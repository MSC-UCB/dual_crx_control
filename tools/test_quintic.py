"""Period coverage, whole-curve limits, and continuous online retargeting."""
import numpy as np
import pytest

from dual_crx_control.interpolation.quintic import QuinticInterpolation, QuinticSegment
from dual_crx_control.interpolation.ruckig import MAX_VELOCITY, MAX_ACCELERATION, MAX_JERK

DT = .002
LIMITS = np.array([MAX_VELOCITY, MAX_ACCELERATION, MAX_JERK])


def planner(period=.1):
    return QuinticInterpolation(DT, input_period=period, max_velocity=MAX_VELOCITY,
                                max_acceleration=MAX_ACCELERATION, max_jerk=MAX_JERK)


def test_small_move_uses_whole_period_and_zero_terminal_derivatives():
    p = planner()
    initial = {'left': np.zeros(6), 'right': np.full(6, np.pi)}
    velocity = {s: np.zeros(6) for s in initial}
    target = {s: q + 1e-4 for s, q in initial.items()}
    p.target(target, initial, velocity, timestamp=0.)
    assert p.duration == pytest.approx(.1)
    for k in range(1, 51):
        q = p.step(timestamp=k*DT)
        u = k/50
        for side in initial:
            np.testing.assert_allclose(q[side], initial[side] + 1e-4*(10*u**3-15*u**4+6*u**5), atol=1e-14)
            if k < 50:
                assert np.all(p.velocity[side] > 0)
            if k < 25:
                assert np.all(p.acceleration[side] > 0)
            elif 25 < k < 50:
                assert np.all(p.acceleration[side] < 0)
    for side in initial:
        np.testing.assert_array_equal(q[side], target[side])
        np.testing.assert_array_equal(p.velocity[side], np.zeros(6))
        np.testing.assert_array_equal(p.acceleration[side], np.zeros(6))


def test_infeasible_period_extends_and_continuous_curve_obeys_limits():
    p = planner(.02)
    initial = {'left': np.zeros(6)}
    target = {'left': np.array([.02, -.03, .04, -.05, .06, -.07])}
    p.target(target, initial, initial)
    assert p.duration > .02
    assert p.extended_count == 1
    c, duration = p.segment.coefficients, p.duration
    # Dense sub-tick evaluation independently verifies all three derivatives.
    for order in (1, 2, 3):
        derivative = np.polynomial.polynomial.polyder(c, m=order, axis=0)
        values = np.polynomial.polynomial.polyval(np.linspace(0, 1, 10001), derivative) / duration**order
        assert np.all(np.max(abs(values), axis=1) <= LIMITS[order-1]+1e-10)
    for _ in range(round(duration/DT)):
        output = p.step()
    np.testing.assert_array_equal(output['left'], target['left'])
    np.testing.assert_array_equal(p.velocity['left'], np.zeros(6))


@pytest.mark.parametrize('period', [.02, .1])
def test_retarget_reversal_dropout_and_resume_preserve_state_and_limits(period):
    p = planner(period)
    rng = np.random.default_rng(8)
    initial = {'left': np.zeros(6), 'right': np.full(6, np.pi)}
    zeros = {s: np.zeros(6) for s in initial}
    previous_a = {s: np.zeros(6) for s in initial}
    for k in range(2000):
        if k % round(period/DT) == 0 and not 450 < k < 800 and k < 1100:
            targets = {s: q + rng.uniform(-.01, .01, 6) for s, q in initial.items()}
            before = {s: (p.positions[s].copy(), p.velocity[s].copy(), p.acceleration[s].copy())
                      for s in p.active}
            p.target(targets, initial, zeros, timestamp=k*DT)
            if before:
                for actual, expected in zip(p.segment.sample(0.),
                                            [np.concatenate([before[t][i] for t in p.sides]) for i in range(3)]):
                    np.testing.assert_allclose(actual, expected, atol=1e-13)
        output = p.step(timestamp=k*DT)
        for side in p.active:
            assert np.isfinite(output[side]).all()
            assert np.all(abs(p.velocity[side]) <= LIMITS[0]+1e-10)
            assert np.all(abs(p.acceleration[side]) <= LIMITS[1]+1e-10)
            assert np.all(abs(p.acceleration[side]-previous_a[side])/DT <= LIMITS[2]+1e-8)
            previous_a[side] = p.acceleration[side].copy()
    for side in initial:
        np.testing.assert_array_equal(output[side], targets[side])
        np.testing.assert_array_equal(p.velocity[side], np.zeros(6))


def test_identical_targets_do_not_restart_and_bad_input_does_not_replace_plan():
    p = planner()
    initial = {'left': np.zeros(6)}
    target = {'left': np.full(6, 1e-4)}
    p.target(target, initial, initial)
    segment = p.segment
    for _ in range(50):
        p.target(target, initial, initial)
        p.step()
    assert p.segment is segment
    np.testing.assert_array_equal(p.positions['left'], target['left'])
    with pytest.raises(ValueError):
        p.target({'right': np.full(6, np.nan)}, initial, initial)
    assert p.segment is segment and p.active == {'left'}


def test_second_arm_initialization_and_partial_update_preserve_existing_state():
    p = planner()
    initial = {'left': np.zeros(6), 'right': np.full(6, np.pi)}
    velocity = {'left': np.zeros(6), 'right': np.full(6, .02)}
    p.target({'left': np.full(6, .01)}, initial, velocity)
    for _ in range(10):
        p.step()
    before = p.positions['left'].copy(), p.velocity['left'].copy(), p.acceleration['left'].copy()
    p.target({'right': initial['right']+.02}, initial, velocity)
    for state, expected in zip(p.segment.sample(0.), before):
        np.testing.assert_allclose(state[:6], expected, atol=1e-14)
    np.testing.assert_array_equal(p.velocity['right'], velocity['right'])
    assert p.targets['left'][0] == .01


def test_bernstein_certificate_bounds_arbitrary_polynomials_between_samples():
    rng = np.random.default_rng(42)
    for _ in range(20):
        q, v, a, target = rng.normal(size=(4, 12))
        segment = QuinticSegment(q, v, a, target, rng.uniform(.02, 2.))
        bounds = segment.bounds()
        for order in (1, 2, 3):
            c = np.polynomial.polynomial.polyder(segment.coefficients, m=order, axis=0)
            values = np.polynomial.polynomial.polyval(np.linspace(0, 1, 10001), c) / segment.duration**order
            assert np.all(np.max(abs(values), axis=1) <= bounds[order-1]+1e-7)


@pytest.mark.parametrize('period', [0., -1., float('nan'), float('inf')])
def test_invalid_period_is_rejected(period):
    with pytest.raises(ValueError):
        planner(period)
