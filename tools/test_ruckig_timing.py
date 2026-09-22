"""Current waypoint contract and meaningful checks on the measurement itself."""
from dataclasses import replace

import numpy as np
import pytest

from dual_crx_control.interpolation.ruckig import (
    RuckigInterpolation, MAX_VELOCITY, MAX_ACCELERATION)
from ruckig_timing import Case, DT, limits, schedule, simulate, summarize


@pytest.mark.parametrize('profile', ['head', 'worktree'])
@pytest.mark.parametrize('joint', [1, 4, 5])
@pytest.mark.parametrize('arms', ['left', 'right', 'both'])
def test_low_rate_waypoints_reproduce_hold_and_obey_limits(profile, joint, arms):
    result, trace = simulate(Case(joint=joint, arms=arms), limits(profile))
    assert np.isfinite(trace).all()
    assert result['native']['within_limits']
    for metrics in result['metrics'].values():
        assert metrics['mean_inter_target_hold_s'] > .025
        assert metrics['final_error_rad'] < 1e-10
        assert metrics['overshoot_rad'] < 1e-10
        assert all(t is not None and 0. < t < .1 for t in metrics['settle_duration_s'])


@pytest.mark.parametrize('timing', ['fixed', 'jitter', 'dropout', 'pause'])
@pytest.mark.parametrize('pattern', ['ramp', 'alternating', 'sine'])
def test_retarget_and_dropout_remain_finite_bounded_and_eventually_rest(timing, pattern):
    result, trace = simulate(Case(timing=timing, pattern=pattern), limits('worktree'))
    assert np.isfinite(trace).all()
    assert result['native']['within_limits']
    for m in result['metrics'].values():
        assert m['final_error_rad'] < 1e-10
        assert m['targets'][-1]['hold_s'] > .5
    np.testing.assert_allclose(trace[-100:, 3:], 0., atol=1e-10)


def test_wrapper_one_shot_holds_and_does_not_reset_to_feedback():
    wrapper = RuckigInterpolation(DT)
    initial = {'left': np.zeros(6), 'right': np.full(6, np.pi)}
    velocity = {s: np.zeros(6) for s in initial}
    target = {s: q+1e-4 for s, q in initial.items()}
    wrapper.target(target, initial, velocity)
    for _ in range(10):
        wrapper.step()
    before = wrapper.step()
    wrapper.target(target, {s: np.full(6, 99.) for s in initial}, velocity)
    after = wrapper.step()
    for side in initial:
        assert np.max(abs(after[side]-before[side])) < 1e-4
    for _ in range(500):
        output = wrapper.step()
    for _ in range(100):
        output = wrapper.step()
        for side in initial:
            np.testing.assert_allclose(output[side], target[side], atol=1e-12, rtol=0)


def test_stream_mode_preserves_motion_between_reference_samples_and_times_out_to_hold():
    period = .1
    generator = RuckigInterpolation(
        DT, mode='stream', input_period=period, target_timeout=2.5 * period)
    initial = {'left': np.zeros(6)}
    velocity = {'left': np.zeros(6)}
    samples = []
    for tick in range(500):
        timestamp = tick * DT
        if tick % 50 == 0 and tick < 350:
            target = np.zeros(6)
            target[0] = (tick // 50 + 1) * 1e-4
            generator.target({'left': target}, initial, velocity, timestamp=timestamp)
        samples.append(generator.step(timestamp=timestamp)['left'][0])
        assert np.isfinite(samples[-1])
        assert np.all(np.abs(generator.velocity['left']) <= np.array(MAX_VELOCITY) + 1e-9)
        assert np.all(np.abs(generator.acceleration['left']) <= np.array(MAX_ACCELERATION) + 1e-8)
    differences = np.diff(samples) / DT
    # The first waypoint starts from rest. Subsequent 100 ms stream windows
    # remain active instead of repeating a rest-to-rest plateau.
    assert all(np.count_nonzero(np.abs(differences[start:start+50]) < 1e-8) == 0
               for start in (50, 100, 150, 200, 250, 300))
    np.testing.assert_allclose(samples[-1], 7e-4, atol=1e-10)
    assert np.count_nonzero(np.abs(differences[350:]) < 1e-8) > 30


def test_stream_mode_rejects_invalid_configuration_and_timestamp():
    with pytest.raises(ValueError):
        RuckigInterpolation(DT, mode='stream')
    with pytest.raises(ValueError):
        RuckigInterpolation(DT, mode='invalid', input_period=.1)
    generator = RuckigInterpolation(DT, mode='stream', input_period=.1)
    with pytest.raises(ValueError):
        generator.target({'left': np.zeros(6)}, {'left': np.zeros(6)},
                         {'left': np.zeros(6)}, timestamp=float('nan'))


def test_measurement_separates_first_crossing_from_settle_and_final_hold():
    t = np.arange(11)*.1
    q = np.array([0., .5, 1., 1.2, 1., 1., 1., 1.5, 2., 2., 2.])
    metrics = summarize(t, q, [(0, 0., 1.), (1, .6, 2.)])
    first = metrics['targets'][0]
    assert first['first_arrival_s'] == pytest.approx(.2)
    assert first['settle_s'] == pytest.approx(.5)
    assert first['overshoot_rad'] == pytest.approx(.2)
    assert metrics['mean_inter_target_hold_s'] == pytest.approx(.1)
    assert metrics['peak_velocity_estimate'] == pytest.approx(5.)


def test_crossing_superseded_target_does_not_count_as_settle():
    metrics = summarize(np.arange(8)*.1, np.arange(8)*.1, [(0, 0., .2), (1, .4, .7)])
    assert metrics['targets'][0]['first_arrival_s'] == pytest.approx(.2)
    assert metrics['targets'][0]['settle_s'] is None
    assert metrics['mean_inter_target_hold_s'] == 0.


def test_deterministic_schedule_accounts_for_intentional_loss_and_pause():
    case = Case()
    events = schedule(case)
    assert events == schedule(case)
    assert len(schedule(replace(case, timing='dropout'))) == len(events)-1
    jitter = np.diff([t for _, t, _ in schedule(replace(case, timing='jitter'))])
    assert set(np.round(jitter, 2)) == {.09, .11}
    pause = np.diff([t for _, t, _ in schedule(replace(case, timing='pause'))])
    assert max(pause) == pytest.approx(.6)


@pytest.mark.parametrize('method', ['linear', 'cubic'])
def test_comparison_methods_finish_and_hold(method):
    result, _ = simulate(Case(method=method), limits('worktree'))
    for metrics in result['metrics'].values():
        assert metrics['final_error_rad'] < 1e-10
        assert metrics['mean_inter_target_hold_s'] < .005
