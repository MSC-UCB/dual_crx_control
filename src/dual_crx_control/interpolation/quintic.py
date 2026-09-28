"""Period-paced quintic references with continuous q/v/a and bounded derivatives.

The nominal horizon is one input period. Infeasible horizons are extended, never
forced by clipping samples. Bernstein convex-hull bounds certify the complete
polynomial, including between the 500 Hz output samples.
"""
from math import ceil, comb, factorial

import numpy as np


def _derivative_hulls(order):
    """Map normalized power coefficients to Bernstein controls on eight intervals."""
    degree = 5 - order
    rows = []
    for start in np.arange(8) / 8:
        for k in range(degree + 1):
            row = np.zeros(6)
            for power in range(order, 6):
                n = power - order
                row[power] = factorial(power) / factorial(n) * sum(
                    comb(n, i) * start ** (n-i) / 8**i * comb(k, i) / comb(degree, i)
                    for i in range(min(k, n) + 1))
            rows.append(row)
    return np.array(rows)


_HULLS = tuple(_derivative_hulls(order) for order in (1, 2, 3))


class QuinticSegment:
    def __init__(self, q, v, a, target, duration):
        self.start = q.copy()
        self.target = target.copy()
        self.duration = duration
        d, vt, at = target - q, v * duration, a * duration**2
        self.coefficients = np.array([
            np.zeros_like(q), vt, at/2,
            10*d - 6*vt - 1.5*at,
            -15*d + 8*vt + 1.5*at,
            6*d - 3*vt - .5*at,
        ])

    def bounds(self):
        return np.array([np.max(np.abs(matrix @ self.coefficients), axis=0) / self.duration**order
                         for order, matrix in enumerate(_HULLS, 1)])

    def sample(self, elapsed):
        if elapsed >= self.duration:
            return self.target.copy(), np.zeros_like(self.target), np.zeros_like(self.target)
        u = max(0., elapsed / self.duration)
        c = self.coefficients
        q = self.start + np.polynomial.polynomial.polyval(u, c)
        v = np.polynomial.polynomial.polyval(u, c[1:] * np.arange(1, 6)[:, None]) / self.duration
        a = np.polynomial.polynomial.polyval(u, c[2:] * np.array([2, 6, 12, 20])[:, None]) / self.duration**2
        return q, v, a


class QuinticInterpolation:
    """Synchronize active arms; use the latest target and finish at zero v/a.

    Like the Ruckig wrapper, advance one fixed dt per step. A new target starts
    from the last output state. Identical targets do not restart the horizon.
    """
    def __init__(self, dt, *, input_period, max_velocity, max_acceleration, max_jerk):
        if not np.isfinite([dt, input_period]).all() or min(dt, input_period) <= 0:
            raise ValueError('Quintic timestep and input period must be positive and finite')
        self.limits = np.asarray([max_velocity, max_acceleration, max_jerk], dtype=float)
        if self.limits.shape != (3, 6) or not np.isfinite(self.limits).all() or np.any(self.limits <= 0):
            raise ValueError('Expected six positive finite limits for each derivative')
        self.dt, self.input_period = dt, input_period
        self.active = set()
        self.positions, self.velocity, self.acceleration, self.targets = {}, {}, {}, {}
        self.segment = None
        self.elapsed = 0.
        self.duration = 0.
        self.extended_count = 0

    @staticmethod
    def _vector(value):
        value = np.asarray(value, dtype=float)
        if value.shape != (6,) or not np.isfinite(value).all():
            raise ValueError('Quintic states must be finite six-joint vectors')
        return value.copy()

    def target(self, commands, positions, velocities, timestamp=None):
        if timestamp is not None and not np.isfinite(timestamp):
            raise ValueError('Quintic target timestamp must be finite')
        if not commands or any(s not in ('left', 'right') for s in commands):
            raise ValueError('Expected left and/or right arm targets')
        targets = dict(self.targets)
        targets.update({s: self._vector(q) for s, q in commands.items()})
        if targets.keys() == self.targets.keys() and all(
                np.array_equal(q, self.targets[s]) for s, q in targets.items()):
            return
        sides = tuple(s for s in ('left', 'right') if s in targets)
        q = np.concatenate([self.positions[s] if s in self.active else self._vector(positions[s]) for s in sides])
        v = np.concatenate([self.velocity[s] if s in self.active else self._vector(velocities[s]) for s in sides])
        a = np.concatenate([self.acceleration[s] if s in self.active else np.zeros(6) for s in sides])
        limits = np.tile(self.limits, (1, len(sides)))
        if np.any(np.abs(v) > limits[0]) or np.any(np.abs(a) > limits[1]):
            raise ValueError('Initial quintic state exceeds velocity or acceleration limits')
        destination = np.concatenate([targets[s] for s in sides])
        duration = max(1, ceil(self.input_period / self.dt - 1e-12)) * self.dt
        # A sufficient bound, not an exact minimum-time solve. Increase the
        # horizon until the complete curve is certified; preserve q/v/a.
        for _ in range(80):
            segment = QuinticSegment(q, v, a, destination, duration)
            bounds = segment.bounds()
            if np.isfinite(bounds).all() and np.all(bounds <= limits * (1 + 1e-12)):
                break
            duration = max(duration + self.dt, ceil(duration * 1.2 / self.dt) * self.dt)
        else:
            # Reject before changing any state: the previous trajectory continues.
            raise ValueError('Unable to find a bounded quintic horizon; retaining previous plan')
        self.targets = targets
        self.active = set(sides)
        self.sides = sides
        self.segment, self.duration, self.elapsed = segment, duration, 0.
        self.extended_count += int(duration > self.input_period + self.dt)
        for i, side in enumerate(sides):
            sl = slice(6*i, 6*i+6)
            self.positions[side], self.velocity[side], self.acceleration[side] = q[sl].copy(), v[sl].copy(), a[sl].copy()

    def step(self, timestamp=None):
        if timestamp is not None and not np.isfinite(timestamp):
            raise ValueError('Quintic step timestamp must be finite')
        if self.segment is None:
            return {}
        self.elapsed = min(self.duration, self.elapsed + self.dt)
        if self.duration - self.elapsed < self.dt * 1e-9:
            self.elapsed = self.duration
        q, v, a = self.segment.sample(self.elapsed)
        for i, side in enumerate(self.sides):
            sl = slice(6*i, 6*i+6)
            self.positions[side], self.velocity[side], self.acceleration[side] = q[sl].copy(), v[sl].copy(), a[sl].copy()
        return {s: q.copy() for s, q in self.positions.items()}
