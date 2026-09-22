"""Time-synchronized Ruckig reference state for one or both arms."""
import math
import time

import numpy as np

from dual_crx_control._ruckig import Generator
from dual_crx_control.robot.joint_config import SIDES

# CRX-5iA model: fanuc_crx_description/urdf/crx5ia_urdf_macro.xacro.
MAX_VELOCITY = tuple(math.radians(v) for v in (150, 150, 180, 225, 225, 225))
# Experimental planning values, NOT verified CRX-5iA hardware acceleration limits.
# Starting point: fanuc_moveit_config/config/joint_limits.yaml (CRX-10iA).
MAX_ACCELERATION = (3.0, 3.0, 4.5, 4.5, 4.5, 4.5)
# Experimental 0.1 s acceleration ramp: jerk = acceleration / 0.1.
MAX_JERK = (30.0, 30.0, 45, 45.0, 45.0, 45.0)


class RuckigInterpolation:
    def __init__(self, dt, *, mode='waypoint', input_period=None, target_timeout=None):
        if mode not in ('waypoint', 'stream'):
            raise ValueError('Ruckig target mode must be waypoint or stream')
        if mode == 'stream' and (
                input_period is None or not math.isfinite(input_period) or input_period <= 0):
            raise ValueError('stream mode requires a positive input period')
        if target_timeout is not None and (
                not math.isfinite(target_timeout) or target_timeout <= 0):
            raise ValueError('Ruckig target timeout must be positive')
        self.generator = Generator(dt, MAX_VELOCITY, MAX_ACCELERATION, MAX_JERK)
        self.active = set()
        self.mode = mode
        self.input_period = input_period
        self.target_timeout = target_timeout
        self.stream_state = {}
        self.last_step_time = None
        self.velocity = {}
        self.acceleration = {}

    def target(self, commands, positions, velocities, timestamp=None):
        timestamp = time.monotonic() if timestamp is None else float(timestamp)
        if not math.isfinite(timestamp):
            raise ValueError('Ruckig target timestamp must be finite')
        for side, q in commands.items():
            q = np.asarray(q, dtype=float)
            if q.shape != (6,) or not np.isfinite(q).all():
                raise ValueError('Ruckig targets must be finite six-joint vectors')
            target_velocity = np.zeros(6)
            target_acceleration = np.zeros(6)
            if self.mode == 'stream':
                state = self.stream_state.get(side)
                if state is not None:
                    elapsed = timestamp - state['timestamp']
                    if elapsed <= 0:
                        elapsed = self.input_period
                    target_velocity = np.clip(
                        (q - state['target']) / elapsed,
                        -np.asarray(MAX_VELOCITY), np.asarray(MAX_VELOCITY))
                    target_acceleration = np.clip(
                        (target_velocity - state['velocity']) / elapsed,
                        -np.asarray(MAX_ACCELERATION), np.asarray(MAX_ACCELERATION))
                self.stream_state[side] = {
                    'target': q.copy(), 'timestamp': timestamp,
                    'velocity': target_velocity.copy(),
                    'acceleration': target_acceleration.copy(), 'holding': False,
                }
            self.generator.target(
                SIDES.index(side), q, positions[side], velocities[side],
                target_velocity, target_acceleration)
            self.active.add(side)

    def step(self, timestamp=None):
        timestamp = time.monotonic() if timestamp is None else float(timestamp)
        if not math.isfinite(timestamp):
            raise ValueError('Ruckig step timestamp must be finite')
        if self.mode == 'stream' and self.target_timeout is not None:
            for side in tuple(self.active):
                state = self.stream_state.get(side)
                if state is None or state['holding']:
                    continue
                if timestamp - state['timestamp'] >= self.target_timeout:
                    zeros = np.zeros(6)
                    self.generator.target(
                        SIDES.index(side), state['target'], zeros, zeros,
                        zeros, zeros)
                    state['holding'] = True
        positions, velocities, accelerations = self.generator.step()
        self.last_step_time = timestamp
        q = np.asarray(positions).reshape(2, 6)
        v = np.asarray(velocities).reshape(2, 6)
        a = np.asarray(accelerations).reshape(2, 6)
        self.velocity = {s: v[i].copy() for i, s in enumerate(SIDES) if s in self.active}
        self.acceleration = {s: a[i].copy() for i, s in enumerate(SIDES) if s in self.active}
        return {s: q[i].copy() for i, s in enumerate(SIDES) if s in self.active}
