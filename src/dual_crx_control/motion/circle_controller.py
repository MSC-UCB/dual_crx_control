#!/usr/bin/env python3
"""Both TCPs draw circles simultaneously with fixed initial orientations."""

import numpy as np

from dual_crx_control.motion.cartesian_controller import DualCartesianController, main as run_controller
from dual_crx_control.motion.circular_trajectory import CircularTrajectory
from dual_crx_control.analysis.motion_recording import MotionRecording
from dual_crx_control.motion.facing_circle import (facing_start_poses, solve_facing_start,
                                           check_joint_approach, scaled_min_singular_value)
from dual_crx_control.motion.startup_motion import InitialJointMove


class DualCircleController(DualCartesianController):
    CIRCLE_DEFAULTS = dict(radius=0.02, plane='xy', direction='ccw', face_each_other=False,
                          center_midpoint=[0.55, -0.38, 0.30], tcp_gap=0.2,
                          minimum_scaled_sigma=0.1)

    def __init__(self, *, node_name='cartesian_circle',
                 motion_defaults=None, circle_defaults=None, **kwargs):
        timing = {'period': 8., 'cycles': 1}
        timing.update(motion_defaults or {})
        super().__init__(node_name=node_name, motion_defaults=timing, **kwargs)
        defaults = self.CIRCLE_DEFAULTS.copy()
        defaults.update(circle_defaults or {})
        parameters = {k: self.declare_parameter(k, v).value for k, v in defaults.items()}
        radius, plane, direction = (parameters[k] for k in ('radius', 'plane', 'direction'))
        self.circle = CircularTrajectory(radius, self.settings['period'], plane, direction)
        self.recording = MotionRecording(self.circle.axes[0], plane=plane, arms=self.active_arms)
        self.face_each_other = parameters['face_each_other']
        if self.face_each_other and len(self.active_arms) != 2:
            raise ValueError('face_each_other / facing_circle requires both active arms')
        self.center_midpoint = parameters['center_midpoint']
        self.tcp_gap = parameters['tcp_gap']
        self.minimum_sigma = parameters['minimum_scaled_sigma']
        if not np.isfinite(self.minimum_sigma) or self.minimum_sigma <= 0:
            raise ValueError('minimum_scaled_sigma must be finite and positive')
        self.facing_poses = facing_start_poses(self.center_midpoint, self.tcp_gap, radius, plane)
        self.facing_move = None
        self.facing_move_started = None
        self.facing_settled_since = None
        self.facing_ready = False

    def prepare_trajectory(self, now):
        if not self.face_each_other or self.facing_ready:
            return True
        p = self.settings
        if self.facing_move is None:
            starts = {s: q.copy() for s, q in self.positions.items()}
            if not self.previous:
                self.previous = {s: q.copy() for s, q in starts.items()}
            targets = solve_facing_start(self.models, self.solvers, starts, self.facing_poses)
            sigma = check_joint_approach(self.models, starts, targets, self.minimum_sigma)
            self.facing_move = InitialJointMove(
                self.models, starts, targets, p['initial_move_time'],
                min(p['initial_max_velocity'], p['max_velocity']), p['initial_max_acceleration'])
            self.get_logger().info(f'Facing approach prepared: gap={self.tcp_gap} m, '
                                   f'center midpoint={self.center_midpoint}, '
                                   f'minimum sampled scaled sigma={sigma:.4f}; local +X inward.')
            # Planning sends nothing. Let the executor refresh feedback before approach.
            return False
        if self.facing_move_started is None:
            self.facing_move_started = now
            self.get_logger().info(f'Approaching facing circle poses over {self.facing_move.duration:.2f} s.')
        for side in self.models:
            error = np.max(np.abs(self.positions[side] - self.previous[side]))
            if error > p['initial_tracking_tolerance']:
                raise ValueError(f'{side}: facing approach tracking error {error:.6g} rad')
        elapsed = now - self.facing_move_started
        self.accept_targets(self.facing_move.sample(elapsed))
        if elapsed < self.facing_move.duration:
            return False
        settled = all(np.max(np.abs(self.positions[s] - self.facing_move.targets[s]))
                      <= p['initial_tolerance'] for s in self.models)
        if settled:
            if self.facing_settled_since is None:
                self.facing_settled_since = now
            if now - self.facing_settled_since >= p['initial_settle_time']:
                self.facing_ready = True
                self.get_logger().info('Both TCPs face each other; circle starts next cycle.')
        else:
            self.facing_settled_since = None
        if elapsed > self.facing_move.duration + p['initial_settle_timeout']:
            raise ValueError('Facing approach settling timeout')
        return False

    def validate_trajectory_targets(self, commands):
        if self.face_each_other:
            for side, q in commands.items():
                sigma = scaled_min_singular_value(self.models[side], q)
                if sigma < self.minimum_sigma:
                    raise ValueError(f'{side}: scaled Jacobian sigma {sigma:.5f} < {self.minimum_sigma}')

    def trajectory_offset(self, elapsed):
        return self.circle.offset(elapsed, self.settings['cycles'])

    def trajectory_description(self):
        c = self.circle
        return (f'Circle radius={c.radius} m, period={c.period} s/lap, '
                f'world plane={c.plane}, direction={c.direction}; '
                'shared phase, fixed orientation; start lies on the circle perimeter')


def main():
    run_controller(DualCircleController)


if __name__ == '__main__':
    main()
