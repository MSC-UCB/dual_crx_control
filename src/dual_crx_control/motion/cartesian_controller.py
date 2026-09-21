#!/usr/bin/env python3
"""Shared-phase Cartesian translation, publishing validated joint command pairs."""

from functools import partial
import math
import time
from xml.parsers.expat import ExpatError

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from dual_crx_control.robot.kinematics import CRXKinematics
from dual_crx_control.robot.ik_solver import DampedLeastSquaresIK
from dual_crx_control.motion.startup_motion import INITIAL_JOINTS_DEG, InitialJointMove
from dual_crx_control.interpolation.client import JointTargetClient
from dual_crx_control.analysis.motion_recording import JointRecording, MotionRecording


class DualCartesianController(Node):
    DEFAULTS = dict(rate=50., max_target_step=0.1,
                        amplitude=0.02, period=4., axis='x', ramp_time=1.,
                        state_timeout=0.25, ready_timeout=10., max_cycle_step=0.03,
                        max_velocity=0.5, damping=0.01, position_tolerance=1e-5,
                        orientation_tolerance=1e-5, max_iterations=100,
                        ik_max_joint_step=0.05, ik_alpha=1., cycles=0, final_hold=0.5,
                        initial_move_time=5., initial_max_velocity=0.1,
                        initial_max_acceleration=0.1, initial_tolerance=0.005,
                        initial_settle_time=0.5, initial_settle_timeout=5.,
                        initial_tracking_tolerance=0.1)

    def __init__(self, *, node_name='cartesian_sine', motion_defaults=None, **kwargs):
        super().__init__(node_name, **kwargs)
        defaults = self.DEFAULTS.copy()
        defaults.update(motion_defaults or {})
        self.settings = {k: self.declare_parameter(k, v).value for k, v in defaults.items()}
        p = self.settings
        if not 0 < p['rate'] <= 500:
            raise ValueError('rate must be in (0, 500], matching interpolation input_rate_hz')
        for key in defaults:
            if key == 'axis':
                continue
            value = p[key]
            if not np.isfinite(value) or value < 0 or (value == 0 and key not in ('amplitude', 'ramp_time', 'cycles')):
                raise ValueError(f'{key} must be finite and positive (amplitude/ramp may be zero)')
        if p['axis'] not in ('x', 'y', 'z'):
            raise ValueError('axis must be x, y, or z')
        if not isinstance(p['cycles'], int) or isinstance(p['cycles'], bool):
            raise ValueError('cycles must be a nonnegative integer; zero repeats indefinitely')
        if p['cycles'] and p['ramp_time'] <= 0:
            raise ValueError('Finite motion requires ramp_time > 0 for a smooth stop')
        if p['initial_settle_timeout'] <= p['initial_settle_time']:
            raise ValueError('initial_settle_timeout must exceed initial_settle_time')
        self.move_to_initial = self.declare_parameter('move_to_initial', True).value
        self.initial_targets = {
            side: np.radians(self.declare_parameter(f'{side}_initial_deg', angles).value)
            for side, angles in INITIAL_JOINTS_DEG.items()}
        self.initial_move = None
        self.initial_move_started = None
        self.initial_settled_since = None
        self.axis = ('x', 'y', 'z').index(p['axis'])
        self.save_plot = self.declare_parameter('save_plot', True).value
        self.output_dir = self.declare_parameter('output_dir', 'motion_recordings').value
        self.recording = MotionRecording(self.axis)
        description = self.declare_parameter('robot_description', '').value
        self.models = {}
        self.description = ''
        self.solvers = {}
        self.arm_publishers = {}
        self.target_client = JointTargetClient(self)
        self.positions, self.received = {}, {}
        self.starts = None
        self.previous = {}
        self.failed = False
        self.finished = False
        self.started_at = None
        self.last_publish = None
        self.segment_target = None
        self.ik_dt = 1. / p['rate']
        self.command_count = 0
        self.ik_count = 0
        self.first_command_time = None
        self.first_ik_time = None
        self.last_ik_time = None
        self.last_rejection_time = -math.inf
        self.cartesian_complete = False
        self.created_at = time.monotonic()
        self.timer = self.create_timer(self.ik_dt, self.cycle, clock=Clock(clock_type=ClockType.STEADY_TIME))
        if description:
            self.configure_model(description)
        else:
            self.create_subscription(
                String, '/robot_description', self.description_callback,
                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.get_logger().info('Waiting for robot description, both joint states, and command subscribers...')
        self.get_logger().info(f"Joint target rate: {p['rate']} Hz; output owned by interpolation node")
        self.create_subscription(JointState, 'interpolated_joint_commands', self.record_command, 10)
        self.joint_recording = JointRecording(self.output_dir, target_source='target') if self.save_plot else None
        if self.joint_recording is not None:
            self.create_timer(1., self.joint_recording.flush, clock=Clock(clock_type=ClockType.STEADY_TIME))


    def description_callback(self, message):
        if self.failed or self.finished:
            return
        if self.description:
            if message.data != self.description:
                self.abort('robot_description changed; restart against the new model')
            return
        try:
            self.configure_model(message.data)
        except (ValueError, KeyError, RuntimeError, SyntaxError, ExpatError) as exc:
            self.abort(f'invalid robot_description: {exc}')

    def configure_model(self, description):
        self.models = {side: CRXKinematics(description, f'{side}_tcp') for side in ('left', 'right')}
        p = self.settings
        for side, model in self.models.items():
            if model.joint_names != [f'{side}_J{i}' for i in range(1, 7)]:
                raise ValueError(f'{side}: command order must be J1..J6')
            if not model.valid_joints(self.initial_targets[side]):
                raise ValueError(f'{side}: invalid initial joint target / joint-limit violation')
            self.solvers[side] = DampedLeastSquaresIK(
                model, damping=p['damping'], position_tolerance=p['position_tolerance'],
                orientation_tolerance=p['orientation_tolerance'], max_iterations=p['max_iterations'],
                max_joint_step=p['ik_max_joint_step'], alpha=p['ik_alpha'])
            self.arm_publishers[side] = self.target_client.arm_publisher(side)
            self.create_subscription(JointState, f'{side}/joint_states',
                                     partial(self.feedback, side), qos_profile_sensor_data)
        self.description = description

    def abort(self, reason):
        if not self.failed:
            self.failed = True
            self.timer.cancel()
            self.get_logger().error(f'ABORT: {reason}; no further targets; interpolation holds the last accepted endpoint. Restart required.')

    def feedback(self, side, message):
        if len(message.name) != len(message.position) or len(set(message.name)) != len(message.name):
            self.invalid_feedback(side, 'malformed joint feedback')
            return
        by_name = dict(zip(message.name, message.position))
        if not all(name in by_name for name in self.models[side].joint_names):
            self.invalid_feedback(side, 'missing joints in feedback')
            return
        q = np.array([by_name[name] for name in self.models[side].joint_names])
        if not self.models[side].valid_joints(q):
            self.invalid_feedback(side, 'invalid feedback / joint-limit violation')
            return
        self.positions[side] = q
        self.received[side] = time.monotonic()
        if self.save_plot:
            stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
            self.recording.measured(side, self.received[side], q, stamp)
            self.joint_recording.add(self.received[side], side, 'feedback', q)

    def invalid_feedback(self, side, reason):
        self.received.pop(side, None)
        if self.starts is None:
            self.abort(f'{side}: {reason}')
        else:
            self.reject_target(f'{side}: {reason}')

    def readiness_error(self, now):
        for side in self.models:
            if side not in self.received or now - self.received[side] > self.settings['state_timeout']:
                return f'{side}: stale or missing feedback'
            if self.arm_publishers[side].get_subscription_count() == 0:
                return f'{side}: missing command subscriber'
        return None

    def cycle(self):
        if self.failed or self.finished:
            return
        try:
            if self.cartesian_complete:
                return  # The output subscription observes the last joint target.
            self._cycle()
        except (ValueError, RuntimeError, np.linalg.LinAlgError) as exc:
            self.abort(str(exc))

    def reject_target(self, reason):
        now = time.monotonic()
        if now - self.last_rejection_time >= 1.:
            self.get_logger().warning(f'Target pair rejected: {reason}; retaining previous segment for both arms.')
            self.last_rejection_time = now

    def accept_targets(self, commands):
        pair = np.array([commands[s] for s in ('left', 'right')])
        old = (self.segment_target if self.segment_target is not None
               else np.array([self.previous[s] for s in ('left', 'right')]))
        for i, side in enumerate(('left', 'right')):
            if not self.models[side].valid_joints(pair[i]):
                raise ValueError(f'{side}: invalid IK target / joint-limit violation')
            delta = np.abs(pair[i] - old[i])
            if np.max(delta) > self.settings['max_target_step']:
                raise ValueError(f'{side}: target step exceeds max_target_step')
            if np.any(delta / self.ik_dt > np.minimum(self.models[side].velocity,
                                                     self.settings['max_velocity'])):
                raise ValueError(f'{side}: target segment exceeds commanded joint velocity limit')
        # Commit the entire pair only after all validation succeeds.
        self.segment_target = pair.copy()
        self.publish_pair(commands)

    def initial_move_cycle(self, now):
        """Return true only after both arms have reached and held their targets."""
        p = self.settings
        if self.initial_move is None:
            self.previous = {s: q.copy() for s, q in self.positions.items()}
            self.initial_move = InitialJointMove(
                self.models, self.previous, self.initial_targets, p['initial_move_time'],
                min(p['initial_max_velocity'], p['max_velocity']), p['initial_max_acceleration'])
            self.initial_move_started = now
            self.get_logger().info(
                f'Moving both arms to initial joints over {self.initial_move.duration:.2f} s.')
            for side in self.models:
                self.get_logger().info(f'{side} initial target [deg]: '
                                       f'{np.degrees(self.initial_targets[side]).tolist()}')
        for side in self.models:
            tracking = float(np.max(np.abs(self.positions[side] - self.previous[side])))
            if tracking > p['initial_tracking_tolerance']:
                raise ValueError(f'{side}: initial-move tracking error {tracking:.6g} rad')
        elapsed = now - self.initial_move_started
        self.accept_targets(self.initial_move.sample(elapsed))
        if elapsed < self.initial_move.duration:
            return False
        errors = {s: float(np.max(np.abs(self.positions[s] - self.initial_targets[s])))
                  for s in self.models}
        if all(error <= p['initial_tolerance'] for error in errors.values()):
            if self.initial_settled_since is None:
                self.initial_settled_since = now
            if now - self.initial_settled_since >= p['initial_settle_time']:
                self.get_logger().info('Both arms reached initial joints; starting Cartesian motion next cycle.')
                return True
        else:
            self.initial_settled_since = None
        if elapsed > self.initial_move.duration + p['initial_settle_timeout']:
            raise ValueError(f'Initial-pose settling timeout; joint errors [rad]: {errors}')
        return False

    def _cycle(self):
        now = time.monotonic()
        if not self.description:
            if now - self.created_at > self.settings['ready_timeout']:
                self.abort('missing robot_description from robot bringup')
            return
        error = self.readiness_error(now)
        if error:
            if self.starts is not None:
                self.reject_target(error)
                return
            if self.initial_move is not None or self.starts is not None or now - self.created_at > self.settings['ready_timeout']:
                self.abort(error)
            return
        if self.move_to_initial and self.starts is None:
            if self.initial_move_cycle(now):
                self.move_to_initial = False
            return
        if self.starts is None and not self.prepare_trajectory(now):
            return
        if self.starts is None:
            if not self.previous:
                self.previous = {s: q.copy() for s, q in self.positions.items()}
            # Preserve the last commanded seed, and capture measured TCP poses
            # only after both arms have settled at the initial joints.
            self.starts = {s: m.fk(self.positions[s]) for s, m in self.models.items()}
            self.started_at = now
            if self.save_plot:
                self.recording.begin(now, self.starts)
            if self.segment_target is None:
                self.segment_target = np.array([self.previous[s] for s in ('left', 'right')])
            for side in self.models:
                self.get_logger().info(f'{side} initial q [rad]: {self.previous[side].tolist()}')
                self.get_logger().info(f'{side} initial TCP in world (4x4):\n{self.starts[side]}')
            p = self.settings
            self.get_logger().info(self.trajectory_description())
        elapsed = now - self.started_at
        p = self.settings
        duration = p['cycles'] * p['period'] if p['cycles'] else math.inf
        offset = self.trajectory_offset(elapsed)
        commands = {}
        self.ik_count += 1
        self.last_ik_time = now
        if self.first_ik_time is None:
            self.first_ik_time = now
        # Both IK solutions are computed and validated before either publication.
        for side in self.models:
            target = self.starts[side].copy()
            target[:3, 3] += offset
            result = self.solvers[side].solve(target, self.segment_target[('left', 'right').index(side)])
            if not result.success:
                self.reject_target(f'IK failed for {side}: {result.reason}; '
                                 f'position error={result.position_error:.6g} m, '
                                 f'orientation error={result.orientation_error:.6g} rad, '
                                 f'iterations={result.iterations}')
                return
            commands[side] = result.q
        error = self.readiness_error(time.monotonic())
        if error:
            self.reject_target(error)
            return
        try:
            self.validate_trajectory_targets(commands)
            self.accept_targets(commands)
        except ValueError as exc:
            self.reject_target(str(exc))
            return
        if elapsed >= duration + p['final_hold']:
            self.cartesian_complete = True

    def trajectory_description(self):
        p = self.settings
        return (f"IK rate={p['rate']} Hz, amplitude={p['amplitude']} m, "
                f"period={p['period']} s, axis={p['axis']}; orientation fixed")

    def prepare_trajectory(self, now):
        """Optional extra approach before capturing the Cartesian starting poses."""
        return True

    def validate_trajectory_targets(self, commands):
        """Optional trajectory-specific target validation before committing a pair."""
        return

    def trajectory_offset(self, elapsed):
        p = self.settings
        duration = p['cycles'] * p['period'] if p['cycles'] else math.inf
        blend = min(elapsed / p['ramp_time'], 1.) if p['ramp_time'] else 1.
        blend = blend**3 * (10. + blend * (-15. + 6. * blend))
        if p['cycles']:
            remaining = min(max((duration - elapsed) / p['ramp_time'], 0.), 1.)
            blend *= remaining**3 * (10. + remaining * (-15. + 6. * remaining))
        offset = np.zeros(3)
        offset[self.axis] = p['amplitude'] * blend * math.sin(2. * math.pi * elapsed / p['period'])
        return offset

    def publish_pair(self, commands):
        published_at = time.monotonic()
        self.target_client.publish(commands)
        if self.save_plot:
            self.recording.target(published_at, commands)
            for side, q in commands.items():
                self.joint_recording.add(published_at, side, 'target', q)
        self.previous = {s: q.copy() for s, q in commands.items()}

    def record_command(self, message):
        if self.finished or self.failed or not self.models:
            return
        if len(message.name) != len(message.position) or len(set(message.name)) != len(message.name):
            return
        by_name = dict(zip(message.name, message.position))
        if any(n not in by_name for model in self.models.values() for n in model.joint_names):
            return
        commands = {s: np.array([by_name[n] for n in m.joint_names]) for s, m in self.models.items()}
        if any(not np.isfinite(q).all() for q in commands.values()):
            return
        received_at = time.monotonic()
        now = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        self.last_publish = now
        self.command_count += 1
        if self.first_command_time is None:
            self.first_command_time = now
        if self.save_plot:
            self.recording.command(now, commands)
            for side, q in commands.items():
                self.joint_recording.add(received_at, side, 'interpolated', q)
        if self.cartesian_complete and all(
                np.array_equal(commands[s], self.segment_target[i]) for i, s in enumerate(('left', 'right'))):
            self.finished = True
            self.timer.cancel()
            self.get_logger().info('Finite motion finished; interpolation keeps holding the last joint target.')

    def report_rates(self):
        command_span = (self.last_publish or 0.) - (self.first_command_time or 0.)
        ik_span = (self.last_ik_time or 0.) - (self.first_ik_time or 0.)
        command_rate = (self.command_count - 1) / command_span if command_span > 0 else 0.
        ik_rate = (self.ik_count - 1) / ik_span if ik_span > 0 else 0.
        summary = (f'Achieved command rate: {command_rate:.2f} Hz; '
                   f'IK rate: {ik_rate:.2f} Hz; command pairs: {self.command_count}; '
                   f'IK updates: {self.ik_count}')
        if self.context.ok():
            self.get_logger().info(summary)
        else:
            print(summary, flush=True)


def main(controller_type=DualCartesianController, parameter_overrides=None):
    rclpy.init()
    node = None
    try:
        node = controller_type(parameter_overrides=parameter_overrides)
        while rclpy.ok() and not node.failed and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.report_rates()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if node is not None and node.save_plot:
            try:
                for path in node.joint_recording.save():
                    print(f'Saved {path}', flush=True)
                settings = {name: node.get_parameter(name).value
                            for name in node.list_parameters([], depth=0).names}
                settings.pop('robot_description', None)
                paths = node.recording.save(node.models, node.joint_recording.output, settings=settings)
                if paths:
                    print('Saved command-versus-feedback plot: ' + str(paths[0]), flush=True)
                    print('Saved motion data: ' + str(paths[1]), flush=True)
            except Exception as exc:
                print(f'Could not save motion plot/data: {exc}', flush=True)

    if node is not None and node.failed:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
