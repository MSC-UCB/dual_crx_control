"""Resample joint sequences at 500 Hz; hold the last target until another arrives."""
from collections import deque
from functools import partial
import time

import numpy as np
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.msg import ParameterDescriptor
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from dual_crx_control.interpolation.trajectory import JointSegment, OUTPUT_RATE_HZ, validate_rate
from dual_crx_control.robot.joint_config import SIDES, JOINT_NAMES


def joint_targets(message):
    """Validate and order one complete arm or both arms, without motion limits."""
    names = list(message.name)
    if len(names) not in (6, 12) or len(set(names)) != len(names) or len(message.position) != len(names):
        raise ValueError('expected 6 or 12 unique joint names with matching positions')
    arms = tuple(s for s in SIDES if set(JOINT_NAMES[s]).issubset(names))
    if set(names) != {n for s in arms for n in JOINT_NAMES[s]}:
        raise ValueError('target must contain complete left and/or right J1..J6')
    values = dict(zip(names, message.position))
    if not np.isfinite(message.position).all():
        raise ValueError('joint positions must be finite')
    return {s: np.array([values[n] for n in JOINT_NAMES[s]]) for s in arms}


class InterpolationNode(Node):
    def __init__(self, **kwargs):
        super().__init__('joint_interpolation', **kwargs)
        self.input_rate = validate_rate(self.declare_parameter(
            'input_rate_hz', 50.0, ParameterDescriptor(read_only=True)).value)
        self.method = self.declare_parameter(
            'method', 'linear', ParameterDescriptor(read_only=True)).value
        if self.method not in ('linear', 'cubic', 'ruckig'):
            raise ValueError('method must be linear, cubic or ruckig')
        self.ruckig = None
        self.velocities = {}
        if self.method == 'ruckig':
            self.ruckig_target_mode = self.declare_parameter(
                'ruckig_target_mode', 'waypoint', ParameterDescriptor(read_only=True)).value
            if self.ruckig_target_mode not in ('waypoint', 'stream'):
                raise ValueError('ruckig_target_mode must be waypoint or stream')
            self.ruckig_target_timeout = self.declare_parameter(
                'ruckig_target_timeout', 2.0 / self.input_rate,
                ParameterDescriptor(read_only=True)).value
            from dual_crx_control.interpolation.ruckig import (
                RuckigInterpolation, MAX_VELOCITY, MAX_ACCELERATION, MAX_JERK)
            self.ruckig = RuckigInterpolation(
                1 / OUTPUT_RATE_HZ, mode=self.ruckig_target_mode,
                input_period=1 / self.input_rate,
                target_timeout=(self.ruckig_target_timeout
                                if self.ruckig_target_mode == 'stream' else None))
            self.get_logger().info(
                f'Ruckig at {OUTPUT_RATE_HZ:g} Hz; max velocity={MAX_VELOCITY} rad/s; '
                f'experimental max acceleration={MAX_ACCELERATION} rad/s²; '
                f'experimental max jerk={MAX_JERK} rad/s³')
        self.positions, self.segments, self.last_q, self.last_publish = {}, {}, {}, {}
        self.history = {s: deque(maxlen=5) for s in SIDES}
        self.pending = None
        self.arm_publishers = {}
        for side in SIDES:
            self.arm_publishers[side] = self.create_publisher(
                Float64MultiArray, f'{side}/forward_position_controller/commands', 1)
            self.create_subscription(JointState, f'{side}/joint_states',
                                     partial(self.feedback, side), qos_profile_sensor_data)
        self.command_pub = self.create_publisher(JointState, 'interpolated_joint_commands', 10)
        self.create_subscription(JointState, 'joint_targets', self.target, 1)
        self.timer = self.create_timer(1 / OUTPUT_RATE_HZ, self.tick,
                                      clock=Clock(clock_type=ClockType.STEADY_TIME))

    def feedback(self, side, msg):
        # Feedback initializes an arm once; it is not a runtime watchdog.
        if side in self.last_q or (self.ruckig is not None and side in self.ruckig.active):
            return
        if (len(msg.name) != len(msg.position) or len(set(msg.name)) != len(msg.name)
                or not set(JOINT_NAMES[side]).issubset(msg.name)):
            return
        values = dict(zip(msg.name, msg.position))
        q = np.array([values[n] for n in JOINT_NAMES[side]])
        if not np.isfinite(q).all():
            return
        self.positions[side] = q
        if self.ruckig is not None:
            velocity = dict(zip(msg.name, msg.velocity))
            if len(msg.velocity) == len(msg.name) and np.isfinite(msg.velocity).all():
                self.velocities[side] = np.array([velocity[n] for n in JOINT_NAMES[side]])
            else:
                self.velocities[side] = np.zeros(6)
                self.get_logger().warning(
                    f'{side}: initialization velocity unavailable; assuming stationary start.',
                    throttle_duration_sec=5.)
        if self.pending is not None and all(s in self.positions or s in self.last_q for s in self.pending):
            pending, self.pending = self.pending, None
            self.accept(pending)

    def target(self, msg):
        try:
            commands = joint_targets(msg)
            if any(s not in self.positions and s not in self.last_q for s in commands):
                self.pending = commands
                return
            self.accept(commands)
            self.pending = None
        except (ValueError, TypeError) as exc:
            self.get_logger().warning(f'Joint data rejected: {exc}', throttle_duration_sec=2.)

    def accept(self, commands):
        if self.ruckig is not None:
            self.ruckig.target(commands, self.positions, self.velocities,
                               timestamp=time.monotonic())
            return
        now = time.monotonic()
        segments = {}
        for side, q in commands.items():
            segments[side] = JointSegment(
                self.last_publish.get(side, now), now + 1/self.input_rate,
                self.last_q.get(side, self.positions.get(side)), q,
                method=self.method, history=list(self.history[side]))
        # Commit the complete pair together after validating both arrays.
        self.segments.update(segments)

    def tick(self):
        if not self.segments and not (self.ruckig is not None and self.ruckig.active):
            return
        now_ns = time.monotonic_ns()
        now = now_ns * 1e-9
        if self.ruckig is not None:
            try:
                commands = self.ruckig.step(timestamp=now)
            except RuntimeError as exc:
                self.get_logger().error(f'{exc}; holding last published positions.', throttle_duration_sec=2.)
                commands = {s: q.copy() for s, q in self.last_q.items()}
                if not commands:
                    return
        else:
            commands = {s: segment.sample(now) for s, segment in self.segments.items()}
        if any(not np.isfinite(q).all() for q in commands.values()):
            self.get_logger().warning('Nonfinite interpolation sample rejected.', throttle_duration_sec=2.)
            return
        report = JointState()
        for side in SIDES:
            if side not in commands:
                continue
            q = commands[side]
            self.arm_publishers[side].publish(Float64MultiArray(data=q.tolist()))
            self.last_q[side], self.last_publish[side] = q, now
            self.history[side].append((now, q.copy()))
            report.name.extend(JOINT_NAMES[side])
            report.position.extend(q.tolist())
        report.header.stamp.sec, report.header.stamp.nanosec = divmod(now_ns, 10**9)
        self.command_pub.publish(report)


def main():
    rclpy.init()
    node = InterpolationNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
