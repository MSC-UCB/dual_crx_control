#!/usr/bin/env python3
"""Send and record synchronized two-position joint targets.

Source ROS and the workspace in each terminal first.
Terminal 1 (mock):
    ros2 launch dual_crx_control dual_arm.launch.py mock:=true rviz:=false input_rate_hz:=100.0
Terminal 2:
    ros2 run dual_crx_control simple_motion.py --joint 1 --range-deg 1 --hold 2 --duration 10 --rate 100 --output-dir motion_recordings

Both arms use left_J1..J6 and right_J1..J6; no configurable prefix.
Motion starts when ready, approaches INITIAL_JOINTS_DEG, then alternates A/B.
Match --rate to launch input_rate_hz. --range-deg is the signed A-to-B angle;
--hold is seconds at each position. Ctrl+C stops targets; interpolation holds.
Each run saves joints.csv and left/right six-joint PNGs in a timestamped folder.
"""

import argparse
from functools import partial
import math
import time

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import JointState

from dual_crx_control.interpolation.client import JointTargetClient
from dual_crx_control.robot.joint_config import INITIAL_JOINTS_DEG, JOINT_NAMES, SIDES
from dual_crx_control.analysis.motion_recording import JointRecording


class SimpleMotion(Node):
    def __init__(self, args, **kwargs):
        super().__init__('simple_motion', **kwargs)
        self.args = args
        self.states = {}
        self.endpoints = None
        self.startup_at = None
        self.start_positions = None
        self.started_at = None
        self.done = False
        self.target_client = JointTargetClient(self)
        self.recording = JointRecording(args.output_dir, target_source='target')
        for side in SIDES:
            self.create_subscription(JointState, f'{side}/joint_states',
                                     partial(self.receive_state, side), qos_profile_sensor_data)
        self.create_subscription(JointState, 'interpolated_joint_commands', self.receive_command, 1000)
        self.create_timer(1., self.recording.flush, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.timer = self.create_timer(1. / args.rate, self.tick,
                                      clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.get_logger().info(f'Recording joints to {self.recording.csv_path}')
        self.get_logger().info('Waiting for both arms\' joint states and interpolation...')

    def receive_state(self, side, message):
        self.record_message('feedback', (side,), message)

    def receive_command(self, message):
        self.record_message('interpolated', SIDES, message)

    def record_message(self, source, sides, message):
        received_at = time.monotonic()
        if len(message.name) != len(message.position) or len(set(message.name)) != len(message.name):
            return
        positions = dict(zip(message.name, message.position))
        for side in sides:
            if not all(name in positions for name in JOINT_NAMES[side]):
                continue
            joints = [positions[name] for name in JOINT_NAMES[side]]
            if all(math.isfinite(q) for q in joints):
                self.recording.add(received_at, side, source, joints)
                if source == 'feedback':
                    self.states[side] = joints

    def tick(self):
        if self.done:
            return
        if self.startup_at is None:
            if len(self.states) != 2 or not self.target_client.available():
                return
            self.start_positions = self.states['left'] + self.states['right']
            a = [math.radians(q) for side in SIDES for q in INITIAL_JOINTS_DEG[side]]
            b = a.copy()
            for offset in (0, 6):
                b[offset + self.args.joint - 1] += math.radians(self.args.range_deg)
            self.endpoints = (a, b)
            self.startup_at = time.monotonic()
            self.get_logger().info(
                f'Moving both arms to INITIAL_JOINTS_DEG over {self.args.startup_duration:g} s.')
        if self.started_at is None:
            elapsed = time.monotonic() - self.startup_at
            u = min(elapsed / self.args.startup_duration, 1.)
            blend = u**3 * (10. + u * (-15. + 6. * u))
            a = self.endpoints[0]
            target = [start + blend * (end - start) for start, end in zip(self.start_positions, a)]
            self.publish_target(a if u >= 1. else target)
            measured = self.states['left'] + self.states['right']
            # Start the step clock only after both arms reach the configured pose.
            if u < 1. or any(abs(q - home) > math.radians(.1) for q, home in zip(measured, a)):
                return
            self.get_logger().info(
                f'Initial pose reached. Both arms J{self.args.joint}: A to A + {self.args.range_deg:g} deg; '
                f'hold {self.args.hold:g} s, duration {self.args.duration:g} s, '
                f'rate {self.args.rate:g} Hz (match launch input_rate_hz).')
            self.started_at = time.monotonic()
            return
        elapsed = time.monotonic() - self.started_at
        if elapsed >= self.args.duration:
            self.done = True
            self.timer.cancel()
            self.get_logger().info('Finished sending targets; interpolation holds the last target.')
            return
        phase = int(elapsed / self.args.hold) % 2
        self.publish_target(self.endpoints[phase])

    def publish_target(self, target):
        commands = {side: target[i*6:(i+1)*6] for i, side in enumerate(SIDES)}
        published_at = time.monotonic()
        self.target_client.publish(commands)
        for side, joints in commands.items():
            self.recording.add(published_at, side, 'target', joints)


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--joint', type=int, choices=(1, 2, 3, 4, 5, 6), default=1)
    parser.add_argument('--range-deg', type=float, default=1.,
                        help='Signed angle from each arm\'s initial position A to B (default: 1 deg)')
    parser.add_argument('--hold', type=float, default=2.,
                        help='Seconds per position; full period is twice this value (default: 2)')
    parser.add_argument('--duration', type=float, default=10.,
                        help='Two-position motion duration after reaching the initial pose (default: 10 s)')
    parser.add_argument('--startup-duration', type=float, default=3.,
                        help='Seconds for the smooth move to INITIAL_JOINTS_DEG before stepping (default: 3)')
    parser.add_argument('--rate', type=float, default=100.,
                        help='Target rate in Hz; match launch input_rate_hz (default: 100)')
    parser.add_argument('--output-dir', default='motion_recordings',
                        help='Parent directory for timestamped joint CSV and left/right plots')
    args = parser.parse_args(argv)
    for name in ('range_deg', 'hold', 'duration', 'rate', 'startup_duration'):
        if not math.isfinite(getattr(args, name)):
            parser.error(f'--{name.replace("_", "-")} must be finite')
    if args.hold <= 0 or args.duration <= 0 or args.startup_duration <= 0:
        parser.error('--hold, --duration and --startup-duration must be positive')
    if not 0 < args.rate <= 500:
        parser.error('--rate must be in (0, 500], matching the interpolation input rate')
    return args


def main():
    args = parse_args(remove_ros_args()[1:])
    rclpy.init()
    node = None
    try:
        node = SimpleMotion(args)
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if node is not None:
            for path in node.recording.save():
                print(f'Saved {path}', flush=True)


if __name__ == '__main__':
    main()
