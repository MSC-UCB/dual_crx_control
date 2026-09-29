"""Same-phase joint sine motion for one or both arms, with streamed recording."""

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
from dual_crx_control.robot.joint_config import (
    INTERPOLATED_COMMANDS_TOPIC, SIDES, arm_topic, ordered_feedback)
from dual_crx_control.analysis.latency_recording import LatencyRecording
from dual_crx_control.analysis.motion_recording import JointRecording


def smootherstep(phase):
    u = min(max(phase, 0.), 1.)
    return u**3 * (10. + u * (-15. + 6. * u))


class JointSine(Node):
    def __init__(self, args, **kwargs):
        super().__init__('joint_sine', **kwargs)
        self.args = args
        self.positions, self.received = {}, {}
        self.starts = None
        self.started_at = None
        self.done = False
        self.failed = False
        self.return_offset = 0.
        self.client = JointTargetClient(self, arms=args.arms)
        self.recording = JointRecording(args.output_dir, target_source='target', arms=args.arms)
        self.events = LatencyRecording() if args.latency_csv else None
        for side in args.arms:
            self.create_subscription(JointState, arm_topic(side, 'joint_states'),
                                     partial(self.feedback, side), qos_profile_sensor_data)
        self.create_subscription(JointState, INTERPOLATED_COMMANDS_TOPIC, self.command, 1000)
        self.create_timer(1., self.recording.flush, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.timer = None

    def feedback(self, side, message):
        now = time.monotonic()
        try:
            joints = ordered_feedback(message, side)
            if not all(math.isfinite(q) for q in joints):
                raise ValueError('nonfinite feedback')
        except (KeyError, ValueError):
            self.received.pop(side, None)
            return
        self.positions[side], self.received[side] = joints, now
        self.recording.add(now, side, 'feedback', joints)
        if self.events:
            stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
            self.events.add(side, 'feedback', joints, stamp=stamp, received_at=now)

    def command(self, message):
        now = time.monotonic()
        for side in self.args.arms:
            try:
                joints = ordered_feedback(message, side)
            except (KeyError, ValueError):
                continue
            if all(math.isfinite(q) for q in joints):
                self.recording.add(now, side, 'interpolated', joints)
                if self.events:
                    self.events.add(side, 'command', joints, received_at=now)

    def ready(self):
        now = time.monotonic()
        return self.client.available() and all(
            side in self.received and now - self.received[side] <= self.args.state_timeout
            for side in self.args.arms)

    def wait_ready(self):
        deadline = time.monotonic() + self.args.ready_timeout
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=.05)
            if self.ready():
                return
            if time.monotonic() >= deadline:
                raise RuntimeError('Timed out waiting for fresh feedback and interpolation subscriber')
        raise RuntimeError('ROS shutdown while waiting for feedback')

    def start(self):
        self.wait_ready()
        self.get_logger().info(
            f'Arms={self.args.arms}, J{self.args.joint}, amplitude={self.args.amplitude_deg:g} deg, '
            f'period={self.args.period:g} s, rate={self.args.rate:g} Hz; starts at measured joints.')
        if not self.args.yes:
            input('Press ENTER to start joint sine motion on the selected arms...')
        self.received.clear()
        self.wait_ready()
        self.starts = {s: self.positions[s].copy() for s in self.args.arms}
        self.started_at = time.monotonic()
        self.timer = self.create_timer(1. / self.args.rate, self.tick,
                                      clock=Clock(clock_type=ClockType.STEADY_TIME))

    def offset(self, elapsed):
        a = self.args
        t = elapsed - a.initial_hold
        if t < 0:
            return 0.
        if a.continuous or t < a.duration:
            scale = smootherstep(t / a.ramp_time) if a.ramp_time else 1.
            self.return_offset = scale * math.radians(a.amplitude_deg) * math.sin(2. * math.pi * t / a.period)
            return self.return_offset
        if a.return_mode == 'smooth':
            return self.return_offset * (1. - smootherstep((t - a.duration) / a.return_duration))
        return 0.

    def tick(self):
        if not self.ready():
            self.failed = True
            self.timer.cancel()
            self.get_logger().error('Stale feedback or missing interpolation subscriber; stopped sending targets.')
            return
        elapsed = time.monotonic() - self.started_at
        offset = self.offset(elapsed)
        return_time = self.args.return_duration if self.args.return_mode == 'smooth' else 0.
        complete = not self.args.continuous and elapsed >= (
            self.args.initial_hold + self.args.duration + return_time + self.args.final_hold)
        if complete:
            offset = 0.
        commands = {side: q.copy() for side, q in self.starts.items()}
        for q in commands.values():
            q[self.args.joint - 1] += offset
        now = time.monotonic()
        for side, q in commands.items():
            if self.events:
                self.events.add(side, 'generated', q, received_at=now)
                self.events.add(side, 'target', q, received_at=now)
        self.client.publish(commands)
        returned_at = time.monotonic()
        for side, q in commands.items():
            self.recording.add(now, side, 'target', q)
            if self.events:
                self.events.add(side, 'target_publish_return', q, received_at=returned_at)
        if complete:
            self.done = True
            self.timer.cancel()
            self.get_logger().info('Motion complete; interpolation holds the start target.')


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('--arms', nargs='+', choices=SIDES, default=list(SIDES))
    parser.add_argument('--joint', type=int, choices=range(1, 7), default=1)
    stop = parser.add_mutually_exclusive_group()
    stop.add_argument('--duration', '--time', type=float, default=10., help='Sine duration in seconds, excluding holds/return')
    stop.add_argument('--continuous', action='store_true', help='Run until Ctrl+C; do not return on interruption')
    parser.add_argument('--period', type=float, default=4., help='Sine period [s]')
    parser.add_argument('--amplitude-deg', '--amplitude', type=float, default=20., help='Sine amplitude [deg]')
    parser.add_argument('--initial-hold', type=float, default=1., help='Hold current joints before motion [s]')
    parser.add_argument('--ramp-time', type=float, default=1., help='Smootherstep amplitude ramp [s]')
    parser.add_argument('--rate', type=float, default=50., help='Target Hz; match launch input_rate_hz')
    parser.add_argument('--return-mode', choices=('smooth', 'direct'), default=None,
                        help='Finite return: single arm defaults to legacy direct; both arms to smooth')
    parser.add_argument('--return-duration', type=float, default=None, help='Smooth return seconds; defaults to max(ramp-time, 1)')
    parser.add_argument('--final-hold', type=float, default=None,
                        help='Finite hold [s]; single arm defaults to 100/rate, both arms to 0.2')
    parser.add_argument('--state-timeout', type=float, default=1., help='Maximum feedback age [s]')
    parser.add_argument('--ready-timeout', type=float, default=10., help='Initial discovery timeout [s]')
    parser.add_argument('--output-dir', default='motion_recordings')
    parser.add_argument('--latency-csv', help='Optional bounded monotonic event trace CSV (legacy event sources preserved)')
    parser.add_argument('--plot-file', help='Optional selected-joint response plot, in addition to six-joint plots')
    parser.add_argument('--show-plot', action='store_true')
    parser.add_argument('--yes', action='store_true', help='Skip the interactive start confirmation')
    args = parser.parse_args(argv)
    if len(set(args.arms)) != len(args.arms):
        parser.error('--arms must contain distinct arms')
    args.return_mode = args.return_mode or ('direct' if len(args.arms) == 1 else 'smooth')
    args.return_duration = max(args.ramp_time, 1.) if args.return_duration is None else args.return_duration
    if not math.isfinite(args.rate) or not 0 < args.rate <= 500:
        parser.error('--rate must be finite and in (0, 500]')
    args.final_hold = (100. / args.rate if len(args.arms) == 1 else .2) if args.final_hold is None else args.final_hold
    for name in ('duration', 'period', 'return_duration', 'state_timeout', 'ready_timeout'):
        if not math.isfinite(getattr(args, name)) or getattr(args, name) <= 0:
            parser.error(f'--{name.replace("_", "-")} must be finite and positive')
    for name in ('amplitude_deg', 'initial_hold', 'ramp_time', 'final_hold'):
        if not math.isfinite(getattr(args, name)) or getattr(args, name) < 0:
            parser.error(f'--{name.replace("_", "-")} must be finite and nonnegative')
    return args


def main():
    args = parse_args(remove_ros_args()[1:])
    rclpy.init()
    node = None
    try:
        node = JointSine(args)
        node.start()
        while rclpy.ok() and not node.done and not node.failed:
            rclpy.spin_once(node, timeout_sec=.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if node is not None:
            if node.events:
                node.events.save(args.latency_csv, command_time='interpolation report receipt',
                                 feedback_time='feedback receipt; header stamp stored separately')
            for path in node.recording.save(response_joint=args.joint, plot_file=args.plot_file, show_plot=args.show_plot):
                print(f'Saved {path}', flush=True)
    if node is not None and node.failed:
        raise SystemExit(1)
