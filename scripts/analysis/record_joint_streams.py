#!/usr/bin/env python3
"""Record generic joint targets, interpolated commands, and arm feedback."""

from functools import partial
import math
import time

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState

from dual_crx_control.robot.joint_config import (
    INTERPOLATED_COMMANDS_TOPIC, JOINT_NAMES, JOINT_TARGETS_TOPIC, SIDES,
    arm_topic)
from dual_crx_control.analysis.motion_recording import JointRecording


class JointStreamRecorder(Node):
    """Record the fixed CRX5IA generic control streams."""

    def __init__(self, **kwargs):
        super().__init__('joint_stream_recorder', **kwargs)
        output_dir = self.declare_parameter(
            'output_dir', '/home/msc-crx/ws_fanuc/joint_recordings').value
        self.recording = JointRecording(output_dir, target_source='target')
        for topic, source in ((JOINT_TARGETS_TOPIC, 'target'),
                              (INTERPOLATED_COMMANDS_TOPIC, 'interpolated')):
            self.create_subscription(JointState, topic, partial(self.receive, source, SIDES), 1000)
        for arm in SIDES:
            self.create_subscription(JointState, arm_topic(arm, 'joint_states'),
                                     partial(self.receive, 'feedback', (arm,)), qos_profile_sensor_data)
        self.create_timer(1., self.recording.flush, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.get_logger().info(f'Recording generic joint streams to {self.recording.csv_path}')

    def receive(self, source, arms, message):
        received_at = time.monotonic()
        if (len(message.name) != len(message.position)
                or len(set(message.name)) != len(message.name)):
            self.get_logger().warning(f'Skipping malformed {source} sample.', throttle_duration_sec=2.)
            return
        positions = dict(zip(message.name, message.position))
        for arm in arms:
            if not all(name in positions for name in JOINT_NAMES[arm]):
                continue
            joints = [positions[name] for name in JOINT_NAMES[arm]]
            if all(math.isfinite(q) for q in joints):
                self.recording.add(received_at, arm, source, joints)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = JointStreamRecorder()
        rclpy.spin(node)
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
