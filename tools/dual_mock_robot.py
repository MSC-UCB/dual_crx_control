#!/usr/bin/env python3
"""Ideal position mock: hold the last accepted command when commands stop.

After sourcing ROS and the built workspace, from the repository root:
    python3 tools/dual_mock_robot.py --ros-args --params-file /path/to/mock.yaml

The parameter file must provide robot_description with the combined dual-arm URDF.
Joint prefixes are left_ and right_. This tool subscribes to each arm's forward
position controller commands and publishes joint states; it has no hardware driver.
Run tools/smoke_motion.py to generate the description and exercise the full mock flow.
"""

from functools import partial

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from dual_crx_control.robot.kinematics import CRXKinematics
from dual_crx_control.robot.joint_config import (
    INITIAL_JOINTS_DEG, JOINT_STATES_TOPIC, arm_topic)


INITIAL = {side: np.radians(angles) for side, angles in INITIAL_JOINTS_DEG.items()}


class DualMockRobot(Node):
    def __init__(self, **kwargs):
        super().__init__('dual_mock_robot', **kwargs)
        description = self.declare_parameter('robot_description', '').value
        rate = self.declare_parameter('rate', 100.).value
        if not np.isfinite(rate) or rate <= 0:
            raise ValueError('Mock rate must be finite and positive')
        self.models = {side: CRXKinematics(description, f'{side}_tcp') for side in INITIAL}
        self.positions = {side: q.copy() for side, q in INITIAL.items()}
        self.arm_publishers = {}
        for side, model in self.models.items():
            if not model.valid_joints(self.positions[side]):
                raise ValueError(f'{side}: invalid initial joints')
            self.arm_publishers[side] = self.create_publisher(
                JointState, arm_topic(side, 'joint_states'), 1)
            self.create_subscription(Float64MultiArray,
                                     arm_topic(side, 'forward_position_controller/commands'),
                                     partial(self.command, side), 1)
        self.combined = self.create_publisher(JointState, JOINT_STATES_TOPIC, 1)
        self.create_timer(1. / rate, self.publish_states, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.get_logger().info('Ideal software mock ready; missing commands hold last position.')

    def command(self, side, message):
        q = np.array(message.data)
        if not self.models[side].valid_joints(q):
            self.get_logger().error(f'{side}: rejected invalid command / joint-limit violation')
            return
        self.positions[side] = q

    def publish_states(self):
        stamp = self.get_clock().now().to_msg()
        combined = JointState()
        combined.header.stamp = stamp
        for side, model in self.models.items():
            message = JointState()
            message.header.stamp = stamp
            message.name = model.joint_names
            message.position = self.positions[side].tolist()
            self.arm_publishers[side].publish(message)
            combined.name.extend(message.name)
            combined.position.extend(message.position)
        self.combined.publish(combined)


def main():
    rclpy.init()
    node = None
    try:
        node = DualMockRobot()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
