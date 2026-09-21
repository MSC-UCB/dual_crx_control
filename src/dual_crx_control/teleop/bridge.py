"""Twelve joint positions in, twelve measured joint states out."""

from functools import partial
import math

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from dual_crx_control.interpolation.client import JointTargetClient


SIDES = ('left', 'right')
JOINT_NAMES = {side: [f'{side}_J{i}' for i in range(1, 7)] for side in SIDES}


class TeleopBridge(Node):
    def __init__(self, **kwargs):
        super().__init__('teleop_bridge', **kwargs)
        rate = self.declare_parameter('state_publish_rate', 100.0).value
        if not math.isfinite(rate) or rate <= 0:
            raise ValueError('state_publish_rate must be finite and positive')

        self.states = {}
        # Legacy adapter: keep the old application protocol isolated from the generic core target topic.
        self.target_client = JointTargetClient(self, target_topic='/interpolation/joint_targets')
        self.pending_command = None
        self.arm_publishers = {}
        for side in SIDES:
            state_topic = self.declare_parameter(
                f'{side}_joint_state_topic', f'/{side}/joint_states').value
            self.arm_publishers[side] = self.target_client.arm_publisher(side)
            self.create_subscription(JointState, state_topic, partial(self.receive_state, side),
                                     qos_profile_sensor_data)
        self.create_subscription(Float64MultiArray, '/teleop/joint_command',
                                 self.receive_command, 1)
        self.state_publisher = self.create_publisher(JointState, '/teleop/joint_states', 1)
        self.create_timer(1.0 / rate, self.publish_state,
                          clock=Clock(clock_type=ClockType.STEADY_TIME))
    def receive_command(self, message):
        if len(message.data) != 12 or not all(math.isfinite(q) for q in message.data):
            self.get_logger().warning('Rejected command: expected exactly 12 finite radians.',
                                      throttle_duration_sec=2.0)
            return
        self.pending_command = list(message.data)
        self.send_target()

    def send_target(self):
        if self.pending_command is None or not self.target_client.available():
            return
        data, self.pending_command = self.pending_command, None
        self.target_client.publish({s: data[i*6:(i+1)*6] for i, s in enumerate(SIDES)})

    def receive_state(self, side, message):
        indices = {name: index for index, name in enumerate(message.name)}
        if (len(indices) != len(message.name)
                or len(message.position) != len(message.name)
                or any(name not in indices for name in JOINT_NAMES[side])):
            self.get_logger().warning(f'Rejected malformed {side} joint state.',
                                      throttle_duration_sec=2.0)
            return
        order = [indices[name] for name in JOINT_NAMES[side]]
        positions = [message.position[index] for index in order]
        if not all(math.isfinite(q) for q in positions):
            self.get_logger().warning(f'Rejected nonfinite {side} joint positions.',
                                      throttle_duration_sec=2.0)
            return
        state = JointState(header=message.header, name=JOINT_NAMES[side], position=positions)
        if len(message.velocity) == len(message.name):
            velocity = [message.velocity[index] for index in order]
            if all(math.isfinite(v) for v in velocity):
                state.velocity = velocity
        # Effort reliability is not established for this driver; leave it empty.
        self.states[side] = state

    def publish_state(self):
        # Retry only a target that could not be sent before discovery completed.
        # Normal commands are forwarded directly by receive_command(), without a rate gate.
        self.send_target()
        if any(side not in self.states for side in SIDES):
            return
        states = [self.states[side] for side in SIDES]
        message = JointState()
        # Preserve the older source timestamp rather than making cached data look new.
        message.header.stamp = min(
            (state.header.stamp for state in states), key=lambda stamp: (stamp.sec, stamp.nanosec))
        message.name = JOINT_NAMES['left'] + JOINT_NAMES['right']
        message.position = list(states[0].position) + list(states[1].position)
        if all(len(state.velocity) == 6 for state in states):
            message.velocity = list(states[0].velocity) + list(states[1].velocity)
        self.state_publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = TeleopBridge()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
