"""Publish joint targets without owning interpolation policy."""
import numpy as np
from sensor_msgs.msg import JointState

from dual_crx_control.robot.joint_config import (
    JOINT_NAMES, JOINT_TARGETS_TOPIC, SIDES, canonical_side)


class JointTargetClient:
    def __init__(self, node, arms=SIDES):
        selected = {canonical_side(a) for a in arms}
        if not selected:
            raise ValueError('at least one arm is required')
        self.arms = tuple(s for s in SIDES if s in selected)
        self.publisher = node.create_publisher(JointState, JOINT_TARGETS_TOPIC, 1)
        self.parts = {}

    def available(self):
        return self.publisher.get_subscription_count() > 0

    def publish(self, commands):
        if set(commands) != set(self.arms):
            raise ValueError('joint target must include all configured arms together')
        values = {s: np.asarray(commands[s], float) for s in self.arms}
        if any(q.shape != (6,) or not np.isfinite(q).all() for q in values.values()):
            raise ValueError('each arm requires six finite joint positions')
        self.publisher.publish(JointState(
            name=sum((JOINT_NAMES[s] for s in self.arms), []),
            position=np.concatenate([values[s] for s in self.arms]).tolist()))

    def arm_publisher(self, namespace):
        return ArmTargetPublisher(self, canonical_side(namespace))


class ArmTargetPublisher:
    """Collect both arms from older loops before publishing one joint target."""
    def __init__(self, client, side):
        self.client, self.side = client, side

    def get_subscription_count(self):
        return self.client.publisher.get_subscription_count()

    def publish(self, message):
        self.client.parts[self.side] = list(message.data)
        if set(self.client.parts) == set(self.client.arms):
            commands, self.client.parts = self.client.parts, {}
            self.client.publish(commands)
