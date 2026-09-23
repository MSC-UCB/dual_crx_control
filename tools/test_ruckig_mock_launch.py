"""Opt-in canonical launch smoke test for Issue #2 Phase 0.

This starts only the software mock stack. Run it explicitly with:

    ISSUE2_ROS_MOCK=1 pytest -q tools/test_ruckig_mock_launch.py

The test is opt-in because a full ROS launch takes longer than the offline
regression and leaves a process group to clean up.
"""
import os
from pathlib import Path
import signal
import subprocess
import time

import numpy as np
import pytest
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from dual_crx_control.robot.joint_config import (
    INTERPOLATED_COMMANDS_TOPIC, JOINT_NAMES, JOINT_STATES_TOPIC,
    JOINT_TARGETS_TOPIC, SIDES, arm_topic)


pytestmark = pytest.mark.skipif(
    os.environ.get('ISSUE2_ROS_MOCK') != '1',
    reason='set ISSUE2_ROS_MOCK=1 to run the canonical mock launch')


ROOT = Path(__file__).resolve().parents[1]
DOMAIN = 192


def spin(node, seconds):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=.01)


def wait_until(node, predicate, timeout=20.):
    deadline = time.monotonic() + timeout
    while not predicate() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=.02)
    assert predicate(), 'ROS mock condition timed out'


@pytest.mark.parametrize('mode', ['waypoint', 'stream'])
def test_canonical_mock_launch_has_500hz_ruckig_stream(tmp_path, mode):
    environment = dict(os.environ,
                       ROS_DOMAIN_ID=str(DOMAIN),
                       ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST')
    log_path = tmp_path / 'dual_arm_mock.log'
    with log_path.open('w') as log:
        process = subprocess.Popen(
            ['ros2', 'launch', 'dual_crx_control', 'dual_arm.launch.py',
             'mock:=true', 'rviz:=false',
             'method:=ruckig', 'input_rate_hz:=10.0',
             f'ruckig_target_mode:={mode}'],
            env=environment, stdout=log, stderr=subprocess.STDOUT,
            start_new_session=True)
    rclpy.init(domain_id=DOMAIN)
    node = Node('issue2_ruckig_mock_probe')
    feedback = []
    arm_feedback = {side: [] for side in SIDES}
    commands = []
    targets = node.create_publisher(JointState, JOINT_TARGETS_TOPIC, 1)
    node.create_subscription(JointState, JOINT_STATES_TOPIC, feedback.append, 10)
    for side in SIDES:
        node.create_subscription(JointState, arm_topic(side, 'joint_states'),
                                 arm_feedback[side].append, 10)
    node.create_subscription(JointState, INTERPOLATED_COMMANDS_TOPIC, commands.append, 100)
    try:
        def has_complete_feedback():
            required = {name for side in SIDES for name in JOINT_NAMES[side]}
            return (targets.get_subscription_count() > 0
                    and node.count_publishers(INTERPOLATED_COMMANDS_TOPIC) > 0
                    and all(arm_feedback[side] for side in SIDES)
                    and any(required.issubset(message.name)
                            and len(message.name) == len(message.position)
                            for message in feedback))

        wait_until(node, has_complete_feedback)
        start = {}
        for side in SIDES:
            state = next(message for message in reversed(arm_feedback[side])
                         if set(JOINT_NAMES[side]).issubset(message.name)
                         and len(message.name) == len(message.position))
            by_name = dict(zip(state.name, state.position))
            start[side] = np.array([by_name[name] for name in JOINT_NAMES[side]])
        target_values = []
        start_time = time.monotonic()
        next_publish = start_time
        while time.monotonic() - start_time < .65:
            now = time.monotonic()
            if now >= next_publish:
                k = len(target_values) + 1
                target = {side: q.copy() for side, q in start.items()}
                for side in SIDES:
                    target[side][0] += k * 1e-4
                targets.publish(JointState(
                    name=sum((JOINT_NAMES[side] for side in SIDES), []),
                    position=np.concatenate([target[side] for side in SIDES]).tolist()))
                target_values.append(target)
                next_publish += .1
            rclpy.spin_once(node, timeout_sec=.002)
        assert len(target_values) >= 5
        spin(node, .35)
        assert len(commands) > 350
        stamps = np.array([m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
                           for m in commands])
        positions = np.array([m.position for m in commands])
        assert np.isfinite(stamps).all() and np.isfinite(positions).all()
        assert np.all(np.diff(stamps) >= 0.)
        observed_hz = (len(stamps) - 1) / (stamps[-1] - stamps[0])
        assert 350. < observed_hz < 650., observed_hz
        expected = np.concatenate([target_values[-1][side] for side in SIDES])
        np.testing.assert_allclose(positions[-1], expected, atol=1e-9, rtol=0.)
        # Keep the observed plateau count as a smoke metric. The deterministic
        # offline harness owns the exact move-hold measurement; ROS scheduling
        # and controller startup add timing jitter here.
        velocity = np.diff(positions[:, 0]) / np.diff(stamps)
        assert np.count_nonzero(np.abs(velocity) < 1e-8) >= 1
    finally:
        node.destroy_node()
        rclpy.shutdown()
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=10.)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
