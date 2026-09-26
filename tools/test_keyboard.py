"""Keyboard regression tests; opt into isolated software-only ROS integration.

After sourcing ROS and the built workspace:
    python3 -m pytest tools/test_keyboard.py
    ROS_DOMAIN_ID=178 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST KEYBOARD_ROS_TEST=1 \
        python3 -m pytest tools/test_keyboard.py -s
"""

import os
import pty
import sys
import termios
import time
from types import SimpleNamespace

import numpy as np
import pytest

from dual_crx_control.teleop.keyboard import (
    DEFAULTS, MOVES, JogTarget, KeyboardControl, Terminal, batch_key)
from dual_crx_control.robot.joint_config import JOINT_TARGETS_TOPIC


class Model:
    velocity = np.ones(6)

    def valid_joints(self, q):
        return np.isfinite(q).all() and np.max(np.abs(q)) < 1.

    def fk(self, q):
        pose = np.eye(4)
        pose[:3, 3] = q[:3]
        return pose


class Solver:
    def solve(self, target, seed):
        return SimpleNamespace(success=True, q=np.r_[target[:3, 3], np.zeros(3)], reason='ok')


@pytest.mark.parametrize('key,axis,sign', [(k, *v) for k, v in MOVES.items()])
def test_cartesian_key_and_fixed_orientation(key, axis, sign):
    jog = JogTarget(Model(), Solver(), np.zeros(6), DEFAULTS)
    pose, q = jog.candidate(key, np.zeros(6))
    assert pose[axis, 3] == pytest.approx(sign * .001)
    np.testing.assert_array_equal(pose[:3, :3], np.eye(3))
    np.testing.assert_array_equal(jog.q, np.zeros(6))  # no commit before publish
    np.testing.assert_array_equal(q[:3], pose[:3, 3])


@pytest.mark.parametrize('data,expected', [
    ('wwww', 'w'), ('w w', ' '), ('we', 'e'), ('e2w', '2'),
    ('wq', 'q'), ('\x03', 'q'), ('\x1b[A', ''), ('?', ''),
])
def test_input_batch_has_no_movement_backlog(data, expected):
    assert batch_key(data) == expected


@pytest.mark.parametrize('failure', ['ik', 'limits', 'step', 'velocity', 'tracking'])
def test_rejection_does_not_accumulate(failure):
    settings = DEFAULTS.copy()
    solver = Solver()
    if failure in ('ik', 'limits'):
        solver.solve = lambda *args: SimpleNamespace(success=failure != 'ik', q=np.ones(6) * 2, reason=failure)
    for name, setting in [('step', 'max_joint_step'), ('velocity', 'max_joint_velocity'),
                          ('tracking', 'tracking_tolerance')]:
        if failure == name:
            settings[setting] = .0001
    jog = JogTarget(Model(), solver, np.zeros(6), settings)
    for _ in range(2):
        with pytest.raises(ValueError):
            jog.candidate('w', np.zeros(6))
        np.testing.assert_array_equal(jog.q, np.zeros(6))
        np.testing.assert_array_equal(jog.pose, np.eye(4))


def test_jog_can_pass_former_displacement_radius():
    jog = JogTarget(Model(), Solver(), np.zeros(6), DEFAULTS)
    for _ in range(100):
        pose, q = jog.candidate('w', jog.q.copy())
        jog.pose, jog.q = pose, q
    assert jog.pose[0, 3] == pytest.approx(.1)


def test_terminal_restores_settings_and_flushes(monkeypatch):
    master, slave = pty.openpty()
    try:
        with os.fdopen(os.dup(slave), 'r') as stream:
            monkeypatch.setattr(sys, 'stdin', stream)
            saved = termios.tcgetattr(slave)
            with pytest.raises(KeyboardInterrupt):
                with Terminal() as terminal:
                    os.write(master, b'wwww')
                    assert select_read(terminal) == 'w'
                    assert terminal.read() == ''
                    os.write(master, b'ww w')
                    assert select_read(terminal) == ' '
                    raise KeyboardInterrupt
            assert termios.tcgetattr(slave) == saved
    finally:
        os.close(master)
        os.close(slave)


def select_read(terminal):
    import select
    assert select.select([terminal.fd], [], [], 1)[0]
    return terminal.read()


def test_non_tty_rejected(monkeypatch):
    with open(os.devnull) as stream:
        monkeypatch.setattr(sys, 'stdin', stream)
        with pytest.raises(ValueError, match='TTY'):
            with Terminal():
                pass


@pytest.mark.skipif(os.environ.get('KEYBOARD_ROS_TEST') != '1', reason='opt-in ROS software mock')
def test_ros_mock_both_arms_and_watchdog():
    from pathlib import Path
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.parameter import Parameter
    from sensor_msgs.msg import JointState
    from ament_index_python.packages import get_package_share_directory
    import xacro
    from dual_crx_control.interpolation.node import InterpolationNode
    from dual_crx_control.robot.joint_config import JOINT_NAMES
    from dual_mock_robot import DualMockRobot

    assert os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE') == 'LOCALHOST'
    assert os.environ.get('ROS_DOMAIN_ID') == '178', 'Use the dedicated test domain'
    description = xacro.process_file(str(Path(get_package_share_directory('dual_crx_control')) /
                                         'urdf/dual_crx.urdf.xacro')).toxml()
    rclpy.init()
    nodes = []
    executor = SingleThreadedExecutor()
    key = ['']
    def read_key():
        value, key[0] = key[0], ''
        return value

    def spin_until(predicate, timeout=5.):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.005)
            if predicate():
                return
        raise AssertionError('Timed out waiting for mock condition')

    def press(value):
        key[0] = value
        spin_until(lambda: key[0] == '')

    try:
        mock = DualMockRobot(parameter_overrides=[Parameter('robot_description', value=description)])
        nodes.append(mock)
        interpolator = InterpolationNode(parameter_overrides=[Parameter('method', value='ruckig')])
        nodes.append(interpolator)
        keyboard = KeyboardControl(read_key, parameter_overrides=[Parameter('robot_description', value=description)])
        nodes.append(keyboard)
        targets = []
        keyboard.create_subscription(JointState, JOINT_TARGETS_TOPIC, targets.append, 10)
        for node in nodes:
            executor.add_node(node)
        spin_until(lambda: keyboard.readiness_error() is None and len(keyboard.positions) == 2)
        assert keyboard.jog is None
        press('w')
        assert targets == []
        keyboard.ruckig_ready = False
        press('e')
        assert keyboard.jog is None
        spin_until(lambda: keyboard.readiness_error() is None)
        for side, selector in [('left', '1'), ('right', '2')]:
            press(selector)
            assert keyboard.jog is None
            spin_until(lambda: keyboard.readiness_error() is None)
            other = 'right' if side == 'left' else 'left'
            other_before = mock.positions[other].copy()
            press('e')
            assert keyboard.jog is not None
            original_jog = keyboard.jog
            press('e')
            assert keyboard.jog is original_jog
            start = keyboard.jog.pose.copy()
            count = len(targets)
            # Pick a reachable inward direction at the mock's initial configuration.
            for direction in MOVES:
                try:
                    keyboard.jog.candidate(direction, keyboard.positions[side])
                    break
                except ValueError:
                    continue
            else:
                raise AssertionError(f'No valid jog direction for {side}')
            press(direction)
            spin_until(lambda: len(targets) == count + 1)
            assert targets[-1].name == JOINT_NAMES[side]
            goal = keyboard.jog.pose.copy()
            assert np.linalg.norm(goal[:3, 3] - start[:3, 3]) == pytest.approx(.001)
            press(' ')
            assert keyboard.jog is None
            # Disarm stops new targets, not the accepted Ruckig segment.
            q_goal = np.array(targets[-1].position)
            spin_until(lambda: np.max(np.abs(mock.positions[side] - q_goal)) < 1e-7)
            actual = keyboard.models[side].fk(mock.positions[side])
            np.testing.assert_allclose(actual[:3, 3], goal[:3, 3], atol=2e-5)
            np.testing.assert_allclose(actual[:3, :3], start[:3, :3], atol=2e-5)
            np.testing.assert_array_equal(mock.positions[other], other_before)
            press(direction)
            assert len(targets) == count + 1
        press('e')
        assert keyboard.jog is not None
        executor.remove_node(mock)
        spin_until(lambda: keyboard.jog is None, timeout=2.)
        press('e')
        assert keyboard.jog is None
        executor.add_node(mock)
        spin_until(lambda: keyboard.readiness_error() is None)
        assert keyboard.jog is None  # no automatic re-enable
        press('e')
        assert keyboard.jog is not None
        keyboard.positions[keyboard.side] = keyboard.jog.q + .2
        keyboard.cycle()
        assert keyboard.jog is None
        spin_until(lambda: keyboard.readiness_error() is None)
        press('e')
        keyboard.feedback(keyboard.side, JointState(name=['bad'], position=[0.]))
        assert keyboard.jog is None
        press('q')
        assert keyboard.finished
    finally:
        executor.shutdown()
        for node in reversed(nodes):
            node.destroy_node()
        rclpy.shutdown()
