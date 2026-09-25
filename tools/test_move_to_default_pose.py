"""Offline tests; no ROS graph, robot, or sensors are started."""
import importlib.util
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace as NS

import numpy as np
import pytest

SCRIPT = Path(__file__).resolve().parents[1] / 'scripts/move_to_default_pose.py'
if not SCRIPT.exists():
    SCRIPT = Path(__file__).with_name('move_to_default_pose.py')
spec = importlib.util.spec_from_file_location('default_pose', SCRIPT)
motion = importlib.util.module_from_spec(spec)
spec.loader.exec_module(motion)


def description():
    return '<robot name="test">' + ''.join(
        f'<joint name="{s}_J{i}" type="revolute"><limit lower="-6" upper="6" velocity="1"/></joint>'
        for s in ('left', 'right') for i in range(1, 7)) + '</robot>'


def test_pendant_roundtrip():
    for side, q in motion.default_targets().items():
        degrees = np.degrees(q)
        degrees[2] -= degrees[1]
        np.testing.assert_allclose(degrees, motion.PENDANT_DEGREES[side], atol=1e-12)


def test_description_limits_and_invalid_data():
    limits = motion.JointLimits(description(), 'left')
    assert limits.valid_joints(np.zeros(6))
    assert not limits.valid_joints(np.full(6, np.nan))
    assert not limits.valid_joints(np.full(6, 7))
    with pytest.raises(ValueError):
        motion.JointLimits(description().replace('velocity="1"', 'velocity="nan"'), 'left')


def test_default_does_not_start_ros(monkeypatch):
    monkeypatch.setattr(motion, 'execute', lambda args: pytest.fail('Unexpected motion'))
    assert motion.main([]) == 0


@pytest.mark.parametrize('option,value', [('rate', '501'), ('rate', 'nan'),
                                        ('state-timeout', '0'), ('max-speed-deg-s', '-1')])
def test_invalid_options(option, value):
    with pytest.raises(SystemExit):
        motion.parse_args(['--' + option, value])


@pytest.mark.parametrize('scenario,expected', [('success', 0), ('stale', 1),
    ('competing', 1), ('missing', 1), ('not_reached', 1)])
def test_lifecycle_without_ros(monkeypatch, scenario, expected):
    # Use the real bounded trajectory implementation with a fake ROS transport.
    root = SCRIPT.parent.parent / 'src'
    if not root.exists():
        root = Path('/home/msc-crx/ws_fanuc/src/dual_crx_control/src')
    monkeypatch.syspath_prepend(str(root))
    from dual_crx_control.robot.joint_config import JOINT_NAMES
    clock, published = [0.], []
    targets = motion.default_targets()
    measured = {s: q.copy() for s, q in targets.items()}
    for q in measured.values():
        q[0] += .01
    monkeypatch.setattr(motion.time, 'monotonic', lambda: clock[0])

    class Node:
        def __init__(self, *args):
            self.subscriptions = []
        def create_subscription(self, typ, topic, callback, qos):
            self.subscriptions.append((topic, callback))
        def get_logger(self):
            return NS(info=lambda *a: None, warning=lambda *a: None, error=lambda *a: None)
        def count_publishers(self, topic):
            return 2 if scenario == 'competing' else 1
        def destroy_node(self):
            pass

    class Client:
        def __init__(self, node):
            pass
        def available(self):
            return True
        def publish(self, q):
            published.append(q)
            if scenario != 'not_reached':
                measured.update({s: a.copy() for s, a in q.items()})

    def spin(node, **kwargs):
        clock[0] += .02
        assert clock[0] < 30, 'Execution failed to terminate'
        for topic, callback in node.subscriptions:
            if topic.endswith('robot_description'):
                callback(NS(data=description()))
            elif scenario != 'missing' and not (scenario == 'stale' and clock[0] > .1):
                side = 'left' if '/left/' in topic else 'right'
                callback(NS(name=JOINT_NAMES[side], position=measured[side]))

    def module(name, **attrs):
        m = ModuleType(name)
        m.__dict__.update(attrs)
        monkeypatch.setitem(sys.modules, name, m)
        return m

    module('rclpy', init=lambda **kw: None, ok=lambda: True,
           shutdown=lambda: None, spin_once=spin)
    module('rclpy.node', Node=Node)
    module('rclpy.executors', ExternalShutdownException=type('Shutdown', (Exception,), {}))
    module('rclpy.qos', QoSProfile=lambda **kw: None,
           DurabilityPolicy=NS(TRANSIENT_LOCAL=1), qos_profile_sensor_data=None)
    module('sensor_msgs.msg', JointState=object)
    module('std_msgs.msg', String=object)
    module('dual_crx_control.interpolation.client', JointTargetClient=Client)
    args = motion.parse_args(['--execute', '--minimum-duration', '.2', '--hold-time', '.1',
                              '--startup-timeout', '.3', '--state-timeout', '.1',
                              '--settle-timeout', '.3', '--tolerance-deg', '.01'])
    assert motion.execute(args) == expected
    if scenario in ('competing', 'missing'):
        assert not published
    if scenario == 'success':
        for s in targets:
            np.testing.assert_allclose(published[-1][s], targets[s])
