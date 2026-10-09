"""Single-arm selection, motion, recording and interlocks (software only)."""
import csv
import importlib.util
from pathlib import Path
import time
import xml.etree.ElementTree as ET

import numpy as np
import pytest
import rclpy
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from std_msgs.msg import String
import xacro

from dual_crx_control.robot.joint_config import JOINT_NAMES, SIDES, validate_active_arms
from dual_crx_control.robot.kinematics import CRXKinematics
from dual_crx_control.interpolation.node import InterpolationNode
from dual_crx_control.motion.cartesian_controller import DualCartesianController
from dual_crx_control.motion.circle_controller import DualCircleController
from dual_crx_control.collision_force_limiter import CollisionForceLimiter, ForceMonitor
from dual_crx_control.teleop.keyboard import KeyboardControl

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('single_admittance', ROOT / 'scripts/admittance_hand_guiding.py')
ad = importlib.util.module_from_spec(spec)
spec.loader.exec_module(ad)


@pytest.fixture
def nodes():
    rclpy.init(domain_id=187)
    created = []
    def create(cls, **params):
        node = cls(parameter_overrides=[Parameter(k, value=v) for k, v in params.items()])
        created.append(node)
        if hasattr(node, 'timer'):
            node.timer.cancel()
        return node
    yield create
    for node in reversed(created):
        if getattr(node, 'joint_recording', None) is not None:
            node.joint_recording.close()
        node.destroy_node()
    rclpy.shutdown()


@pytest.fixture(scope='module')
def xml():
    return xacro.process_file(str(ROOT / 'urdf/dual_crx.urdf.xacro')).toxml()


def single_xml(side):
    return xacro.process_file(str(ROOT / 'urdf/dual_crx.urdf.xacro'), mappings={
        f'enable_{s}': str(s == side).lower() for s in SIDES}).toxml()


@pytest.mark.parametrize('side', SIDES)
def test_selected_model_preserves_world_chain_and_removes_other_arm(xml, side):
    description = single_xml(side)
    root = ET.fromstring(description)
    links = {link.attrib['name'] for link in root.findall('link')}
    other = next(s for s in SIDES if s != side)
    assert {'world', 'table_frame', 'calibration_table_surface', f'{side}_tcp'} <= links
    assert not any(name.startswith(f'{other}_') for name in links)
    assert {joint.attrib['name'] for joint in root.findall('joint')
            if joint.attrib['type'] == 'revolute'} == set(JOINT_NAMES[side])
    for joint in root.findall('joint'):
        assert joint.find('parent').attrib['link'] in links
        assert joint.find('child').attrib['link'] in links
    # Both TCP and flange (admittance) keep their calibrated world transforms.
    for tip in (f'{side}_tcp', f'{side}_fanuc_flange'):
        before, after = CRXKinematics(xml, tip), CRXKinematics(description, tip)
        assert before.joint_names == after.joint_names == JOINT_NAMES[side]
        for fraction in (.25, .5, .75):
            q = before.lower + fraction * (before.upper - before.lower)
            np.testing.assert_allclose(after.fk(q), before.fk(q), atol=1e-12, rtol=0)
            np.testing.assert_allclose(after.jacobian(q), before.jacobian(q), atol=1e-12, rtol=0)


def test_default_world_model_still_has_both_arms(xml):
    joints = {joint.attrib['name'] for joint in ET.fromstring(xml).findall('joint')
              if joint.attrib['type'] == 'revolute'}
    assert joints == set(JOINT_NAMES['left'] + JOINT_NAMES['right'])


@pytest.mark.parametrize('side', SIDES)
def test_keyboard_accepts_single_world_model(nodes, side):
    keys = iter(['1' if side == 'left' else '2', 'e'])
    node = nodes(lambda **kwargs: KeyboardControl(lambda: next(keys), **kwargs),
                 robot_description=single_xml(side))
    assert set(node.models) == {side} and not node.model_fault
    node.quintic_ready, node.method_checked = True, time.monotonic()
    node.target_clients[side].available = lambda: True
    node.feedback(side, JointState(name=JOINT_NAMES[side], position=[0.] * 6))
    node.cycle()
    node.cycle()
    assert node.jog is not None and node.side == side


@pytest.mark.parametrize('arms', [[], ['left', 'left'], ['other'], ['left_'], 'left'])
def test_bad_selection(arms):
    with pytest.raises(ValueError, match='active_arms'):
        validate_active_arms(arms)


def test_selection_preserves_global_order():
    assert validate_active_arms(['right', 'left']) == SIDES
    assert validate_active_arms(['right']) == ('right',)


def assert_no_inactive_endpoints(node, side):
    other = 'right' if side == 'left' else 'left'
    endpoints = [entity.topic_name for entity in (*node.publishers, *node.subscriptions)]
    assert not any(f'/crx5ia/{other}/' in topic for topic in endpoints)
    assert not any(f'/crx5ia/{other}/' in client.srv_name for client in node.clients)


@pytest.mark.parametrize('side', SIDES)
@pytest.mark.parametrize('method', ['linear', 'cubic', 'ruckig', 'quintic'])
def test_interpolation_isolated_endpoints_and_atomic_rejection(nodes, side, method):
    node = nodes(InterpolationNode, active_arms=[side], method=method, input_rate_hz=100.)
    assert_no_inactive_endpoints(node, side)
    assert set(node.arm_publishers) == {side}
    q = np.arange(6) * .01
    target = q + .001
    message = JointState(name=JOINT_NAMES[side], position=target.tolist())
    node.target(message)
    assert set(node.pending) == {side}
    # Mixed commands must not replace even a valid, not-yet-initialized target.
    pending = node.pending
    node.target(JointState(name=JOINT_NAMES['left'] + JOINT_NAMES['right'], position=[1.] * 12))
    assert node.pending is pending
    node.feedback(side, JointState(name=JOINT_NAMES[side], position=q.tolist(), velocity=[0.] * 6))
    assert node.pending is None
    deadline = time.monotonic() + 2.
    while time.monotonic() < deadline:
        node.tick()
        if side in node.last_q and np.allclose(node.last_q[side], target, atol=1e-10, rtol=0):
            break
        time.sleep(.002)
    np.testing.assert_allclose(node.last_q[side], target, atol=1e-10, rtol=0)
    assert set(node.last_q) == {side}
    other = next(s for s in SIDES if s != side)
    node.target(JointState(name=JOINT_NAMES[other], position=[2.] * 6))
    node.tick()
    assert node.pending is None and set(node.last_q) == {side}
    np.testing.assert_allclose(node.last_q[side], target, atol=1e-10, rtol=0)


@pytest.mark.parametrize('side', SIDES)
@pytest.mark.parametrize('cls', [DualCartesianController, DualCircleController])
def test_cartesian_initial_motion_completion_and_recording(nodes, xml, tmp_path, side, cls):
    node = nodes(cls, active_arms=[side], robot_description=single_xml(side), output_dir=str(tmp_path))
    assert_no_inactive_endpoints(node, side)
    assert set(node.models) == set(node.initial_targets) == {side}
    q = node.initial_targets[side].copy()
    node.feedback(side, JointState(name=JOINT_NAMES[side], position=q.tolist()))
    assert node.readiness_error(time.monotonic()) == f'{side}: missing command subscriber'
    sent = []
    node.target_client.publish = lambda commands: sent.append(commands)
    node.arm_publishers[side].get_subscription_count = lambda: 1
    assert node.readiness_error(time.monotonic()) is None
    now = time.monotonic()
    node.initial_move_cycle(now)
    assert set(node.initial_move.sample(10000.)) == {side}
    node.initial_move_cycle(now + node.initial_move.duration)
    assert node.initial_move_cycle(now + node.initial_move.duration + 1.)
    node.move_to_initial = False
    node._cycle()  # Capture FK, solve IK and publish from the selected model only.
    assert not node.failed and node.starts is not None
    assert sent and all(set(command) == {side} for command in sent)
    node.cartesian_complete = True
    node.record_command(JointState(name=JOINT_NAMES[side], position=node.segment_target[0].tolist()))
    assert node.finished
    paths = node.recording.save(node.models, tmp_path)
    assert paths and all(p.is_file() for p in paths)
    with paths[1].open() as stream:
        assert {row['arm'] for row in csv.DictReader(stream)} == {side}
    assert node.joint_recording.arms == (side,)


@pytest.mark.parametrize('side', SIDES)
def test_single_arm_force_monitor_does_not_wait_for_other_side(side):
    monitor = ForceMonitor(20., 0., [side])
    monitor.receive(side, (0., 0., 0.), 0.)
    monitor.update(0., True)
    monitor.receive(side, (0., 0., 0.), 3.1)
    monitor.update(3.1, True)
    assert monitor.state == 'ARMED'
    monitor.receive(side, (21., 0., 0.), 3.11)
    assert monitor.state == 'TRIPPED'


@pytest.mark.parametrize('side', SIDES)
def test_limiter_only_requests_selected_arm(nodes, side):
    node = nodes(CollisionForceLimiter, active_arms=[side])
    assert_no_inactive_endpoints(node, side)
    assert set(node.manager_clients) == {side}
    node.monitor.trip('test force')
    node.process_stop(time.monotonic())
    assert set(node.stops) == {side}


@pytest.mark.parametrize('side', SIDES)
def test_admittance_single_arm_enable_and_competing_sender(nodes, xml, side):
    node = nodes(ad.AdmittanceNode, active_arms=[side])
    assert_no_inactive_endpoints(node, side)
    node.configure(String(data=single_xml(side)))
    assert not node.model_error and set(node.arms) == {side}
    other = next(s for s in SIDES if s != side)
    node.request(other, 'enable')
    assert not node.requests
    node.target_clients[(side,)].available = lambda: True
    # One own publisher plus one competitor must block single-arm enable.
    node.count_publishers = lambda topic: 2 if topic == ad.JOINT_TARGETS_TOPIC else 1
    node.cycle()
    assert node.chain_error == 'another joint-target publisher is active'
    node.count_publishers = lambda topic: 1
    node.next_graph_check = 0.
    node.limiter(String(data='ARMED'))
    arm = node.arms[side]
    arm.feedback(JointState(name=JOINT_NAMES[side], position=[0.] * 6, velocity=[0.] * 6), time.monotonic())
    wrench = ad.WrenchStamped()
    wrench.header.frame_id = ad.FRAME[side]
    arm.wrench(wrench, time.monotonic())
    sent = []
    node.target_clients[(side,)].publish = lambda commands: sent.append(commands)
    node.request(side, 'enable')
    node.cycle()
    assert not node.chain_error and arm.active
    assert sent and set(sent[-1]) == {side}
    assert set(node.read_snapshot()) == {side}
    assert not node.read_snapshot()[side]['error']
    node.limiter(String(data='TRIPPED'))
    assert not arm.active and arm.state == 'Fault'


@pytest.mark.parametrize('cls', [InterpolationNode, CollisionForceLimiter, ad.AdmittanceNode])
def test_existing_default_still_uses_both_arms(nodes, cls):
    node = nodes(cls)
    assert node.active_arms == SIDES


@pytest.mark.parametrize('side', SIDES)
def test_gui_keeps_inactive_panel_disabled(nodes, side):
    import tkinter as tk
    from tkinter import ttk
    node = nodes(ad.AdmittanceNode, active_arms=[side])
    try:
        window = ad.AdmittanceWindow(node)
    except tk.TclError as error:
        pytest.skip(f'GUI display unavailable: {error}')
    window.root.withdraw()
    try:
        other = next(s for s in SIDES if s != side)
        assert set(window.buttons) == set(SIDES)
        assert window.labels[other]['state'].get() == 'Not active'
        assert not window.labels[other]['error'].get()
        panel = window.buttons[other][0].master
        for widget in panel.winfo_children():
            if isinstance(widget, (ttk.Button, ttk.Checkbutton, ttk.Entry)):
                assert widget.instate(['disabled'])
        window.buttons[other][0].invoke()
        assert other not in node.requests
        assert window.buttons[side][0].instate(['!disabled'])
    finally:
        window.root.destroy()
