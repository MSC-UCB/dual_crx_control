"""Single-file GUI/controller tests. Full mock and GUI checks are opt-in."""
import importlib.util
import math
import os
from pathlib import Path
import threading
import time
from types import SimpleNamespace

import numpy as np
import pytest
from scipy.linalg import expm
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.executors import SingleThreadedExecutor
from geometry_msgs.msg import WrenchStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import String


ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('admittance_gui', ROOT/'scripts/admittance_hand_guiding.py')
ad = importlib.util.module_from_spec(spec)
spec.loader.exec_module(ad)


def wrench(side='left', values=(0.,)*6, frame=None):
    msg = WrenchStamped()
    msg.header.frame_id = ad.FRAME[side] if frame is None else frame
    for part, vector in [(values[:3], msg.wrench.force), (values[3:], msg.wrench.torque)]:
        vector.x, vector.y, vector.z = map(float, part)
    return msg


def joints(side='left', q=None):
    return JointState(name=ad.JOINT_NAMES[side], position=list(np.zeros(6) if q is None else q),
                      velocity=[0.]*6)


class Model:
    velocity = np.ones(6)

    def valid_joints(self, q):
        return np.shape(q) == (6,) and np.isfinite(q).all() and np.max(np.abs(q)) < 10

    def fk(self, q):
        pose = np.eye(4)
        pose[:3, 3] = q[:3]
        pose[:3, :3] = Rotation.from_rotvec(q[3:]).as_matrix()
        return pose


class Solver:
    def solve(self, pose, seed):
        q = np.r_[pose[:3, 3], Rotation.from_matrix(pose[:3, :3]).as_rotvec()]
        return SimpleNamespace(success=True, q=q, reason='test')


def arm(side='left', now=0.):
    result = ad.ArmAdmittance(side)
    result.model, result.solver = Model(), Solver()
    result.feedback(joints(side), now)
    result.wrench(wrench(side), now)
    return result


@pytest.mark.parametrize('axis', range(6))
@pytest.mark.parametrize('sign', [-1, 1])
def test_continuous_deadband_per_axis(axis, sign):
    value = np.zeros(6)
    value[axis] = sign*ad.DEADBAND[axis]
    np.testing.assert_array_equal(ad.effective_wrench(value), np.zeros(6))
    value[axis] += sign*.01
    expected = np.zeros(6)
    expected[axis] = sign*.01
    np.testing.assert_allclose(ad.effective_wrench(value), expected, atol=1e-15)


@pytest.mark.parametrize('scale', [0., 1e-9, .2, 2.])
def test_se3_exponential_against_matrix_expm(scale):
    increment = np.array([.3, -.4, .2, -.2, .3, .4])*scale
    x, y, z = increment[3:]
    algebra = np.zeros((4, 4))
    algebra[:3, :3] = [[0, -z, y], [z, 0, -x], [-y, x, 0]]
    algebra[:3, 3] = increment[:3]
    np.testing.assert_allclose(ad.se3_exp(increment), expm(algebra), atol=1e-12)


def test_body_translation_uses_initial_rotation():
    initial = np.eye(4)
    initial[:3, :3] = Rotation.from_euler('z', 90, degrees=True).as_matrix()
    moved = initial @ ad.se3_exp(np.array([.01, 0, 0, 0, 0, 0]))
    np.testing.assert_allclose(moved[:3, 3], [0, .01, 0], atol=1e-15)


@pytest.mark.parametrize('axis,input_value,steady', [(0, 10., .085), (3, .2, .425)])
def test_nominal_response_and_release_stays_at_last_position(axis, input_value, steady):
    pose, velocity, raw = np.eye(4), np.zeros(6), np.zeros(6)
    raw[axis] = input_value
    for _ in range(500):
        pose, velocity = ad.integrate(pose, velocity, ad.effective_wrench(raw),
                                     ad.M_DEFAULT, ad.D_DEFAULT, .01)
    assert velocity[axis] == pytest.approx(steady, abs=1e-6)
    for _ in range(500):
        pose, velocity = ad.integrate(pose, velocity, np.zeros(6), ad.M_DEFAULT, ad.D_DEFAULT, .01)
    assert np.max(np.abs(velocity)) < 1e-10
    assert not np.allclose(pose, np.eye(4))
    settled = pose.copy()
    for _ in range(50):
        pose, velocity = ad.integrate(pose, velocity, np.zeros(6), ad.M_DEFAULT, ad.D_DEFAULT, .01)
    np.testing.assert_allclose(pose, settled, atol=1e-10)


def test_limits_hold_during_multi_axis_force_and_reversal():
    pose, velocity = np.eye(4), np.zeros(6)
    for index in range(300):
        old = velocity.copy()
        force = np.array([500., -300., 200., 5., -8., 9.]) * (1 if index < 150 else -1)
        pose, velocity = ad.integrate(pose, velocity, force, ad.M_DEFAULT, ad.D_DEFAULT, .01)
        assert np.all(np.abs(velocity) <= ad.VELOCITY_LIMIT + 1e-12)
        assert np.all(np.abs(velocity-old) <= ad.ACCELERATION_LIMIT*.01 + 1e-12)
        assert np.linalg.norm(velocity[:3]) <= .1+1e-12
        assert np.linalg.norm(velocity[3:]) <= .5+1e-12


@pytest.mark.parametrize('value', [0., -1., math.nan, math.inf, 1e-9])
def test_invalid_or_unstable_gains_rejected(value):
    mass = ad.M_DEFAULT.copy()
    mass[0] = value
    with pytest.raises(ValueError):
        ad.validate_gains(mass, ad.D_DEFAULT)


def test_filter_uses_sample_interval_and_enable_resets_it():
    a = arm()
    raw = np.array([10., 0., 0., 0., 0., 0.])
    a.wrench(wrench(values=raw), .01)
    np.testing.assert_allclose(a.filtered, raw*(1-np.exp(-2*np.pi*ad.FILTER_HZ*.01)))
    a.enable(.01)
    np.testing.assert_array_equal(a.filtered, raw)
    assert a.state == 'Enabling' and not np.any(a.velocity)
    candidate = a.candidate(.01)
    np.testing.assert_array_equal(candidate[0], a.actual_pose)
    a.commit(candidate)
    candidate = a.candidate(.02)
    assert 0 < candidate[1][0] < .001


def test_disable_and_reenable_use_new_actual_pose():
    a = arm()
    a.enable(0.)
    a.commit(a.candidate(0.))
    a.velocity[:] = .1
    a.stop()
    assert a.state == 'Disabled' and not a.velocity.any() and a.command_pose is None
    q = np.array([.001, .002, .003, .004, .005, .006])
    a.feedback(joints(q=q), .02)
    a.wrench(wrench(), .02)
    a.enable(.02)
    np.testing.assert_allclose(a.command_pose, a.model.fk(q))


@pytest.mark.parametrize('frame', ['', 'left_tcp', 'right_fanuc_flange'])
def test_wrong_frame_faults_only_that_arm(frame):
    left, right = arm(), arm('right')
    left.enable(0.)
    right.enable(0.)
    left.wrench(wrench(frame=frame), .01)
    assert left.state == 'Fault' and right.active


@pytest.mark.parametrize('kind', ['wrench', 'pose'])
def test_late_fresh_sample_does_not_hide_data_dropout(kind):
    a = arm()
    a.enable(0.)
    if kind == 'wrench':
        a.wrench(wrench(), .2)
    else:
        a.feedback(joints(), .2)
    assert a.state == 'Fault' and not a.velocity.any()


def test_timeout_tracking_dt_and_bad_feedback():
    a = arm()
    a.enable(0.)
    a.commit(a.candidate(0.))
    assert a.candidate(.001) is None and a.last_step == 0.
    with pytest.raises(ValueError, match='dt'):
        a.candidate(.06)
    with pytest.raises(ValueError, match='timeout'):
        a.candidate(.2)
    a.actual_pose[0, 3] = .03
    with pytest.raises(ValueError, match='tracking'):
        a.candidate(.01)
    a.feedback(joints(q=[math.nan]*6), .01)
    assert a.state == 'Fault'


def test_gains_apply_atomically_and_smoothly():
    a = arm()
    a.enable(0.)
    a.commit(a.candidate(0.))
    a.apply_gains(ad.M_DEFAULT*2, ad.D_DEFAULT*2)
    candidate = a.candidate(.01)
    assert np.all(candidate[3] > ad.M_DEFAULT) and np.all(candidate[3] < ad.M_DEFAULT*2)
    previous = a.target_mass.copy()
    with pytest.raises(ValueError):
        a.apply_gains([-1.]*6, ad.D_DEFAULT)
    np.testing.assert_array_equal(a.target_mass, previous)


@pytest.fixture
def ros_node():
    rclpy.init(domain_id=185)
    node = ad.AdmittanceNode()
    node.timer.cancel()
    node.limiter_state, node.chain_error, node.next_graph_check = 'ARMED', '', math.inf
    node.arms = {s: arm(s, time.monotonic()) for s in ad.SIDES}
    sent = []
    for client in node.target_clients.values():
        client.publish = lambda commands: sent.append(commands)
    try:
        yield node, sent
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_node_merges_arms_and_isolates_bad_ik(ros_node):
    node, sent = ros_node
    for s in ad.SIDES:
        node.request(s, 'enable')
    node.cycle()
    assert len(sent) == 1 and set(sent[0]) == set(ad.SIDES)
    def fail(*args):
        raise ValueError('test IK failure')
    node.arms['left'].solver.solve = fail
    for a in node.arms.values():
        a.last_step -= .01
    node.cycle()
    assert node.arms['left'].state == 'Fault'
    assert node.arms['right'].active and set(sent[-1]) == {'right'}


def test_disable_during_ik_cannot_publish_candidate(ros_node):
    node, sent = ros_node
    node.request('left', 'enable')
    node.cycle()
    count = len(sent)
    a = node.arms['left']
    solve = a.solver.solve
    def disable(pose, seed):
        node.request('left', 'disable')
        return solve(pose, seed)
    a.solver.solve = disable
    a.last_step -= .01
    node.cycle()
    assert len(sent) == count and not a.active


def test_close_during_ik_discards_both_candidates(ros_node):
    node, sent = ros_node
    for side in ad.SIDES:
        node.request(side, 'enable')
    node.cycle()
    original = node.arms['right'].solver.solve
    def close(pose, seed):
        node.close()
        return original(pose, seed)
    node.arms['right'].solver.solve = close
    for a in node.arms.values():
        a.last_step -= .01
    node.cycle()
    assert len(sent) == 1 and node.shutdown_requested.is_set()


def test_disable_enable_between_ticks_still_reinitializes(ros_node):
    node, sent = ros_node
    node.request('left', 'enable')
    node.cycle()
    a = node.arms['left']
    a.velocity[:] = .1
    node.request('left', 'disable')
    node.request('left', 'enable')
    node.cycle()
    assert a.active and not a.velocity.any()
    np.testing.assert_allclose(a.command_pose, a.actual_pose)


def test_limiter_trip_stops_both_and_no_recovery_on_armed(ros_node):
    node, sent = ros_node
    for side in ad.SIDES:
        node.request(side, 'enable')
    node.cycle()
    node.limiter(String(data='TRIPPED'))
    node.cycle()
    assert all(a.state == 'Fault' for a in node.arms.values())
    assert len(sent) == 1
    node.limiter(String(data='ARMED'))
    node.cycle()
    assert len(sent) == 1


@pytest.mark.skipif(os.environ.get('ADMITTANCE_ROS_MOCK') != '1', reason='opt-in full bimanual mock')
def test_bimanual_mock_hand_guiding(tmp_path):
    from test_collision_force_launch import stack
    with stack(tmp_path, []) as (probe, process, log):
        args = ['--ros-args']
        for side in ad.SIDES:
            args += ['-r', f'{ad.arm_topic(side, "force_torque_sensor_broadcaster/wrench")}:=/collision_test/{side}/wrench']
        node = ad.AdmittanceNode(cli_args=args, use_global_arguments=False)
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        executor.add_node(probe)
        received = []
        probe.create_subscription(JointState, ad.JOINT_TARGETS_TOPIC,
                                  lambda m: received.append((time.monotonic(), m)), 100)
        publishers = {s: probe.create_publisher(WrenchStamped, f'/collision_test/{s}/wrench', 1)
                      for s in ad.SIDES}
        forces = {s: np.zeros(6) for s in ad.SIDES}
        probe.create_timer(.01, lambda: [p.publish(wrench(s, forces[s])) for s, p in publishers.items()])

        def wait(predicate, timeout=20.):
            deadline = time.monotonic()+timeout
            while not predicate() and time.monotonic() < deadline:
                executor.spin_once(timeout_sec=.002)
            assert predicate(), (node.read_snapshot(), log.read_text()[-3000:])

        def spin(duration):
            until = time.monotonic()+duration
            while time.monotonic() < until:
                executor.spin_once(timeout_sec=.002)

        try:
            wait(lambda: node.limiter_state == 'ARMED' and all(
                not a.readiness(time.monotonic()) for a in node.arms.values()) and not node.chain_error)
            assert not received
            start = {s: a.actual_pose.copy() for s, a in node.arms.items()}
            for side in ad.SIDES:
                node.request(side, 'enable')
            wait(lambda: all(a.state == 'Enabled' for a in node.arms.values()))
            # A small local-Z force exercises both flange-frame IK chains.
            forces['left'][2] = 2.
            forces['right'][2] = -2.
            spin(.6)
            assert all(a.active for a in node.arms.values()), node.read_snapshot()
            assert all(np.linalg.norm(a.actual_pose[:3, 3]-start[s][:3, 3]) > 1e-5
                       for s, a in node.arms.items())
            for side, a in node.arms.items():
                displacement = a.actual_pose[:3, 3]-start[side][:3, 3]
                expected = start[side][:3, 2] * (1 if side == 'left' else -1)
                assert np.dot(displacement, expected)/np.linalg.norm(displacement) > .9
            assert any(len(m.name) == 12 for _, m in received)
            node.request('left', 'disable')
            wait(lambda: node.arms['left'].state == 'Disabled')
            spin(.05)
            since = len(received)
            spin(.1)
            assert received[since:] and all(m.name == ad.JOINT_NAMES['right'] for _, m in received[since:])
            forces['left'][:] = 0.
            # Do not re-enable until the existing interpolator has finished its tail.
            wait(lambda: np.max(np.abs(node.arms['left'].joint_velocity)) < ad.ENABLE_JOINT_SPEED)
            node.request('left', 'enable')
            wait(lambda: node.arms['left'].active)
            np.testing.assert_allclose(node.arms['left'].command_pose, node.arms['left'].actual_pose, atol=1e-4)
            forces['right'][2] = 21.
            wait(lambda: node.limiter_state == 'TRIPPED')
            assert all(not a.active for a in node.arms.values())
            wait(lambda: 'Both hardware components confirmed inactive' in log.read_text())
            assert process.poll() is None
        finally:
            executor.shutdown()
            node.destroy_node()


@pytest.mark.skipif(os.environ.get('ADMITTANCE_GUI_TEST') != '1', reason='opt-in display smoke test')
def test_gui_buttons_gains_and_shutdown():
    rclpy.init(domain_id=186)
    node = ad.AdmittanceNode()
    node.timer.cancel()
    node.limiter_state, node.chain_error, node.next_graph_check = 'ARMED', '', math.inf
    node.arms = {s: arm(s, time.monotonic()) for s in ad.SIDES}
    for client in node.target_clients.values():
        client.publish = lambda _: None
    window = ad.AdmittanceWindow(node)
    window.root.withdraw()
    def fake_feedback_cycle():
        now = time.monotonic()
        for side, a in node.arms.items():
            a.feedback(joints(side), now)
            a.wrench(wrench(side), now)
        node.cycle()
    node.create_timer(.01, fake_feedback_cycle)
    worker = threading.Thread(target=ad.run_ros, args=(node,))
    worker.start()
    def wait(predicate):
        deadline = time.monotonic()+3.
        while not predicate() and time.monotonic() < deadline:
            window.root.update()
            time.sleep(.005)
        assert predicate(), node.read_snapshot()
    try:
        window.buttons['left'][0].invoke()
        wait(lambda: node.read_snapshot()['left']['state'] in ('Enabling', 'Enabled'))
        assert node.read_snapshot()['right']['state'] == 'Disabled'
        window.fields['left'][0][0].set('25')
        window.apply('left')
        wait(lambda: node.arms['left'].target_mass[0] == 25.)
        window.buttons['left'][1].invoke()
        wait(lambda: node.read_snapshot()['left']['state'] == 'Disabled')
        window.fields['left'][0][0].set('-1')
        window.apply('left')
        wait(lambda: 'positive' in node.read_snapshot()['left']['error'])
        assert node.arms['left'].target_mass[0] == 25.
        window.close()
        deadline = time.monotonic()+3.
        while not node.done.is_set() and time.monotonic() < deadline:
            window.root.update()
            time.sleep(.01)
        assert node.done.is_set()
        worker.join(timeout=1.)
        assert not worker.is_alive()
        assert all(not a.active and not a.velocity.any() for a in node.arms.values())
    finally:
        node.close()
        worker.join(timeout=2.)
        try:
            window.root.destroy()
        except window.tk.TclError:
            pass
