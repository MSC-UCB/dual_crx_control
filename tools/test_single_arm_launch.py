"""Opt-in full single-arm mock bringup, motion, admittance and force-stop checks.

SINGLE_ARM_ROS_MOCK=1 python3 -m pytest tools/test_single_arm_launch.py
Never starts a physical driver. Each launch is isolated in a private ROS domain.
"""
from contextlib import contextmanager
import importlib.util
import os
from pathlib import Path
import signal
import subprocess
import time
import xml.etree.ElementTree as ET

import numpy as np
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile
from controller_manager_msgs.srv import ListControllers, ListHardwareComponents
from rcl_interfaces.srv import GetParameters
from geometry_msgs.msg import WrenchStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage

from dual_crx_control.motion.cartesian_controller import DualCartesianController
from dual_crx_control.robot.joint_config import (
    INTERPOLATED_COMMANDS_TOPIC, JOINT_NAMES, JOINT_STATES_TOPIC, JOINT_TARGETS_TOPIC,
    ROBOT_DESCRIPTION_TOPIC)

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('single_launch_admittance', ROOT / 'scripts/admittance_hand_guiding.py')
ad = importlib.util.module_from_spec(spec)
spec.loader.exec_module(ad)
pytestmark = pytest.mark.skipif(os.environ.get('SINGLE_ARM_ROS_MOCK') != '1',
                                reason='set SINGLE_ARM_ROS_MOCK=1 for software mock bringup')
DOMAIN = 189


def wait(executor, predicate, timeout=25.):
    end = time.monotonic() + timeout
    while not predicate() and time.monotonic() < end:
        executor.spin_once(timeout_sec=.005)
    assert predicate(), 'single-arm mock condition timed out'


@contextmanager
def stack(tmp_path, side, read_only, control='none'):
    wrapper = tmp_path / 'single_test.launch.py'
    wrapper.write_text('''
from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import SetRemap
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    remaps = [SetRemap(
        src=f'collision_force_limiter:/crx5ia/{side}/force_torque_sensor_broadcaster/wrench',
        dst='/single_arm_test/wrench') for side in ('left', 'right')]
    include = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        get_package_share_directory('dual_crx_control') + '/launch/single_arm.launch.py'))
    return LaunchDescription([GroupAction([*remaps, include])])
''')
    log_path = tmp_path / 'single_mock.log'
    env = dict(os.environ, ROS_DOMAIN_ID=str(DOMAIN), ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST')
    with log_path.open('w') as log:
        process = subprocess.Popen(
            ['ros2', 'launch', str(wrapper), 'mock:=true', 'rviz:=false',
             f'prefix:={side}_', f'read_only:={str(read_only).lower()}', f'control:={control}',
             'input_rate_hz:=100.0' if control == 'admittance' else 'input_rate_hz:=50.0'],
            env=env, cwd=tmp_path, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    rclpy.init(domain_id=DOMAIN)
    probe = Node('single_arm_launch_probe')
    executor = SingleThreadedExecutor()
    executor.add_node(probe)
    try:
        yield probe, executor, process, log_path
    except Exception:
        print(log_path.read_text())
        raise
    finally:
        executor.shutdown()
        probe.destroy_node()
        rclpy.shutdown()
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=10.)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=5.)


def assert_isolated(probe, side):
    other = 'right' if side == 'left' else 'left'
    assert not any(f'/crx5ia/{other}/' in name for name, _ in probe.get_topic_names_and_types())
    assert not any(f'/crx5ia/{other}/' in name for name, _ in probe.get_service_names_and_types())
    assert not any(namespace == f'/crx5ia/{other}' for _, namespace in probe.get_node_names_and_namespaces())


@pytest.mark.parametrize('side', ['left', 'right'])
@pytest.mark.parametrize('read_only', [False, True])
def test_single_mock_full_chain(tmp_path, side, read_only):
    with stack(tmp_path, side, read_only) as (probe, executor, process, log):
        feedback, commands, states = [], [], []
        descriptions, frames = [], set()
        probe.create_subscription(String, ROBOT_DESCRIPTION_TOPIC,
                                  lambda msg: descriptions.append(msg.data),
                                  QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        def receive_tf(msg):
            frames.update(t.child_frame_id for t in msg.transforms)
        probe.create_subscription(TFMessage, '/tf', receive_tf, 100)
        probe.create_subscription(TFMessage, '/tf_static', receive_tf,
                                  QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        probe.create_subscription(JointState, JOINT_STATES_TOPIC, feedback.append, 10)
        if not read_only:
            probe.create_subscription(JointState, INTERPOLATED_COMMANDS_TOPIC, commands.append, 100)
            probe.create_subscription(String, ad.LIMITER_TOPIC, lambda msg: states.append(msg.data),
                                      QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
            forces = [0.]
            wrench_pub = probe.create_publisher(WrenchStamped, '/single_arm_test/wrench', 1)
            def send_force():
                msg = WrenchStamped()
                msg.wrench.force.x = forces[0]
                wrench_pub.publish(msg)
            probe.create_timer(.01, send_force)
        manager = f'/crx5ia/{side}/controller_manager'
        controller_client = probe.create_client(ListControllers, manager + '/list_controllers')
        hardware_client = probe.create_client(ListHardwareComponents, manager + '/list_hardware_components')
        wait(executor, lambda: feedback and len(feedback[-1].position) == 6 and controller_client.service_is_ready())
        assert feedback[-1].name == JOINT_NAMES[side]
        wait(executor, lambda: descriptions and f'{side}_tcp' in frames
             and f'{side}_fanuc_flange' in frames)
        other = 'right' if side == 'left' else 'left'
        model = ET.fromstring(descriptions[-1])
        assert not any(link.attrib['name'].startswith(f'{other}_') for link in model.findall('link'))
        assert not any(frame.startswith(f'{other}_') for frame in frames)
        def query(client, service):
            future = client.call_async(service.Request())
            wait(executor, future.done)
            return future.result()
        def active_controllers():
            return {c.name for c in query(controller_client, ListControllers).controller if c.state == 'active'}
        required = {'joint_state_broadcaster', 'force_torque_sensor_broadcaster'}
        if not read_only:
            required.add('forward_position_controller')
        wait(executor, lambda: required.issubset(active_controllers()))
        assert_isolated(probe, side)
        if read_only:
            assert 'forward_position_controller' not in active_controllers()
            assert 'joint_interpolation' not in probe.get_node_names()
            assert 'collision_force_limiter' not in probe.get_node_names()
            assert probe.count_publishers(JOINT_TARGETS_TOPIC) == 0
        else:
            wait(executor, lambda: states and states[-1] == 'ARMED')
            target_pub = probe.create_publisher(JointState, JOINT_TARGETS_TOPIC, 1)
            wait(executor, lambda: target_pub.get_subscription_count() > 0)
            goal = np.array(feedback[-1].position)
            goal[0] += .001
            target_pub.publish(JointState(name=JOINT_NAMES[side], position=goal.tolist()))
            wait(executor, lambda: commands and np.allclose(commands[-1].position, goal, atol=1e-9, rtol=0))
            wait(executor, lambda: np.allclose(feedback[-1].position, goal, atol=1e-8, rtol=0))
            assert all(msg.name == JOINT_NAMES[side] for msg in commands)
            probe.destroy_publisher(target_pub)

            motion = DualCartesianController(parameter_overrides=[
                Parameter('active_arms', value=[side]), Parameter('save_plot', value=False),
                Parameter('move_to_initial', value=False), Parameter('amplitude', value=.0002),
                Parameter('period', value=.5), Parameter('cycles', value=1),
                Parameter('ramp_time', value=.1), Parameter('final_hold', value=.1)])
            executor.add_node(motion)
            try:
                wait(executor, lambda: motion.finished or motion.failed)
                assert motion.finished and not motion.failed
                assert set(motion.models) == {side}
            finally:
                executor.remove_node(motion)
                motion.destroy_node()

            guiding = ad.AdmittanceNode(parameter_overrides=[Parameter('active_arms', value=[side])])
            executor.add_node(guiding)
            try:
                try:
                    wait(executor, lambda: not guiding.read_snapshot()[side]['error'], timeout=5.)
                except AssertionError:
                    pytest.fail(str(guiding.read_snapshot()))
                # Wait for the last interpolation segment to settle before enable.
                arm = guiding.arms[side]
                wait(executor, lambda: arm.joint_velocity is not None and np.max(np.abs(arm.joint_velocity)) < .02)
                guiding.request(side, 'enable')
                wait(executor, lambda: arm.active or arm.state == 'Fault')
                assert arm.active, arm.error
                assert_isolated(probe, side)
                guiding.request(side, 'disable')
                wait(executor, lambda: not arm.active)
            finally:
                executor.remove_node(guiding)
                guiding.destroy_node()
            forces[0] = 21.
            wait(executor, lambda: 'TRIPPED' in states)
            wait(executor, lambda: 'Selected hardware component confirmed inactive' in log.read_text())
            hardware = query(hardware_client, ListHardwareComponents)
            assert any(component.name == 'crx5ia' and component.state.id == 2 for component in hardware.component)
            assert 'forward_position_controller' not in active_controllers()
            assert states[-1] == 'TRIPPED'
        assert process.poll() is None, log.read_text()
        assert_isolated(probe, side)
        assert 'Traceback' not in log.read_text()


@pytest.mark.parametrize('control', ['cartesian', 'admittance'])
def test_launch_routes_optional_control_parameters(tmp_path, control):
    if control == 'admittance':
        import tkinter as tk
        try:
            root = tk.Tk()
        except tk.TclError as error:
            pytest.skip(f'GUI display unavailable: {error}')
        root.withdraw()
        root.destroy()
    with stack(tmp_path, 'right', False, control) as (probe, executor, process, log):
        name = 'cartesian_sine' if control == 'cartesian' else 'admittance_hand_guiding'
        client = probe.create_client(GetParameters, f'/crx5ia/{name}/get_parameters')
        commands = []
        probe.create_subscription(JointState, INTERPOLATED_COMMANDS_TOPIC, commands.append, 100)
        wait(executor, client.service_is_ready)
        future = client.call_async(GetParameters.Request(names=['active_arms']))
        wait(executor, future.done)
        assert future.result().values[0].string_array_value == ['right']
        if control == 'cartesian':
            wait(executor, lambda: len(commands) > 20)
            assert all(msg.name == JOINT_NAMES['right'] for msg in commands)
        assert_isolated(probe, 'right')
        assert process.poll() is None and 'process has died' not in log.read_text()
