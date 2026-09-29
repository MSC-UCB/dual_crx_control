"""Opt-in full bimanual mock integration; never starts a physical driver.

COLLISION_ROS_MOCK=1 python3 -m pytest -q tools/test_collision_force_launch.py
"""
from contextlib import contextmanager
import os
from pathlib import Path
import re
import signal
import subprocess
import time

import pytest
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from controller_manager_msgs.srv import ListControllers, ListHardwareComponents
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import String


pytestmark = pytest.mark.skipif(os.environ.get('COLLISION_ROS_MOCK') != '1',
                                reason='set COLLISION_ROS_MOCK=1 for full mock launch')
DOMAIN = 184


def wait(node, predicate, timeout=25.):
    deadline = time.monotonic() + timeout
    while not predicate() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=.01)
    assert predicate(), 'bimanual mock condition timed out'


@contextmanager
def stack(tmp_path, arguments):
    # Node-qualified remaps affect only limiter subscriptions, not broadcasters.
    wrapper = tmp_path / 'collision_test.launch.py'
    wrapper.write_text('''
from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import SetRemap
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    remaps = [SetRemap(
        src=f'collision_force_limiter:/crx5ia/{side}/force_torque_sensor_broadcaster/wrench',
        dst=f'/collision_test/{side}/wrench') for side in ('left', 'right')]
    include = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        get_package_share_directory('bimanual_manipulation') + '/launch/bimanual_system.launch.py'))
    return LaunchDescription([GroupAction([*remaps, include])])
''')
    log_path = tmp_path / 'bimanual_mock.log'
    environment = dict(os.environ, ROS_DOMAIN_ID=str(DOMAIN),
                       ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST')
    with log_path.open('w') as log:
        process = subprocess.Popen(
            ['ros2', 'launch', str(wrapper), 'crx_mock:=true', 'sharpa_backend:=mock',
             'rviz:=false', 'viewer:=false', *arguments],
            env=environment, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    rclpy.init(domain_id=DOMAIN)
    node = Node('collision_mock_probe')
    try:
        yield node, process, log_path
    finally:
        node.destroy_node()
        rclpy.shutdown()
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=15.)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=5.)


def clients(node, kind, suffix):
    result = {s: node.create_client(kind, f'/crx5ia/{s}/controller_manager/{suffix}')
              for s in ('left', 'right')}
    wait(node, lambda: all(c.service_is_ready() for c in result.values()))
    return result


def query(node, services, kind):
    futures = {s: client.call_async(kind.Request()) for s, client in services.items()}
    wait(node, lambda: all(f.done() for f in futures.values()))
    return {s: f.result() for s, f in futures.items()}


@pytest.mark.parametrize('threshold', [None, 25.])
def test_bimanual_default_and_threshold_override_stop_both(tmp_path, threshold):
    arguments = [] if threshold is None else [f'collision_force_threshold_n:={threshold}']
    with stack(tmp_path, arguments) as (node, process, log):
        states, forces = [], {'left': 0., 'right': (threshold or 20.) - .5}
        node.create_subscription(String, '/crx5ia/collision_force_limiter/state',
                                 lambda msg: states.append(msg.data),
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        publishers = {s: node.create_publisher(WrenchStamped, f'/collision_test/{s}/wrench', 1)
                      for s in ('left', 'right')}

        def send():
            for side, publisher in publishers.items():
                msg = WrenchStamped()
                msg.wrench.force.y = forces[side]
                publisher.publish(msg)

        node.create_timer(.01, send)
        hardware = clients(node, ListHardwareComponents, 'list_hardware_components')
        controllers = clients(node, ListControllers, 'list_controllers')
        wait(node, lambda: 'ARMED' in states)
        assert process.poll() is None, log.read_text()
        names = node.get_node_names()
        assert names.count('collision_force_limiter') == 1
        assert all(any(c.name == 'crx5ia' and c.state.id == 3 for c in r.component)
                   for r in query(node, hardware, ListHardwareComponents).values())
        forces['right'] = -((threshold or 20.) + .1)
        wait(node, lambda: 'TRIPPED' in states)

        def stopped():
            return 'Both hardware components confirmed inactive' in log.read_text()

        wait(node, stopped)
        assert all(any(c.name == 'crx5ia' and c.state.id == 2 for c in r.component)
                   for r in query(node, hardware, ListHardwareComponents).values())
        assert all(any(c.name == 'forward_position_controller' and c.state == 'inactive'
                       for c in r.controller)
                   for r in query(node, controllers, ListControllers).values())
        forces['right'] = 0.
        until = time.monotonic() + .4
        while time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=.01)
        assert states[-1] == 'TRIPPED' and process.poll() is None
        assert 'STOP_FAILED' not in states


@pytest.mark.parametrize('option', ['collision_force_limit:=false', 'read_only:=true'])
def test_bimanual_disabled_and_readonly_have_no_limiter(tmp_path, option):
    with stack(tmp_path, [option]) as (node, process, log):
        hardware = clients(node, ListHardwareComponents, 'list_hardware_components')
        assert len(query(node, hardware, ListHardwareComponents)) == 2
        wait(node, lambda: 'dual_crx_joint_state_merger' in node.get_node_names())
        assert 'collision_force_limiter' not in node.get_node_names()
        assert 'collision_force_limiter.py' not in log.read_text()
        assert process.poll() is None


def test_limiter_exit_shuts_down_bimanual_launch(tmp_path):
    with stack(tmp_path, []) as (node, process, log):
        pattern = r'\[collision_force_limiter.py-\d+\]: process started with pid \[(\d+)\]'
        wait(node, lambda: re.search(pattern, log.read_text()) is not None)
        wait(node, lambda: 'collision_force_limiter' in node.get_node_names())
        pid = int(re.search(pattern, log.read_text()).group(1))
        os.kill(pid, signal.SIGTERM)
        wait(node, lambda: process.poll() is not None, timeout=20.)
        # launch completion waits for its children; no unprotected stack remains.
        assert 'sending signal' in log.read_text()
