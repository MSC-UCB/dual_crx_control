"""Shared pose and mock-only injection; no ROS nodes or hardware."""

import importlib.util
import math
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace
import xml.etree.ElementTree as ET

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))
from dual_crx_control.robot import initial_pose


def test_source_pose_and_independent_copies():
    assert initial_pose.initial_pose_path() == ROOT / 'config/initial_pose.yaml'
    expected = {'left': [0, 30, -30, 0, 60, 0], 'right': [-90, -30, 210, 0, -60, 0]}
    assert initial_pose.load_initial_degrees() == expected
    first = initial_pose.load_initial_degrees()
    first['left'][0] = 99
    assert initial_pose.load_initial_degrees() == expected
    from dual_crx_control.robot.joint_config import INITIAL_JOINTS_DEG
    assert INITIAL_JOINTS_DEG == expected
    for side, values in initial_pose.load_initial_radians().items():
        assert values == pytest.approx([math.radians(v) for v in expected[side]])


def test_installed_package_share_lookup(tmp_path, monkeypatch):
    module_file = tmp_path / 'lib/python/site-packages/dual_crx_control/robot/initial_pose.py'
    monkeypatch.setattr(initial_pose, '__file__', str(module_file))
    share = tmp_path / 'share/dual_crx_control'
    (share / 'config').mkdir(parents=True)
    config = share / 'config/initial_pose.yaml'
    config.write_text('left: [1, 2, 3, 4, 5, 6]\nright: [6, 5, 4, 3, 2, 1]\n')
    package = ModuleType('ament_index_python')
    packages = ModuleType('ament_index_python.packages')
    def lookup(name):
        assert name == 'dual_crx_control'
        return str(share)
    packages.get_package_share_directory = lookup
    monkeypatch.setitem(sys.modules, 'ament_index_python', package)
    monkeypatch.setitem(sys.modules, 'ament_index_python.packages', packages)
    assert initial_pose.initial_pose_path() == config
    assert initial_pose.load_initial_degrees()['left'] == [1, 2, 3, 4, 5, 6]


@pytest.mark.parametrize('data', [None, [], {'left': [0]*6},
    {'left': [0]*6, 'right': [0]*6, 'units': 'radians'},
    {'left': [0]*5, 'right': [0]*6}, {'left': [True]*6, 'right': [0]*6},
    {'left': ['0']*6, 'right': [0]*6}, {'left': [float('nan')]*6, 'right': [0]*6},
    {'left': [0]*6, 'right': [float('inf')]*6}])
def test_bad_config_fails(tmp_path, data):
    path = tmp_path / 'initial_pose.yaml'
    path.write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError):
        initial_pose.load_initial_degrees(path)


def test_missing_config_fails(tmp_path):
    with pytest.raises(FileNotFoundError):
        initial_pose.load_initial_degrees(tmp_path / 'missing.yaml')


def load_description(monkeypatch, xml):
    xacro = ModuleType('xacro')
    calls = []
    def process(path, mappings):
        calls.append(mappings)
        return SimpleNamespace(toxml=lambda: xml)
    xacro.process_file = process
    monkeypatch.setitem(sys.modules, 'xacro', xacro)
    spec = importlib.util.spec_from_file_location(
        'dual_crx_control.robot._description_test', ROOT / 'src/dual_crx_control/robot/description.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module, calls


@pytest.mark.parametrize('side', ['left', 'right'])
def test_mock_injects_shared_pose(side, monkeypatch):
    xml = '<robot><ros2_control>' + ''.join(
        f'<joint name="{side}_J{i}"><state_interface name="position">'
        '<param name="initial_value">99</param></state_interface></joint>'
        for i in range(1, 7)) + '</ros2_control></robot>'
    module, calls = load_description(monkeypatch, xml)
    result = ET.fromstring(module.arm_description('driver.xacro', side, 'unused', True))
    values = [float(p.text) for p in result.findall('.//param')]
    assert values == pytest.approx(initial_pose.load_initial_radians()[side])
    assert calls[0]['use_mock'] == 'true'


def test_real_mode_does_not_read_pose_or_inject_positions(monkeypatch):
    xml = '<robot name="physical"><ros2_control/></robot>'
    module, calls = load_description(monkeypatch, xml)
    def unexpected():
        pytest.fail('Physical driver description must not read mock initial pose')
    monkeypatch.setattr(module, 'load_initial_radians', unexpected)
    assert module.arm_description('driver.xacro', 'left', 'unused', False) == xml
    assert calls[0]['use_mock'] == 'false'
