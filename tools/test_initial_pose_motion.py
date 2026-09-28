"""Offline motion checks using installed ROS Python/KDL libraries; no nodes."""

import importlib.util
from pathlib import Path
from unittest.mock import Mock, patch

import numpy as np
import pytest

pytest.importorskip('PyKDL')
xacro = pytest.importorskip('xacro')
pytest.importorskip('rclpy')
from dual_crx_control.robot.initial_pose import load_initial_radians
from dual_crx_control.robot.kinematics import CRXKinematics
from dual_crx_control.robot.ik_solver import DampedLeastSquaresIK
from dual_crx_control.motion.facing_circle import (
    facing_start_poses, solve_facing_start, check_joint_approach, scaled_min_singular_value)
from dual_crx_control.motion.circular_trajectory import CircularTrajectory
from dual_crx_control.motion.cli import parse_parameters

ROOT = Path(__file__).resolve().parents[1]


def test_cartesian_initial_override_preserved():
    assert parse_parameters('cartesian_sine', []) == []
    for kind in ('cartesian_sine', 'cartesian_circle', 'facing_circle'):
        params = parse_parameters(kind, ['--left-initial-deg', '1', '2', '3', '4', '5', '6'])
        assert {p.name: p.value for p in params} == {'left_initial_deg': [1., 2., 3., 4., 5., 6.]}


def test_simple_motion_approaches_config_before_oscillating():
    spec = importlib.util.spec_from_file_location('simple_motion_pose_test',
        ROOT / 'scripts/preplanned_trajectory/simple_motion.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    home = load_initial_radians()
    starts = {s: [v + .1 for v in q] for s, q in home.items()}
    node = Mock(args=module.parse_args(['--startup-duration', '2']), states=starts,
                done=False, startup_at=None, started_at=None)
    node.target_client.available.return_value = True
    with patch.object(module.time, 'monotonic', return_value=10.) as clock:
        module.SimpleMotion.tick(node)
        assert node.endpoints[0] == home['left'] + home['right']
        clock.return_value = 12.
        module.SimpleMotion.tick(node)
        assert node.started_at is None
        assert node.publish_target.call_args.args[0] == home['left'] + home['right']
        node.states = home
        module.SimpleMotion.tick(node)
        assert node.started_at == 12.


@pytest.mark.parametrize('center,gap,period,max_velocity', [
    ([.55, -.38, .35], .02, 3., 3.), ([.55, -.38, .30], .2, 4., 2.)])
def test_facing_approach_and_circle_from_shared_pose(center, gap, period, max_velocity):
    xml = xacro.process_file(str(ROOT / 'urdf/dual_crx.urdf.xacro')).toxml()
    home = {s: np.asarray(q) for s, q in load_initial_radians().items()}
    models = {s: CRXKinematics(xml, s + '_tcp') for s in home}
    solvers = {s: DampedLeastSquaresIK(m) for s, m in models.items()}
    assert all(models[s].valid_joints(q) for s, q in home.items())
    poses = facing_start_poses(center, gap, .1, 'xz')
    seeds = solve_facing_start(models, solvers, home, poses)
    assert check_joint_approach(models, home, seeds, .1) >= .1
    curve = CircularTrajectory(.1, period, 'xz', 'cw')
    for t in np.linspace(0., period, 201):
        for side, model in models.items():
            pose = poses[side].copy()
            pose[:3, 3] += curve.offset(t, 1)
            result = solvers[side].solve(pose, seeds[side])
            assert result.success, result.reason
            assert model.valid_joints(result.q)
            assert scaled_min_singular_value(model, result.q) >= .1
            assert np.max(np.abs(result.q - seeds[side])) / (period / 200) < max_velocity
            seeds[side] = result.q
