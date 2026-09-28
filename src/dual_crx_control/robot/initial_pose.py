"""Shared initial pose; ROS joint convention, degrees in YAML, radians on wire."""

import math
from pathlib import Path

import yaml


def initial_pose_path():
    # Source checkout / symlink install: keep offline tools independent of ROS.
    source = Path(__file__).resolve().parents[3] / 'config' / 'initial_pose.yaml'
    if source.is_file():
        return source
    from ament_index_python.packages import get_package_share_directory

    return Path(get_package_share_directory('dual_crx_control')) / 'config' / 'initial_pose.yaml'


def load_initial_degrees(path=None):
    """Read fresh copies of six ROS degree values per side; reject malformed data."""
    path = initial_pose_path() if path is None else Path(path)
    with path.open() as stream:
        data = yaml.safe_load(stream)
    if not isinstance(data, dict) or set(data) != {'left', 'right'}:
        raise ValueError(f'{path}: expected exactly left and right ROS degree lists')
    for side, values in data.items():
        if (not isinstance(values, list) or len(values) != 6
                or any(isinstance(v, bool) or not isinstance(v, (int, float))
                       or not math.isfinite(v) for v in values)):
            raise ValueError(f'{path}: {side} must contain six finite numeric ROS angles')
    return {side: [float(v) for v in data[side]] for side in ('left', 'right')}


def load_initial_radians(path=None):
    return {side: [math.radians(v) for v in values]
            for side, values in load_initial_degrees(path).items()}
