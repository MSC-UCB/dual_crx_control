#!/usr/bin/env python3
"""Run synchronized world-axis sine translation with fixed TCP orientations.

Source ROS and the workspace in each terminal first.
Terminal 1 (mock):
    ros2 launch dual_crx_control dual_arm.launch.py mock:=true rviz:=false input_rate_hz:=50.0
Terminal 2:
    ros2 run dual_crx_control cartesian_sine.py --axis x --amplitude-m 0.02 --period 4 --cycles 2 --rate 50 --output-dir motion_recordings

Both arms are required: left_J1..J6 and right_J1..J6. No arbitrary prefix.
Defaults: X axis, 0.02 m amplitude, 4 s period, 50 Hz; cycles=0 repeats.
Match --rate to launch input_rate_hz. Startup approaches INITIAL_JOINTS_DEG;
--no-move-to-initial starts from measured poses instead. Motion starts when ready.
Ctrl+C stops targets and saves output; interpolation holds its last target.
Each timestamped output directory contains joints.csv, left/right joint plots,
and additional TCP CSV, plot and JSON. TCP positions use joint FK, not external measurements.
Use --help for CLI options. Existing ROS parameters (--ros-args -p name:=value)
remain supported; explicit CLI options override matching ROS parameter values.
"""

from dual_crx_control.motion.cli import run


if __name__ == '__main__':
    run('cartesian_sine')
