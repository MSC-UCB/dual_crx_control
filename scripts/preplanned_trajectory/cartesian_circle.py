#!/usr/bin/env python3
"""Run synchronized TCP circles in a world-coordinate plane.

Source ROS and the workspace in each terminal first.
Terminal 1 (mock):
    ros2 launch dual_crx_control dual_arm.launch.py mock:=true rviz:=false input_rate_hz:=50.0
Terminal 2:
    ros2 run dual_crx_control cartesian_circle.py --plane xy --radius-m 0.02 --direction ccw --period 8 --cycles 1 --rate 50 --output-dir motion_recordings

Both arms use left_J1..J6 and right_J1..J6. No arbitrary prefix.
Defaults: XY plane, 0.02 m radius, counterclockwise, 8 s/lap, 1 lap, 50 Hz.
Match --rate to launch input_rate_hz. --cycles 0 repeats until Ctrl+C.
Startup approaches INITIAL_JOINTS_DEG; --no-move-to-initial skips that approach.
Motion starts when ready. Ctrl+C stops targets; interpolation holds its last target.
Timestamped output: joints.csv, left/right joint plots, TCP plane CSV/plot/JSON.
TCP positions are computed from joint FK. Use --help for all options;
existing ROS parameters remain supported, with explicit CLI options taking precedence.
"""

from dual_crx_control.motion.cli import run


if __name__ == '__main__':
    run('cartesian_circle')
