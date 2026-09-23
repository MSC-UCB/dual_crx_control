#!/usr/bin/env python3
"""Run synchronized inward-facing TCP circles at the configured world center.

Source ROS and the workspace in each terminal first.
Terminal 1 (mock):
    ros2 launch dual_crx_control dual_arm.launch.py mock:=true rviz:=false input_rate_hz:=50.0
Terminal 2:
    ros2 run dual_crx_control facing_circle.py --plane xz --radius-m 0.1 --direction cw --period 3 --cycles 10 --rate 50 --tcp-gap-m 0.02 --center-midpoint 0.55 -0.38 0.35 --output-dir motion_recordings

Both arms use left_J1..J6 and right_J1..J6. Local TCP +X faces inward,
and +Z stays world-up. Defaults match the command above; max_velocity=3 rad/s.
Startup approaches initial joints, then the facing poses, with IK/conditioning checks.
Placement was checked kinematically in mock; physical tool/link collisions were not checked.
Match --rate to launch input_rate_hz. --cycles 0 repeats until Ctrl+C.
Motion starts when ready. Ctrl+C stops targets and saves output; interpolation holds.
Timestamped output: joints.csv, left/right joint plots, TCP plane CSV/plot/JSON.
Use --help for all options. ROS parameters remain supported; explicit CLI values take precedence.
"""

from dual_crx_control.motion.cli import run


if __name__ == '__main__':
    run('facing_circle')
