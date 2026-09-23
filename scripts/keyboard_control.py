#!/usr/bin/env python3
"""Keyboard Cartesian Control (Linux / WSL interactive terminal).

Setup (in both terminals, from your ROS workspace):
    source /opt/ros/jazzy/setup.bash
    source install/setup.bash

Usage:
    # Terminal 1: software/mock bringup with Ruckig
    ros2 launch dual_crx_control dual_arm.launch.py mock:=true method:=ruckig
    # Terminal 2: keyboard input (keep this terminal focused)
    ros2 run dual_crx_control keyboard_control.py
    # Optional smaller step:
    ros2 run dual_crx_control keyboard_control.py --ros-args -p step_m:=0.0005

Keys:
    1 / 2     Select left / right arm; always disarms.
    e         Enable selected arm at its CURRENT measured TCP pose.
    w / s     World X + / -
    a / d     World Y + / -
    r / f     World Z + / -
    Space     Disarm and discard pending keyboard input.
    q, Ctrl+C Exit and restore terminal settings.

Example: press 1, e, w to move the left TCP +1 mm in world X.
Then press 2, e, r to move the right TCP +1 mm in world Z.
No automatic initial-pose move. Targets keep the orientation captured at enable.
Control points are left_tcp / right_tcp, NOT the flanges. Each input batch applies
at most one step; holding a key uses OS repeat, not key-release/deadman detection.
Enable and arm selection discard other keys in the same batch: press separately.

Parameters (ROS --ros-args -p name:=value):
    step_m=0.001                 Cartesian increment; maximum 0.005 m.
    state_timeout=0.25          Feedback age limit, seconds.
    max_joint_step=0.03         Maximum per-target joint change, radians.
    max_joint_velocity=0.5      Target increment / 0.02 s limit, rad/s.
    tracking_tolerance=0.1      Maximum target-to-feedback joint error, radians.
    robot_description=''       Otherwise read transient-local /crx5ia/robot_description.
Input checks run at 50 Hz; existing Ruckig interpolation outputs at 500 Hz with
its configured velocity/acceleration/jerk limits, NOT a fixed 20 ms arrival time.
The node requires /crx5ia/joint_interpolation to report method=ruckig before enabling.

Safety:
    No Cartesian displacement radius is enforced around the enable position.
    Use ONE command source only; stop other teleop/motion publishers first.
    Start in mock. No collision checking or certified safety stop is provided.
    Space, switching arms and exit do NOT cancel an accepted Ruckig trajectory:
    it can finish the last target before holding. Use the hardware emergency stop
    when needed. The unselected arm receives no new targets from this program.
    Joint-space interpolation does not guarantee a straight TCP path or constant
    orientation between endpoints. Invalid/stale feedback disarms; press e again
    after resolving the cause. Limits are experimental, not safety certification.
"""

from dual_crx_control.teleop.keyboard import main


if __name__ == '__main__':
    main()
