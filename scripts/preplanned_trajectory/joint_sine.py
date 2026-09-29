#!/usr/bin/env python3
"""Run same-phase sine motion from the measured joints of one or both arms.

Source ROS and the workspace in each terminal first.
Terminal 1 (mock):
    ros2 launch dual_crx_control dual_arm.launch.py mock:=true rviz:=false input_rate_hz:=50.0
Terminal 2 (both arms):
    ros2 run dual_crx_control joint_sine.py --arms left right --joint 1 --amplitude-deg 1 --period 4 --duration 20 --rate 50
Single arm / continuous:
    ros2 run dual_crx_control joint_sine.py --arms left --joint 1 --amplitude-deg 1 --duration 20 --rate 50
    ros2 run dual_crx_control joint_sine.py --arms left right --amplitude-deg 1 --continuous --rate 50

Prefixes are fixed: left_J1..J6 and right_J1..J6; select arms with --arms.
Defaults: both arms, J1, 20 deg amplitude, 4 s period, 10 s sine duration,
1 s initial hold/ramp, 50 Hz. Match --rate to launch input_rate_hz.
Single-arm finite motion returns directly then holds for 100/rate seconds;
dual-arm finite motion returns smoothly over max(ramp-time, 1) seconds,
then holds 0.2 s. Override with --return-mode/--return-duration/--final-hold.
Press ENTER to start, or pass --yes. Ctrl+C stops publishing without returning.
Interpolation holds its last target. No automatic move to the home pose.
--output-dir defaults to motion_recordings; each run saves joints.csv and
one six-joint PNG per selected arm. --latency-csv preserves optional event traces.
Use --help for all options, including --plot-file and --show-plot.
"""

from dual_crx_control.motion.joint_sine import main


if __name__ == '__main__':
    main()
