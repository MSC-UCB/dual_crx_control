# Move once to the Sharpa default pose

`scripts/move_to_default_pose.py` is a source-only entry point. It reuses the
installed `dual_crx_control` Python package. Build/source the updated package once
to install the shared pose loader and `config/initial_pose.yaml`. Subsequent source
config edits take effect on the next invocation with a symlink install.

Joint order is J1 through J6:

| Convention | Left (degrees) | Right (degrees) |
| --- | --- | --- |
| FANUC pendant | 0, 30, -60, 0, 60, 0 | -90, -30, 240, 0, -60, 0 |
| ROS / URDF | 0, 30, -30, 0, 60, 0 | -90, -30, 210, 0, -60, 0 |

Both J6 targets are zero, matching the CRX + Sharpa robot configs in
`retargeting_crx`. ROS teleoperation uses measured feedback for calibration;
changing its configured initial pose does not automatically move the arms.
Mock startup, planned-motion scripts and this script all read
`config/initial_pose.yaml`. Its values use ROS / URDF degrees, J1 through J6.

The script converts the configured ROS degrees to radians for publication.
Pendant J3 is derived only for display: `J3_pendant = J3_ROS - J2_ROS`.
It never applies pendant-to-ROS conversion to the configured targets or feedback.

Source ROS and the workspace in your terminal:

```bash
source /opt/ros/jazzy/setup.bash
source /home/msc-crx/ws_fanuc/install/setup.bash
cd /home/msc-crx/ws_fanuc/src/dual_crx_control
python3 scripts/move_to_default_pose.py
```

Without `--execute`, it only prints targets and does not initialize ROS. To move
with already-running robot drivers and interpolation:

```bash
python3 scripts/move_to_default_pose.py --execute --rate 100
```

Stop teleoperation and other motion publishers first. The script refuses to
publish when another publisher is discovered on `/crx5ia/joint_targets`; this
check is not an exclusive control lock and cannot detect other command topics.
Match `--rate` to the running interpolation node's `input_rate_hz`.

The script waits up to 15 seconds for the transient-local robot description,
both arms' feedback, and a target subscriber. It reads limits from the description
and uses `InitialJointMove` for synchronized quintic joint interpolation. Defaults
are 10 degrees/s maximum trajectory speed, 20 degrees/s² acceleration, and a
minimum duration of 5 seconds; large moves extend the duration automatically.
These bounds apply to the generated targets, not a guarantee of physical tracking
or the downstream interpolation's derivatives.

After the trajectory it waits for both arms to stay within 0.5 degrees of the
target for 1 second, with a 15-second settling timeout. Feedback is checked by
local receipt time and expires after 0.5 seconds. Invalid feedback, changed
description, lost subscriber, out-of-limit feedback, or a competing publisher
cause a nonzero exit and stop new targets. Success exits with code 0.

This does not enable controllers, plan around collisions, or move the hands.
Check the entire path including attached hands and the other arm. On interruption
or failure the existing interpolation node retains its last accepted target;
Ctrl+C is not an emergency stop. Keep the physical stop available during use.

Offline regression tests (no ROS graph or hardware):

```bash
/home/msc-crx/retargeting_crx/.venv/bin/python -m pytest tools/test_move_to_default_pose.py -q
```
