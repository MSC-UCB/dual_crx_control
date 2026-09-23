# Dual CRX Control

ROS 2 tools for controlling two FANUC CRX-5iA arms: joint and Cartesian trajectories, keyboard control, external teleoperation, and motion recording. Start with mock hardware and RViz before configuring a connection to physical robots.

For a first run, follow **Setup and build** and **Quick start** below. All commands use a **Linux or WSL Bash terminal**, not Windows Command Prompt or PowerShell.

## Setup and build

The project targets ROS 2 Jazzy. Install ROS 2, `colcon`, `rosdep`, and Git LFS first.

Download the official [FANUC ROS 2 driver](https://github.com/FANUC-CORPORATION/fanuc_driver) and robot descriptions into the same workspace:

```bash
sudo apt install git-lfs
git lfs install
cd <workspace>/src
git clone https://github.com/FANUC-CORPORATION/fanuc_description.git
git clone --recurse-submodules https://github.com/FANUC-CORPORATION/fanuc_driver.git
```

The official driver's `main` branch targets ROS 2 Jazzy. See [package.xml](package.xml) for the remaining dependencies.

Place this package in the `src` directory of a ROS workspace, for example:

```text
~/ros2_ws/
└── src/
    ├── dual_crx_control/
    └── ... FANUC driver and description packages, unless already installed
```

The commands below assume that the package is located at `<workspace>/src/dual_crx_control`. Replace `<workspace>` with the root directory of your ROS workspace.

Run these commands from the workspace root:

```bash
cd <workspace>
source /opt/ros/jazzy/setup.bash
# If FANUC packages are built in another workspace, source its install/setup.bash here.
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-up-to dual_crx_control
source install/setup.bash
ros2 pkg executables dual_crx_control
```

The last command should list executables such as `joint_sine.py`, `keyboard_control.py`, and `interpolation_node`. If the FANUC dependencies cannot be resolved, obtain and build the corresponding driver and description packages first; they are not included in this repository.

**In every new terminal, prepare the environment before running commands:**

```bash
cd <workspace>
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

## Quick start: a small joint motion in mock mode

Before start, please jog the robot near the following pose

| Arm | J1–J6, degrees |
| --- | --- |
| Left | `0, 0, 0, 0, -90, 0` |
| Right | `-90, 0, 180, 0, 90, 0` |

Then make sure there is no alarm and the manual mode is off.

![Dual CRX control system overview](docs/images/robot_overview.png)

### Terminal 1: start the control system

```bash
ros2 launch dual_crx_control dual_arm.launch.py \
  mock:=true rviz:=true method:=linear input_rate_hz:=50.0
```

This starts both simulated arms, their controllers, the interpolation node, and RViz. Wait for the controllers to finish starting before sending motion commands. Use `rviz:=false` if a graphical display is unavailable.

![Dual CRX rviz overview](docs/images/robot_in_riviz.png)

### Terminal 2: send a trajectory

After sourcing the environment as shown above, run:

```bash
ros2 run dual_crx_control joint_sine.py \
  --arms left right --joint 1 --amplitude-deg 1 \
  --period 4 --duration 10 --rate 50
```

Press **Enter** when prompted. Both arms move J1 around their current measured positions with a 1-degree sine amplitude and a 4-second period. `--duration 10` specifies the sine-motion duration; initial hold, return, and final hold add to the total runtime.

When the motion finishes, the script saves joint CSV data and plots. The control system in terminal 1 stays running.

To interrupt the motion, press `Ctrl+C` in terminal 2. The script stops sending targets and saves its recordings; the interpolator may finish the last accepted target before holding position. To end the mock session, then press `Ctrl+C` in terminal 1.

Run only one control launch and one motion-command source for the same pair of arms. Stop the current command source before trying another mode below. If changing launch files or launch settings, stop the previous launch first.

## Choose a control mode

| Task | Launch | Command source |
| --- | --- | --- |
| Run predefined trajectories | `dual_arm.launch.py` | One of the trajectory scripts below |
| Move a TCP with the keyboard | `dual_arm.launch.py method:=ruckig` | `keyboard_control.py` |
| Connect an external teleoperation program | `dual_arm.launch.py` | Publish `JointState` to `/crx5ia/joint_targets` |

### Predefined trajectories

Use these examples with the **50 Hz** launch from the quick start. Run one at a time.

```bash
# Left-arm J1 sine motion; press Enter when prompted.
ros2 run dual_crx_control joint_sine.py --arms left --joint 1 --amplitude-deg 1 --period 4 --duration 10 --rate 50

# Both arms alternate between two joint positions, holding each for 2 seconds.
ros2 run dual_crx_control simple_motion.py --joint 1 --range-deg 1 --hold 2 --duration 10 --rate 50

# Both TCPs oscillate along world X with 0.02 m amplitude for 2 cycles.
ros2 run dual_crx_control cartesian_sine.py --axis x --amplitude-m 0.02 --period 4 --cycles 2 --rate 50

# Both TCPs trace a circle in the XY plane with a 0.02 m radius.
ros2 run dual_crx_control cartesian_circle.py --plane xy --radius-m 0.02 --direction ccw --period 8 --cycles 1 --rate 50

# Facing TCPs trace circles; inspect the pose and placement in mock mode first.
ros2 run dual_crx_control facing_circle.py --plane xz --radius-m 0.1 --direction cw --period 3 --cycles 10 --rate 50
```

`simple_motion.py` and the Cartesian scripts **start automatically when ready**. By default, they first approach these configured initial joint positions:

| Arm | J1–J6, degrees |
| --- | --- |
| Left | `0, 0, 0, 0, -90, 0` |
| Right | `-90, 0, 180, 0, 90, 0` |

A small trajectory amplitude therefore does not imply a small total move from the current pose. `joint_sine.py` instead uses the current measured joints as its center, without this initial-pose move. Its default amplitude is 20 degrees; explicitly set a small amplitude for initial trials, as in the examples.

Use `--help` when you need the full argument list. The most common options are the motion amplitude or radius, period, duration/cycle count, selected arm, publishing rate, and output directory. Cartesian scripts also accept ROS parameters through `--ros-args -p` and parameter files. Explicit CLI options take precedence for the same parameter.

```bash
ros2 run dual_crx_control joint_sine.py --help
ros2 run dual_crx_control cartesian_circle.py --help
```

The bundled [facing_circle_mock.yaml](config/facing_circle_mock.yaml) targets the `cartesian_circle` node. Load it with:

```bash
ros2 run dual_crx_control cartesian_circle.py --ros-args \
  --params-file "$(ros2 pkg prefix --share dual_crx_control)/config/facing_circle_mock.yaml"
```

This configuration uses a 0.2 m TCP gap, unlike the 0.02 m default of `facing_circle.py`. For detailed motion behavior, recording formats, and migration from older executable names, see [docs/motion_scripts.md](docs/motion_scripts.md).

### Keyboard TCP control

In terminal 1, launch with Ruckig:

```bash
ros2 launch dual_crx_control dual_arm.launch.py mock:=true rviz:=true method:=ruckig
```

Ruckig keeps independent absolute targets as rest-to-rest waypoints by default.
For a low-frequency reference stream, opt in explicitly:

```bash
ros2 launch dual_crx_control dual_arm.launch.py \
  mock:=true rviz:=false method:=ruckig \
  input_rate_hz:=10.0 ruckig_target_mode:=stream \
  ruckig_target_timeout:=0.2
```

In `stream` mode, `input_rate_hz` describes the expected reference period. The
interpolator estimates bounded target derivatives from arrival times and keeps
tracking between samples. A dropout longer than `ruckig_target_timeout` changes
the terminal derivatives to zero and holds the last target. The reference can
therefore have up to one input period of phase lag; velocity, acceleration and
jerk limits remain enforced. `waypoint` remains the default for motion scripts.

In terminal 2, start keyboard input and keep that terminal focused:

```bash
ros2 run dual_crx_control keyboard_control.py
```

| Key | Action |
| --- | --- |
| `1` / `2` | Select the left / right arm; selection disarms control |
| `e` | Enable control at the selected arm's current measured TCP pose |
| `w` / `s` | World X positive / negative |
| `a` / `d` | World Y positive / negative |
| `r` / `f` | World Z positive / negative |
| Space | Disarm and discard pending keyboard input |
| `q` / `Ctrl+C` | Exit |

For example, press `1`, then `e`, then `w` separately to move the left TCP one step along world +X. The default step is **1 mm**. There is no automatic move to an initial pose. Targets preserve the orientation captured when enabling control; holding a direction key uses the operating system's key repeat.

For a 0.5 mm step:

```bash
ros2 run dual_crx_control keyboard_control.py --ros-args -p step_m:=0.0005
```

The maximum step is 0.005 m. This mode requires `method:=ruckig` and an interactive Linux/WSL terminal. Space, arm switching, and exit do not cancel the last target already accepted by the interpolator.

The keyboard program does not record automatically. To record a session, run the standalone recorder shown under **Recording and analysis** in another terminal.

### External teleoperation

External teleoperation publishes a generic, named `sensor_msgs/msg/JointState`
target stream. This package owns one fixed CRX5IA topic tree; use mock hardware
for headless testing.

```bash
ros2 launch dual_crx_control dual_arm.launch.py \
  mock:=true rviz:=false method:=linear input_rate_hz:=100.0
```

The retargeting script connects to the same fixed `/crx5ia` topic tree:

```bash
.venv/bin/python scripts/run_crx_joint_teleop.py \
  --command-hz 20 --publish-hz 100 \
  --output-interpolation cubic --interpolation-horizon-ms 50
```

The target message contains `left_J1`–`left_J6` and `right_J1`–`right_J6` in
radians. The merged measured feedback is read from `/crx5ia/joint_states`. The core
interpolator publishes the accepted command stream at 500 Hz and holds the last
accepted target until a fresh target arrives. The legacy
`dual_arm_teleop.launch.py`, `teleop_bridge`, and `/teleop/*` topics
have been removed. External publishers must use the fixed `/crx5ia` interface.

| Fixed topic | Message type | Content |
| --- | --- | --- |
| `/crx5ia/joint_targets` | `sensor_msgs/msg/JointState` | One complete arm or both arms, by fixed joint name |
| `/crx5ia/joint_states` | `sensor_msgs/msg/JointState` | Merged measured feedback for both arms |
| `/crx5ia/interpolated_joint_commands` | `sensor_msgs/msg/JointState` | Core interpolated command stream |
| `/crx5ia/left/joint_states`, `/crx5ia/right/joint_states` | `sensor_msgs/msg/JointState` | Individual arm feedback |

Inspect the fixed stream with:

```bash
ros2 topic echo /crx5ia/joint_states --once
ros2 topic hz /crx5ia/joint_targets
ros2 topic hz /crx5ia/interpolated_joint_commands
```

## ROS topics

The canonical launch uses a namespace (default `crx5ia`). The paths below show
the default fully qualified names; remove the `/crx5ia` prefix when referring to
relative names inside a namespaced node.

| Topic | Type | Direction and purpose |
| --- | --- | --- |
| `/crx5ia/left/joint_states` | `sensor_msgs/msg/JointState` | Left driver feedback |
| `/crx5ia/right/joint_states` | `sensor_msgs/msg/JointState` | Right driver feedback |
| `/crx5ia/joint_states` | `sensor_msgs/msg/JointState` | Merged measured feedback |
| `/crx5ia/joint_targets` | `sensor_msgs/msg/JointState` | External or motion target input; one arm or both |
| `/crx5ia/interpolated_joint_commands` | `sensor_msgs/msg/JointState` | 500 Hz interpolated command stream |
| `/crx5ia/left/forward_position_controller/commands` | `std_msgs/msg/Float64MultiArray` | Left controller command |
| `/crx5ia/right/forward_position_controller/commands` | `std_msgs/msg/Float64MultiArray` | Right controller command |

Joint names are fixed as `left_J1`–`left_J6` and `right_J1`–`right_J6`. A normal
command follows:

```text
external publisher / motion script
        -> /crx5ia/joint_targets
        -> interpolation_node
        -> /crx5ia/left|right/forward_position_controller/commands
        -> arm driver
        -> /crx5ia/left|right/joint_states
        -> /crx5ia/joint_states
```

The interpolator publishes at 500 Hz. For linear/cubic modes, `input_rate_hz`
sets the expected input rate and interpolation horizon; it does not throttle the
target publisher. For Ruckig, it sets the reference period only when
`ruckig_target_mode:=stream`; waypoint mode keeps its limit-selected arrival
time. Use `ros2 topic list`, `ros2 topic info <topic>`, and
`ros2 topic echo <topic> --once` to inspect a running system.

## Recording and analysis

By default, trajectory scripts save results under `motion_recordings/<timestamp>/` relative to the current directory. On normal completion or `Ctrl+C`, inspect:

- `joints.csv`: targets, interpolated commands, and feedback for each selected arm, with joint angles in radians.
- `left_joints.png` and `right_joints.png`: six-joint plots for the selected arms.
- Additional `axis_*` or `circle_*` CSV, PNG, and JSON files for Cartesian motion.

For keyboard control or another command source, start the standalone recorder and press `Ctrl+C` when finished to save its plots:

```bash
ros2 run dual_crx_control record_joint_streams.py --ros-args \
  -p output_dir:="$PWD/joint_recordings"
```

The canonical `dual_arm.launch.py` does not start a recorder automatically. All recorders label sources as `target`, `interpolated`, and `feedback`.

To estimate the delay between interpolated commands and feedback, replace the CSV path with an actual recording:

```bash
ros2 run dual_crx_control analyze_latency.py \
  'motion_recordings/<timestamp>/joints.csv' --output latency_report.json
```

CSV timestamps represent local publication or receipt times. The estimated delay includes feedback transport; it is not the robot's execution timestamp or a packet round-trip measurement. TCP traces are forward-kinematics estimates from joint feedback, not external tracking measurements.

## Physical robots

First prepare the FANUC driver configuration, network connections, and controller-side motion access. Then set `mock:=false` and specify the actual robot IP addresses:

```bash
ros2 launch dual_crx_control dual_arm.launch.py \
  mock:=false rviz:=false method:=linear input_rate_hz:=50.0 \
  left_robot_ip:=192.168.2.100 right_robot_ip:=192.168.1.100
```

This starts drivers with motion control enabled and activates position controllers. Before sending trajectories, confirm that the modeled base placements and TCP offsets match the installation.

These settings are in [dual_crx.urdf.xacro](urdf/dual_crx.urdf.xacro). Each TCP currently has a 0.035 m offset along its `ee_mount` X axis. The relative arm placement reflects an existing installation and is not calibrated automatically.

The package does not provide complete link/tool collision checking. Successful mock execution or IK checks do not establish that a physical path is collision-free. `Ctrl+C` and keyboard Space stop new input; they are not hardware emergency stops. Use the installation's hardware emergency stop when an immediate stop is required.

There is also a `dual_arm_readonly.launch.py` using the driver's `motion_control=0` and `initial_controller=none` settings. It currently hard-codes the IP addresses above, physical hardware, and RViz. It does not expose the main launch's `mock` and related arguments; check its settings and driver support before use.

## Troubleshooting

| Symptom | What to check |
| --- | --- |
| `Package 'dual_crx_control' not found` | Confirm the build succeeded and source the workspace's `install/setup.bash` in the current terminal |
| Missing FANUC package or Xacro | Install/build and source both the driver and description packages; mock mode needs them too |
| Waiting for joint states or interpolation | Check that the launch is running, both controllers are active, and terminals use the same `ROS_DOMAIN_ID` |
| Keyboard control will not enable | Use `method:=ruckig`, check for fresh feedback, and press the arm-selection key and `e` separately in an interactive terminal |
| RViz cannot open | Check the graphical display environment, or run mock mode with `rviz:=false` |
| Missing or unloadable `_ruckig` | Check that the native extension built successfully and that the active ROS/Python environment matches the build |
| Recording directory is not writable | Set `--output-dir`, `record_output_dir`, or the recorder's `output_dir` to a writable location |
| Missing `record_wrench.py` or `wrench` option | This version has neither that executable nor that launch argument; use the joint-recording workflow above |

Useful checks:

```bash
ros2 control list_controllers -c /crx5ia/left/controller_manager
ros2 control list_controllers -c /crx5ia/right/controller_manager
ros2 topic echo /crx5ia/left/joint_states --once
ros2 topic echo /crx5ia/right/joint_states --once
```

## Development and advanced tools

The main directories are `launch/` for startup configuration, `config/` for controller and trajectory parameters, `scripts/` for executable entry points, `src/dual_crx_control/` for reusable implementations, and `tools/` for tests and offline utilities.

After building, run package tests from the workspace root:

```bash
colcon test --packages-select dual_crx_control
colcon test-result --verbose
```

Run the following tools from the **package root**, after sourcing ROS and the built workspace:

```bash
# Software-only integration scenarios; no FANUC hardware driver is started.
ROS_DOMAIN_ID=177 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST python3 tools/smoke_motion.py

# Show options for offline facing-circle placement search; no motion is published.
python3 tools/search_facing_circle.py --help
```

[scripts/calibrate_arm_bases.py](scripts/calibrate_arm_bases.py) estimates the right base placement relative to the left from paired TCP measurements. Replace its example measurement points before use. It prints the transform and fit errors without modifying the model.
