# Motion scripts

Source entry points are grouped by purpose:

```text
scripts/
  preplanned_trajectory/
    simple_motion.py
    joint_sine.py
    cartesian_sine.py
    cartesian_circle.py
    facing_circle.py
  util/
    interpolation_node
  analysis/
    record_joint_streams.py
    analyze_latency.py
  calibrate_arm_bases.py
```

CMake installs the nine ROS entry points into `lib/dual_crx_control` using their
existing basenames. These source directories do not change `ros2 run` commands or
launch executable names. Calibration remains a source-only script. Preplanned
trajectories define motion patterns; Cartesian IK is still evaluated at runtime.

Reusable implementation lives under `src/dual_crx_control/`:

| Package | Responsibility |
| --- | --- |
| `robot` | Joint conventions and feedback ordering, robot descriptions, FK/Jacobians and IK |
| `interpolation` | Joint-target clients, interpolation node and interpolation backends |
| `motion` | Motion controllers, startup approaches and circular trajectories |
| `analysis` | Joint/TCP recording, plotting and latency analysis |
| `teleop` | Interactive keyboard control |

Shared imports now use these packages, e.g. `dual_crx_control.robot.joint_config`
and `dual_crx_control.analysis.motion_recording`. `ordered_feedback()` lives in
`robot.joint_config`; reading joint feedback does not require the latency recorder.
The native Ruckig extension remains `dual_crx_control._ruckig`.

Source ROS Jazzy and the workspace in each terminal. Use one motion sender at a time.
Joint prefixes are fixed: `left_J1` through `left_J6`, and `right_J1` through `right_J6`.
The launch argument `input_rate_hz` describes the sender's actual rate; match it to `--rate`.

```bash
ros2 launch dual_crx_control dual_arm.launch.py mock:=true rviz:=false input_rate_hz:=50.0
```

In another terminal, choose one motion:

```bash
ros2 run dual_crx_control joint_sine.py --arms left right --joint 1 --amplitude-deg 1 --period 4 --duration 20 --rate 50
ros2 run dual_crx_control cartesian_sine.py --axis x --amplitude-m 0.02 --period 4 --cycles 2 --rate 50
ros2 run dual_crx_control cartesian_circle.py --plane xy --radius-m 0.02 --direction ccw --period 8 --cycles 1 --rate 50
ros2 run dual_crx_control facing_circle.py --plane xz --radius-m 0.1 --direction cw --period 3 --cycles 10 --rate 50
```

These examples use mock bringup. The facing-circle placement is kinematically checked;
it does not include physical tool/link collision checking.
Each script has an English header with complete examples and a `--help` command.

## Migration

| Old executable | New executable / options |
| --- | --- |
| `test2_j1_periotic.py` | `joint_sine.py --arms left` |
| `dual_test2_periodic.py` | `joint_sine.py --arms right` (or `left`) |
| `dual_test3_sychronized_motion.py` | `joint_sine.py --arms left right` |
| `dual_test3_sychronized_motion_infinite.py` | `joint_sine.py --arms left right --continuous` |
| `dual_test_5_cartesion_sychro_motion.py` | `cartesian_sine.py` |
| `dual_test_6_cartesian_circle_motion.py` | `cartesian_circle.py` |
| `dual_test_7_cartesian_facing_circle_motion.py` | `facing_circle.py` |

`simple_motion.py` remains the two-position joint motion tool.
Old executable names are no longer installed; use a fresh
build/install directory when checking a release so stale scripts cannot mask migration errors.
The Python package `motion` has moved to `dual_crx_control.motion`.

### Joint sine

The merged tool defaults to both arms. Numeric defaults are preserved: J1, 20 deg
amplitude, 4 s period, 10 s sine duration, 1 s initial hold, 1 s amplitude ramp, 50 Hz.
It uses current measured joints as the center, without moving to a home pose.

- `--arms left`, `--arms right`, or `--arms left right` replaces old namespace options.
- `--duration` replaces `--time` (the latter is accepted as an alias).
- `--amplitude-deg` replaces `--amplitude` (also retained as an alias).
- `--continuous` is mutually exclusive with `--duration` and ends on Ctrl+C.
- Finite single-arm motion preserves the direct return to the start and `100/rate`
  seconds of final hold. Use `--return-mode smooth` to select a smooth return instead.
- Finite dual-arm motion preserves the quintic return lasting `max(ramp-time, 1)`
  seconds, followed by 0.2 s hold. `--return-duration` and `--final-hold` are configurable.
- Ctrl+C stops sending targets immediately, without an automatic return. The interpolation
  node continues holding its last accepted target.
- Press ENTER to confirm startup, or explicitly pass `--yes` for automated mock runs.
- `--plot-file` still provides an optional selected-joint response plot;
  `--show-plot` displays figures interactively. Standard per-arm plots are always saved.
- `--latency-csv` retains optional bounded event tracing, including `generated`, `target`,
  `target_publish_return`, interpolated `command`, and `feedback`.

### Cartesian options

The controllers retain their initial approach, IK, joint limits, feedback freshness,
step/velocity checks and circle-conditioning checks. They start when ready, as before.
`--no-move-to-initial` disables the configured initial-joint approach; it does not
disable the facing-pose preparation of a facing circle.

| ROS parameter | CLI option | Units / meaning |
| --- | --- | --- |
| `rate` | `--rate` | Target frequency, Hz |
| `amplitude` | `--amplitude-m` | Sine translation, metres |
| `radius` | `--radius-m` | Circle radius, metres |
| `period` | `--period` | Seconds per cycle |
| `cycles` | `--cycles` | Zero repeats indefinitely |
| `tcp_gap` | `--tcp-gap-m` | Facing gap, metres |
| `center_midpoint` | `--center-midpoint X Y Z` | World coordinates, metres |
| `move_to_initial` | `--move-to-initial` / `--no-move-to-initial` | Initial approach |
| `save_plot` | `--save-plot` / `--no-save-plot` | All recording and plots |
| `output_dir` | `--output-dir` | Recording parent directory |

Other parameter names replace underscores with hyphens, such as `--max-velocity`
and `--initial-move-time`. Existing `--ros-args -p` overrides and parameter files
remain supported; explicit CLI options take precedence for the same parameter.
`config/facing_circle_mock.yaml` now targets the renamed `cartesian_circle` node.

Cartesian sine defaults remain amplitude 0.02 m, period 4 s, axis X, cycles 0.
Circle defaults remain radius 0.02 m, plane XY, CCW, period 8 s, one cycle.
Facing defaults remain radius 0.1 m, plane XZ, CW, period 3 s, ten cycles,
gap 0.02 m, center `[0.55, -0.38, 0.35]`, and max joint velocity 3 rad/s.

## Recording

The default parent directory is `motion_recordings`, relative to the working directory.
Each run creates a timestamped directory containing:

- `joints.csv`: `time_s,arm,source,J1_rad,...,J6_rad`, with sources `target`,
  `interpolated`, and `feedback`. Times are local monotonic publication/receipt times
  relative to recorder startup. Feedback receipt is not the robot's execution time.
- `left_joints.png` and/or `right_joints.png`: six-joint plots for selected arms.
  CSV data remain complete; plots are sampled to approximately 10,000 points per trace
  to bound memory during long runs. Save/close is idempotent.
- For Cartesian motion, additional `axis_*.csv/png/json` or `circle_*.csv/png/json`:
  the existing FK-based TCP displacement/plane analysis, sample counts, truncation flag,
  and controller settings. The TCP analysis retains its bounded recent sample window;
  the complete joint stream remains in `joints.csv`.

TCP analysis begins after startup/pose preparation. Its time origin differs from
`joints.csv`; its JSON describes the timestamp basis. TCP values are FK estimates from
joint states, not measurements from an external tracking system. The controller also
reports achieved command and IK rates at shutdown.

`analyze_latency.py` can read the common `joints.csv` using the `interpolated` stream.
Its phase-delay estimate includes feedback transport and is not a packet round trip.

## Development tools

`tools/search_facing_circle.py` searches offline for well-conditioned facing-circle
placements. It is source-only and is not installed as a ROS executable. From the
repository root, after sourcing ROS and the built workspace:

```bash
python3 tools/search_facing_circle.py --help
python3 tools/search_facing_circle.py --gap 0.2 --output-dir test_results/facing_circle
```

It writes a search report and suggested YAML settings without publishing motion.
`tools/dual_mock_robot.py` is also source-only. The smoke runner starts it with
Python from the tools directory and supplies a generated robot-description parameter file.

## Verification

`colcon test --packages-select dual_crx_control` runs `tools/test_motion_refactor.py`.
After building and sourcing a fresh install, run the software-only ROS scenarios with:

```bash
ROS_DOMAIN_ID=177 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST python3 tools/smoke_motion.py
```

The smoke runner starts `dual_mock_robot.py` and the interpolation executable, exercises
single/dual finite sine, continuous sine interrupted by Ctrl+C, Cartesian sine,
circle and facing circle, then checks CSV sources and saved joint/TCP outputs.
It uses an ideal position mock, small test motions and shortened startup settings;
it does not start a FANUC hardware driver or certify physical motion.
