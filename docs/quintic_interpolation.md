# Period-paced quintic interpolation

Select this third alternative to Ruckig `waypoint` and `stream` with
`method:=quintic`. The existing modes and launch defaults are preserved.

```bash
source /opt/ros/jazzy/setup.bash
source /home/msc-crx/ws_fanuc/install/setup.bash
ros2 launch dual_crx_control dual_arm.launch.py \
  mock:=true rviz:=false method:=quintic input_rate_hz:=10.0
```

The expected input period is `T = 1/input_rate_hz`; output remains 500 Hz.
This does not change the sender's rate. For a stationary initial state, the
position profile is

```text
u = elapsed / T
q(u) = q0 + (q1 - q0) * (10*u^3 - 15*u^4 + 6*u^5)
```

Velocity and acceleration are zero at both endpoints. Acceleration acts throughout
the first half and deceleration throughout the second half, with no inserted
constant-speed plateau. Acceleration is not constant. The general trajectory
matches the current planned position, velocity, and acceleration and ends at the
new target with zero velocity and acceleration. Mid-motion replans need not have
a symmetric accelerate/decelerate shape; preserving continuity takes precedence.
Jerk is bounded, but may jump at segment boundaries; this is C2, not C3 continuity.

## Timing and limit behavior

- A new target is planned from the last emitted reference state, not from zero
  velocity or delayed feedback. Feedback seeds each arm once, including its
  measured velocity when available. Targets for both active arms share a horizon.
- The nominal horizon starts when a target is accepted; the next actual arrival
  time is unknown. Early targets replace the unfinished trajectory continuously;
  late targets may leave a hold interval. This does not buffer future targets.
- The planner checks velocity, acceleration, and jerk over the **entire curve**.
  Derivatives are converted to Bernstein control points on eight subintervals;
  their convex hull gives a sufficient bound, including between output samples.
- If the nominal horizon fails these bounds, it grows by approximately 20% and
  is rounded up to a 2 ms output tick, until all active joints pass. This is a
  conservative feasible duration, not the shortest possible quintic duration.
- Limits are shared with Ruckig's `MAX_VELOCITY`, `MAX_ACCELERATION`, and
  `MAX_JERK`. These acceleration/jerk values remain experimental planning limits.
- Repeated identical targets do not restart the horizon. With no further target,
  the current trajectory finishes and holds with exactly zero velocity and
  acceleration. `ruckig_target_timeout` does not apply to this method.
- Invalid/unplannable targets are rejected before replacing the previous plan.
  During replanning a reversing trajectory can overshoot a target; derivative
  bounds do not enforce joint position limits or collision clearance.
- Like the existing Ruckig wrapper, planning time advances one 2 ms step per
  timer callback. Delayed callbacks lengthen wall-clock execution. Polynomial
  derivative bounds describe reference time, not a guarantee about ROS scheduling
  or physical feedback derivatives.

For a rest-to-rest move of distance `d`, the exact single-joint peaks are:

```text
peak speed        = 1.875 * abs(d) / T
peak acceleration = (10 / sqrt(3)) * abs(d) / T^2
peak jerk         = 60 * abs(d) / T^3
```

For J1 with jerk limit 30 rad/s³ and a 0.0001 rad step, a 100 ms horizon is
feasible (peak jerk 6 rad/s³). A 20 ms horizon would require 750 rad/s³, so it
must be extended. The jerk-only minimum is about 58.5 ms; the implemented
duration search selects a conservative tick-aligned horizon. Continuously
retargeting a zero-terminal-velocity plan can increase tracking lag substantially.

Setting only Ruckig's `minimum_duration` does not give the same profile: a
0.0001 rad / 100 ms J1 experiment still used maximum jerk at the edges and
a long constant-speed middle section. The quintic method deliberately spreads
the change across the horizon instead.

## Reproduce the offline evaluation

From the workspace root, after building and sourcing it:

```bash
OPENBLAS_NUM_THREADS=1 python3 -m pytest -q \
  src/dual_crx_control/tools/test_quintic.py \
  src/dual_crx_control/tools/test_ruckig_timing.py

OPENBLAS_NUM_THREADS=1 python3 src/dual_crx_control/tools/evaluate_quintic.py \
  --output-dir interpolation_evaluation/quintic
```

The evaluation creates `report.md`, `metrics.json`, 15 full CSV traces and five
plots comparing waypoint, stream, and quintic. Cases cover small 10 Hz steps,
50 Hz steps that cannot finish within one period, clean/noisy sine targets,
reversals and input dropout. Both arms are active; plots show left J1. Noise
uses a fixed seed, 0.0005 rad position standard deviation, and 0–8 ms arrival
delay. Source hashes and all limits are recorded in `metrics.json`.

The tests separately exercise all six joints, nonzero initial velocities, partial
arm activation, identical targets, invalid input, continuous retargeting and
whole-curve derivative bounds. The existing software-mock launch test includes
the new method; run it explicitly with `ISSUE2_ROS_MOCK=1`.

These are reference-generator and software-mock checks, not physical robot tests.
Use the existing joint-stream recorder to compare measured robot feedback before
choosing this mode for a live teleoperation session.
