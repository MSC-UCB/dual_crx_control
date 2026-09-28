#!/usr/bin/env python3
"""Offline comparison of waypoint, stream, and period-paced quintic references.

Source the built workspace, then run with --output-dir PATH. No ROS nodes,
robot connections, or GUI are created. Results are synthetic, not hardware tests.
"""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import time

import numpy as np

from dual_crx_control.interpolation.quintic import QuinticInterpolation
from dual_crx_control.interpolation.ruckig import (
    RuckigInterpolation, MAX_VELOCITY, MAX_ACCELERATION, MAX_JERK)

DT = .002
MODES = ('waypoint', 'stream', 'quintic')
LIMITS = np.array([MAX_VELOCITY, MAX_ACCELERATION, MAX_JERK])


def make_planner(mode, rate):
    if mode == 'quintic':
        return QuinticInterpolation(DT, input_period=1/rate, max_velocity=MAX_VELOCITY,
                                    max_acceleration=MAX_ACCELERATION, max_jerk=MAX_JERK)
    return RuckigInterpolation(DT, mode=mode, input_period=1/rate, target_timeout=3/rate)


def run(mode, case):
    rate = 50 if case.startswith('sine') or case == 'fast_ramp' else 10
    end = 8. if case.startswith('sine') else 2.
    source_times = np.arange(0., end, 1/rate)
    noisy = case == 'sine_noisy'
    rng = np.random.default_rng(20260928)
    arrivals = source_times + (rng.uniform(0., .008, len(source_times)) if noisy else 0.)
    if case.startswith('sine'):
        target = .1*np.sin(np.pi*source_times)
    else:
        target = 1e-4*(np.arange(len(source_times))+1)
    if noisy:
        target += rng.normal(0., .0005, len(target))
    if case == 'reversal_dropout':
        target = .002*np.sin(2*np.pi*source_times)
        keep = ~((source_times > .7) & (source_times < 1.2))
        arrivals, source_times, target = arrivals[keep], source_times[keep], target[keep]
    planner = make_planner(mode, rate)
    starts = {'left': np.zeros(6), 'right': np.full(6, np.pi)}
    zeros = {s: np.zeros(6) for s in starts}
    traces, planning_times, step_times, durations = [], [], [], []
    cursor = 0
    latest = 0.
    previous_a = np.zeros(12)
    peaks = np.zeros((3, 12))
    for k in range(round((end+2.)/DT)):
        now = k*DT
        while cursor < len(arrivals) and arrivals[cursor] <= now+1e-12:
            latest = target[cursor]
            commands = {s: q+np.array([latest, 0., 0., 0., 0., 0.]) for s, q in starts.items()}
            began = time.perf_counter()
            planner.target(commands, starts, zeros, timestamp=float(arrivals[cursor]))
            planning_times.append(time.perf_counter()-began)
            if mode == 'quintic':
                durations.append(planner.duration)
            cursor += 1
        if not planner.active:
            continue
        began = time.perf_counter()
        output = planner.step(timestamp=now)
        step_times.append(time.perf_counter()-began)
        q = np.concatenate([output[s] for s in starts])
        v = np.concatenate([planner.velocity[s] for s in starts])
        a = np.concatenate([planner.acceleration[s] for s in starts])
        jerk = (a-previous_a)/DT
        peaks = np.maximum(peaks, np.array([abs(v), abs(a), abs(jerk)]))
        previous_a = a
        traces.append([now+DT, latest, q[0], v[0], a[0], jerk[0]])
    trace = np.array(traces)
    moving = trace[(trace[:, 0] >= (2. if case.startswith('sine') else .2)) & (trace[:, 0] < end)]
    reference = (.1*np.sin(np.pi*moving[:, 0]) if case.startswith('sine')
                 else moving[:, 1])
    assert np.isfinite(trace).all()
    assert np.all(peaks <= np.tile(LIMITS, (1, 2)) + 1e-7), (mode, case, peaks)
    assert abs(trace[-1, 2]-target[-1]) < 1e-10
    assert np.max(abs(trace[-1, 3:5])) < 1e-10
    metrics = dict(
        mode=mode, case=case, rate_hz=rate,
        position_rmse_rad=float(np.sqrt(np.mean((moving[:, 2]-reference)**2))),
        acceleration_rms=float(np.sqrt(np.mean(moving[:, 4]**2))),
        jerk_rms=float(np.sqrt(np.mean(moving[:, 5]**2))),
        peak_jerk=float(np.max(abs(trace[:, 5]))),
        rest_fraction=float(np.mean((abs(moving[:, 3]) < 1e-8) & (abs(moving[:, 4]) < 1e-7))),
        target_p99_ms=float(np.percentile(planning_times, 99)*1000),
        step_p99_ms=float(np.percentile(step_times, 99)*1000),
        within_limits=True, final_error_rad=float(abs(trace[-1, 2]-target[-1])),
        extended_targets=getattr(planner, 'extended_count', 0),
        median_horizon_s=float(np.median(durations)) if durations else None,
    )
    return metrics, trace


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', type=Path, required=True)
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    results = []
    for case in ('small_ramp', 'fast_ramp', 'sine_clean', 'sine_noisy', 'reversal_dropout'):
        fig, axes = plt.subplots(4, 1, figsize=(12, 10), sharex=True)
        for mode in MODES:
            metrics, trace = run(mode, case)
            results.append(metrics)
            with (args.output_dir / f'{case}_{mode}.csv').open('w', newline='') as stream:
                writer = csv.writer(stream)
                writer.writerow(['time_s', 'target_rad', 'position_rad', 'velocity_rad_s',
                                 'acceleration_rad_s2', 'jerk_estimate_rad_s3'])
                writer.writerows(trace)
            # Show a short interval for fine structure; the complete traces are in CSV.
            view = trace[(trace[:, 0] >= (2 if case.startswith('sine') else 0)) &
                         (trace[:, 0] <= (4 if case.startswith('sine') else 2.5))]
            for i, ax in enumerate(axes):
                ax.plot(view[:, 0], view[:, 2+i], label=mode, linewidth=1)
            if mode == 'waypoint':
                axes[0].step(view[:, 0], view[:, 1], where='post', label='received target', color='gray', alpha=.6)
        for ax, label in zip(axes, ('Position [rad]', 'Velocity [rad/s]',
                                  'Acceleration [rad/s²]', 'Jerk estimate [rad/s³]')):
            ax.set_ylabel(label)
            ax.grid(True)
        axes[0].legend(loc='upper right')
        axes[-1].set_xlabel('Simulation time [s]')
        fig.suptitle(f'{case}: left J1, synthetic input, 500 Hz output')
        fig.tight_layout()
        fig.savefig(args.output_dir / f'{case}.png', dpi=150)
        plt.close(fig)
    source = Path(__file__).resolve().parents[1] / 'src/dual_crx_control/interpolation'
    metadata = dict(dt=DT, limits=LIMITS.tolist(), seed=20260928,
                    source_sha256={p.name: hashlib.sha256(p.read_bytes()).hexdigest()
                                   for p in (source/'quintic.py', source/'ruckig.py')},
                    note='Synthetic offline reference states; no ROS or robot feedback. '
                         'Wall-clock timings are observations, not realtime guarantees. '
                         'Sine RMSE uses the clean continuous sine; other RMSE uses latest received target.')
    (args.output_dir/'metrics.json').write_text(json.dumps(dict(metadata=metadata, results=results), indent=2)+'\n')
    lines = [
        '# 第三版 quintic 离线评估', '',
        '比较当前 waypoint、stream 和新增的 quintic。未连接机器人；两臂均参与规划，指标和图显示左臂 J1。', '',
        '| 场景 | 模式 | 位置 RMSE (rad) | 加速度 RMS (rad/s²) | jerk RMS (rad/s³) | 静止采样占比 | quintic 中位规划时长 (s) |',
        '|---|---|---:|---:|---:|---:|---:|',
    ]
    for result in results:
        horizon = result['median_horizon_s']
        lines.append(f"| {result['case']} | {result['mode']} | {result['position_rmse_rad']:.6g} | "
                     f"{result['acceleration_rms']:.4g} | {result['jerk_rms']:.4g} | "
                     f"{result['rest_fraction']:.1%} | {horizon if horizon is not None else '—'} |")
    lines += [
        '', '小步进场景：每步 0.0001 rad。正弦场景：幅度 0.1 rad、频率 0.5 Hz、输入 50 Hz。',
        '带噪声场景加入标准差 0.0005 rad 的位置噪声和 0–8 ms 到达延迟；所有模式使用同一输入。',
        '正弦误差相对于干净连续正弦；其余场景相对于最新收到的目标，包含正常插值过程的偏差。',
        'jerk 由相邻规划加速度差分估计。静止采样包括段末恰好停稳的单个采样点，不等同于额外停顿时间。', '',
        '**结论：** 小而可行的低频步进能够铺满周期，明显降低 jerk；高频、大幅连续目标会使时长延长，增加跟踪滞后。',
        '因此它适合优先追求柔和、段末停稳的运动，不能替代低延迟连续跟踪。',
        '15 组场景均通过有限值、速度/加速度/离散 jerk 限幅和最终停稳断言；整条 quintic 曲线另有保守解析限幅检查。',
        '本机运行耗时记录于 metrics.json，不能视为实时调度保证。', '',
    ]
    for case in ('small_ramp', 'fast_ramp', 'sine_clean', 'sine_noisy', 'reversal_dropout'):
        lines += [f'## {case}', '', f'![{case}]({case}.png)', '']
    (args.output_dir/'report.md').write_text('\n'.join(lines)+'\n')
    print(json.dumps(results, indent=2))


if __name__ == '__main__':
    main()
