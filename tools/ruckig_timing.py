#!/usr/bin/env python3
"""Deterministic, hardware-free timing baseline for Issue #2.

Run after sourcing ROS and the built workspace. No ROS nodes are started.
"""
import argparse
import ast
from collections import deque
import csv
from dataclasses import asdict, dataclass, replace
import hashlib
import itertools
import json
from pathlib import Path
import subprocess

import numpy as np

from dual_crx_control import _ruckig
from dual_crx_control.interpolation.ruckig import MAX_VELOCITY
from dual_crx_control.interpolation.trajectory import JointSegment, validate_rate

ROOT = Path(__file__).resolve().parents[1]
DT = .002
POSITION_TOL = 1e-10
VELOCITY_TOL = 1e-8


@dataclass(frozen=True)
class Case:
    rate: float = 10.
    step: float = 1e-4
    joint: int = 1
    arms: str = 'both'
    method: str = 'ruckig'
    pattern: str = 'ramp'
    timing: str = 'fixed'
    count: int = 12
    tail: float = 1.

    def __post_init__(self):
        validate_rate(self.rate)
        if not np.isfinite([self.step, self.tail]).all() or self.step <= 0 or self.tail <= 0:
            raise ValueError('step and tail must be finite and positive')
        if self.joint not in range(1, 7) or self.count < 3:
            raise ValueError('joint must be 1..6 and count must be at least 3')
        for value, allowed in ((self.arms, ('left', 'right', 'both')),
                               (self.method, ('ruckig', 'linear', 'cubic')),
                               (self.pattern, ('ramp', 'alternating', 'sine')),
                               (self.timing, ('fixed', 'jitter', 'dropout', 'pause'))):
            if value not in allowed:
                raise ValueError(f'unsupported case value: {value}')

    @property
    def sides(self):
        return ('left', 'right') if self.arms == 'both' else (self.arms,)

    @property
    def key(self):
        return (f'{self.method}_{self.rate:g}Hz_{self.step:g}_J{self.joint}_'
                f'{self.arms}_{self.pattern}_{self.timing}')


def schedule(case):
    """Absolute targets; pause resumes after five periods; final tail always stops input."""
    elapsed = 0.
    events = []
    for k in range(case.count):
        if k:
            elapsed += (1. + (.1 * (-1)**k if case.timing == 'jitter' else 0.)) / case.rate
        if case.timing == 'pause' and k == case.count // 2:
            elapsed += 5. / case.rate
        if case.timing == 'dropout' and k == case.count // 2:
            continue
        value = case.step * (k + 1)
        if case.pattern == 'alternating':
            value = case.step * (-1)**k
        elif case.pattern == 'sine':
            value = case.step * np.sin(2 * np.pi * (k + 1) / case.count)
        events.append((k, elapsed, float(value)))
    return events


def git(*args):
    return subprocess.check_output(['git', '-C', str(ROOT), *args], text=True)


def limits(profile):
    """Read committed/worktree constants without editing or importing historical code."""
    path = 'src/dual_crx_control/interpolation/ruckig.py'
    source = git('show', f'HEAD:{path}') if profile == 'head' else (ROOT / path).read_text()
    values = {}
    for node in ast.parse(source).body:
        if isinstance(node, ast.Assign) and isinstance(node.targets[0], ast.Name):
            name = node.targets[0].id
            if name in ('MAX_ACCELERATION', 'MAX_JERK'):
                values[name] = ast.literal_eval(node.value)
    return tuple(MAX_VELOCITY), tuple(values['MAX_ACCELERATION']), tuple(values['MAX_JERK'])


def provenance(profile, bounds):
    binary = Path(_ruckig.__file__).resolve()
    return dict(revision=git('rev-parse', 'HEAD').strip(), profile=profile,
                comparison='same current native binding, HEAD/worktree planning constants',
                wrapper_diff=(git('diff', 'HEAD', '--', 'src/dual_crx_control/interpolation/ruckig.py')
                              if profile == 'worktree' else ''),
                native_binding_diff=git('diff', 'HEAD', '--', 'src/ruckig_binding.cpp'),
                binding_path=str(binary), binding_sha256=hashlib.sha256(binary.read_bytes()).hexdigest(),
                max_velocity=bounds[0], max_acceleration=bounds[1], max_jerk=bounds[2],
                dt=DT, position_tolerance=POSITION_TOL, velocity_tolerance=VELOCITY_TOL)


def derivatives(t, q):
    """Backward differences located at interval midpoints, including nonuniform ROS timing."""
    result = []
    for _ in range(3):
        q = np.diff(q) / np.diff(t)
        t = (t[1:] + t[:-1]) / 2
        result.append((t, q))
    return result


def summarize(t, q, targets, initial=0.):
    """Times share one clock. A crossing is not a settle; settle must persist to next input.

    targets are (sequence, observed arrival, absolute position). The last target's
    hold is reported separately and never inflates the inter-target plateau metric.
    """
    t, q = np.asarray(t), np.asarray(q)
    if len(t) < 4 or len(t) != len(q) or not np.isfinite([t, q]).all() or np.any(np.diff(t) <= 0):
        raise ValueError('need at least four finite, strictly increasing samples')
    estimates = derivatives(t, q)
    velocity = np.r_[0., estimates[0][1]]
    rows = []
    for i, (sequence, arrival, target) in enumerate(targets):
        end = targets[i+1][1] if i+1 < len(targets) else t[-1]
        indices = np.flatnonzero((t >= arrival - 1e-12) & (t < end - 1e-12))
        before = np.flatnonzero(t < arrival - 1e-12)
        start = q[before[-1]] if len(before) else initial
        row = dict(sequence=sequence, arrival_s=arrival, target_rad=target,
                   first_arrival_s=None, settle_s=None, hold_s=0., overshoot_rad=0.,
                   superseded=i+1 < len(targets))
        if len(indices):
            close = np.abs(q[indices] - target) <= POSITION_TOL
            hit = np.flatnonzero(close)
            if len(hit):
                row['first_arrival_s'] = float(t[indices[hit[0]]] - arrival)
            rest = close & (np.abs(velocity[indices]) <= VELOCITY_TOL)
            if rest[-1]:
                moving = np.flatnonzero(~rest)
                settled = moving[-1]+1 if len(moving) else 0
                row['settle_s'] = float(t[indices[settled]] - arrival)
                row['hold_s'] = max(0., float(end - t[indices[settled]]))
            low, high = sorted((start, target))
            row['overshoot_rad'] = float(max(0., np.max(q[indices]-high), np.max(low-q[indices])))
        rows.append(row)
    inter = rows[:-1]
    return dict(targets=rows, observed_output_hz=float((len(t)-1)/(t[-1]-t[0])),
                observed_segment_duration_s=[r['first_arrival_s'] for r in rows],
                settle_duration_s=[r['settle_s'] for r in rows],
                mean_inter_target_hold_s=float(np.mean([r['hold_s'] for r in inter])) if inter else 0.,
                max_inter_target_hold_s=max((r['hold_s'] for r in inter), default=0.),
                peak_velocity_estimate=float(np.max(np.abs(estimates[0][1]))),
                peak_acceleration_estimate=float(np.max(np.abs(estimates[1][1]))),
                peak_jerk_estimate=float(np.max(np.abs(estimates[2][1]))),
                overshoot_rad=max((r['overshoot_rad'] for r in rows), default=0.),
                final_error_rad=float(abs(q[-1]-targets[-1][2])))


def simulate(case, bounds):
    generator = _ruckig.Generator(DT, *bounds) if case.method == 'ruckig' else None
    events = schedule(case)
    q = np.zeros(12)
    # Exercise subtraction around real nonzero absolute right-arm angles as well.
    q[6:] = [-np.pi/2, 0., np.pi, 0., np.pi/2, 0.]
    initial = q.copy()
    axes = [case.joint - 1 + (6 if s == 'right' else 0) for s in case.sides]
    traces = [[0., *q[axes], *np.zeros(2*len(axes))]]
    accepted, histories, segments = [], {s: deque(maxlen=5) for s in case.sides}, {}
    cursor = 0
    previous_a = np.zeros(12)
    peaks = np.zeros((3, 12))
    end = events[-1][1] + case.tail
    for tick in range(int(np.ceil(end / DT))):
        now = tick * DT
        while cursor < len(events) and events[cursor][1] <= now + 1e-12:
            sequence, scheduled, displacement = events[cursor]
            accepted.append((sequence, now, displacement))
            for side in case.sides:
                arm = int(side == 'right')
                sl = slice(arm*6, arm*6+6)
                target = initial[sl].copy()
                target[case.joint-1] += displacement
                if generator:
                    generator.target(arm, target, initial[sl], np.zeros(6),
                                     np.zeros(6), np.zeros(6))
                else:
                    segments[side] = JointSegment(now, now+1/case.rate, q[sl].copy(), target,
                                                  method=case.method, history=list(histories[side]))
            cursor += 1
        if generator:
            q, v, a = (np.asarray(x) for x in generator.step())
            peaks = np.maximum(peaks, [abs(v), abs(a), abs(a-previous_a)/DT])
            previous_a = a.copy()
        else:
            for side, segment in segments.items():
                arm = int(side == 'right')
                sl = slice(arm*6, arm*6+6)
                q[sl] = segment.sample(now+DT)
                histories[side].append((now+DT, q[sl].copy()))
            v = a = np.full(12, np.nan)
        traces.append([now+DT, *q[axes], *v[axes], *a[axes]])
    trace = np.asarray(traces)
    metrics = {}
    for index, side in enumerate(case.sides):
        targets = [(seq, t, initial[axes[index]]+d) for seq, t, d in accepted]
        metrics[side] = summarize(trace[:, 0], trace[:, index+1], targets, initial[axes[index]])
    native = None
    if generator:
        native = dict(peak_velocity=peaks[0].tolist(), peak_acceleration=peaks[1].tolist(),
                      peak_jerk=peaks[2].tolist(),
                      within_limits=bool(np.all(peaks <= np.tile(np.array(bounds), (1, 2)) + 1e-7)))
    result = dict(case=asdict(case), target_period_s=1/case.rate,
                  intentionally_dropped_target_count=case.count-len(events),
                  dropped_target_count=0, timing_quantization_bound_s=DT,
                  native=native, metrics=metrics)
    return result, trace


def matrix():
    """Rate/step/joint/arm grid plus pattern/timing grid (not a full Cartesian product)."""
    cases = []
    for rate, step, joint, arms, method in itertools.product(
            (5., 10., 20., 50.), (1e-5, 1e-4, 1e-3, 1e-2), (1, 4, 5),
            ('left', 'right', 'both'), ('ruckig', 'linear', 'cubic')):
        cases.append(Case(rate=rate, step=step, joint=joint, arms=arms, method=method))
    for pattern, timing, joint, arms, method in itertools.product(
            ('ramp', 'alternating', 'sine'), ('jitter', 'dropout', 'pause'),
            (1, 4, 5), ('left', 'right', 'both'), ('ruckig', 'linear', 'cubic')):
        cases.append(Case(pattern=pattern, timing=timing, joint=joint, arms=arms, method=method))
    return cases


def write_results(output, metadata, results):
    output.mkdir(parents=True, exist_ok=True)
    (output / 'metrics.json').write_text(json.dumps(dict(metadata=metadata, results=results),
                                                   indent=2, allow_nan=False)+'\n')
    rows = []
    for result in results:
        for side, metrics in result.get('metrics', {}).items():
            rows.append(dict(**result['case'], side=side,
                             **{k: v for k, v in metrics.items() if not isinstance(v, list)}))
    if rows:
        with (output / 'metrics.csv').open('w') as stream:
            writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=Path('test_results/issue2'))
    parser.add_argument('--profile', choices=('head', 'worktree'), default='worktree')
    parser.add_argument('--matrix', action='store_true')
    parser.add_argument('--traces', action='store_true', help='also save each matrix trace')
    for field, kind in (('rate', float), ('step', float), ('joint', int), ('arms', str),
                        ('method', str), ('pattern', str), ('timing', str), ('count', int), ('tail', float)):
        parser.add_argument('--'+field, type=kind, default=getattr(Case(), field))
    args = parser.parse_args()
    case = Case(**{k: getattr(args, k) for k in asdict(Case())})
    bounds = limits(args.profile)
    output = args.output / args.profile
    output.mkdir(parents=True, exist_ok=True)
    metadata = provenance(args.profile, bounds)
    metadata['matrix'] = 'rate/step/joint/arms + pattern/timing/joint/arms' if args.matrix else 'single case'
    results = []
    for index, current in enumerate(matrix() if args.matrix else [case]):
        try:
            result, trace = simulate(current, bounds)
            if args.traces or not args.matrix:
                header = ['time_s'] + [f'{field}_{s}' for field in ('q', 'v_native', 'a_native') for s in current.sides]
                np.savetxt(output / (current.key+'.csv'), trace, delimiter=',',
                           header=','.join(header), comments='')
            results.append(result)
        except (RuntimeError, ValueError) as exc:
            results.append(dict(case=asdict(current), error=str(exc)))
        if (index+1) % 100 == 0:
            print(f'{args.profile}: {index+1} cases', flush=True)
    write_results(output, metadata, results)
    errors = sum('error' in r for r in results)
    print(f'{output}: {len(results)} cases, {errors} solver/measurement errors', flush=True)
    if errors:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
