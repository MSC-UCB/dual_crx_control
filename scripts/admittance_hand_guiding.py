#!/usr/bin/env python3
"""Dual CRX flange-frame hand guiding. Start the existing control launch first.

ros2 run dual_crx_control admittance_hand_guiding.py

Both arms start Disabled. Stop other joint-target publishers before enabling.
Disable stops new targets; the interpolator may finish its last accepted segment.
"""
import math
import signal
import threading
import time
from functools import partial

import numpy as np
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import WrenchStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from dual_crx_control.interpolation.client import JointTargetClient
from dual_crx_control.robot.ik_solver import DampedLeastSquaresIK
from dual_crx_control.robot.joint_config import (
    JOINT_NAMES, JOINT_TARGETS_TOPIC, ROBOT_DESCRIPTION_TOPIC, SIDES, arm_topic, ordered_feedback)
from dual_crx_control.robot.kinematics import CRXKinematics


# ---- User settings: all axes are [x, y, z, rx, ry, rz], SI units. K is zero. ----
M_DEFAULT = np.array([20., 20., 20., 1., 1., 1.])
D_DEFAULT = np.array([100., 100., 100., 10., 10., 10.])
DEADBAND = np.array([1.5, 1.5, 1.5, .06, .06, .06])
FILTER_HZ = np.full(6, 8.)
VELOCITY_LIMIT = np.array([.1, .1, .1, .5, .5, .5])
SPEED_NORM_LIMIT = (.1, .5)
ACCELERATION_LIMIT = np.array([.25, .25, .25, 1., 1., 1.])
CONTROL_HZ, GUI_HZ = 100., 20.
ENABLE_RAMP_SEC, GAIN_SMOOTHING_SEC = .5, .3
WRENCH_TIMEOUT_SEC, POSE_TIMEOUT_SEC = .1, .1
DT_MIN_SEC, DT_MAX_SEC = .002, .05
EULER_RATIO_LIMIT, MAX_SUBSTEPS = .25, 20
IK_ITERATIONS, IK_BUDGET_SEC = 12, .008
MAX_JOINT_STEP, MAX_JOINT_VELOCITY = .06, 1.0
ENABLE_JOINT_SPEED = .02
FRAME = {side: f'{side}_fanuc_flange' for side in SIDES}
AXES = ('x', 'y', 'z', 'rx', 'ry', 'rz')
LIMITER_TOPIC = '/crx5ia/collision_force_limiter/state'
# No tare, extra gravity compensation, wrench transform or frame selector.


def validate_gains(mass, damping):
    mass, damping = np.asarray(mass, float), np.asarray(damping, float)
    if (mass.shape != (6,) or damping.shape != (6,)
            or not np.isfinite([mass, damping]).all() or np.any(mass <= 0) or np.any(damping <= 0)):
        raise ValueError('M and D must each contain six finite positive values')
    if np.max(damping / mass) > MAX_SUBSTEPS * EULER_RATIO_LIMIT / DT_MAX_SEC:
        raise ValueError('D/M too large for the integration budget; increase M or reduce D')
    return mass.copy(), damping.copy()


def effective_wrench(filtered):
    return np.sign(filtered) * np.maximum(np.abs(filtered) - DEADBAND, 0.)


def se3_exp(increment):
    """SE(3) exponential of [body translation; rotation vector], not Euler angles."""
    rho, phi = increment[:3], increment[3:]
    theta = np.linalg.norm(phi)
    x, y, z = phi
    skew = np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])
    if theta < 1e-4:
        a = .5 - theta**2 / 24. + theta**4 / 720.
        b = 1./6. - theta**2 / 120. + theta**4 / 5040.
    else:
        a = (1. - math.cos(theta)) / theta**2
        b = (theta - math.sin(theta)) / theta**3
    result = np.eye(4)
    result[:3, :3] = Rotation.from_rotvec(phi).as_matrix()
    result[:3, 3] = (np.eye(3) + a*skew + b*(skew @ skew)) @ rho
    return result


def integrate(pose, velocity, wrench, mass, damping, dt, gain=1., axes=None):
    """Bounded semi-implicit Euler, including body-frame pose integration."""
    if not math.isfinite(dt) or not 0 < dt <= DT_MAX_SEC:
        raise ValueError('invalid integration dt')
    count = max(1, math.ceil(dt * float(np.max(damping / mass)) / EULER_RATIO_LIMIT))
    if count > MAX_SUBSTEPS:
        raise ValueError('integration substep budget exceeded')
    h, pose, velocity = dt / count, pose.copy(), velocity.copy()
    mask = np.ones(6, dtype=bool) if axes is None else np.asarray(axes, dtype=bool)
    if mask.shape != (6,):
        raise ValueError('expected six axis selections')
    wrench = np.where(mask, wrench, 0.)
    velocity[~mask] = 0.
    with np.errstate(over='raise', invalid='raise', divide='raise'):
        for _ in range(count):
            acceleration = np.clip((gain*wrench - damping*velocity) / mass,
                                   -ACCELERATION_LIMIT, ACCELERATION_LIMIT)
            goal = np.clip(velocity + acceleration*h, -VELOCITY_LIMIT, VELOCITY_LIMIT)
            for start, cap in zip((0, 3), SPEED_NORM_LIMIT):
                block = slice(start, start+3)
                goal[block] *= min(1., cap / max(np.linalg.norm(goal[block]), 1e-15))
                delta = goal[block] - velocity[block]
                # After norm projection, re-limit acceleration along the feasible segment.
                scale = min(1., float(np.min(ACCELERATION_LIMIT[block]*h /
                                             np.maximum(np.abs(delta), 1e-15))))
                velocity[block] += scale * delta
            velocity[~mask] = 0.
            pose = pose @ se3_exp(velocity*h)
    if not np.isfinite(pose).all() or not np.isfinite(velocity).all():
        raise ValueError('nonfinite integration result')
    return pose, velocity


class ArmAdmittance:
    def __init__(self, side):
        self.side, self.frame = side, FRAME[side]
        self.model = self.solver = None
        self.state, self.error = 'Disabled', ''
        self.wrench_error = self.pose_error_message = 'waiting for data'
        self.received_frame = None
        self.raw = self.filtered = None
        self.wrench_at = self.pose_at = -math.inf
        self.q = self.actual_pose = self.joint_velocity = None
        self.command_pose = self.command_q = None
        self.velocity = np.zeros(6)
        self.axes = np.ones(6, dtype=bool)
        self.mass, self.damping = validate_gains(M_DEFAULT, D_DEFAULT)
        self.target_mass, self.target_damping = self.mass.copy(), self.damping.copy()
        self.last_step = self.enabled_at = None
        self.initial_pending = False

    @property
    def active(self):
        return self.state in ('Enabling', 'Enabled')

    def configure(self, description):
        self.model = CRXKinematics(description, self.frame)
        if self.model.joint_names != JOINT_NAMES[self.side]:
            raise ValueError('unexpected joint order')
        self.solver = DampedLeastSquaresIK(self.model, max_iterations=IK_ITERATIONS)

    def stop(self, reason='', fault=False):
        self.state, self.error = ('Fault' if fault else 'Disabled'), reason
        self.velocity[:] = 0.
        self.command_pose = self.command_q = None
        self.initial_pending = False

    def wrench(self, message, now):
        self.received_frame = message.header.frame_id
        force, torque = message.wrench.force, message.wrench.torque
        raw = np.array([force.x, force.y, force.z, torque.x, torque.y, torque.z])
        error = ('wrench frame must be ' + self.frame if self.received_frame != self.frame else
                 'nonfinite wrench' if not np.isfinite(raw).all() else '')
        if self.active and (error or now - self.wrench_at > WRENCH_TIMEOUT_SEC):
            self.stop(error or 'wrench timeout', fault=True)
        if error:
            self.wrench_error, self.wrench_at = error, -math.inf
            return
        dt = now - self.wrench_at
        if self.filtered is None or not 0 < dt <= WRENCH_TIMEOUT_SEC:
            self.filtered = raw.copy()
        else:
            alpha = -np.expm1(-2.*math.pi*FILTER_HZ*dt)
            self.filtered = (1.-alpha)*self.filtered + alpha*raw
        self.raw, self.wrench_at, self.wrench_error = raw, now, ''

    def feedback(self, message, now):
        if self.model is None:
            return
        try:
            q = np.asarray(ordered_feedback(message, self.side), float)
            if not self.model.valid_joints(q):
                raise ValueError('invalid joints / joint limits')
            pose = self.model.fk(q)
            if len(message.velocity) == len(message.name) and message.velocity:
                values = dict(zip(message.name, message.velocity))
                velocity = np.array([values[n] for n in JOINT_NAMES[self.side]])
            elif self.q is not None and now > self.pose_at:
                velocity = (q-self.q) / (now-self.pose_at)
            else:
                velocity = None
            if velocity is not None and not np.isfinite(velocity).all():
                raise ValueError('invalid joint velocity')
        except (ValueError, KeyError, TypeError, RuntimeError) as exc:
            self.pose_at, self.pose_error_message = -math.inf, str(exc)
            if self.active:
                self.stop(str(exc), fault=True)
            return
        if self.active and now-self.pose_at > POSE_TIMEOUT_SEC:
            self.stop('joint feedback timeout', fault=True)
        self.q, self.actual_pose, self.joint_velocity = q, pose, velocity
        self.pose_at, self.pose_error_message = now, ''

    def readiness(self, now):
        if self.model is None:
            return 'waiting for robot description'
        if self.wrench_error or now-self.wrench_at > WRENCH_TIMEOUT_SEC:
            return self.wrench_error or 'wrench timeout'
        if self.pose_error_message or now-self.pose_at > POSE_TIMEOUT_SEC:
            return self.pose_error_message or 'joint feedback timeout'
        return ''

    def apply_gains(self, mass, damping):
        self.target_mass, self.target_damping = validate_gains(mass, damping)
        if not self.active:
            self.mass, self.damping = self.target_mass.copy(), self.target_damping.copy()

    def select_axes(self, axes):
        mask = np.asarray(axes)
        if mask.shape != (6,) or mask.dtype != np.bool_:
            raise ValueError('expected six boolean axis selections')
        # Start from measured rest on the next explicit Enable, rather than
        # continuing an integrated target on a newly disabled axis.
        self.stop('axis selection changed; wait for rest, then Enable')
        self.axes = mask.copy()

    def enable(self, now):
        if not self.axes.any():
            raise ValueError('select at least one axis before Enable')
        reason = self.readiness(now)
        if reason:
            raise ValueError(reason)
        if self.joint_velocity is None or np.max(np.abs(self.joint_velocity)) > ENABLE_JOINT_SPEED:
            raise ValueError('wait for stationary joint feedback before Enable')
        self.command_pose, self.command_q = self.actual_pose.copy(), self.q.copy()
        self.velocity[:] = 0.
        self.filtered = self.raw.copy()
        self.last_step = self.enabled_at = now
        self.state, self.error, self.initial_pending = 'Enabling', '', True

    def candidate(self, now):
        """Return a proposal; commit it only after final checks and publication."""
        reason = self.readiness(now)
        if reason:
            raise ValueError(reason)
        if self.initial_pending:
            return (self.command_pose.copy(), self.velocity.copy(), self.command_q.copy(),
                    self.mass.copy(), self.damping.copy(), now)
        dt = now-self.last_step
        if not math.isfinite(dt) or dt <= 0 or dt > DT_MAX_SEC:
            raise ValueError(f'control dt out of range: {dt:.4g}s')
        if dt < DT_MIN_SEC:
            return None
        blend = -math.expm1(-dt/GAIN_SMOOTHING_SEC)
        mass = (1.-blend)*self.mass + blend*self.target_mass
        damping = (1.-blend)*self.damping + blend*self.target_damping
        ramp = min(1., (now-self.enabled_at)/ENABLE_RAMP_SEC)
        gain = ramp*ramp*(3.-2.*ramp)
        pose, velocity = integrate(self.command_pose, self.velocity,
                                   effective_wrench(self.filtered), mass, damping, dt, gain,
                                   axes=self.axes)
        started = time.monotonic()
        result = self.solver.solve(pose, self.command_q)
        if time.monotonic()-started > IK_BUDGET_SEC:
            raise ValueError('IK time budget exceeded')
        if not result.success or not self.model.valid_joints(result.q):
            raise ValueError(f'IK rejected: {result.reason}')
        delta = np.abs(result.q-self.command_q)
        if (np.any(delta > MAX_JOINT_STEP)
                or np.any(delta/dt > np.minimum(self.model.velocity, MAX_JOINT_VELOCITY))):
            raise ValueError('joint target step/velocity limit')
        return pose, velocity, result.q.copy(), mass, damping, now

    def commit(self, candidate):
        (self.command_pose, self.velocity, self.command_q,
         self.mass, self.damping, self.last_step) = candidate
        self.initial_pending = False
        if self.last_step-self.enabled_at >= ENABLE_RAMP_SEC:
            self.state = 'Enabled'


class AdmittanceNode(Node):
    def __init__(self, **kwargs):
        super().__init__('admittance_hand_guiding', **kwargs)
        self.arms = {s: ArmAdmittance(s) for s in SIDES}
        self.target_clients = {subset: JointTargetClient(self, arms=subset)
                               for subset in [('left',), ('right',), SIDES]}
        self.lock = threading.Lock()
        self.requests, self.snapshot = {}, {}
        self.generations = {s: 0 for s in SIDES}
        self.shutdown_requested = threading.Event()
        self.done = threading.Event()
        self.worker_error = ''
        self.input_errors = {s: '' for s in SIDES}
        self.limiter_state, self.description, self.model_error = '', '', ''
        self.chain_error, self.next_graph_check = 'waiting for control stack', 0.
        sensor_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        for side, arm in self.arms.items():
            self.create_subscription(JointState, arm_topic(side, 'joint_states'),
                                     lambda msg, a=arm: a.feedback(msg, time.monotonic()), sensor_qos)
            self.create_subscription(WrenchStamped, arm_topic(side, 'force_torque_sensor_broadcaster/wrench'),
                                     partial(self.receive_wrench, side), sensor_qos)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, ROBOT_DESCRIPTION_TOPIC, self.configure, latched)
        self.create_subscription(String, LIMITER_TOPIC, self.limiter, latched)
        self.timer = self.create_timer(1./CONTROL_HZ, self.cycle,
                                      clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.update_snapshot(time.monotonic())

    def receive_wrench(self, side, message):
        arm = self.arms[side]
        if message.header.frame_id != arm.received_frame:
            self.get_logger().info(f'{side}: wrench frame={message.header.frame_id!r}, '
                                   f'expected={arm.frame!r}; FK/IK root=world')
        arm.wrench(message, time.monotonic())

    def configure(self, message):
        if self.model_error or message.data == self.description:
            return
        try:
            if self.description:
                raise ValueError('robot description changed; restart GUI')
            for arm in self.arms.values():
                arm.configure(message.data)
            self.description = message.data
        except Exception as exc:
            self.model_error = f'robot description: {exc}'
            for arm in self.arms.values():
                arm.stop(self.model_error, fault=True)

    def limiter(self, message):
        self.limiter_state = message.data
        if self.limiter_state != 'ARMED':
            for arm in self.arms.values():
                if arm.active:
                    arm.stop(f'collision limiter: {self.limiter_state}', fault=True)

    def request(self, side, action, value=None):
        """GUI thread entry; last request wins and Disable invalidates in-flight work."""
        with self.lock:
            request = self.requests.setdefault(side, {})
            if action == 'axes':
                self.generations[side] += 1
                request['axes'] = value
            elif action in ('enable', 'disable'):
                self.generations[side] += 1
                request['action'] = action
                if action == 'disable':
                    request['reset'] = True
            else:
                request['gains'] = value

    def close(self):
        with self.lock:
            self.shutdown_requested.set()

    def read_snapshot(self):
        with self.lock:
            return self.snapshot.copy()

    def update_snapshot(self, now):
        data = {}
        for side, arm in self.arms.items():
            data[side] = dict(state=arm.state, frame=arm.received_frame or '(no frame)',
                              axes=tuple(bool(v) for v in arm.axes),
                              mass=tuple(arm.mass), damping=tuple(arm.damping),
                              target_mass=tuple(arm.target_mass), target_damping=tuple(arm.target_damping),
                              gain_error=self.input_errors[side],
                              filtered=None if arm.filtered is None else tuple(arm.filtered),
                              effective=None if arm.filtered is None else tuple(effective_wrench(arm.filtered)),
                              velocity=tuple(arm.velocity), wrench_age=now-arm.wrench_at,
                              pose_age=now-arm.pose_at,
                              error=arm.error or arm.readiness(now) or
                              self.model_error or self.chain_error or
                              ('' if self.limiter_state == 'ARMED' else 'waiting for collision limiter ARMED'))
        with self.lock:
            self.snapshot = data

    def cycle(self):
        now = time.monotonic()
        with self.lock:
            requests, self.requests = self.requests, {}
            generations = self.generations.copy()
        if self.shutdown_requested.is_set():
            return
        if now >= self.next_graph_check:
            self.next_graph_check = now+.5
            self.chain_error = (
                'missing interpolation subscriber' if not self.target_clients[SIDES].available() else
                'collision limiter unavailable' if self.count_publishers(LIMITER_TOPIC) == 0 else
                'another joint-target publisher is active' if self.count_publishers(JOINT_TARGETS_TOPIC) > 3 else '')
        proposals = {}
        for side, arm in self.arms.items():
            request = requests.get(side, {})
            if 'gains' in request:
                try:
                    arm.apply_gains(*request['gains'])
                    self.input_errors[side] = ''
                    self.get_logger().info(
                        f'{side}: accepted M={arm.target_mass.tolist()}, '
                        f'D={arm.target_damping.tolist()}')
                except (ValueError, TypeError) as exc:
                    self.input_errors[side] = str(exc)
                    self.get_logger().warning(f'{side}: M/D rejected: {exc}')
            action = request.get('action')
            if action == 'disable' or request.get('reset'):
                arm.stop()
            if 'axes' in request:
                try:
                    arm.select_axes(request['axes'])
                    selected = ' '.join(a for a, use in zip(AXES, arm.axes) if use) or '(none)'
                    self.get_logger().info(f'{side}: selected flange axes: {selected}; arm Disabled')
                except ValueError as exc:
                    arm.stop(str(exc), fault=True)
                # A simultaneous Enable must not bypass the selection stop.
                action = None
            blocked = self.model_error or self.chain_error or (
                '' if self.limiter_state == 'ARMED' else f'collision limiter: {self.limiter_state or "waiting"}')
            try:
                if blocked and (arm.active or action == 'enable'):
                    raise ValueError(blocked)
                if action == 'enable' and not arm.active:
                    arm.enable(now)
                if arm.active:
                    candidate = arm.candidate(now)
                    if candidate is not None:
                        proposals[side] = candidate
            except (ValueError, RuntimeError, FloatingPointError, np.linalg.LinAlgError) as exc:
                arm.stop(str(exc), fault=True)
                self.get_logger().warning(f'{side}: {exc}')
        # GUI Disable/close can arrive during IK. Serialize the final check and publish.
        with self.lock:
            if self.shutdown_requested.is_set():
                proposals.clear()
            for side in tuple(proposals):
                arm = self.arms[side]
                reason = arm.readiness(time.monotonic())
                if self.generations[side] != generations[side]:
                    arm.stop()
                    del proposals[side]
                elif reason:
                    arm.stop(reason, fault=True)
                    del proposals[side]
            if proposals:
                subset = tuple(s for s in SIDES if s in proposals)
                self.target_clients[subset].publish({s: c[2] for s, c in proposals.items()})
                for side, candidate in proposals.items():
                    self.arms[side].commit(candidate)
        self.update_snapshot(time.monotonic())


class AdmittanceWindow:
    def __init__(self, node):
        import tkinter as tk
        from tkinter import ttk
        self.node, self.tk = node, tk
        self.root = tk.Tk()
        self.root.title('Dual CRX — flange admittance')
        self.closing = False
        self.fields, self.labels, self.buttons = {}, {}, {}
        self.axis_variables, self.axis_buttons = {}, {}
        self.gain_input_errors = {side: '' for side in SIDES}
        ttk.Label(self.root, text='Stop other arm command scripts. K = 0. Disable holds the last accepted target.').grid(
            row=0, column=0, columnspan=2, padx=12, pady=8)
        for col, side in enumerate(SIDES):
            panel = ttk.LabelFrame(self.root, text=side.upper(), padding=10)
            panel.grid(row=1, column=col, sticky='nsew', padx=8, pady=8)
            labels = {key: tk.StringVar() for key in
                      ('state', 'frame', 'age', 'error', 'values', 'gains', 'gain_error')}
            self.labels[side] = labels
            ttk.Label(panel, textvariable=labels['state']).grid(row=0, column=0, columnspan=4)
            enable = ttk.Button(panel, text='Enable', command=lambda s=side: node.request(s, 'enable'))
            enable.grid(row=1, column=0)
            disable = ttk.Button(panel, text='Disable', command=lambda s=side: node.request(s, 'disable'))
            disable.grid(row=1, column=1)
            self.buttons[side] = (enable, disable)
            for index, title in enumerate(('Flange axis', 'Enable axis', 'M', 'D')):
                ttk.Label(panel, text=title).grid(row=2, column=index)
            mass, damping = [], []
            self.axis_variables[side], self.axis_buttons[side] = [], []
            for row, axis in enumerate(AXES, start=3):
                ttk.Label(panel, text=axis).grid(row=row, column=0)
                selected = tk.BooleanVar(value=True)
                self.axis_variables[side].append(selected)
                checkbox = ttk.Checkbutton(panel, variable=selected,
                                           command=lambda s=side: self.select_axes(s))
                checkbox.grid(row=row, column=1)
                self.axis_buttons[side].append(checkbox)
                for column, defaults, output in [(2, M_DEFAULT, mass), (3, D_DEFAULT, damping)]:
                    variable = tk.StringVar(value=f'{defaults[row-3]:g}')
                    ttk.Entry(panel, textvariable=variable, width=12).grid(row=row, column=column)
                    output.append(variable)
            self.fields[side] = (mass, damping)
            ttk.Button(panel, text='Apply M / D', command=lambda s=side: self.apply(s)).grid(
                row=9, column=0, columnspan=4, pady=8)
            ttk.Label(panel, justify='left', text=(
                'Edit, then click Apply M / D for this arm.\n'
                'x/y/z: M [kg], D [N s/m]; rx/ry/rz: M [kg m²], D [Nm s/rad].\n'
                'M/D must stay positive. Checkboxes select local flange axes.\n'
                'Changing axes disables this arm; wait for rest, then Enable.')).grid(
                    row=10, column=0, columnspan=4, sticky='w', pady=4)
            for row, key in enumerate(('gains', 'gain_error', 'frame', 'age', 'values', 'error'), start=11):
                ttk.Label(panel, textvariable=labels[key], justify='left', wraplength=520).grid(
                    row=row, column=0, columnspan=4, sticky='w', pady=4)
        self.root.protocol('WM_DELETE_WINDOW', self.close)
        self.refresh()

    def select_axes(self, side):
        self.node.request(side, 'axes', [v.get() for v in self.axis_variables[side]])

    def apply(self, side):
        try:
            values = [[float(v.get()) for v in group] for group in self.fields[side]]
            validate_gains(*values)
            self.gain_input_errors[side] = ''
            self.node.request(side, 'gains', values)
        except ValueError as exc:
            # Keep the exact error visible independently of arm readiness/faults.
            # Reject the edit without sending a dummy NaN request to the controller.
            self.gain_input_errors[side] = f'M/D rejected: {exc}'
            self.labels[side]['gain_error'].set(self.gain_input_errors[side])

    def refresh(self):
        if self.closing:
            if self.node.done.is_set():
                self.root.destroy()
                return
        else:
            for side, data in self.node.read_snapshot().items():
                labels = self.labels[side]
                labels['state'].set(data['state'])
                labels['frame'].set(data['frame'])
                labels['age'].set(f'Wrench age: {data["wrench_age"]:.3f}s | Pose age: {data["pose_age"]:.3f}s')
                def row(values):
                    return 'waiting' if values is None else ' '.join(f'{v: .3f}' for v in values)
                self.labels[side]['gains'].set(
                    f'Accepted M: {row(data["target_mass"])}\n'
                    f'Accepted D: {row(data["target_damping"])}\n'
                    f'In use M:   {row(data["mass"])}\n'
                    f'In use D:   {row(data["damping"])}')
                labels['gain_error'].set(self.gain_input_errors[side] or data['gain_error'])
                selected = ' '.join(a for a, use in zip(AXES, data['axes']) if use) or '(none)'
                labels['values'].set(f'Selected flange axes: {selected}\n'
                                     'Axes: x y z rx ry rz\n'
                                     f'Filtered [N, Nm]: {row(data["filtered"])}\n'
                                     f'Effective [N, Nm]: {row(data["effective"])}\n'
                                     f'Velocity [m/s, rad/s]: {row(data["velocity"])}')
                labels['error'].set(data['error'])
            if self.node.done.is_set():
                self.close()
        self.root.after(round(1000./GUI_HZ), self.refresh)

    def close(self):
        self.closing = True
        self.node.close()
        for pair in self.buttons.values():
            for button in pair:
                button.state(['disabled'])
        for checkboxes in self.axis_buttons.values():
            for checkbox in checkboxes:
                checkbox.state(['disabled'])
        self.root.title('Dual CRX — shutting down')


def run_ros(node):
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        while rclpy.ok() and not node.shutdown_requested.is_set():
            executor.spin_once(timeout_sec=.02)
    except ExternalShutdownException:
        pass
    except Exception as exc:
        node.worker_error = str(exc)
        node.get_logger().error(f'Admittance stopped: {exc}')
    finally:
        node.close()
        for arm in node.arms.values():
            arm.stop(node.worker_error or 'shutdown', fault=bool(node.worker_error))
        node.update_snapshot(time.monotonic())
        node.timer.cancel()
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()
        node.done.set()


def main():
    rclpy.init()
    node = AdmittanceNode()
    try:
        window = AdmittanceWindow(node)
    except Exception:
        node.destroy_node()
        rclpy.try_shutdown()
        raise
    worker = threading.Thread(target=run_ros, args=(node,), name='admittance-ros')
    previous = signal.signal(signal.SIGINT, lambda *_: window.close())
    worker.start()
    try:
        window.root.mainloop()
    finally:
        node.close()
        worker.join(timeout=2.)
        signal.signal(signal.SIGINT, previous)
    return 1 if node.worker_error else 0


if __name__ == '__main__':
    raise SystemExit(main())
