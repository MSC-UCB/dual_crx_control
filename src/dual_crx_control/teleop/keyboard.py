"""Small, event-driven Cartesian jogger using the shared Ruckig interpolator."""

from functools import partial
import os
import select
import sys
import termios
import time
import tty
from xml.parsers.expat import ExpatError

import numpy as np
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter_client import AsyncParameterClient
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from dual_crx_control.interpolation.client import JointTargetClient
from dual_crx_control.robot.ik_solver import DampedLeastSquaresIK
from dual_crx_control.robot.joint_config import JOINT_NAMES, SIDES, ordered_feedback
from dual_crx_control.robot.kinematics import CRXKinematics


RATE = 50.
MOVES = {'w': (0, 1), 's': (0, -1), 'a': (1, 1),
         'd': (1, -1), 'r': (2, 1), 'f': (2, -1)}
DEFAULTS = dict(step_m=.001, state_timeout=.25,
                max_joint_step=.03, max_joint_velocity=.5, tracking_tolerance=.1)


def batch_key(data):
    """Safety/control keys take priority; never queue repeated movement keys."""
    if 'q' in data or '\x03' in data:
        return 'q'
    if ' ' in data:
        return ' '
    # Discard escape sequences (arrows etc.), rather than treating their bytes as keys.
    if '\x1b' in data:
        return ''
    for group in ('12', 'e', ''.join(MOVES)):
        keys = [key for key in data if key in group]
        if keys:
            return keys[-1]
    return ''


class Terminal:
    def __enter__(self):
        if not sys.stdin.isatty():
            raise ValueError('Keyboard control requires an interactive TTY terminal')
        self.fd = sys.stdin.fileno()
        self.saved = termios.tcgetattr(self.fd)
        try:
            tty.setcbreak(self.fd)
            termios.tcflush(self.fd, termios.TCIFLUSH)
        except BaseException:
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.saved)
            raise
        return self

    def read(self):
        if not select.select([self.fd], [], [], 0)[0]:
            return ''
        data = os.read(self.fd, 4096)
        termios.tcflush(self.fd, termios.TCIFLUSH)
        return batch_key(data.decode('ascii', errors='ignore')) if data else 'q'

    def __exit__(self, *exc):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.saved)


class JogTarget:
    """Commit only validated IK endpoints; rejection leaves the target unchanged."""
    def __init__(self, model, solver, q, settings):
        self.model, self.solver, self.settings = model, solver, settings
        self.q = q.copy()
        self.pose = model.fk(q)

    def candidate(self, key, feedback):
        p = self.settings
        target = self.pose.copy()
        axis, sign = MOVES[key]
        target[axis, 3] += sign * p['step_m']
        result = self.solver.solve(target, self.q)
        if not result.success or not self.model.valid_joints(result.q):
            raise ValueError(f'IK rejected: {result.reason}')
        delta = np.abs(result.q - self.q)
        if np.any(delta > p['max_joint_step']):
            raise ValueError('joint step limit')
        if np.any(delta * RATE > np.minimum(self.model.velocity, p['max_joint_velocity'])):
            raise ValueError('joint target velocity limit')
        if np.max(np.abs(result.q - feedback)) > p['tracking_tolerance']:
            raise ValueError('candidate exceeds tracking tolerance')
        return target, result.q.copy()


class KeyboardControl(Node):
    def __init__(self, read_key, **kwargs):
        super().__init__('keyboard_control', **kwargs)
        self.settings = {k: self.declare_parameter(k, v).value for k, v in DEFAULTS.items()}
        if any(isinstance(v, bool) or not np.isfinite(v) or v <= 0 for v in self.settings.values()):
            raise ValueError('All jog limits must be finite and positive')
        if self.settings['step_m'] > .005:
            raise ValueError('step_m must not exceed 0.005')
        self.read_key = read_key
        self.side, self.jog, self.finished = 'left', None, False
        self.models, self.solvers, self.positions, self.received = {}, {}, {}, {}
        self.description, self.model_fault = '', False
        self.target_clients = {s: JointTargetClient(self, arms=(s,)) for s in SIDES}
        self.method_client = AsyncParameterClient(self, '/joint_interpolation')
        self.method_future = None
        self.method_checked = -float('inf')
        self.ruckig_ready = False
        for side in SIDES:
            self.create_subscription(JointState, f'{side}/joint_states',
                                     partial(self.feedback, side), qos_profile_sensor_data)
        self.create_subscription(String, '/robot_description', self.configure_message,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        description = self.declare_parameter('robot_description', '').value
        if description:
            self.configure(description)
        clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(1., self.check_method, clock=clock)
        self.create_timer(1. / RATE, self.cycle, clock=clock)
        self.get_logger().info('DISARMED left | 1/2 select, e enable, w/s X, a/d Y, r/f Z, Space disarm, q exit')
        self.get_logger().warning('One command source only. Disarm/exit is NOT an emergency stop.')

    def check_method(self):
        if self.method_future is not None and not self.method_future.done():
            return
        if self.method_client.services_are_ready():
            self.method_future = self.method_client.get_parameters(['method'])
            self.method_future.add_done_callback(self.method_result)

    def method_result(self, future):
        try:
            self.ruckig_ready = future.result().values[0].string_value == 'ruckig'
        except Exception:
            self.ruckig_ready = False
        self.method_checked = time.monotonic()

    def configure_message(self, message):
        try:
            self.configure(message.data)
        except (ValueError, KeyError, RuntimeError, SyntaxError, ExpatError) as exc:
            self.model_fault = True
            self.disarm(f'invalid robot_description: {exc}; restart required')

    def configure(self, description):
        if self.description:
            if description != self.description:
                self.model_fault = True
                self.disarm('robot_description changed; restart required')
            return
        models = {s: CRXKinematics(description, f'{s}_tcp') for s in SIDES}
        if any(models[s].joint_names != JOINT_NAMES[s] for s in SIDES):
            raise ValueError('Expected left/right J1..J6 joint order')
        self.models = models
        self.solvers = {s: DampedLeastSquaresIK(m) for s, m in models.items()}
        self.description = description

    def feedback(self, side, message):
        if side not in self.models:
            return
        try:
            q = np.array(ordered_feedback(message, side), dtype=float)
            if not self.models[side].valid_joints(q):
                raise ValueError('invalid joint positions')
        except (ValueError, KeyError, TypeError):
            self.received.pop(side, None)
            if side == self.side and self.jog is not None:
                self.disarm('invalid feedback')
            return
        self.positions[side], self.received[side] = q, time.monotonic()

    def readiness_error(self):
        now = time.monotonic()
        if self.model_fault or not self.description:
            return 'robot description unavailable/invalid'
        if not self.ruckig_ready or now - self.method_checked > 2.5:
            return 'waiting for /joint_interpolation with method=ruckig'
        if not self.target_clients[self.side].available():
            return 'missing interpolation subscriber'
        if now - self.received.get(self.side, -float('inf')) > self.settings['state_timeout']:
            return 'stale/missing feedback'
        if self.jog is not None and np.max(np.abs(self.jog.q - self.positions[self.side])) > self.settings['tracking_tolerance']:
            return 'tracking error exceeded tolerance'
        return None

    def disarm(self, reason):
        self.jog = None
        self.get_logger().info(f'DISARMED {self.side}: {reason}')

    def cycle(self):
        key = self.read_key()
        if key == 'q':
            self.disarm('exit; last accepted interpolation may still finish')
            self.finished = True
            return
        if key in ('1', '2', ' '):
            if key != ' ':
                self.side = 'left' if key == '1' else 'right'
            self.disarm('press e to enable; last accepted interpolation may still finish')
            return
        error = self.readiness_error()
        if error:
            if self.jog is not None or key == 'e':
                self.disarm(error)
            return
        try:
            if key == 'e':
                if self.jog is None:
                    self.jog = JogTarget(self.models[self.side], self.solvers[self.side],
                                         self.positions[self.side], self.settings)
                    self.get_logger().info(f'ENABLED {self.side}: TCP world XYZ {self.jog.pose[:3, 3].tolist()}')
                return
            if key not in MOVES or self.jog is None:
                return
            pose, q = self.jog.candidate(key, self.positions[self.side])
            # IK may take time; do not send if feedback became stale during the solve.
            error = self.readiness_error()
            if error:
                self.disarm(error)
                return
            self.target_clients[self.side].publish({self.side: q})
            self.jog.pose, self.jog.q = pose, q
            self.get_logger().info(f'{self.side} target XYZ {pose[:3, 3].round(5).tolist()}', throttle_duration_sec=.5)
        except (ValueError, RuntimeError, np.linalg.LinAlgError) as exc:
            self.get_logger().warning(f'Target rejected: {exc}', throttle_duration_sec=1.)


def main():
    node = None
    try:
        with Terminal() as terminal:
            rclpy.init()
            try:
                node = KeyboardControl(terminal.read)
                while rclpy.ok() and not node.finished:
                    rclpy.spin_once(node, timeout_sec=.02)
            finally:
                if node is not None:
                    node.destroy_node()
                if rclpy.ok():
                    rclpy.shutdown()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except (ValueError, OSError) as exc:
        raise SystemExit(str(exc)) from exc
