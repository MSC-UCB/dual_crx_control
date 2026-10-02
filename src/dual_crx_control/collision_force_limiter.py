"""Latched dual-arm force stop using the existing controller-manager services.

This is a software interlock, not a hard real-time or hardware emergency stop.
Only the force threshold is configurable; timing constants stay internal.
"""
from functools import partial
import math
import time

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rcl_interfaces.msg import ParameterDescriptor
from controller_manager_msgs.srv import (
    ListControllers, ListHardwareComponents, SetHardwareComponentState, SwitchController)
from geometry_msgs.msg import WrenchStamped
from lifecycle_msgs.msg import State
from std_msgs.msg import String


SIDES = ('left', 'right')
STARTUP_GRACE_SEC = 3.0
WRENCH_TIMEOUT_SEC = 0.2
SERVICE_TIMEOUT_SEC = 3.0
STOP_TIMEOUT_SEC = 8.0
READINESS_POLL_SEC = 0.5
READINESS_MAX_AGE_SEC = 1.5


class ForceMonitor:
    """Clock-driven policy independent of ROS transport, with no automatic reset."""

    def __init__(self, threshold, now):
        if not math.isfinite(threshold) or threshold <= 0:
            raise ValueError('collision_force_threshold_n must be finite and positive')
        self.threshold = threshold
        self.created_at = now
        self.state = 'WAITING'
        self.samples = {}
        self.warmup_at = None
        self.reason = ''

    def trip(self, reason):
        if self.state != 'TRIPPED':
            self.state, self.reason = 'TRIPPED', reason

    def receive(self, side, force, now):
        self.samples[side] = (tuple(force), now)
        if self.state in ('ARMED', 'DEGRADED'):
            self._check(side)

    def _check(self, side):
        force, _ = self.samples[side]
        if not all(math.isfinite(v) for v in force):
            return
        for axis, value in zip('xyz', force):
            if abs(value) > self.threshold:
                self.trip(f'{side}: F{axis}={value:.6g} N exceeds '
                          f'|F{axis}| > {self.threshold:.6g} N')
                return

    def fresh(self, side, now):
        return (side in self.samples
                and now - self.samples[side][1] <= WRENCH_TIMEOUT_SEC
                and all(math.isfinite(v) for v in self.samples[side][0]))

    def update(self, now, ready):
        if self.state == 'TRIPPED':
            return
        if self.state in ('ARMED', 'DEGRADED'):
            # Data loss is diagnostic only. Keep checking every valid incoming
            # sample, including the other arm, and recover without a new warmup.
            self.state = 'ARMED' if all(self.fresh(s, now) for s in SIDES) else 'DEGRADED'
            return
        if not ready or not all(self.fresh(side, now) for side in SIDES):
            self.state, self.warmup_at = 'WAITING', None
            return
        if self.warmup_at is None:
            self.state, self.warmup_at = 'WARMUP', now
        if now - self.warmup_at >= STARTUP_GRACE_SEC:
            self.state = 'ARMED'
            for side in SIDES:
                self._check(side)


class ArmStop:
    """One bounded asynchronous stop sequence; never reissue a stage request."""

    def __init__(self, side, switch, hardware, now, log):
        self.side, self.switch, self.hardware, self.log = side, switch, hardware, log
        self.stage = 'switch'
        self.deadline = now + SERVICE_TIMEOUT_SEC
        self.future = None
        self.success = False

    def _advance(self, now, response=None, error=''):
        if self.stage == 'switch':
            if error or response is None or not response.ok:
                self.log(f'{self.side}: controller stop unconfirmed ({error or "rejected"}); '
                         'requesting hardware inactive anyway')
            self.stage, self.future = 'hardware', None
            self.deadline = now + SERVICE_TIMEOUT_SEC
        else:
            self.success = (not error and response is not None and response.ok
                            and response.state.id == State.PRIMARY_STATE_INACTIVE)
            if not self.success:
                self.log(f'{self.side}: hardware stop FAILED ({error or "inactive not confirmed"})')
            self.stage = 'done'

    def tick(self, now):
        if self.stage == 'done':
            return
        if self.future is not None and self.future.done():
            try:
                self._advance(now, response=self.future.result())
            except Exception as exc:
                self._advance(now, error=str(exc))
        elif now >= self.deadline:
            # A timeout does not cancel the server-side operation. Do not retry it.
            self._advance(now, error='service timeout')
        if self.stage == 'done' or self.future is not None:
            return
        client = self.switch if self.stage == 'switch' else self.hardware
        if not client.service_is_ready():
            return
        if self.stage == 'switch':
            request = SwitchController.Request()
            request.deactivate_controllers = ['forward_position_controller']
            request.strictness = SwitchController.Request.BEST_EFFORT
            request.timeout.sec = int(SERVICE_TIMEOUT_SEC)
        else:
            request = SetHardwareComponentState.Request()
            request.name = 'crx5ia'
            request.target_state.id = State.PRIMARY_STATE_INACTIVE
        try:
            self.future = client.call_async(request)
        except Exception as exc:
            self._advance(now, error=str(exc))


class CollisionForceLimiter(Node):
    def __init__(self, **kwargs):
        super().__init__('collision_force_limiter', **kwargs)
        threshold = self.declare_parameter(
            'collision_force_threshold_n', 20.0,
            ParameterDescriptor(read_only=True)).value
        now = time.monotonic()
        self.monitor = ForceMonitor(threshold, now)
        self.stops = None
        self.stop_started = None
        self.stop_finished = False
        self.exit_code = None
        self.last_state = None
        self.manager_clients, self.probes, self.ready_results = {}, {}, {}
        self.next_probe = now
        self.status = self.create_publisher(
            String, '/crx5ia/collision_force_limiter/state',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        for side in SIDES:
            root = f'/crx5ia/{side}/controller_manager'
            self.manager_clients[side] = {
                'switch': self.create_client(SwitchController, root + '/switch_controller'),
                'hardware': self.create_client(
                    SetHardwareComponentState, root + '/set_hardware_component_state'),
                'controllers': self.create_client(ListControllers, root + '/list_controllers'),
                'components': self.create_client(
                    ListHardwareComponents, root + '/list_hardware_components'),
            }
            self.create_subscription(
                WrenchStamped, f'/crx5ia/{side}/force_torque_sensor_broadcaster/wrench',
                partial(self.receive, side),
                QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.timer = self.create_timer(
            0.02, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.get_logger().info(
            f'Collision force limit: |Fx/Fy/Fz| > {threshold:g} N on either arm stops BOTH arms; '
            f'{STARTUP_GRACE_SEC:g}s warmup after readiness. Wait for ARMED before motion.')
        self.publish_state()

    def readiness(self, now):
        for key, (future, sent_at) in list(self.probes.items()):
            if future.done():
                try:
                    response = future.result()
                    if key[1] == 'controllers':
                        active = {c.name for c in response.controller if c.state == 'active'}
                        ready = {'forward_position_controller',
                                 'force_torque_sensor_broadcaster'}.issubset(active)
                    else:
                        ready = any(c.name == 'crx5ia' and c.state.id == State.PRIMARY_STATE_ACTIVE
                                    for c in response.component)
                    self.ready_results[key] = (ready, now)
                except Exception:
                    self.ready_results[key] = (False, now)
                del self.probes[key]
            elif now - sent_at >= SERVICE_TIMEOUT_SEC:
                self.manager_clients[key[0]][key[1]].remove_pending_request(future)
                del self.probes[key]
                self.ready_results[key] = (False, now)
        if now >= self.next_probe:
            self.next_probe = now + READINESS_POLL_SEC
            for side in SIDES:
                for kind, service in [('controllers', ListControllers),
                                      ('components', ListHardwareComponents)]:
                    key, client = (side, kind), self.manager_clients[side][kind]
                    if key not in self.probes and client.service_is_ready():
                        self.probes[key] = (client.call_async(service.Request()), now)
        return (all(self.manager_clients[s][k].service_is_ready()
                    for s in SIDES for k in ('switch', 'hardware'))
                and all(self.ready_results.get((s, k), (False, 0))[0]
                        and now - self.ready_results[(s, k)][1] <= READINESS_MAX_AGE_SEC
                        for s in SIDES for k in ('controllers', 'components')))

    def receive(self, side, message):
        f = message.wrench.force
        now = time.monotonic()
        self.monitor.receive(side, (f.x, f.y, f.z), now)
        self.process_stop(now)
        self.publish_state()

    def process_stop(self, now):
        if self.monitor.state != 'TRIPPED' or self.stop_finished:
            return
        if self.stops is None:
            self.stop_started = now
            self.get_logger().error(f'TRIPPED at monotonic={now:.6f}: {self.monitor.reason}')
            self.stops = {
                s: ArmStop(s, self.manager_clients[s]['switch'], self.manager_clients[s]['hardware'],
                           now, self.get_logger().error) for s in SIDES}
        for stop in self.stops.values():
            stop.tick(now)
        if (all(stop.stage == 'done' for stop in self.stops.values())
                or now - self.stop_started >= STOP_TIMEOUT_SEC):
            self.stop_finished = True
            failed = [s for s, stop in self.stops.items() if not stop.success]
            if failed:
                self.get_logger().error(
                    f'STOP_FAILED: inactive not confirmed for {failed}; requesting launch shutdown')
                self.exit_code = 1
            else:
                self.get_logger().warning(
                    f'Both hardware components confirmed inactive after {now-self.stop_started:.3f}s; '
                    'TRIPPED is latched. Stop external senders before restarting launch. '
                    'This acknowledgment does not measure physical stopping time.')

    def publish_state(self):
        state = ('STOP_FAILED' if self.stop_finished and self.stops
                 and not all(s.success for s in self.stops.values()) else self.monitor.state)
        if state != self.last_state:
            self.status.publish(String(data=state))
            self.get_logger().info(f'Collision force limiter: {state}')
            self.last_state = state

    def tick(self):
        now = time.monotonic()
        ready = self.readiness(now) if self.monitor.state in ('WAITING', 'WARMUP') else False
        self.monitor.update(now, ready)
        if self.monitor.state != 'TRIPPED':
            missing = [s for s in SIDES if not self.monitor.fresh(s, now)]
            if missing:
                self.get_logger().warning(
                    f'Force feedback missing/stale/invalid for {missing}; no stop requested. '
                    'Force threshold cannot be checked for these samples; waiting for valid data.',
                    throttle_duration_sec=5.0)
        self.process_stop(now)
        self.publish_state()


def main():
    rclpy.init()
    node = None
    code = 0
    try:
        node = CollisionForceLimiter()
        while rclpy.ok() and node.exit_code is None:
            rclpy.spin_once(node, timeout_sec=0.1)
        code = node.exit_code or 0
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return code
