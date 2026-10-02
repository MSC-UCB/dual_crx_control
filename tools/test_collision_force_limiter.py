"""Force-stop policy and ROS transport tests; fake managers, no robot connection."""
import math
import time
from types import SimpleNamespace

import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.task import Future
from controller_manager_msgs.msg import ControllerState, HardwareComponentState
from controller_manager_msgs.srv import (
    ListControllers, ListHardwareComponents, SetHardwareComponentState, SwitchController)
from geometry_msgs.msg import WrenchStamped
from lifecycle_msgs.msg import State

from dual_crx_control import collision_force_limiter as limiter


def samples(monitor, now, value=(0., 0., 0.)):
    for side in limiter.SIDES:
        monitor.receive(side, value, now)


def armed():
    monitor = limiter.ForceMonitor(20., 0.)
    samples(monitor, 0.)
    monitor.update(0., True)
    samples(monitor, 3.)
    monitor.update(3., True)
    assert monitor.state == 'ARMED'
    return monitor


@pytest.mark.parametrize('side', limiter.SIDES)
@pytest.mark.parametrize('axis', range(3))
@pytest.mark.parametrize('sign', [-1, 1])
def test_each_axis_stops_both_directions(side, axis, sign):
    monitor = armed()
    force = [0., 0., 0.]
    force[axis] = sign * 20.001
    monitor.receive(side, force, 3.01)
    assert monitor.state == 'TRIPPED'
    assert f'{side}: F{"xyz"[axis]}' in monitor.reason
    reason = monitor.reason
    samples(monitor, 4.)
    monitor.update(4., True)
    assert monitor.state == 'TRIPPED' and monitor.reason == reason


def test_boundary_and_component_rule():
    monitor = armed()
    samples(monitor, 3.1, (20., -20., 20.))
    monitor.update(3.1, True)
    assert monitor.state == 'ARMED'  # resultant > 20 N does not trip


@pytest.mark.parametrize('value', [math.nan, math.inf, -math.inf])
def test_invalid_force_does_not_stop_and_other_arm_still_protects(value):
    monitor = armed()
    monitor.receive('right', (0., value, 0.), 3.01)
    monitor.update(3.01, True)
    assert monitor.state == 'DEGRADED' and not monitor.reason
    monitor.receive('left', (21., 0., 0.), 3.02)
    assert monitor.state == 'TRIPPED' and 'left: Fx=' in monitor.reason


@pytest.mark.parametrize('value', [0., -1., math.nan, math.inf])
def test_invalid_threshold_rejected(value):
    with pytest.raises(ValueError):
        limiter.ForceMonitor(value, 0.)


def test_warmup_ignores_force_but_checks_latest_on_arm():
    monitor = limiter.ForceMonitor(20., 0.)
    samples(monitor, 0., (21., 0., 0.))
    monitor.update(0., False)
    assert monitor.state == 'WAITING'
    monitor.update(0., True)
    samples(monitor, 2.99, (21., 0., 0.))
    monitor.update(2.99, True)
    assert monitor.state == 'WARMUP'
    samples(monitor, 3., (21., 0., 0.))
    monitor.update(3., True)
    assert monitor.state == 'TRIPPED'


def test_warmup_loss_restarts_grace_without_startup_shutdown():
    monitor = limiter.ForceMonitor(20., 0.)
    samples(monitor, 0.)
    monitor.update(0., True)
    monitor.update(.21, True)
    assert monitor.state == 'WAITING'
    samples(monitor, 179.)
    monitor.update(179., True)
    assert monitor.state == 'WARMUP'
    samples(monitor, 180.)
    monitor.update(180., True)
    assert monitor.state == 'WARMUP'
    samples(monitor, 182.)
    monitor.update(182., True)
    assert monitor.state == 'ARMED'


def test_stale_side_recovers_without_stop_or_new_warmup():
    monitor = armed()
    monitor.receive('left', (0., 0., 0.), 3.21)
    monitor.update(3.21, False)
    assert monitor.state == 'DEGRADED' and not monitor.reason
    monitor.receive('right', (0., 0., 0.), 3.22)
    monitor.update(3.22, False)
    assert monitor.state == 'ARMED'


def test_indefinite_missing_feedback_never_requests_stop():
    monitor = limiter.ForceMonitor(20., 0.)
    monitor.update(10000., True)
    assert monitor.state == 'WAITING' and not monitor.reason
    monitor = armed()
    monitor.samples.clear()
    monitor.update(10000., False)
    assert monitor.state == 'DEGRADED' and not monitor.reason
    # The first restored valid sample is checked immediately, even if the
    # other side is still missing, rather than hiding it behind another warmup.
    monitor.receive('right', (0., -21., 0.), 10001.)
    assert monitor.state == 'TRIPPED' and 'right: Fy=' in monitor.reason


class FakeClient:
    def __init__(self, ready=True):
        self.ready = ready
        self.requests = []
        self.future = Future()

    def service_is_ready(self):
        return self.ready

    def call_async(self, request):
        self.requests.append(request)
        return self.future


def hardware_response(ok=True, state=State.PRIMARY_STATE_INACTIVE):
    return SimpleNamespace(ok=ok, state=SimpleNamespace(id=state))


@pytest.mark.parametrize('failure', ['rejected', 'exception', 'timeout', 'unavailable'])
def test_switch_failure_still_attempts_hardware_once(failure):
    switch, hardware = FakeClient(ready=failure != 'unavailable'), FakeClient()
    stop = limiter.ArmStop('left', switch, hardware, 0., lambda _: None)
    stop.tick(0.)
    if failure == 'rejected':
        switch.future.set_result(SimpleNamespace(ok=False))
    if failure == 'exception':
        switch.future.set_exception(RuntimeError('test failure'))
    stop.tick(3.1)
    assert len(hardware.requests) == 1
    request = hardware.requests[0]
    assert request.name == 'crx5ia' and request.target_state.id == 2
    hardware.future.set_result(hardware_response())
    stop.tick(3.2)
    for now in [4., 5., 10.]:
        stop.tick(now)
    assert stop.success and stop.stage == 'done'
    assert len(hardware.requests) == 1
    assert len(switch.requests) == (0 if failure == 'unavailable' else 1)


@pytest.mark.parametrize('response', [hardware_response(False), hardware_response(True, 3)])
def test_hardware_rejection_or_wrong_state_is_not_success(response):
    switch, hardware = FakeClient(), FakeClient()
    stop = limiter.ArmStop('left', switch, hardware, 0., lambda _: None)
    stop.tick(0.)
    switch.future.set_result(SimpleNamespace(ok=True))
    stop.tick(.1)
    hardware.future.set_result(response)
    stop.tick(.2)
    assert stop.stage == 'done' and not stop.success


def test_hardware_timeout_is_bounded_and_does_not_retry():
    switch, hardware = FakeClient(), FakeClient()
    stop = limiter.ArmStop('left', switch, hardware, 0., lambda _: None)
    stop.tick(0.)
    stop.tick(3.)
    stop.tick(6.)
    stop.tick(20.)
    assert stop.stage == 'done' and not stop.success
    assert len(switch.requests) == len(hardware.requests) == 1


def test_unresponsive_left_does_not_delay_right_stop():
    monitor = armed()
    monitor.trip('test bilateral stop')
    node = SimpleNamespace(
        monitor=monitor, stops=None, stop_started=None, stop_finished=False, exit_code=None,
        manager_clients={s: {'switch': FakeClient(), 'hardware': FakeClient()}
                         for s in limiter.SIDES},
        get_logger=lambda: SimpleNamespace(error=lambda _: None, warning=lambda _: None))
    tick = lambda now: limiter.CollisionForceLimiter.process_stop(node, now)
    tick(0.)
    node.manager_clients['right']['switch'].future.set_result(SimpleNamespace(ok=True))
    tick(.01)
    node.manager_clients['right']['hardware'].future.set_result(hardware_response())
    tick(.02)
    assert node.stops['right'].success
    assert node.stops['left'].stage == 'switch' and not node.stop_finished
    tick(3.1)  # Left switch timeout still attempts hardware, without delaying right.
    node.manager_clients['left']['hardware'].future.set_result(hardware_response())
    tick(3.2)
    assert node.stop_finished
    assert node.exit_code is None


def wait(executor, predicate, timeout=8.):
    deadline = time.monotonic() + timeout
    while not predicate() and time.monotonic() < deadline:
        executor.spin_once(timeout_sec=.01)
    assert predicate(), 'ROS fake-manager condition timed out'


@pytest.mark.parametrize('fail_left', [False, True])
def test_ros_transport_readiness_and_bilateral_latched_stop(monkeypatch, fail_left):
    # Isolated domain, no physical driver; speed up only the internal warmup in this test.
    monkeypatch.setattr(limiter, 'STARTUP_GRACE_SEC', .1)
    rclpy.init(domain_id=183)
    manager = Node('fake_collision_managers')
    node = limiter.CollisionForceLimiter()
    executor = SingleThreadedExecutor()
    executor.add_node(manager)
    executor.add_node(node)
    calls = []
    publish = True
    forces = {s: 0. for s in limiter.SIDES}
    hardware_active = {s: False for s in limiter.SIDES}
    publishers = {}

    def controllers(request, response):
        response.controller = [ControllerState(name=name, state='active') for name in
                               ('forward_position_controller', 'force_torque_sensor_broadcaster')]
        return response

    def components(side, request, response):
        response.component = [HardwareComponentState(
            name='crx5ia', state=State(id=3 if hardware_active[side] else 2))]
        return response

    def switch(side, request, response):
        calls.append((side, 'switch'))
        assert request.deactivate_controllers == ['forward_position_controller']
        assert not request.activate_controllers
        response.ok = True
        return response

    def hardware(side, request, response):
        calls.append((side, 'hardware'))
        assert request.name == 'crx5ia' and request.target_state.id == 2
        response.ok = not (fail_left and side == 'left')
        response.state.id = 2 if response.ok else 3
        return response

    def send():
        if publish:
            for side, publisher in publishers.items():
                msg = WrenchStamped()
                msg.wrench.force.z = forces[side]
                publisher.publish(msg)

    from functools import partial
    try:
        for side in limiter.SIDES:
            root = f'/crx5ia/{side}/controller_manager'
            manager.create_service(ListControllers, root + '/list_controllers', controllers)
            manager.create_service(ListHardwareComponents, root + '/list_hardware_components',
                                   partial(components, side))
            manager.create_service(SwitchController, root + '/switch_controller', partial(switch, side))
            manager.create_service(SetHardwareComponentState, root + '/set_hardware_component_state',
                                   partial(hardware, side))
            publishers[side] = manager.create_publisher(
                WrenchStamped, f'/crx5ia/{side}/force_torque_sensor_broadcaster/wrench', 1)
        manager.create_timer(.01, send)
        wait(executor, lambda: all(s in node.monitor.samples for s in limiter.SIDES))
        assert node.monitor.state == 'WAITING' and not calls
        hardware_active.update(left=True, right=True)
        wait(executor, lambda: node.monitor.state == 'ARMED')
        publish = False
        wait(executor, lambda: node.monitor.state == 'DEGRADED')
        assert not calls and node.stops is None and node.exit_code is None
        publish = True
        forces['right'] = math.nan
        wait(executor, lambda: math.isnan(node.monitor.samples['right'][0][2]))
        assert not calls
        forces['right'] = 0.
        wait(executor, lambda: node.monitor.state == 'ARMED')
        forces['right'] = -21.
        wait(executor, lambda: node.stop_finished)
        assert set(calls) == {(s, kind) for s in limiter.SIDES for kind in ('switch', 'hardware')}
        assert all(calls.index((s, 'switch')) < calls.index((s, 'hardware')) for s in limiter.SIDES)
        assert len(calls) == 4
        assert node.stops['right'].success
        assert node.exit_code == (1 if fail_left else None)
        assert node.last_state == ('STOP_FAILED' if fail_left else 'TRIPPED')
        forces['right'] = 0.
        publish = False
        until = time.monotonic() + .3
        while time.monotonic() < until:
            executor.spin_once(timeout_sec=.01)
        assert node.monitor.state == 'TRIPPED' and len(calls) == 4
    finally:
        executor.shutdown()
        node.destroy_node()
        manager.destroy_node()
        rclpy.shutdown()
