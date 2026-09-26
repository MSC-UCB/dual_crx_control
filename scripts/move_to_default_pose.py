#!/usr/bin/env python3
"""Move both CRX arms once to the confirmed Sharpa initial pose.

Source ROS Jazzy and ws_fanuc/install/setup.bash first. With no --execute,
only print the target (no ROS node). Stop all other motion senders first.
Example: python3 scripts/move_to_default_pose.py --execute --rate 100
Match --rate to the interpolation node's input_rate_hz.
This is joint interpolation, without collision planning. On exit the downstream
interpolator retains its last target; Ctrl+C is NOT an emergency stop.
"""

import argparse
import math
import time
import xml.etree.ElementTree as ET

import numpy as np


PENDANT_DEGREES = {
    'left': [0., 30., -60., 0., 60., 90.],
    'right': [-90., -30., 240., 0., -60., -90.],
}


def default_targets():
    targets = {}
    for side, angles in PENDANT_DEGREES.items():
        q = np.radians(angles)
        q[2] += q[1]  # Pendant -> ROS; never apply this to ROS feedback.
        targets[side] = q
    return targets


class JointLimits:
    """The joint-limit interface required by InitialJointMove; no FK needed."""

    def __init__(self, description, side):
        root = ET.fromstring(description)
        rows = []
        for i in range(1, 7):
            joint = root.find(f"joint[@name='{side}_J{i}']")
            if joint is None or joint.get('type') != 'revolute':
                raise ValueError(f'Missing revolute {side}_J{i}')
            limit = joint.find('limit')
            if limit is None:
                raise ValueError(f'Missing limits for {side}_J{i}')
            rows.append([float(limit.get(k, 'nan')) for k in ('lower', 'upper', 'velocity')])
        self.lower, self.upper, self.velocity = np.asarray(rows).T
        if (not np.isfinite(rows).all() or np.any(self.lower >= self.upper)
                or np.any(self.velocity <= 0)):
            raise ValueError('Invalid URDF joint limits')

    def valid_joints(self, q):
        q = np.asarray(q)
        return (q.shape == (6,) and np.isfinite(q).all()
                and np.all(q >= self.lower) and np.all(q <= self.upper))


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute', action='store_true', help='Explicitly publish robot motion')
    for name, value in (('rate', 100.), ('minimum-duration', 5.),
                        ('max-speed-deg-s', 10.), ('max-accel-deg-s2', 20.),
                        ('state-timeout', .5), ('startup-timeout', 15.),
                        ('settle-timeout', 15.), ('tolerance-deg', .5), ('hold-time', 1.)):
        parser.add_argument('--' + name, type=float, default=value)
    args = parser.parse_args(argv)
    for name, value in vars(args).items():
        if name != 'execute' and (not math.isfinite(value) or value <= 0):
            parser.error(f'--{name.replace("_", "-")} must be finite and positive')
    if args.rate > 500:
        parser.error('--rate must be <= 500')
    return args


def execute(args):
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String
    from dual_crx_control.interpolation.client import JointTargetClient
    from dual_crx_control.motion.startup_motion import InitialJointMove
    from dual_crx_control.robot.joint_config import (
        SIDES, JOINT_TARGETS_TOPIC, ROBOT_DESCRIPTION_TOPIC, arm_topic, ordered_feedback)

    rclpy.init(args=[])
    node = Node('move_to_default_pose')
    states, models = {}, {}
    description, fault = '', None
    client = JointTargetClient(node)

    def feedback(side, message):
        nonlocal fault
        try:
            q = np.asarray(ordered_feedback(message, side), dtype=float)
            if not np.isfinite(q).all():
                raise ValueError('Nonfinite positions')
            states[side] = (q, time.monotonic())
        except (ValueError, KeyError, TypeError) as exc:
            fault = f'{side}: invalid feedback: {exc}'

    def receive_description(message):
        nonlocal description, fault
        if description and message.data != description:
            fault = 'Robot description changed during execution'
            return
        try:
            models.update({side: JointLimits(message.data, side) for side in SIDES})
            description = message.data
        except (ValueError, ET.ParseError) as exc:
            fault = f'Invalid robot description: {exc}'

    for side in SIDES:
        node.create_subscription(JointState, arm_topic(side, 'joint_states'),
                                 lambda msg, side=side: feedback(side, msg), qos_profile_sensor_data)
    node.create_subscription(String, ROBOT_DESCRIPTION_TOPIC, receive_description,
                             QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    targets = default_targets()
    plan = None
    began = last_tick = settled = None
    deadline = time.monotonic() + args.startup_timeout
    node.get_logger().info('Waiting for description, fresh feedback and interpolation subscriber.')
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=min(.01, 1 / args.rate))
            now = time.monotonic()
            if fault:
                raise RuntimeError(fault)
            if node.count_publishers(JOINT_TARGETS_TOPIC) > 1:
                raise RuntimeError('Another joint-target publisher exists; stop it before running this script')
            fresh = all(s in states and now - states[s][1] <= args.state_timeout for s in SIDES)
            if plan is None:
                if not (description and fresh and client.available()):
                    if now > deadline:
                        raise RuntimeError('Startup timeout waiting for description, feedback or subscriber')
                    continue
                plan = InitialJointMove(models, {s: states[s][0] for s in SIDES}, targets,
                                        args.minimum_duration, math.radians(args.max_speed_deg_s),
                                        math.radians(args.max_accel_deg_s2))
                began = now
                node.get_logger().info(f'Moving to default pose over {plan.duration:.2f} s')
            if not fresh or not client.available():
                raise RuntimeError('Feedback expired or interpolation subscriber disappeared')
            if any(not models[s].valid_joints(states[s][0]) for s in SIDES):
                raise RuntimeError('Measured position outside URDF limits')
            if last_tick is not None and now - last_tick > args.state_timeout:
                raise RuntimeError('Publication loop stalled')
            if last_tick is not None and now - last_tick < 1 / args.rate:
                continue
            elapsed = now - began
            client.publish(plan.sample(elapsed))
            last_tick = now
            if elapsed >= plan.duration:
                reached = all(np.max(np.abs(states[s][0] - targets[s])) <=
                              math.radians(args.tolerance_deg) for s in SIDES)
                settled = (now if settled is None else settled) if reached else None
                if settled is not None and now - settled >= args.hold_time:
                    node.get_logger().info('Both arms reached default pose; finished.')
                    return 0
                if elapsed > plan.duration + args.settle_timeout:
                    raise RuntimeError('Timed out waiting for both arms to settle at target')
        return 1
    except (KeyboardInterrupt, ExternalShutdownException):
        node.get_logger().warning('Interrupted: downstream interpolation retains its last target.')
        return 130
    except (RuntimeError, ValueError) as exc:
        node.get_logger().error(f'{exc}. Stopped sending; downstream retains its last target.')
        return 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main(argv=None):
    args = parse_args(argv)
    for side, target in default_targets().items():
        print(f'{side}: pendant degrees={PENDANT_DEGREES[side]}', flush=True)
        print(f'  ROS degrees={np.degrees(target).round(6).tolist()}, radians={target.tolist()}', flush=True)
    if not args.execute:
        print('Preview only. Add --execute to move; stop other motion senders and check the path first.')
        return 0
    return execute(args)


if __name__ == '__main__':
    raise SystemExit(main())
