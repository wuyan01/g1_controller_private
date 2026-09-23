#!/usr/bin/env python3
"""Preview or run one small native ScaleBFM wrist lift/return in the simulator."""
import argparse
import copy
import json
import math
import os
from pathlib import Path
import signal
import sys
import time

import yaml

from scalebfm_wrist_geometry import wrist_poses


def norm(values):
    return math.sqrt(sum(value * value for value in values))


def offset_poses(initial, side, delta, fraction):
    result = copy.deepcopy(initial)
    result[side]['position'] = [p + fraction * d for p, d in
                                zip(initial[side]['position'], delta)]
    return result


def progress(elapsed):
    """One-second hold, 2s outward ramp, 2s hold, 2s return, 1s hold."""
    if elapsed < 1:
        return 'initial_hold', 0.
    if elapsed < 3:
        return 'outward', (elapsed - 1) / 2
    if elapsed < 5:
        return 'target_hold', 1.
    if elapsed < 7:
        return 'return', (7 - elapsed) / 2
    return 'final_hold', 0.


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute', action='store_true', help='Publish the eight-second test; default only previews')
    parser.add_argument('--side', choices=['left', 'right'], default='right')
    parser.add_argument('--dx', type=float, default=0., help='Offset in odom metres')
    parser.add_argument('--dy', type=float, default=0., help='Offset in odom metres')
    parser.add_argument('--dz', type=float, default=.02, help='Offset in odom metres; use 0 for a current-pose hold test')
    parser.add_argument('--controller-node', default='/g1_controller/g1_controller')
    parser.add_argument('--output', type=Path, help='Optional new JSON report path; existing files refused')
    args = parser.parse_args()
    if args.execute and args.output is None:
        args.output = Path('/tmp') / f'scalebfm_wrist_test_{time.time_ns()}_{os.getpid()}.json'
    delta = [args.dx, args.dy, args.dz]
    if not all(math.isfinite(x) for x in delta) or norm(delta) > .050001:
        parser.error('The free-space probe permits a finite offset of at most 5 cm')
    if args.output and args.output.exists():
        parser.error('Output already exists; choose a fresh path')

    # ROS imports stay inside main so geometry/planning checks need no ROS.
    import rclpy
    from rclpy.parameter_client import AsyncParameterClient
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.signals import SignalHandlerOptions
    from ament_index_python.packages import get_package_share_directory
    from geometry_msgs.msg import Pose, PoseArray
    from sensor_msgs.msg import JointState
    from nav_msgs.msg import Odometry
    from std_msgs.msg import Empty

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('scalebfm_wrist_test')
    def interrupted(_signum, _frame):
        raise KeyboardInterrupt('SIGTERM')
    previous_sigterm = signal.signal(signal.SIGTERM, interrupted)
    records, report = [], {'executed': False, 'qualification': 'Exploratory free-space test; command acceptance is not acknowledged by this interface'}
    sent = False
    goal_pub = clear_pub = None
    failure = None
    try:
        client = AsyncParameterClient(node, args.controller_node)
        if not client.wait_for_services(timeout_sec=5.):
            raise RuntimeError('Controller parameters unavailable; launch ScaleBFM first')
        future = client.get_parameters(['config_file', 'simulation', 'network', 'steering_ros_enabled'])
        rclpy.spin_until_future_complete(node, future, timeout_sec=5.)
        if not future.done() or future.result() is None:
            raise RuntimeError('Controller parameter lookup timed out')
        config_param, simulation, network, steering = future.result().values
        if not simulation.bool_value or network.string_value != 'lo':
            raise RuntimeError('This probe requires simulation:=true network:=lo')
        if steering.bool_value:
            raise RuntimeError('Use steering_ros_enabled:=false for the isolated wrist test')
        share = Path(get_package_share_directory('g1_controller'))
        config_path = Path(config_param.string_value)
        if not config_path.is_absolute():
            config_path = share / 'config' / config_path
        config = yaml.safe_load(config_path.read_text())
        controller = config['FSM'].get('ScaleBFM')
        if not isinstance(controller, dict):
            raise RuntimeError('The active configuration is not a ScaleBFM profile')
        enabled = controller.get('enable_wrist_targets', False) is True
        if args.execute and not enabled:
            raise RuntimeError('Restart the controller with enable_wrist_targets:=true before --execute')
        metadata_path = share / controller['policy_dir'] / 'exported' / 'deployment.yaml'
        metadata = yaml.safe_load(metadata_path.read_text())
        latest = {}

        def receive(key, message):
            latest[key] = (message, time.monotonic())

        node.create_subscription(JointState, '/g1_controller/joint_states',
                                 lambda msg: receive('joints', msg), qos_profile_sensor_data)
        node.create_subscription(Odometry, '/g1_controller/sport_mode/odom',
                                 lambda msg: receive('odom', msg), qos_profile_sensor_data)

        def snapshot():
            if set(latest) != {'joints', 'odom'}:
                raise RuntimeError('JointState or native sport odometry is unavailable')
            joints, joint_received = latest['joints']
            odom, odom_received = latest['odom']
            now = node.get_clock().now().nanoseconds * 1e-9
            stamps = [m.header.stamp.sec + m.header.stamp.nanosec * 1e-9 for m in (joints, odom)]
            ages = [now - stamp for stamp in stamps]
            if any(age < -.01 or age > .2 for age in ages) or abs(stamps[0] - stamps[1]) > .05:
                raise RuntimeError('State messages are stale or more than 50 ms apart')
            if time.monotonic() - min(joint_received, odom_received) > .2:
                raise RuntimeError('State reception stopped')
            if odom.header.frame_id != 'odom' or odom.child_frame_id != 'pelvis':
                raise RuntimeError('Expected native odom -> pelvis sport odometry')
            if len(joints.name) != len(joints.position) or len(set(joints.name)) != len(joints.name):
                raise RuntimeError('Malformed joint state')
            p, q = odom.pose.pose.position, odom.pose.pose.orientation
            root, quat = [p.x, p.y, p.z], [q.w, q.x, q.y, q.z]
            velocity = odom.twist.twist.linear
            linear = [velocity.x, velocity.y, velocity.z]
            if not all(math.isfinite(x) for x in root + quat + linear):
                raise RuntimeError('Nonfinite pelvis state')
            tilt = math.acos(max(-1., min(1., 1 - 2 * (q.x*q.x + q.y*q.y))))
            poses = wrist_poses(metadata, dict(zip(joints.name, joints.position)), root, quat)
            return poses, root, tilt, norm(linear[:2]), ages

        deadline = time.monotonic() + 5
        while set(latest) != {'joints', 'odom'} and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.05)
        initial, root_initial, tilt, speed, ages = snapshot()
        report.update(checkpoint_sha256=metadata['checkpoint_sha256'], onnx_sha256=metadata['onnx_sha256'],
                      config_file=str(config_path), wrist_targets_enabled=enabled,
                      mode='2: pelvis + both wrists', frame='odom', side=args.side, offset_m=delta,
                      initial=initial, target=offset_poses(initial, args.side, delta, 1.),
                      pelvis_position=root_initial, tilt_rad=tilt, horizontal_speed_m_s=speed)
        print(json.dumps(report, indent=2), flush=True)
        if not args.execute:
            print('PREVIEW ONLY: no commands published. Enter ScaleBFM, center the sticks, and use --execute for the test.', flush=True)
            return 0
        if root_initial[2] < .6 or tilt > .25 or speed > .10:
            raise RuntimeError('Begin the test upright and stationary, with centered sticks')
        goal_pub = node.create_publisher(PoseArray, '/scalebfm/wrist_targets', 1)
        clear_pub = node.create_publisher(Empty, '/scalebfm/clear_wrist_targets', 1)
        deadline = time.monotonic() + 3
        while (not goal_pub.get_subscription_count() or not clear_pub.get_subscription_count()) and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.02)
        if not goal_pub.get_subscription_count() or not clear_pub.get_subscription_count():
            raise RuntimeError('Wrist command or clear subscriber is unavailable')
        # Resample after discovery so the world-frame targets start at the latest pose.
        initial, root_initial, tilt, speed, ages = snapshot()
        if root_initial[2] < .6 or tilt > .25 or speed > .10:
            raise RuntimeError('Robot moved during setup; settle and retry')
        report.update(initial=initial, target=offset_poses(initial, args.side, delta, 1.), pelvis_position=root_initial)
        started, next_publish, next_log = time.monotonic(), time.monotonic(), 0.
        print('Publishing current-pose hold, small lift, hold, return, then clear. Keep the sticks centered.', flush=True)
        while time.monotonic() - started < 8.:
            rclpy.spin_once(node, timeout_sec=.01)
            now = time.monotonic()
            if now < next_publish:
                continue
            if now - next_publish > .2:
                raise RuntimeError('Probe scheduling stalled; clearing wrist targets')
            elapsed = now - started
            phase, fraction = progress(elapsed)
            measured, root, tilt, speed, ages = snapshot()
            if root[2] < .55 or tilt > .4 or speed > .20 or norm([a-b for a,b in zip(root[:2], root_initial[:2])]) > .10:
                raise RuntimeError('Free-space probe body-motion bound exceeded; clearing wrist targets')
            desired = offset_poses(initial, args.side, delta, fraction)
            message = PoseArray()
            message.header.stamp = node.get_clock().now().to_msg()
            message.header.frame_id = 'odom'
            for side in ('left', 'right'):
                pose = Pose()
                pose.position.x, pose.position.y, pose.position.z = desired[side]['position']
                pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z = desired[side]['quaternion_wxyz']
                message.poses.append(pose)
            # A transport exception can leave delivery uncertain; attempt clear
            # after any publication attempt, including one that raises.
            sent = True
            goal_pub.publish(message)
            report['executed'] = True
            row = {'elapsed_s': elapsed, 'phase': phase, 'pelvis_position': root, 'tilt_rad': tilt, 'horizontal_speed_m_s': speed}
            for side in ('left', 'right'):
                row[side + '_position_error_m'] = norm([a-b for a,b in zip(measured[side]['position'], desired[side]['position'])])
                dot = abs(sum(a*b for a,b in zip(measured[side]['quaternion_wxyz'], desired[side]['quaternion_wxyz'])))
                row[side + '_orientation_error_deg'] = math.degrees(2 * math.acos(min(1., dot)))
                row[side + '_measured_pose'] = measured[side]
                row[side + '_target_pose'] = desired[side]
            records.append(row)
            if elapsed >= next_log:
                print(f"{phase}: left error {1000*row['left_position_error_m']:.1f} mm, right error {1000*row['right_position_error_m']:.1f} mm", flush=True)
                next_log = elapsed + .5
            next_publish = now + .05
        print('Sequence complete. Inspect tracking errors and controller logs; this does not qualify manipulation.', flush=True)
        return 0
    except (Exception, KeyboardInterrupt) as error:
        failure = str(error) or 'Interrupted'
        print('Wrist test stopped: ' + failure, file=sys.stderr, flush=True)
        return 1
    finally:
        cleanup_errors, cleared = [], 0
        try:
            if sent and clear_pub is not None:
                for _ in range(3):
                    try:
                        if not rclpy.ok():
                            raise RuntimeError('ROS context is unavailable for clear')
                        clear_pub.publish(Empty())
                        cleared += 1
                        rclpy.spin_once(node, timeout_sec=.05)
                    except (Exception, KeyboardInterrupt) as error:
                        cleanup_errors.append(str(error) or 'Interrupted during clear')
                if cleared:
                    print('Sent clear_wrist_targets; the controller returns to pelvis navigation if still active.', flush=True)
                else:
                    print('Clear delivery could not be requested; use the manual clear command or X to exit wrist mode.', file=sys.stderr, flush=True)
            report.update(failure=failure, samples=records, clear_requested=sent,
                          clear_messages_published=cleared, clear_acknowledged=False,
                          cleanup_errors=cleanup_errors)
            if args.output:
                try:
                    args.output.parent.mkdir(parents=True, exist_ok=True)
                    with args.output.open('x') as stream:
                        json.dump(report, stream, indent=2, allow_nan=False)
                        stream.write('\n')
                    print('Report: ' + str(args.output), flush=True)
                except (Exception, KeyboardInterrupt) as error:
                    print('Could not save report: ' + str(error), file=sys.stderr, flush=True)
        finally:
            try:
                node.destroy_node()
            finally:
                try:
                    if rclpy.ok():
                        rclpy.shutdown()
                finally:
                    signal.signal(signal.SIGTERM, previous_sigterm)


if __name__ == '__main__':
    raise SystemExit(main())
