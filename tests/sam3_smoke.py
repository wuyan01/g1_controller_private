#!/usr/bin/env python3
"""Real GPU smoke test: live MuJoCo -> SAM 3 -> RViz, without a controller.

Source ROS, then run: python3 tests/sam3_smoke.py --checkpoint /opt/g1-models/sam3.pt
Defaults to /opt/g1-sam3-venv/bin/python; test uses ROS domain 89 and Unitree domain 90.
"""
import argparse
import importlib.util
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

os.environ['ROS_DOMAIN_ID'] = '89'
ROOT = Path(__file__).resolve().parents[1]


def rviz_only():
    from launch import LaunchDescription, LaunchService
    from launch.actions import OpaqueFunction, SetLaunchConfiguration
    spec = importlib.util.spec_from_file_location('controller_launch', ROOT / 'launch/g1_controller_sim.launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    service = LaunchService()
    service.include_launch_description(LaunchDescription([
        SetLaunchConfiguration('sam3', 'true'), OpaqueFunction(function=module.launch_rviz)]))
    return service.run()


def main():
    if '--rviz-only' in sys.argv:
        return rviz_only()
    parser = argparse.ArgumentParser()
    parser.add_argument('--checkpoint', required=True)
    parser.add_argument('--scene', default='scene_29dof.xml')
    parser.add_argument('--prompt', default='floor')
    # This test does not run the controller/joint publisher. Camera-to-torso TF
    # is static and available; timestamped pelvis transforms have a separate test.
    parser.add_argument('--target-frame', default='torso_link')
    parser.add_argument('--python', default='/opt/g1-sam3-venv/bin/python')
    args = parser.parse_args()
    import numpy as np
    from PIL import Image as PILImage
    import rclpy
    from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import Image
    from std_msgs.msg import String
    from vision_msgs.msg import Detection2DArray, Detection3DArray
    from cv_bridge import CvBridge
    from ament_index_python.packages import get_package_prefix
    binary = Path(get_package_prefix('g1_controller')) / 'lib/g1_controller/g1_mujoco_camera'
    commands = [
        ['ros2', 'launch', 'g1_description', 'g1_description.launch.py'],
        ['ros2', 'launch', 'g1_controller', 'g1_camera.launch.py', 'simulation:=true'],
        [str(binary), '-r', 'g1', '-s', args.scene, '-i', '90', '-n', 'lo'],
        ['ros2', 'launch', 'g1_controller', 'g1_sam3.launch.py',
         f'sam3_python:={args.python}', f'sam3_checkpoint:={args.checkpoint}',
         f'sam3_prompt:={args.prompt}', 'sam3_rate:=2.0',
         f'sam3_target_frame:={args.target_frame}'],
        [sys.executable, str(Path(__file__).resolve()), '--rviz-only'],
    ]
    rclpy.init()
    observer = rclpy.create_node('sam3_smoke')
    bridge = CvBridge()
    cache = {key: {} for key in ('rgb', 'overlay', 'labels', 'detections', 'positions')}
    def receive(msg, key):
        stamp = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        cache[key][stamp] = msg
        if len(cache[key]) > 300:
            cache[key].pop(next(iter(cache[key])))
    subscriptions = [observer.create_subscription(kind, topic,
        lambda msg, key=key: receive(msg, key),
        QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)
        if key in ('overlay', 'labels', 'detections') else qos_profile_sensor_data)
        for kind, topic, key in [
            (Image, '/camera/color/image_raw', 'rgb'),
            (Image, '/sam3/overlay', 'overlay'),
            (Image, '/sam3/instance_labels', 'labels'),
            (Detection2DArray, '/sam3/detections', 'detections'),
            (Detection3DArray, '/sam3/detections_3d', 'positions')]]
    localization_status = []
    subscriptions.append(observer.create_subscription(String, '/sam3/localization_status',
        lambda msg: localization_status.append(json.loads(msg.data)), 1))
    prompt_pub = observer.create_publisher(String, '/sam3/prompt', 1)
    children, logs = [], []
    try:
        for index, command in enumerate(commands):
            log = open(f'/tmp/g1-sam3-smoke-{index}.log', 'w')
            logs.append(log)
            children.append(subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                              start_new_session=True))
        def wait_frame(expect_detection, after=(0, 0)):
            deadline = time.monotonic() + 90
            while time.monotonic() < deadline:
                rclpy.spin_once(observer, timeout_sec=0.1)
                assert all(child.poll() is None for child in children), 'Child exited; inspect /tmp/g1-sam3-smoke-*.log'
                common = set.intersection(*(set(messages) for messages in cache.values()))
                for stamp in sorted(common, reverse=True):
                    if (stamp > after and
                            bool(cache['detections'][stamp].detections) == expect_detection and
                            bool(cache['positions'][stamp].detections) == expect_detection):
                        return stamp
            raise AssertionError(f'Timed out waiting for synchronized SAM 3 outputs: {localization_status[-3:]}')
        stamp = wait_frame(True)
        for key, messages in cache.items():
            assert messages[stamp].header.frame_id == (
                args.target_frame if key == 'positions' else 'camera_color_optical_frame')
        labels = bridge.imgmsg_to_cv2(cache['labels'][stamp])
        overlay = bridge.imgmsg_to_cv2(cache['overlay'][stamp], 'rgb8')
        rgb = bridge.imgmsg_to_cv2(cache['rgb'][stamp], 'rgb8')
        assert labels.shape == rgb.shape[:2] and labels.any()
        assert overlay.shape == rgb.shape and np.any(overlay != rgb)
        detections = cache['detections'][stamp].detections
        assert all(d.results[0].hypothesis.class_id == args.prompt for d in detections)
        assert set(np.unique(labels)) - {0} <= {int(d.id) for d in detections}
        positions = cache['positions'][stamp].detections
        assert positions, 'No valid depth/TF localization for the detected object'
        assert {d.id for d in positions} <= {d.id for d in detections}
        xyz = [[d.results[0].pose.pose.position.x, d.results[0].pose.pose.position.y,
                d.results[0].pose.pose.position.z] for d in positions]
        assert np.isfinite(xyz).all()
        print('PASS: live masked depth positions in', args.target_frame, xyz, flush=True)
        endpoints = observer.get_subscriptions_info_by_topic('/sam3/overlay')
        assert any(item.node_name.startswith('rviz') and
                   item.qos_profile.reliability == ReliabilityPolicy.BEST_EFFORT
                   for item in endpoints), 'RViz overlay subscription must accept best-effort images'
        PILImage.fromarray(overlay).save('/tmp/g1-sam3-live-overlay.png')
        print('PASS: real GPU inference on live MuJoCo, matching image headers, labels, '
              'Detection2DArray, and automatic RViz overlay; scores:',
              [d.results[0].hypothesis.score for d in detections], flush=True)
        prompt_pub.publish(String(data=''))
        empty_stamp = wait_frame(False, after=stamp)
        assert not bridge.imgmsg_to_cv2(cache['labels'][empty_stamp]).any()
        assert not cache['positions'][empty_stamp].detections
        np.testing.assert_array_equal(bridge.imgmsg_to_cv2(cache['overlay'][empty_stamp], 'rgb8'),
                                      bridge.imgmsg_to_cv2(cache['rgb'][empty_stamp], 'rgb8'))
        print('PASS: live prompt update clears masks and overlay', flush=True)
    finally:
        for child in children:
            if child.poll() is None:
                # Signal launch parents once; they forward to their child processes.
                child.send_signal(signal.SIGINT)
        for child in children:
            try:
                child.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid, signal.SIGKILL)
                child.wait()
        for log in logs:
            log.close()
        observer.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
