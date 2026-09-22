#!/usr/bin/env python3
"""Isolated, controller-free camera smoke test; run inside the sourced ROS workspace.

python3 tests/camera_smoke.py /tmp/g1-camera-build/g1_mujoco_camera
Add --rviz to verify the saved RViz configuration subscribes to RGB and points.
Uses ROS/Unitree domain 87. Creates /tmp/g1-camera-smoke-* logs and RGB preview.
"""
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

os.environ['ROS_DOMAIN_ID'] = '87'
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, PointCloud2


def main():
    root = Path(__file__).resolve().parents[1]
    children, logs = [], []
    commands = [
        ['ros2', 'launch', 'g1_description', 'g1_description.launch.py'],
        ['ros2', 'launch', str(root / 'launch/g1_camera.launch.py'), 'simulation:=true'],
        [sys.argv[1], '-r', 'g1', '-s', 'scene_29dof.xml', '-i', '87', '-n', 'lo'],
    ]
    if '--rviz' in sys.argv:
        commands.append(['rviz2', '-d', str(root / 'config/rviz/config.rviz'),
                         '-f', 'camera_color_optical_frame'])
    rclpy.init()
    node = rclpy.create_node('camera_smoke')
    received = {}
    counts = {}
    subscriptions = []
    topics = [
        (Image, '/camera/color/image_raw'),
        (Image, '/camera/aligned_depth_to_color/image_raw'),
        (CameraInfo, '/camera/color/camera_info'),
        (CameraInfo, '/camera/aligned_depth_to_color/camera_info'),
        (PointCloud2, '/camera/depth/color/points'),
    ]
    def record(msg, topic):
        received[topic] = msg
        counts[topic] = counts.get(topic, 0) + 1
    for kind, topic in topics:
        subscriptions.append(node.create_subscription(
            kind, topic, lambda msg, topic=topic: record(msg, topic), qos_profile_sensor_data))
    try:
        for i, command in enumerate(commands):
            log = open(f'/tmp/g1-camera-smoke-{i}.log', 'w')
            logs.append(log)
            children.append(subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                              start_new_session=True))
        deadline = time.monotonic() + 40
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if len(counts) == len(topics) and min(counts.values()) >= 5:
                stamps = {(m.header.stamp.sec, m.header.stamp.nanosec) for m in received.values()}
                if len(stamps) == 1:
                    break
            assert all(p.poll() is None for p in children), 'Child exited; check /tmp/g1-camera-smoke-*.log'
        assert len(counts) == len(topics) and min(counts.values()) >= 5, counts
        assert len(stamps) == 1, 'RGB, depth, calibration and cloud timestamps differ'
        rgb, depth, info, depth_info, cloud = [received[t] for _, t in topics]
        assert rgb.encoding == 'rgb8' and depth.encoding == '16UC1'
        assert all(m.header.frame_id == 'camera_color_optical_frame' for m in received.values())
        assert all((m.width, m.height) == (640, 480) for m in received.values())
        assert np.array_equal(info.k, depth_info.k) and info.k[0] > 0 and info.r[8] == 1
        colors = np.frombuffer(rgb.data, dtype=np.uint8).reshape(480, 640, 3)
        depth_mm = np.frombuffer(depth.data, dtype='>u2' if depth.is_bigendian else '<u2').reshape(480, 640)
        fields = {f.name: f.offset for f in cloud.fields}
        xyz = np.ndarray((480, 640, 3), dtype=np.float32, buffer=cloud.data,
                         strides=(cloud.row_step, cloud.point_step, 4))
        valid = depth_mm > 0
        assert valid.any() and colors.std() > 1, 'Empty camera render'
        assert np.allclose(xyz[:, :, 2][valid] * 1000, depth_mm[valid], atol=0.51)
        ys, xs = np.nonzero(valid)
        assert np.allclose(xyz[:, :, 0][valid], (xs-info.k[2])*xyz[:, :, 2][valid]/info.k[0], atol=1e-5)
        assert np.allclose(xyz[:, :, 1][valid], (ys-info.k[5])*xyz[:, :, 2][valid]/info.k[4], atol=1e-5)
        packed = np.ndarray((480, 640, 4), dtype=np.uint8, buffer=cloud.data,
                            offset=fields['rgb'], strides=(cloud.row_step, cloud.point_step, 1))
        assert np.array_equal(packed[:, :, :3][:, :, ::-1], colors)
        if '--rviz' in sys.argv:
            for topic in ('/camera/color/image_raw', '/camera/depth/color/points'):
                endpoints = node.get_subscriptions_info_by_topic(topic)
                names = [endpoint.node_name for endpoint in endpoints]
                print(topic, 'subscribers:', names)
                assert any(name.startswith('rviz') for name in names), (topic, names)
        from PIL import Image as PILImage
        PILImage.fromarray(colors).save('/tmp/g1-camera-smoke-rgb.png')
        print('PASS: synchronized RGB/depth/calibration/cloud, optical frame, metric depth, '
              'XYZ projection, RGB packing; counts:', counts)
        print('Valid depth pixels:', int(valid.sum()), 'range (mm):', int(depth_mm[valid].min()), int(depth_mm.max()))
    finally:
        for child in children:
            if child.poll() is None:
                os.killpg(child.pid, signal.SIGINT)
        for child in children:
            try:
                child.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid, signal.SIGKILL)
                child.wait()
        for log in logs:
            log.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
