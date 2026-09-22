"""Metric depth, delayed synchronization, and timestamped pelvis TF tests."""
from pathlib import Path
import sys
import time
import unittest

import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TransformStamped, PointStamped
from sensor_msgs.msg import CameraInfo
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose, Detection3DArray
from std_msgs.msg import Bool
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from sam3_localization import Sam3Localization, surface_points, transform_points


def calibration():
    info = CameraInfo(width=13, height=13, distortion_model='plumb_bob', d=[0.0]*5)
    info.k = [100.0, 0.0, 6.0, 0.0, 100.0, 6.0, 0.0, 0.0, 1.0]
    info.header.frame_id = 'camera_color_optical_frame'
    return info


class GeometryTests(unittest.TestCase):
    def test_units_outliers_and_invalid_depth(self):
        labels = np.ones((13, 13), dtype=np.uint16)
        depth = np.full((13, 13), 2000, dtype=np.uint16)
        depth[1, 1] = 0
        depth[2, 2] = 6000
        mm = surface_points(labels, 1, depth, '16UC1', calibration())
        meters = depth.astype(np.float32)/1000
        meters[1, 1] = np.nan
        floating = surface_points(labels, 1, meters, '32FC1', calibration())
        np.testing.assert_allclose(mm, floating)
        np.testing.assert_allclose(mm[:, 2], 2.0)
        self.assertIsNone(surface_points(labels, 1, depth*0, '16UC1', calibration()))
        with self.assertRaises(ValueError):
            surface_points(labels, 1, depth, '8UC1', calibration())

    def test_rotated_optical_frame(self):
        transform = TransformStamped().transform
        transform.rotation.x, transform.rotation.y = -0.5, 0.5
        transform.rotation.z, transform.rotation.w = -0.5, 0.5
        transform.translation.x = 0.1
        transform.translation.y = 0.2
        transform.translation.z = 0.3
        point = transform_points(np.array([[0, 0, 2.0]]), transform)
        np.testing.assert_allclose(point, [[2.1, 0.2, 0.3]])


class LocalizationTests(unittest.TestCase):
    def setUp(self):
        rclpy.init()
        self.node = Sam3Localization()
        self.observer = rclpy.create_node('localization_test')
        self.arrays, self.points, self.visible = [], [], []
        self.subs = [
            self.observer.create_subscription(Detection3DArray, '/sam3/detections_3d', self.arrays.append, qos_profile_sensor_data),
            self.observer.create_subscription(PointStamped, '/sam3/target_position', self.points.append, qos_profile_sensor_data),
            self.observer.create_subscription(Bool, '/sam3/target_visible', self.visible.append, 1),
        ]
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.executor.add_node(self.observer)
        self.wait(lambda: self.node.point_pub.get_subscription_count() > 0)
        for sec, x in [(10, 1.0), (11, 2.0)]:
            transform = TransformStamped()
            transform.header.stamp.sec = sec
            transform.header.frame_id = 'pelvis'
            transform.child_frame_id = 'camera_color_optical_frame'
            transform.transform.rotation.w = 1.0
            transform.transform.translation.x = x
            self.node.buffer.set_transform(transform, 'test')

    def wait(self, predicate):
        deadline = time.monotonic()+3
        while not predicate() and time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=0.01)
        self.assertTrue(predicate())

    def frame(self, sec=10, nanosec=250000000):
        labels = self.node.bridge.cv2_to_imgmsg(np.ones((13,13), np.uint16), '16UC1')
        depth = self.node.bridge.cv2_to_imgmsg(np.full((13,13), 2000, np.uint16), '16UC1')
        info = calibration()
        detections = Detection2DArray()
        detection = Detection2D(id='1')
        hypothesis = ObjectHypothesisWithPose()
        hypothesis.hypothesis.class_id = 'chair'
        hypothesis.hypothesis.score = 0.9
        detection.results = [hypothesis]
        detections.detections = [detection]
        for msg in (labels, depth, info, detections):
            msg.header.stamp.sec, msg.header.stamp.nanosec = sec, nanosec
            msg.header.frame_id = 'camera_color_optical_frame'
        return labels, depth, info, detections

    def test_delayed_mask_uses_capture_time_tf_and_clears_loss(self):
        labels, depth, info, detections = self.frame()
        self.node.receive('depth', depth)
        self.node.receive('info', info)
        self.node.receive('labels', labels)
        self.node.receive('detections', detections)
        self.wait(lambda: bool(self.points))
        point = self.points[-1]
        self.assertEqual(point.header.frame_id, 'pelvis')
        self.assertEqual(point.header.stamp.nanosec, 250000000)
        # TF is interpolated at t=10.25; using the latest t=11 would give x=2.
        np.testing.assert_allclose([point.point.x, point.point.y, point.point.z], [1.25, 0, 2])
        self.wait(lambda: bool(self.arrays) and bool(self.visible))
        self.assertTrue(self.visible[-1].data)
        self.assertEqual(self.arrays[-1].detections[0].id, '1')
        labels, _, _, detections = self.frame(nanosec=500000000)
        detections.detections = []
        self.node.receive('labels', labels)
        self.node.receive('detections', detections)
        self.wait(lambda: len(self.arrays) == 2 and len(self.visible) == 2)
        self.assertFalse(self.arrays[-1].detections)
        self.assertFalse(self.visible[-1].data)
        self.assertEqual(len(self.points), 1)

    def test_missing_capture_tf_does_not_fall_back_to_latest(self):
        labels, depth, info, detections = self.frame(sec=20)
        for key, msg in [('labels', labels), ('depth', depth), ('info', info), ('detections', detections)]:
            self.node.receive(key, msg)
        self.wait(lambda: bool(self.arrays))
        self.assertFalse(self.arrays[-1].detections)
        self.assertFalse(self.points)

    def test_unmatched_depth_is_rejected(self):
        labels, depth, info, detections = self.frame()
        depth.header.stamp.nanosec += 100000000
        for key, msg in [('labels', labels), ('depth', depth), ('info', info), ('detections', detections)]:
            self.node.receive(key, msg)
        self.wait(lambda: bool(self.arrays))
        self.assertFalse(self.arrays[-1].detections)
        self.assertFalse(self.points)

    def test_stale_target_is_invalidated(self):
        self.node.visible = True
        self.node.last_result_time = time.monotonic() - 4.0
        self.node.process()
        self.wait(lambda: bool(self.arrays) and bool(self.visible))
        self.assertFalse(self.arrays[-1].detections)
        self.assertFalse(self.visible[-1].data)
        self.assertFalse(self.points)

    def tearDown(self):
        self.executor.shutdown()
        self.node.destroy_node()
        self.observer.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    unittest.main()
