"""Ensure timestamp/frame alignment and explicit object association for metrics."""
from pathlib import Path
import sys
import unittest
import time
import numpy as np
import rclpy
from rclpy.parameter import Parameter
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from vision_msgs.msg import Detection3DArray, Detection3D, ObjectHypothesisWithPose
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import String
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from sam3_benchmark import compare, Sam3Benchmark


def pair():
    marker = Marker()
    marker.text = 'desk_Furniture_Monitor'
    marker.header.frame_id = 'pelvis'
    marker.header.stamp.sec = 123
    marker.pose.position.x = 1.0
    marker.pose.orientation.z = marker.pose.orientation.w = np.sqrt(0.5)
    hypothesis = ObjectHypothesisWithPose()
    hypothesis.hypothesis.class_id = 'monitor'
    hypothesis.pose.pose.position.x = 1.0
    hypothesis.pose.pose.position.y = 0.5
    detection = Detection3D(results=[hypothesis])
    array = Detection3DArray(detections=[detection])
    array.header.frame_id = 'pelvis'
    array.header.stamp.sec = 123
    return array, marker


class ComparisonTests(unittest.TestCase):
    def test_late_start_reads_retained_prompt_and_live_command_wins(self):
        rclpy.init()
        source = rclpy.create_node('retained_prompt_test')
        publisher = source.create_publisher(String, '/sam3/status', QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL))
        publisher.publish(String(data='{"state":"running","prompt":"keyboard"}'))
        node = Sam3Benchmark()
        executor = SingleThreadedExecutor()
        executor.add_node(source)
        executor.add_node(node)
        try:
            deadline = time.monotonic() + 5
            while not node.prompt_received and time.monotonic() < deadline:
                executor.spin_once(timeout_sec=0.05)
            self.assertEqual(node.label, 'keyboard')
            self.assertEqual(node.body, 'desk_Props_Keyboard')
            node.count = 7
            node.on_status(String(data='{"prompt":"keyboard"}'))
            self.assertEqual(node.count, 7)
            node.on_prompt(String(data='mouse'))
            node.on_status(String(data='{"prompt":"keyboard"}'))
            self.assertEqual(node.label, 'mouse')
            self.assertEqual(node.body, 'desk_Props_Mouse')
        finally:
            executor.shutdown()
            node.destroy_node()
            source.destroy_node()
            rclpy.shutdown()

    def test_bootstrap_ignores_invalid_status_and_honors_empty_prompt(self):
        rclpy.init()
        node = Sam3Benchmark()
        try:
            for text in ('invalid', '[]', '{"state":"loading"}', '{"prompt":null}'):
                node.on_status(String(data=text))
                self.assertFalse(node.prompt_received)
            node.on_status(String(data='{"prompt":""}'))
            self.assertTrue(node.prompt_received)
            self.assertIsNone(node.body)
            self.assertEqual(node.label, '')
        finally:
            node.destroy_node()
            rclpy.shutdown()

    def test_local_reference_rotates_into_pelvis(self):
        detections, marker = pair()
        estimate, truth = compare(detections, marker, 'monitor', [0.5, 0, 0])
        np.testing.assert_allclose(estimate, truth, atol=1e-12)

    def test_frame_and_timestamp_must_match(self):
        for mismatch in ('frame', 'time'):
            detections, marker = pair()
            if mismatch == 'frame':
                marker.header.frame_id = 'world'
            else:
                marker.header.stamp.nanosec = 1
            with self.assertRaises(ValueError):
                compare(detections, marker, 'monitor', [0, 0, 0])

    def test_no_guess_when_missing_or_ambiguous(self):
        detections, marker = pair()
        with self.assertRaises(ValueError):
            compare(detections, marker, 'chair', [0, 0, 0])
        detections.detections.append(detections.detections[0])
        with self.assertRaises(ValueError):
            compare(detections, marker, 'monitor', [0, 0, 0])

    def test_nonfinite_rejected(self):
        detections, marker = pair()
        detections.detections[0].results[0].pose.pose.position.x = float('nan')
        with self.assertRaises(ValueError):
            compare(detections, marker, 'monitor', [0, 0, 0])

    def test_delayed_truth_and_duplicate_samples(self):
        rclpy.init()
        node = Sam3Benchmark(parameter_overrides=[Parameter('reference_offset', value=[0.5, 0., 0.])])
        try:
            detections, marker = pair()
            node.on_detections(detections)
            node.process()
            self.assertEqual(node.count, 0)
            node.on_truth(MarkerArray(markers=[marker]))
            node.process()
            self.assertEqual(node.count, 1)
            self.assertAlmostEqual(node.sum_squared, 0)
            node.on_detections(detections)
            node.process()
            self.assertEqual(node.count, 1)
        finally:
            node.destroy_node()
            rclpy.shutdown()

    def test_switch_resets_metrics_rejects_old_frames_and_restores_offset(self):
        rclpy.init()
        node = Sam3Benchmark(parameter_overrides=[Parameter('reference_offset', value=[0.5, 0., 0.])])
        try:
            detections, marker = pair()
            node.on_truth(MarkerArray(markers=[marker]))
            node.on_detections(detections)
            node.process()
            self.assertEqual(node.count, 1)
            node.on_prompt(String(data='keyboard'))
            self.assertEqual(node.body, 'desk_Props_Keyboard')
            self.assertEqual(node.label, 'keyboard')
            self.assertEqual(node.offset, [0., 0., 0.])
            self.assertEqual(node.count, 0)
            self.assertEqual(node.sum_squared, 0)
            self.assertFalse(node.truth)
            self.assertFalse(node.pending)
            node.on_detections(detections)
            self.assertFalse(node.pending)
            detections.header.stamp = node.get_clock().now().to_msg()
            marker.header.stamp = detections.header.stamp
            marker.text = 'desk_Props_Keyboard'
            detections.detections[0].results[0].hypothesis.class_id = 'keyboard'
            node.on_truth(MarkerArray(markers=[marker]))
            node.on_detections(detections)
            node.process()
            self.assertEqual(node.count, 1)
            node.on_prompt(String(data='unmapped object'))
            self.assertIsNone(node.body)
            node.on_detections(detections)
            self.assertFalse(node.pending)
            node.on_prompt(String(data='monitor'))
            self.assertEqual(node.body, 'desk_Furniture_Monitor')
            self.assertEqual(node.offset, [0.5, 0., 0.])
            self.assertEqual(node.count, 0)
        finally:
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    unittest.main()
