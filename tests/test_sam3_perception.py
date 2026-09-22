"""ROS integration tests with a deliberately slow, injected inference backend.

Run with ROS sourced: ROS_DOMAIN_ID=88 python3 -m unittest discover -s tests -p test_sam3_perception.py
No checkpoint or GPU is needed; tests never select a mock backend in production.
"""
from pathlib import Path
import sys
import threading
import time
import unittest

import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String
from vision_msgs.msg import Detection2DArray

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from sam3_perception import Prediction, Sam3Perception, compose_outputs


class OutputTests(unittest.TestCase):
    def test_overlap_filtering_and_clipping(self):
        rgb = np.zeros((4, 6, 3), dtype=np.uint8)
        masks = np.zeros((3, 4, 6), dtype=bool)
        masks[0, :, :4] = True
        masks[1, :, 2:] = True
        masks[2] = True
        prediction = Prediction(masks, np.array([[-1, -1, 4, 4], [2, 0, 8, 4], [0, 0, 6, 4]]),
                                np.array([0.6, 0.9, 0.1]))
        labels, overlay, records = compose_outputs(rgb, prediction, 'chair', 0.5, 100)
        np.testing.assert_array_equal(labels[:, :2], 2)
        np.testing.assert_array_equal(labels[:, 2:], 1)
        self.assertEqual(records[0][2], (2, 0, 6, 4))
        self.assertEqual(records[1][2], (0, 0, 4, 4))
        self.assertEqual(overlay.shape, rgb.shape)
        self.assertEqual(labels.dtype, np.uint16)
        labels, _, records = compose_outputs(rgb, prediction, 'chair', 0.5, 1)
        self.assertEqual(len(records), 1)
        self.assertEqual(labels.max(), 1)

    def test_no_detections_and_invalid_shape(self):
        rgb = np.ones((4, 6, 3), dtype=np.uint8) * 20
        prediction = Prediction(np.empty((0, 4, 6)), np.empty((0, 4)), np.empty(0))
        labels, overlay, records = compose_outputs(rgb, prediction, 'chair', 0.5, 100)
        self.assertFalse(labels.any())
        np.testing.assert_array_equal(overlay, rgb)
        self.assertFalse(records)
        prediction.masks = np.empty((0, 6, 4))
        with self.assertRaises(ValueError):
            compose_outputs(rgb, prediction, 'chair', 0.5, 100)


class NodeTests(unittest.TestCase):
    def setUp(self):
        rclpy.init()
        self.entered = threading.Event()
        self.release = threading.Event()
        self.calls = []
        test = self

        class SlowBackend:
            def __init__(self, *args):
                pass

            def predict(self, rgb, prompt):
                test.calls.append((int(rgb[0, 0, 0]), prompt, time.monotonic()))
                if len(test.calls) == 1:
                    test.entered.set()
                    if not test.release.wait(5):
                        raise RuntimeError('Test failed to release inference')
                h, w = rgb.shape[:2]
                return Prediction(np.ones((1, h, w), dtype=bool),
                                  np.array([[0, 0, w, h]]), np.array([0.9]))

        self.node = Sam3Perception(backend_factory=SlowBackend, parameter_overrides=[
            Parameter('max_rate_hz', value=5.0), Parameter('prompt', value='chair')])
        self.observer = rclpy.create_node('sam3_test_observer')
        self.outputs = {'overlay': [], 'labels': [], 'detections': []}
        self.subs = [
            self.observer.create_subscription(kind, topic,
                lambda msg, key=key: self.outputs[key].append(msg), qos_profile_sensor_data)
            for kind, topic, key in [(Image, '/sam3/overlay', 'overlay'),
                                     (Image, '/sam3/instance_labels', 'labels'),
                                     (Detection2DArray, '/sam3/detections', 'detections')]
        ]
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.executor.add_node(self.observer)
        self.wait_for(lambda: self.node.overlay_pub.get_subscription_count() > 0)

    def wait_for(self, condition, timeout=5):
        deadline = time.monotonic() + timeout
        while not condition() and time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=0.01)
        self.assertTrue(condition(), 'Timed out waiting for ROS/worker result')

    def send(self, sequence):
        msg = self.node.bridge.cv2_to_imgmsg(np.full((16, 20, 3), sequence, np.uint8), 'rgb8')
        msg.header.stamp.sec = sequence
        msg.header.frame_id = 'camera_color_optical_frame'
        self.node.on_image(msg)

    def test_latest_frame_prompt_change_and_empty_prompt(self):
        self.send(1)
        self.wait_for(self.entered.is_set)
        # A prompt callback must proceed even though GPU work is still blocked.
        publisher = self.observer.create_publisher(String, '/sam3/prompt', 1)
        self.wait_for(lambda: publisher.get_subscription_count() > 0)
        publisher.publish(String(data='table'))
        self.wait_for(lambda: self.node.prompt == 'table')
        for sequence in range(2, 10):
            self.send(sequence)
        self.release.set()
        self.wait_for(lambda: all(self.outputs.values()))
        self.assertEqual([(x, prompt) for x, prompt, _ in self.calls], [(1, 'chair'), (9, 'table')])
        self.assertGreaterEqual(self.calls[1][2]-self.calls[0][2], 0.19)
        for outputs in self.outputs.values():
            self.assertEqual(len(outputs), 1)  # Obsolete in-flight prompt result was dropped.
            self.assertEqual(outputs[0].header.stamp.sec, 9)
            self.assertEqual(outputs[0].header.frame_id, 'camera_color_optical_frame')
        detection = self.outputs['detections'][0].detections[0]
        self.assertEqual(detection.id, '1')
        self.assertEqual(detection.results[0].hypothesis.class_id, 'table')
        self.assertAlmostEqual(detection.bbox.center.position.x, 10)
        self.assertAlmostEqual(detection.bbox.size_y, 16)
        self.assertEqual(self.outputs['labels'][0].encoding, '16UC1')
        publisher.publish(String(data=''))
        self.wait_for(lambda: self.node.prompt == '')
        self.send(10)
        self.wait_for(lambda: all(len(outputs) == 2 for outputs in self.outputs.values()))
        self.assertEqual(len(self.calls), 2)  # Empty prompt does not call inference.
        self.assertFalse(self.outputs['detections'][-1].detections)
        labels = self.node.bridge.imgmsg_to_cv2(self.outputs['labels'][-1])
        self.assertFalse(labels.any())

    def tearDown(self):
        self.release.set()
        self.node.stop()
        self.executor.shutdown()
        self.node.destroy_node()
        self.observer.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    unittest.main()
