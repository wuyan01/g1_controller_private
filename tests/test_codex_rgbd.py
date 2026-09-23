from pathlib import Path
import sys
import unittest

import cv2
from cv_bridge import CvBridge
import numpy as np
from sensor_msgs.msg import CameraInfo

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from codex_rgbd import RgbdCache


class RgbdTests(unittest.TestCase):
    def setUp(self):
        self.now = 10.
        self.cache = RgbdCache(lambda: int(self.now*1e9), clock=lambda: self.now)
        self.converter = CvBridge()

    def frame(self, encoding='16UC1', depth=None, stamp=10):
        rgb = self.converter.cv2_to_imgmsg(np.full((4, 4, 3), [255, 0, 0], np.uint8), 'rgb8')
        if depth is None:
            depth = np.full((4, 4), 2000, np.uint16) if encoding == '16UC1' else np.full((4, 4), 2., np.float32)
        depth = self.converter.cv2_to_imgmsg(depth, encoding)
        info = CameraInfo(width=4, height=4, distortion_model='plumb_bob', d=[0.]*5)
        info.k = [100., 0., 1.5, 0., 100., 1.5, 0., 0., 1.]
        messages = dict(rgb=rgb, depth=depth, info=info)
        for msg in messages.values():
            msg.header.stamp.sec = stamp
            msg.header.frame_id = 'camera_optical'
        return messages

    def put(self, messages):
        for key, msg in messages.items():
            self.cache.receive(key, msg)

    def test_mm_and_meters_match_and_image_is_rgb(self):
        for encoding in ('16UC1', '32FC1'):
            self.cache.frames = {key: type(value)() for key, value in self.cache.frames.items()}
            self.put(self.frame(encoding))
            token, messages = self.cache.capture()
            result, _ = self.cache.region(token, 1, 1, 3, 3)
            self.assertEqual(result['depth_median_m'], 2.)
            np.testing.assert_allclose(result['point_optical_m'], [0, 0, 2])
            images = self.cache.images(messages, True)
            decoded = cv2.imdecode(np.frombuffer(images[0][1], np.uint8), cv2.IMREAD_COLOR)
            self.assertEqual(decoded.shape, (4, 4, 3))
            self.assertGreater(int(decoded[0, 0, 2]), 250)  # Red, not BGR-swapped blue.
            self.assertEqual(images[1][0], 'image/png')

    def test_missing_stale_future_and_unsynchronized_frames(self):
        with self.assertRaises(ValueError):
            self.cache.capture()
        messages = self.frame()
        messages['depth'].header.stamp.nanosec = 50_000_000
        self.put(messages)
        with self.assertRaises(ValueError):
            self.cache.capture()
        self.cache.frames['depth'].clear()
        self.cache.receive('depth', self.frame()['depth'])
        self.now = 12.
        with self.assertRaises(ValueError):
            self.cache.capture()
        self.now = 9.
        with self.assertRaises(ValueError):
            self.cache.capture()

    def test_snapshot_keeps_its_depth_and_reports_age(self):
        self.put(self.frame())
        token, _ = self.cache.capture()
        self.now = 11.
        self.put(self.frame(depth=np.full((4, 4), 5000, np.uint16), stamp=11))
        result, _ = self.cache.region(token, 0, 0, 4, 4)
        self.assertEqual(result['depth_median_m'], 2.)
        self.now = 12.
        self.assertFalse(self.cache.region(token, 0, 0, 4, 4)[0]['fresh'])
        self.now = 71.
        with self.assertRaises(ValueError):
            self.cache.region(token, 0, 0, 4, 4)

    def test_reasoning_delay_revalidates_current_data_without_changing_original(self):
        self.put(self.frame())
        token, original = self.cache.capture()
        self.cache.region(token, 1, 1, 3, 3)
        self.now = 50.  # Forty seconds of reasoning, with a live camera.
        self.put(self.frame(stamp=50))
        current = self.cache.revalidate(token)
        self.assertTrue(self.cache.metadata(token, current)['fresh'])
        self.assertFalse(self.cache.metadata(token, original)['fresh'])
        self.assertEqual(original['rgb'].header.stamp.sec, 10)
        self.cache.invalidate()
        with self.assertRaises(ValueError):
            self.cache.revalidate(token)

    def test_revalidation_rejects_camera_loss_scene_changes_and_bad_depth(self):
        for change in ('loss', 'depth', 'rgb', 'calibration', 'invalid', 'region'):
            with self.subTest(change=change):
                self.setUp()
                self.put(self.frame())
                token, _ = self.cache.capture()
                self.cache.region(token, 0, 0, 1, 1)
                self.now = 20.
                current = self.frame(stamp=20)
                if change == 'depth':
                    current = self.frame(depth=np.full((4, 4), 3000, np.uint16), stamp=20)
                elif change == 'invalid':
                    current = self.frame(depth=np.zeros((4, 4), np.uint16), stamp=20)
                elif change == 'rgb':
                    current['rgb'].data = [0] * len(current['rgb'].data)
                elif change == 'calibration':
                    current['info'].k[0] = 120.
                elif change == 'region':
                    depth = np.full((4, 4), 2000, np.uint16)
                    depth[0, 0] = 2500
                    current = self.frame(depth=depth, stamp=20)
                if change != 'loss':
                    self.put(current)
                with self.assertRaises(ValueError):
                    self.cache.revalidate(token)

    def test_invalid_pixels_and_bounds(self):
        depth = np.full((4, 4), np.nan, np.float32)
        depth[0, 0], depth[0, 1], depth[1, 0], depth[1, 1] = 0., 20., 2., 4.
        self.put(self.frame('32FC1', depth))
        token, _ = self.cache.capture()
        result, _ = self.cache.region(token, 0, 0, 2, 2)
        self.assertEqual(result['valid_pixels'], 2)
        self.assertEqual(result['valid_fraction'], .5)
        self.assertEqual(result['depth_median_m'], 3.)
        self.assertFalse(self.cache.region(token, 2, 2, 4, 4)[0]['valid'])
        for bounds in [(-1, 0, 2, 2), (0, 0, 5, 2), (0, 0, 0, 1), (True, 0, 2, 2)]:
            with self.assertRaises(ValueError):
                self.cache.region(token, *bounds)

    def test_alignment_calibration_and_snapshot_capacity(self):
        messages = self.frame()
        messages['depth'].header.frame_id = 'different_camera'
        self.put(messages)
        with self.assertRaises(ValueError):
            self.cache.capture()
        messages['depth'].header.frame_id = 'camera_optical'
        messages['info'].k[0] = 0.
        with self.assertRaises(ValueError):
            self.cache.capture()
        messages['info'].k[0] = 100.
        first, _ = self.cache.capture()
        for _ in range(4):
            self.cache.capture()
        with self.assertRaises(ValueError):
            self.cache.get(first)


if __name__ == '__main__':
    unittest.main()
