"""Bounded RGB-D cache. Conversion happens on tool requests, not control ticks."""
from collections import OrderedDict
import threading
import time
import uuid

import cv2
import numpy as np
from cv_bridge import CvBridge


def stamp_ns(msg):
    return msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec


class RgbdCache:
    def __init__(self, ros_now, clock=time.monotonic, max_age=1.5, slop=.03, retention=60.):
        self.ros_now, self.clock = ros_now, clock
        self.max_age, self.slop, self.retention = max_age, slop, retention
        self.lock = threading.Lock()
        self.frames = {key: OrderedDict() for key in ('rgb', 'depth', 'info')}
        self.snapshots = OrderedDict()
        self.bridge = CvBridge()

    def receive(self, key, msg):
        with self.lock:
            cache = self.frames[key]
            stamp = stamp_ns(msg)
            # Do not refresh the arrival age of repeatedly published old frames.
            if stamp not in cache:
                cache[stamp] = (msg, self.clock())
            while len(cache) > 8:
                cache.popitem(last=False)

    def capture(self):
        with self.lock:
            now, clock = self.ros_now(), self.clock()
            for stamp in sorted(self.frames['rgb'], reverse=True):
                rgb, received = self.frames['rgb'][stamp]
                if not 0 <= (now-stamp)/1e9 <= self.max_age or clock-received > self.max_age:
                    continue
                messages = {'rgb': rgb}
                arrivals = [received]
                for key in ('depth', 'info'):
                    if not self.frames[key]:
                        break
                    other_stamp = min(self.frames[key], key=lambda value: abs(value-stamp))
                    msg, arrived = self.frames[key][other_stamp]
                    if (abs(other_stamp-stamp)/1e9 > self.slop or clock-arrived > self.max_age or
                            not 0 <= (now-other_stamp)/1e9 <= self.max_age):
                        break
                    messages[key] = msg
                    arrivals.append(arrived)
                if len(messages) != 3:
                    continue
                self.validate(messages)
                messages['_received_at'] = min(arrivals)
                token = uuid.uuid4().hex
                self.snapshots[token] = (messages, clock)
                for old, (_, created) in list(self.snapshots.items()):
                    if clock-created > self.retention:
                        del self.snapshots[old]
                while len(self.snapshots) > 4:
                    self.snapshots.popitem(last=False)
                return token, messages
        raise ValueError('No fresh synchronized RGB-D pair: check RGB, aligned depth and camera_info topics')

    @staticmethod
    def validate(messages):
        rgb, depth, info = (messages[key] for key in ('rgb', 'depth', 'info'))
        if not rgb.header.frame_id or any(m.header.frame_id != rgb.header.frame_id for m in (depth, info)):
            raise ValueError('RGB, aligned depth and camera_info must share an optical frame')
        if not 0 < rgb.width <= 4096 or not 0 < rgb.height <= 2160:
            raise ValueError('Invalid or unsupported RGB dimensions')
        if any((m.width, m.height) != (rgb.width, rgb.height) for m in (depth, info)):
            raise ValueError('RGB, aligned depth and camera calibration dimensions differ')
        if depth.encoding not in ('16UC1', '32FC1'):
            raise ValueError('Expected 16UC1 millimetres or 32FC1 metres for depth')
        if info.binning_x > 1 or info.binning_y > 1 or info.roi.x_offset or info.roi.y_offset:
            raise ValueError('Binned/cropped camera calibration is unsupported')
        if not np.isfinite(info.k).all() or info.k[0] <= 0 or info.k[4] <= 0 or not np.isfinite(info.d).all():
            raise ValueError('Invalid camera intrinsics')

    def get(self, token):
        with self.lock:
            if token not in self.snapshots:
                raise ValueError('Unknown RGB-D snapshot; call observe_rgbd again')
            messages, created = self.snapshots[token]
            if self.clock()-created > self.retention:
                del self.snapshots[token]
                raise ValueError('RGB-D snapshot expired; call observe_rgbd again')
            return messages

    def invalidate(self):
        with self.lock:
            self.snapshots.clear()

    def revalidate(self, token):
        """Compare a retained decision image with fresh data; never renew old timestamps.

        This is a conservative scene-change check, not semantic object tracking.
        Check selected depth regions individually so small targets are not diluted
        by the rest of the image. Conversion runs outside the command timer lock.
        """
        original = self.get(token)
        _, current = self.capture()
        a, b = original['info'], current['info']
        if (original['rgb'].header.frame_id != current['rgb'].header.frame_id or
                (a.width, a.height, a.distortion_model) != (b.width, b.height, b.distortion_model) or
                not np.array_equal(a.k, b.k) or not np.array_equal(a.d, b.d)):
            raise ValueError('Camera geometry changed; observe_rgbd again')
        old_depth, new_depth = (self.depth_meters(m['depth']) for m in (original, current))
        old_rgb, new_rgb = (np.asarray(self.bridge.imgmsg_to_cv2(m['rgb'], 'bgr8'), dtype=np.int16)
                            for m in (original, current))
        regions = [(0, 0, a.width, a.height)] + list(original.get('_regions', ()))
        for x0, y0, x1, y1 in regions:
            d0, d1 = old_depth[y0:y1, x0:x1], new_depth[y0:y1, x0:x1]
            v0, v1 = (np.isfinite(d) & (d >= .1) & (d <= 10.) for d in (d0, d1))
            shared = v0 & v1
            if shared.mean() < .5:
                raise ValueError('Insufficient current depth to revalidate visual action; observe_rgbd again')
            changed_depth = (v0 != v1) | (shared & (np.abs(d0-d1) > .10))
            changed_rgb = np.max(np.abs(old_rgb[y0:y1, x0:x1]-new_rgb[y0:y1, x0:x1]), axis=2) > 30
            if changed_depth.mean() > .05 or changed_rgb.mean() > .05:
                raise ValueError('RGB-D scene or selected region changed during reasoning; observe_rgbd again')
        return current

    def metadata(self, token, messages):
        rgb, depth = messages['rgb'], messages['depth']
        age = (self.ros_now()-stamp_ns(rgb))/1e9
        return {'rgbd_snapshot_id': token, 'width': rgb.width, 'height': rgb.height,
                'frame_id': rgb.header.frame_id, 'capture_stamp_ns': stamp_ns(rgb),
                'age_seconds': age,
                'fresh': 0 <= age <= self.max_age and self.clock()-messages['_received_at'] <= self.max_age,
                'depth_offset_seconds': (stamp_ns(depth)-stamp_ns(rgb))/1e9,
                'coordinates': 'Original image pixels: x right, y down; region maximum bounds exclusive',
                'depth_units': 'metres (optical Z, not radial distance)',
                'query_retention_seconds': self.retention,
                'note': 'Retained depth is from this image. Visual actions revalidate against current RGB-D before starting.'}

    def depth_meters(self, message):
        depth = np.asarray(self.bridge.imgmsg_to_cv2(message, desired_encoding='passthrough'), dtype=np.float64)
        return depth * .001 if message.encoding == '16UC1' else depth

    def images(self, messages, include_depth_preview=False):
        bgr = self.bridge.imgmsg_to_cv2(messages['rgb'], desired_encoding='bgr8')
        success, data = cv2.imencode('.jpg', bgr, [cv2.IMWRITE_JPEG_QUALITY, 85])
        if not success:
            raise ValueError('RGB image encoding failed')
        output = [('image/jpeg', data.tobytes())]
        if include_depth_preview:
            meters = self.depth_meters(messages['depth'])
            valid = np.isfinite(meters) & (meters >= .1) & (meters <= 10.)
            scale = np.zeros(meters.shape, np.uint8)
            scale[valid] = np.clip((meters[valid]-.1)/9.9*255, 0, 255).astype(np.uint8)
            colored = cv2.applyColorMap(scale, cv2.COLORMAP_TURBO)
            colored[~valid] = 0
            success, data = cv2.imencode('.png', colored)
            if not success:
                raise ValueError('Depth preview encoding failed')
            output.append(('image/png', data.tobytes()))
        return output

    def region(self, token, x_min, y_min, x_max, y_max):
        messages = self.get(token)
        rgb, info = messages['rgb'], messages['info']
        if (any(type(value) is not int for value in (x_min, y_min, x_max, y_max)) or
                not 0 <= x_min < x_max <= rgb.width or not 0 <= y_min < y_max <= rgb.height):
            raise ValueError('Region must use integer original-image pixels within image bounds')
        with self.lock:
            regions = messages.setdefault('_regions', [])
            region = (x_min, y_min, x_max, y_max)
            if region not in regions:
                if len(regions) >= 16:
                    raise ValueError('Too many regions; observe_rgbd again')
                regions.append(region)
        meters = self.depth_meters(messages['depth'])[y_min:y_max, x_min:x_max]
        valid = np.isfinite(meters) & (meters >= .1) & (meters <= 10.)
        count = int(valid.sum())
        result = dict(self.metadata(token, messages), region=[x_min, y_min, x_max, y_max],
                      valid_pixels=count, valid_fraction=count/valid.size,
                      note='Region statistics can mix foreground/background; select an interior object region or use SAM3.')
        if not count:
            return dict(result, valid=False, reason='No valid depth within 0.1–10 metres'), messages
        ys, xs = np.nonzero(valid)
        pixels = np.column_stack((xs+x_min, ys+y_min)).astype(np.float64).reshape(-1, 1, 2)
        k = np.asarray(info.k).reshape(3, 3)
        distortion = np.asarray(info.d, dtype=np.float64)
        if info.distortion_model in ('', 'plumb_bob', 'rational_polynomial'):
            rays = cv2.undistortPoints(pixels, k, distortion if len(distortion) else None)
        elif info.distortion_model == 'equidistant':
            rays = cv2.fisheye.undistortPoints(pixels, k, distortion)
        else:
            raise ValueError('Unsupported distortion model')
        z = meters[valid]
        points = np.column_stack((rays.reshape(-1, 2)*z[:, None], z))
        return dict(result, valid=True, depth_median_m=float(np.median(z)),
                    depth_p10_m=float(np.percentile(z, 10)), depth_p90_m=float(np.percentile(z, 90)),
                    point_optical_m=np.median(points, axis=0).tolist(),
                    optical_axes='x right, y down, z forward'), messages
