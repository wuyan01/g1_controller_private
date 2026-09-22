#!/usr/bin/env python3
"""Localize delayed SAM 3 masks with aligned RGB-D and timestamped TF."""
from collections import OrderedDict
import copy
import json
import time

import cv2
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool, String, Header
from tf2_ros import Buffer, TransformListener, TransformException
from vision_msgs.msg import Detection2DArray, Detection3D, Detection3DArray
from visualization_msgs.msg import Marker, MarkerArray


def stamp_ns(msg):
    return msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec


def surface_points(labels, instance_id, depth, encoding, info, min_pixels=20):
    """Back-project interior mask pixels; reject invalid/foreground-edge depth."""
    if depth.shape != labels.shape or (info.height, info.width) != labels.shape:
        raise ValueError('Depth, labels, and calibration must have matching dimensions')
    if info.binning_x > 1 or info.binning_y > 1 or info.roi.x_offset or info.roi.y_offset:
        raise ValueError('Binned/cropped calibration is not supported')
    if encoding == '16UC1':
        meters = depth.astype(np.float64) * 0.001
    elif encoding == '32FC1':
        meters = depth.astype(np.float64)
    else:
        raise ValueError(f'Unsupported depth encoding: {encoding}')
    mask = cv2.erode((labels == instance_id).astype(np.uint8), np.ones((3, 3), np.uint8)) > 0
    valid = mask & np.isfinite(meters) & (meters >= 0.1) & (meters <= 10.0)
    ys, xs = np.nonzero(valid)
    z = meters[valid]
    if len(z) < min_pixels:
        return None
    median = np.median(z)
    mad = np.median(np.abs(z - median))
    keep = np.abs(z - median) <= max(0.03, 3 * 1.4826 * mad)
    xs, ys, z = xs[keep], ys[keep], z[keep]
    if len(z) < min_pixels:
        return None
    k = np.asarray(info.k, dtype=np.float64).reshape(3, 3)
    if not np.isfinite(k).all() or k[0, 0] <= 0 or k[1, 1] <= 0:
        raise ValueError('Invalid camera intrinsics')
    pixels = np.column_stack((xs, ys)).astype(np.float64).reshape(-1, 1, 2)
    distortion = np.asarray(info.d, dtype=np.float64)
    if not np.isfinite(distortion).all():
        raise ValueError('Invalid distortion coefficients')
    if info.distortion_model in ('plumb_bob', 'rational_polynomial', ''):
        rays = cv2.undistortPoints(pixels, k, distortion if len(distortion) else None)
    elif info.distortion_model == 'equidistant':
        rays = cv2.fisheye.undistortPoints(pixels, k, distortion)
    else:
        raise ValueError(f'Unsupported distortion model: {info.distortion_model}')
    return np.column_stack((rays.reshape(-1, 2) * z[:, None], z))


def transform_points(points, transform):
    q, t = transform.rotation, transform.translation
    quaternion = np.array([q.x, q.y, q.z, q.w], dtype=float)
    norm = np.linalg.norm(quaternion)
    if not np.isfinite(norm) or norm < 1e-12:
        raise ValueError('Invalid TF quaternion')
    x, y, z, w = quaternion / norm
    rotation = np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)],
    ])
    return points @ rotation.T + np.array([t.x, t.y, t.z])


class Sam3Localization(Node):
    def __init__(self, **kwargs):
        super().__init__('sam3_localization', **kwargs)
        self.target_frame = self.declare_parameter('target_frame', 'pelvis').value
        self.slop = float(self.declare_parameter('sync_slop', 0.02).value)
        self.wait_seconds = float(self.declare_parameter('match_wait', 0.5).value)
        self.stale_seconds = float(self.declare_parameter('stale_timeout', 3.0).value)
        self.cache_size = int(self.declare_parameter('cache_frames', 120).value)
        self.min_pixels = int(self.declare_parameter('min_depth_pixels', 20).value)
        if not self.target_frame or not all(np.isfinite(v) and v > 0 for v in
                (self.slop, self.wait_seconds, self.stale_seconds)) or self.cache_size < 1 or self.min_pixels < 1:
            raise ValueError('Invalid localization parameters')
        self.bridge = CvBridge()
        self.buffer = Buffer(cache_time=Duration(seconds=10))
        self.listener = TransformListener(self.buffer, self)
        self.cache = {key: OrderedDict() for key in ('labels', 'detections', 'depth', 'info')}
        self.received = {}
        self.last_completed = -1
        self.last_result_time = time.monotonic()
        self.visible = False
        self.expected_prompt = None
        sensor_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.results_pub = self.create_publisher(Detection3DArray, '/sam3/detections_3d', sensor_qos)
        self.point_pub = self.create_publisher(PointStamped, '/sam3/target_position', sensor_qos)
        self.visible_pub = self.create_publisher(Bool, '/sam3/target_visible', 1)
        self.markers_pub = self.create_publisher(MarkerArray, '/sam3/object_markers', sensor_qos)
        self.status_pub = self.create_publisher(String, '/sam3/localization_status', 1)
        result_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)
        for kind, topic, key in [
            (Image, '/sam3/instance_labels', 'labels'),
            (Detection2DArray, '/sam3/detections', 'detections'),
            (Image, '/camera/aligned_depth_to_color/image_raw', 'depth'),
            (CameraInfo, '/camera/aligned_depth_to_color/camera_info', 'info'),
        ]:
            self.create_subscription(kind, topic, lambda msg, key=key: self.receive(key, msg),
                                     result_qos if key in ('labels', 'detections') else sensor_qos)
        self.create_subscription(String, '/sam3/prompt', self.on_prompt, 1)
        self.create_timer(0.05, self.process)

    def on_prompt(self, msg):
        self.expected_prompt = msg.data.strip()
        self.cache['labels'].clear()
        self.cache['detections'].clear()
        self.received.clear()
        self.publish_result(Header(stamp=self.get_clock().now().to_msg()), [], 'prompt_changed')

    def receive(self, key, msg):
        stamp = stamp_ns(msg)
        if key in ('labels', 'detections'):
            if stamp <= self.last_completed:
                return
            self.received.setdefault(stamp, time.monotonic())
        cache = self.cache[key]
        cache[stamp] = msg
        while len(cache) > self.cache_size:
            dropped, _ = cache.popitem(last=False)
            if key in ('labels', 'detections'):
                if dropped not in self.cache['labels'] and dropped not in self.cache['detections']:
                    self.received.pop(dropped, None)

    def nearest(self, key, stamp):
        cache = self.cache[key]
        if not cache:
            return None
        best = min(cache, key=lambda item: abs(item-stamp))
        return cache[best] if abs(best-stamp) <= self.slop * 1e9 else None

    def process(self):
        now = time.monotonic()
        # Newest first: delayed older results must never replace a newer target.
        for stamp in sorted(self.received, reverse=True):
            if stamp <= self.last_completed:
                continue
            labels_msg = self.cache['labels'].get(stamp)
            detections = self.cache['detections'].get(stamp)
            if labels_msg is None or detections is None:
                if now-self.received[stamp] > self.wait_seconds:
                    source = labels_msg if labels_msg is not None else detections
                    self.publish_result(source.header, [], 'missing_segmentation_pair')
                    self.last_completed = stamp
                continue
            reason = 'no_detections'
            output = []
            try:
                if detections.detections:
                    if self.expected_prompt is not None and any(
                            not d.results or d.results[0].hypothesis.class_id != self.expected_prompt
                            for d in detections.detections):
                        raise ValueError('Obsolete prompt result')
                    depth_msg, info = self.nearest('depth', stamp), self.nearest('info', stamp)
                    if depth_msg is None or info is None:
                        raise ValueError('Synchronized depth/calibration unavailable')
                    if not labels_msg.header.frame_id or any(
                            msg.header.frame_id != labels_msg.header.frame_id
                            for msg in (detections, depth_msg, info)):
                        raise ValueError('RGB labels and aligned depth must share an optical frame')
                    if labels_msg.encoding != '16UC1':
                        raise ValueError('Expected a 16UC1 instance-label image')
                    transform = self.buffer.lookup_transform(
                        self.target_frame, labels_msg.header.frame_id,
                        Time.from_msg(labels_msg.header.stamp))
                    labels = self.bridge.imgmsg_to_cv2(labels_msg)
                    depth = self.bridge.imgmsg_to_cv2(depth_msg)
                    for detection in detections.detections:
                        if not detection.results:
                            continue
                        points = surface_points(labels, int(detection.id), depth,
                                                depth_msg.encoding, info, self.min_pixels)
                        if points is None:
                            continue
                        points = transform_points(points, transform.transform)
                        center = np.median(points, axis=0)
                        lower, upper = np.percentile(points, [5, 95], axis=0)
                        result = Detection3D()
                        result.header = copy.deepcopy(labels_msg.header)
                        result.header.frame_id = self.target_frame
                        result.id = detection.id
                        result.results = copy.deepcopy(detection.results)
                        for hypothesis in result.results:
                            hypothesis.pose.pose.position.x, hypothesis.pose.pose.position.y, hypothesis.pose.pose.position.z = map(float, center)
                            hypothesis.pose.pose.orientation.w = 1.0
                        result.bbox.center.position.x, result.bbox.center.position.y, result.bbox.center.position.z = map(float, (lower+upper)/2)
                        result.bbox.center.orientation.w = 1.0
                        result.bbox.size.x, result.bbox.size.y, result.bbox.size.z = map(float, upper-lower)
                        output.append(result)
                    reason = 'localized' if output else 'no_valid_depth'
            except (ValueError, TransformException, cv2.error, CvBridgeError) as error:
                if now-self.received[stamp] <= self.wait_seconds:
                    continue  # Allow depth and timestamped joint TF to arrive.
                reason = str(error)
                output = []
            self.publish_result(labels_msg.header, output, reason)
            self.last_completed = stamp
        for stamp in list(self.received):
            if stamp <= self.last_completed:
                self.received.pop(stamp)
                self.cache['labels'].pop(stamp, None)
                self.cache['detections'].pop(stamp, None)
        if self.visible and now-self.last_result_time > self.stale_seconds:
            self.publish_result(Header(stamp=self.get_clock().now().to_msg()), [], 'stale_stream')

    def publish_result(self, header, detections, reason):
        header = copy.deepcopy(header)
        header.frame_id = self.target_frame
        result = Detection3DArray(header=header, detections=detections)
        self.results_pub.publish(result)
        self.visible = bool(detections)
        self.visible_pub.publish(Bool(data=self.visible))
        markers = MarkerArray(markers=[Marker(action=Marker.DELETEALL)])
        for i, detection in enumerate(detections):
            marker = Marker(header=header, ns='sam3_positions', id=i, type=Marker.SPHERE,
                            action=Marker.ADD, pose=copy.deepcopy(detection.results[0].pose.pose))
            marker.scale.x = marker.scale.y = marker.scale.z = 0.08
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = 0.1, 1.0, 0.2, 1.0
            marker.lifetime = Duration(seconds=self.stale_seconds).to_msg()
            markers.markers.append(marker)
        self.markers_pub.publish(markers)
        if detections:
            selected = max(detections, key=lambda d: d.results[0].hypothesis.score)
            self.point_pub.publish(PointStamped(header=header, point=selected.results[0].pose.pose.position))
        self.status_pub.publish(String(data=json.dumps(dict(
            valid=self.visible, reason=reason, frame_id=self.target_frame, count=len(detections),
            stamp={'sec': header.stamp.sec, 'nanosec': header.stamp.nanosec}))))
        self.last_result_time = time.monotonic()


def main():
    rclpy.init()
    node = Sam3Localization()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
