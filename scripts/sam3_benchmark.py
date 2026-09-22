#!/usr/bin/env python3
"""Compare a SAM3 visible-surface position to a timestamped MuJoCo reference point."""
from collections import OrderedDict
import json
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Transform
from std_msgs.msg import String
from vision_msgs.msg import Detection3DArray
from visualization_msgs.msg import MarkerArray

from sam3_localization import stamp_ns, transform_points

# Explicit associations for the photo desk scene; never infer truth by proximity.
DESK_TARGETS = {
    'monitor': 'desk_Furniture_Monitor',
    'desk': 'desk_Furniture_Table',
    'table': 'desk_Furniture_Table',
    'keyboard': 'desk_Props_Keyboard',
    'mouse': 'desk_Props_Mouse',
    'headphones': 'desk_Props_Headphones',
    'laptop': 'desk_Props_Laptop',
    'notebook': 'desk_Props_Notebook',
    'pen': 'desk_Props_Pen',
    'food container': 'desk_Props_FoodContainer',
    'toothpaste tube': 'desk_Props_ToothpasteTube',
    'cloth': 'desk_Props_BeigeCloth',
    'lanyard': 'desk_Props_GreenLanyard',
    'pouch': 'desk_Props_DarkPouch',
    'twist tie': 'desk_Props_SmallTwistTie',
}


def compare(detections, marker, label, offset):
    """Require unambiguous semantic association; never choose by proximity to truth."""
    if detections.header.frame_id != marker.header.frame_id:
        raise ValueError('Frame mismatch: localization must use pelvis')
    if stamp_ns(detections) != stamp_ns(marker):
        raise ValueError('Capture timestamp mismatch')
    matches = [d for d in detections.detections if d.results and
               d.results[0].hypothesis.class_id == label]
    if len(matches) != 1:
        raise ValueError(f'Expected one {label!r} detection, got {len(matches)}')
    p = matches[0].results[0].pose.pose.position
    estimated = np.array([p.x, p.y, p.z])
    transform = Transform(rotation=marker.pose.orientation)
    transform.translation.x = marker.pose.position.x
    transform.translation.y = marker.pose.position.y
    transform.translation.z = marker.pose.position.z
    truth = transform_points(np.asarray([offset]), transform)[0]
    if not np.isfinite(estimated).all() or not np.isfinite(truth).all():
        raise ValueError('Non-finite position')
    return estimated, truth


class Sam3Benchmark(Node):
    def __init__(self, **kwargs):
        super().__init__('sam3_benchmark', **kwargs)
        self.body = self.declare_parameter('target_body', 'desk_Furniture_Monitor').value
        self.label = self.declare_parameter('target_label', 'monitor').value
        self.offset = self.declare_parameter('reference_offset', [0.0, 0.0, 0.0]).value
        self.print_hz = float(self.declare_parameter('print_hz', 1.0).value)
        if (not self.body or not self.label or len(self.offset) != 3 or
                not np.isfinite(self.offset).all() or not np.isfinite(self.print_hz) or self.print_hz <= 0):
            raise ValueError('Invalid benchmark parameters')
        self.initial_target = (self.label, self.body, list(self.offset))
        self.prompt_received = False
        self.truth = OrderedDict()
        self.pending = OrderedDict()
        self.last_stamp = -1
        self.count = 0
        self.sum_error = self.sum_squared = 0.0
        self.next_print = 0.0
        self.last_pair = time.monotonic()
        self.status = 'waiting for detections and ground truth'
        self.publisher = self.create_publisher(String, '/sam3/benchmark', 10)
        self.create_subscription(MarkerArray, '/simulation/body_poses', self.on_truth, 10)
        self.create_subscription(Detection3DArray, '/sam3/detections_3d', self.on_detections,
                                 qos_profile_sensor_data)
        self.create_subscription(String, '/sam3/prompt', self.on_prompt, 1)
        # The command topic is volatile. Bootstrap late subscribers from the
        # perception node's retained status, then let live commands take priority.
        self.create_subscription(String, '/sam3/status', self.on_status, QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_timer(0.1, self.process)
        self.get_logger().info(f'Waiting for current perception target; configured fallback '
                               f'{self.label!r} -> {self.body} + local offset '
                               f'{list(self.offset)}; meters in pelvis, exact capture timestamps. '
                               'Visible-surface median versus reference point: geometric bias is included.')

    def on_status(self, msg):
        if self.prompt_received:
            return
        try:
            status = json.loads(msg.data)
        except (ValueError, TypeError):
            return
        if isinstance(status, dict) and isinstance(status.get('prompt'), str):
            self.on_prompt(String(data=status['prompt']))

    def on_prompt(self, msg):
        self.prompt_received = True
        self.label = msg.data.strip()
        initial_label, initial_body, initial_offset = self.initial_target
        if self.label == initial_label:
            self.body, self.offset = initial_body, list(initial_offset)
        else:
            self.body = DESK_TARGETS.get(self.label.lower())
            self.offset = [0.0, 0.0, 0.0]
        self.pending.clear()
        self.truth.clear()
        # Reject delayed pre-switch frames, including when switching back to a
        # previously used label. All camera stamps use this ROS wall clock.
        self.last_stamp = self.get_clock().now().nanoseconds
        self.count = 0
        self.sum_error = self.sum_squared = 0.0
        self.last_pair = time.monotonic()
        self.next_print = 0.0
        if self.body:
            self.status = f'waiting for {self.label!r} / {self.body}'
            self.get_logger().info(f'Target switched: {self.label!r} -> {self.body}; statistics reset')
        else:
            self.status = f'benchmark paused: no body mapping for prompt {self.label!r}'
            self.get_logger().warn(self.status)

    def on_truth(self, msg):
        for marker in msg.markers:
            if marker.text == self.body:
                self.truth[stamp_ns(marker)] = marker
        while len(self.truth) > 600:  # About 20 seconds at 30 Hz; inference is delayed.
            self.truth.popitem(last=False)

    def on_detections(self, msg):
        if not self.body:
            return
        stamp = stamp_ns(msg)
        if stamp > self.last_stamp:
            self.pending[stamp] = (msg, time.monotonic())
        while len(self.pending) > 120:
            self.pending.popitem(last=False)

    def process(self):
        now = time.monotonic()
        for stamp in sorted(list(self.pending)):
            msg, received = self.pending[stamp]
            if stamp <= self.last_stamp:
                del self.pending[stamp]
                continue
            marker = self.truth.get(stamp)
            if marker is None and now - received < 2.0:
                continue
            del self.pending[stamp]
            self.last_stamp = stamp
            try:
                if marker is None:
                    raise ValueError(f'No exact-stamp ground truth for body {self.body!r}')
                estimated, truth = compare(msg, marker, self.label, self.offset)
            except ValueError as error:
                self.status = str(error)
                continue
            delta = estimated - truth
            error = float(np.linalg.norm(delta))
            self.count += 1
            self.sum_error += error
            self.sum_squared += error * error
            self.last_pair = now
            result = dict(stamp_ns=stamp, frame_id=msg.header.frame_id, target_body=self.body,
                          target_label=self.label, reference_offset=list(self.offset),
                          reference_definition='body_origin_plus_local_offset',
                          estimate_definition='visible_surface_median',
                          estimated=estimated.tolist(), ground_truth=truth.tolist(),
                          delta=delta.tolist(), distance_m=error, samples=self.count,
                          mean_distance_m=self.sum_error/self.count,
                          rmse_m=float(np.sqrt(self.sum_squared/self.count)))
            self.publisher.publish(String(data=json.dumps(result)))
            if now >= self.next_print:
                fmt = lambda v: '(' + ', '.join(f'{x:+.4f}' for x in v) + ')'
                self.get_logger().info(
                    f'{self.label} pelvis [m] stamp={stamp}: estimated={fmt(estimated)} '
                    f'ground_truth={fmt(truth)} delta={fmt(delta)} '
                    f'distance={error:.4f} RMSE={result["rmse_m"]:.4f} n={self.count}')
                self.next_print = now + 1.0/self.print_hz
        if now - self.last_pair >= 5.0 and now >= self.next_print:
            self.get_logger().warn(f'No valid benchmark pair: {self.status}')
            self.next_print = now + 5.0


def main():
    rclpy.init()
    node = Sam3Benchmark()
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
