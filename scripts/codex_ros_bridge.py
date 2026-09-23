#!/usr/bin/env python3
"""Codex MCP tools and a 50 Hz ROS supervisor. Stdout is MCP-only."""
import argparse
import asyncio
import base64
import fcntl
import json
import math
import os
import signal
import threading
import time

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool, String
from vision_msgs.msg import Detection3DArray
from tf2_ros import Buffer, TransformListener, TransformException
from mcp.server.fastmcp import FastMCP
from mcp.types import ImageContent, TextContent

from codex_ros_core import CommandSupervisor, heading_point, rotate_point
from codex_rgbd import RgbdCache


class RosBridge(Node):
    def __init__(self, dry_run=True, **kwargs):
        super().__init__('codex_ros_bridge', **kwargs)
        self.dry_run = dry_run
        self.lock = threading.RLock()
        self.supervisor = CommandSupervisor()
        self.rgbd_action = None
        self.action_revision = 0
        self.prompt = None
        self.prompt_stamp = 0
        self.detections = None
        self.received = 0.0
        self.ready = False
        self.ready_received = 0.0
        self.world_frame = self.declare_parameter('world_frame', 'odom').value
        self.base_frame = self.declare_parameter('base_frame', 'pelvis').value
        # G1Articulation uses torso odometry for the policy's root quaternion.
        self.control_frame = self.declare_parameter('control_frame', 'torso_link').value
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.rgbd = RgbdCache(lambda: self.get_clock().now().nanoseconds)
        for key, kind, default in [
                ('rgb', Image, '/camera/color/image_raw'),
                ('depth', Image, '/camera/aligned_depth_to_color/image_raw'),
                ('info', CameraInfo, '/camera/aligned_depth_to_color/camera_info')]:
            camera_topic = self.declare_parameter(f'rgbd_{key}_topic', default).value
            self.create_subscription(kind, camera_topic,
                                     lambda msg, key=key: self.rgbd.receive(key, msg), qos_profile_sensor_data)
        topic = '/codex/steering_preview' if dry_run else '/codex/steering_command'
        self.command_pub = self.create_publisher(TwistStamped, topic, 1)
        self.prompt_pub = self.create_publisher(String, '/sam3/prompt', 1)
        self.status_pub = self.create_publisher(String, '/codex/bridge_status', 1)
        self.create_subscription(Detection3DArray, '/sam3/detections_3d', self.on_detections,
                                 qos_profile_sensor_data)
        self.create_subscription(Bool, '/codex/steering_ready', self.on_ready, 1)
        self.create_subscription(String, '/sam3/prompt', self.on_prompt, 1)
        self.create_subscription(Bool, '/codex/stop', self.on_stop, 1)
        # Keeps publishing and expiring commands even if ROS simulation time pauses.
        self.timer = self.create_timer(0.02, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def on_detections(self, msg):
        with self.lock:
            self.detections = msg
            self.received = time.monotonic()

    def on_ready(self, msg):
        with self.lock:
            self.ready, self.ready_received = msg.data, time.monotonic()

    def on_prompt(self, msg):
        with self.lock:
            if msg.data.strip() != self.prompt:
                self.reset_target(msg.data.strip())

    def on_stop(self, msg):
        if msg.data:
            self.stop('external_stop')

    def reset_target(self, prompt):
        self.action_revision += 1
        self.rgbd.invalidate()
        self.supervisor.stop('target_changed')
        self.prompt = prompt
        self.prompt_stamp = self.get_clock().now().nanoseconds
        self.detections = None
        self.publish((0.0, 0.0, 0.0))

    def set_target(self, prompt):
        if not prompt.strip() or len(prompt) > 200:
            raise ValueError('Target prompt must contain 1–200 characters')
        with self.lock:
            self.reset_target(prompt.strip())
            self.prompt_pub.publish(String(data=self.prompt))
        return {'target': self.prompt, 'status': 'waiting_for_fresh_perception'}

    def controller_ready(self):
        return self.ready and time.monotonic()-self.ready_received < 0.25

    def scene(self):
        msg = self.detections
        result = {'valid': False, 'target': self.prompt, 'frame': 'heading',
                  'dry_run': self.dry_run, 'controller_ready': self.controller_ready()}
        if msg is None or not self.prompt:
            return dict(result, reason='set_target_and_wait_for_detections')
        stamp = Time.from_msg(msg.header.stamp).nanoseconds
        age = (self.get_clock().now().nanoseconds-stamp)/1e9
        result['age_seconds'] = age
        if (stamp <= self.prompt_stamp or not 0 <= age <= self.supervisor.max_scene_age or
                time.monotonic()-self.received > self.supervisor.max_scene_age):
            return dict(result, reason='stale_perception')
        if msg.header.frame_id != self.base_frame:
            return dict(result, reason='localization_must_use_base_frame')
        candidates = [d for d in msg.detections if d.results and
                      d.results[0].hypothesis.class_id == self.prompt and
                      math.isfinite(d.results[0].hypothesis.score) and d.results[0].hypothesis.score >= 0.5]
        if not candidates:
            return dict(result, reason='target_not_visible')
        try:
            rotation = self.buffer.lookup_transform(self.world_frame, self.control_frame,
                                                   Time.from_msg(msg.header.stamp)).transform.rotation
            body = self.buffer.lookup_transform(self.control_frame, self.base_frame,
                                               Time.from_msg(msg.header.stamp)).transform
            objects = []
            for detection in candidates:
                hypothesis = detection.results[0]
                p = hypothesis.pose.pose.position
                q = body.rotation
                relative = rotate_point((p.x, p.y, p.z), (q.x, q.y, q.z, q.w))
                relative = [a+b for a, b in zip(relative, (body.translation.x, body.translation.y, body.translation.z))]
                xyz = heading_point(relative, (rotation.x, rotation.y, rotation.z, rotation.w))
                objects.append({'id': detection.id, 'confidence': hypothesis.hypothesis.score,
                                'position': xyz, 'distance_xy': math.hypot(*xyz[:2])})
            return dict(result, valid=True, objects=objects,
                        selected=max(objects, key=lambda obj: obj['confidence']))
        except (TransformException, ValueError) as error:
            return dict(result, reason=str(error))

    def runtime_status(self):
        return {'bridge_pid': os.getpid(), 'limits': {
            'speed_mps': self.supervisor.max_speed,
            'yaw_rate_radps': self.supervisor.max_yaw_rate,
            'duration_seconds': self.supervisor.max_duration,
            'velocity_duration_seconds': self.supervisor.max_velocity_duration,
            'snapshot_lifetime_seconds': self.supervisor.max_scene_age}}

    def observe(self):
        with self.lock:
            scene = self.scene()
            scene['snapshot_id'] = self.supervisor.observe() if scene['valid'] else None
            if scene['valid']:
                # The lease starts at image capture, not when a delayed result is read.
                self.supervisor.observed -= scene['age_seconds']
            else:
                self.supervisor.token = None
            scene.update(self.runtime_status())
            scene['status'] = self.supervisor.reason
            return scene

    def publish(self, velocity):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'heading'  # Semantic yaw-only frame; not a TF frame.
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.angular.z = velocity
        self.command_pub.publish(msg)

    def observe_rgbd(self, include_depth_preview=False):
        token, messages = self.rgbd.capture()
        metadata = self.rgbd.metadata(token, messages)
        with self.lock:
            metadata.update(dry_run=self.dry_run, controller_ready=self.controller_ready())
            metadata.update(self.runtime_status())
        metadata['images'] = ['RGB at original pixel dimensions']
        if include_depth_preview:
            metadata['images'].append('Depth visualization: TURBO scale 0.1–10 m; invalid pixels black')
        content = [TextContent(type='text', text=json.dumps(metadata))]
        for mime, data in self.rgbd.images(messages, include_depth_preview):
            content.append(ImageContent(type='image', data=base64.b64encode(data).decode(), mimeType=mime))
        return content

    def get_depth_region(self, snapshot_id, x_min, y_min, x_max, y_max):
        result, messages = self.rgbd.region(snapshot_id, x_min, y_min, x_max, y_max)
        if not result['valid']:
            return result
        try:
            stamp = Time.from_msg(messages['rgb'].header.stamp)
            body = self.buffer.lookup_transform(self.control_frame, result['frame_id'], stamp).transform
            q = body.rotation
            point = rotate_point(result['point_optical_m'], (q.x, q.y, q.z, q.w))
            point = [a+b for a, b in zip(point, (body.translation.x, body.translation.y, body.translation.z))]
            q = self.buffer.lookup_transform(self.world_frame, self.control_frame, stamp).transform.rotation
            point = heading_point(point, (q.x, q.y, q.z, q.w))
            result.update(point_heading_m=point, distance_xy_m=math.hypot(*point[:2]),
                          heading_axes='x forward, y left, z up; relative to torso heading at capture time')
        except (TransformException, ValueError) as error:
            result['heading_unavailable'] = str(error)
        return result

    def tick(self):
        with self.lock:
            try:
                if self.supervisor.requires_scene and self.rgbd_action is not None:
                    scene_valid = self.rgbd.metadata(*self.rgbd_action)['fresh']
                else:
                    scene_valid = self.scene()['valid'] if self.supervisor.requires_scene else True
                velocity = self.supervisor.tick(scene_valid, self.dry_run or self.controller_ready())
                self.publish(velocity)
                self.status_pub.publish(String(data=json.dumps({
                    'dry_run': self.dry_run, 'status': self.supervisor.reason,
                    'requires_scene': self.supervisor.requires_scene,
                    'velocity': velocity, 'controller_ready': self.controller_ready(),
                    **self.runtime_status()})))
            except Exception:
                self.supervisor.stop('supervisor_error')
                self.publish((0.0, 0.0, 0.0))
                raise

    def stop(self, reason='stopped'):
        with self.lock:
            self.action_revision += 1
            self.rgbd.invalidate()
            self.supervisor.stop(reason)
            self.publish((0.0, 0.0, 0.0))
            return {'status': reason, 'dry_run': self.dry_run}

    async def execute(self, snapshot_id, vx, vy, yaw_rate, duration):
        with self.lock:
            self.supervisor.start(snapshot_id, vx, vy, yaw_rate, duration,
                                  self.scene()['valid'], self.dry_run or self.controller_ready())
            self.rgbd_action = None
            self.action_revision += 1
            self.rgbd.invalidate()
        return await self.wait_for_action()

    async def execute_velocity(self, vx, vy, yaw_rate, duration, rgbd_snapshot_id=None):
        with self.lock:
            if self.supervisor.active:
                raise ValueError('An action is already executing')
            revision = self.action_revision
        visual = None
        if rgbd_snapshot_id is not None:
            if duration > self.supervisor.max_duration:
                raise ValueError('RGB-D-guided actions are limited to 1 second')
            messages = await asyncio.to_thread(self.rgbd.revalidate, rgbd_snapshot_id)
            visual = (rgbd_snapshot_id, messages)
        with self.lock:
            if revision != self.action_revision:
                raise ValueError('Control state changed during revalidation; observe again')
            if visual and not self.rgbd.metadata(*visual)['fresh']:
                raise ValueError('Current RGB-D expired during revalidation; observe again')
            self.supervisor.start_velocity(vx, vy, yaw_rate, duration,
                                           self.dry_run or self.controller_ready())
            self.action_revision += 1
            self.rgbd.invalidate()
            self.rgbd_action = visual
            self.supervisor.requires_scene = visual is not None
        return await self.wait_for_action()

    async def wait_for_action(self):
        try:
            while True:
                await asyncio.sleep(0.02)
                with self.lock:
                    if not self.supervisor.active:
                        return {'status': self.supervisor.reason, 'dry_run': self.dry_run,
                                'requires_scene': self.supervisor.requires_scene,
                                'scene': self.scene() if self.supervisor.requires_scene and self.rgbd_action is None else None,
                                'rgbd_snapshot_id': self.rgbd_action[0] if self.rgbd_action is not None else None,
                                'note': 'Command window ended; this does not certify physical displacement.'}
        finally:
            with self.lock:
                if self.supervisor.active:
                    self.stop('cancelled')


def make_server(node):
    server = FastMCP('g1-ros', instructions=(
        'Read live limits from observe_scene or observe_rgbd; do not rely on old conversation limits. '
        'For visual tasks use observe_rgbd, then get_depth_region for metric distance in original image pixels. '
        'Pass rgbd_snapshot_id to execute_velocity for a short visual action; refresh before every action. '
        'For relative motion use execute_velocity without an image ID: no perception required. SAM3 is optional. '
        'For relative directions such as "move left for 3 seconds", use execute_velocity: no object, SAM3, '
        'TF or snapshot is required. For optional SAM3 guidance use set_target, observe_scene and execute_steering '
        'with a fresh snapshot. Positive vx is forward, vy left, yaw_rate counterclockwise; units m/s, rad/s. '
        'Use 0.1 m/s for "slowly". Wait for each action to finish; idle output is zero. '
        'Velocity actions allow up to 3 seconds; target-guided actions allow 1 second. '
        'Controller readiness, cancellation and joystick takeover apply to both. No obstacle avoidance. '
        'Dry-run publishes preview commands only; never report it as physical movement.'))

    @server.tool(structured_output=False)
    def observe_rgbd(include_depth_preview: bool = False) -> list[TextContent | ImageContent]:
        """Read one fresh aligned RGB-D snapshot on demand; no SAM3 target is needed.

        Returns RGB image and rgbd_snapshot_id. Optional second image visualizes depth.
        No image resizing: depth queries use the returned image's original pixel coordinates.
        This never moves the robot. No continuous image stream is sent to Codex.
        """
        return node.observe_rgbd(include_depth_preview)

    @server.tool()
    def get_depth_region(rgbd_snapshot_id: str, x_min: int, y_min: int, x_max: int, y_max: int) -> dict:
        """Query metric depth for a region of the exact image returned by observe_rgbd.

        Pixel bounds: minimum inclusive, maximum exclusive; origin top left.
        Returns optical Z statistics and 3D point; torso-heading point if timestamped TF is available.
        Regions may mix surfaces; select inside the object or use SAM3 for a mask.
        Retained frames expire after 60s. Actions revalidate the view and queried regions against fresh RGB-D.
        """
        return node.get_depth_region(rgbd_snapshot_id, x_min, y_min, x_max, y_max)

    @server.tool()
    def observe_scene() -> dict:
        """Get target positions, snapshot ID, dry_run and controller readiness.

        'valid' describes target perception only. execute_velocity does not require it.
        """
        return node.observe()

    @server.tool()
    def set_target(prompt: str) -> dict:
        """Stop motion and set SAM3's object description. Wait for new detections afterwards."""
        return node.set_target(prompt)

    @server.tool()
    async def execute_steering(snapshot_id: str, vx: float, vy: float,
                               yaw_rate: float, duration: float) -> dict:
        """Execute a bounded command, returning only after zero output resumes. Requires fresh perception."""
        return await node.execute(snapshot_id, vx, vy, yaw_rate, duration)

    @server.tool()
    async def execute_velocity(vx: float, vy: float, yaw_rate: float, duration: float,
                               rgbd_snapshot_id: str | None = None) -> dict:
        """Move without object tracking, at most 3 seconds, 0.5 m/s, 1.0 rad/s.

        No target, snapshot, SAM3 or TF needed. Positive vy means robot-left.
        For slowly left for 3 seconds: vx=0, vy=0.1, yaw_rate=0, duration=3.
        For visual tasks pass rgbd_snapshot_id from observe_rgbd (retained for 60 seconds).
        The bridge compares fresh RGB-D with that image and queried depth regions before starting.
        Actions last at most 1 second; revalidated image expiry stops the action.
        Each action consumes pending images; observe again after movement. No SAM3 needed.
        Execution requires a ready controller. Returns after zero output resumes.
        """
        return await node.execute_velocity(vx, vy, yaw_rate, duration, rgbd_snapshot_id)

    @server.tool()
    def stop() -> dict:
        """Immediately cancel the active command and invalidate pending observations."""
        return node.stop()

    return server


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--execute', action='store_true', help='Publish to controller; default is preview only')
    args, ros_args = parser.parse_known_args()
    # One bridge per domain; preview and execution must not race SAM3 prompts.
    lock_file = open(f'/tmp/g1-codex-ros-{os.getuid()}-{os.environ.get("ROS_DOMAIN_ID", "0")}.lock', 'w')
    fcntl.flock(lock_file, fcntl.LOCK_EX | fcntl.LOCK_NB)
    # rclpy's default SIGTERM handler only stops ROS, leaving the MCP stdin
    # server alive and its singleton lock held after the Docker client exits.
    def terminate(_signum, _frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, terminate)
    rclpy.init(args=ros_args, signal_handler_options=SignalHandlerOptions.NO)
    node = RosBridge(dry_run=not args.execute)
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        make_server(node).run(transport='stdio')
    finally:
        node.stop('disconnected')
        executor.shutdown()
        thread.join(timeout=2)
        node.destroy_node()
        rclpy.shutdown()
        lock_file.close()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass
