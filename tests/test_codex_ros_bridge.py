"""Run in a separate ROS_DOMAIN_ID; exercises real DDS and MCP stdio."""
import asyncio
import base64
import json
import math
import os
from pathlib import Path
import sys
import threading
import time
import unittest

import rclpy
from rclpy.executors import SingleThreadedExecutor
from geometry_msgs.msg import TransformStamped, TwistStamped
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
from std_msgs.msg import Bool
from vision_msgs.msg import Detection3DArray, Detection3D, ObjectHypothesisWithPose
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from mcp import ClientSession, StdioServerParameters
from mcp.client.stdio import stdio_client

SCRIPTS = Path(__file__).resolve().parents[1] / 'scripts'
sys.path.insert(0, str(SCRIPTS))
from codex_ros_bridge import RosBridge


class BridgeTests(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        rclpy.init()
        self.fixture = rclpy.create_node('codex_test_fixture')
        self.pub = self.fixture.create_publisher(Detection3DArray, '/sam3/detections_3d', 1)
        self.ready_pub = self.fixture.create_publisher(Bool, '/codex/steering_ready', 1)
        self.camera_enabled = False
        self.cv = CvBridge()
        self.rgb_pub = self.fixture.create_publisher(Image, '/camera/color/image_raw', 1)
        self.depth_pub = self.fixture.create_publisher(Image, '/camera/aligned_depth_to_color/image_raw', 1)
        self.info_pub = self.fixture.create_publisher(CameraInfo, '/camera/aligned_depth_to_color/camera_info', 1)
        self.tf = StaticTransformBroadcaster(self.fixture)
        transform = TransformStamped()
        transform.header.stamp = self.fixture.get_clock().now().to_msg()
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'pelvis'
        transform.transform.rotation.w = 1.0
        self.tf.sendTransform(transform)
        torso = TransformStamped()
        torso.header.stamp = transform.header.stamp
        torso.header.frame_id = 'pelvis'
        torso.child_frame_id = 'torso_link'
        torso.transform.translation.x = .1
        torso.transform.rotation.z = math.sqrt(.5)
        torso.transform.rotation.w = math.sqrt(.5)
        self.tf.sendTransform([transform, torso])
        self.visible = True
        self.ready = True
        self.preview = []
        self.commands = []
        self.subs = [self.fixture.create_subscription(TwistStamped, topic, sink.append, 10)
                     for topic, sink in [('/codex/steering_preview', self.preview),
                                         ('/codex/steering_command', self.commands)]]
        self.fixture.create_timer(.025, self.frame)
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.fixture)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()

    def frame(self):
        msg = Detection3DArray()
        msg.header.stamp = self.fixture.get_clock().now().to_msg()
        msg.header.frame_id = 'pelvis'
        if self.visible:
            detection = Detection3D(id='1')
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = 'chair'
            hypothesis.hypothesis.score = .9
            hypothesis.pose.pose.position.x = 2.0
            detection.results = [hypothesis]
            msg.detections = [detection]
        self.pub.publish(msg)
        self.ready_pub.publish(Bool(data=self.ready))
        if self.camera_enabled:
            rgb = self.cv.cv2_to_imgmsg(np.full((6, 8, 3), 127, np.uint8), 'rgb8')
            depth = self.cv.cv2_to_imgmsg(np.full((6, 8), 2000, np.uint16), '16UC1')
            info = CameraInfo(width=8, height=6, distortion_model='plumb_bob', d=[0.]*5)
            info.k = [100., 0., 3.5, 0., 100., 2.5, 0., 0., 1.]
            for image_msg, pub in ((rgb, self.rgb_pub), (depth, self.depth_pub), (info, self.info_pub)):
                image_msg.header.stamp = msg.header.stamp
                image_msg.header.frame_id = 'camera_optical'
                pub.publish(image_msg)

    async def wait(self, predicate, timeout=5):
        deadline = time.monotonic()+timeout
        while not predicate() and time.monotonic() < deadline:
            await asyncio.sleep(.025)
        self.assertTrue(predicate())

    async def asyncTearDown(self):
        self.executor.shutdown()
        self.thread.join(timeout=2)
        self.fixture.destroy_node()
        rclpy.shutdown()

    async def test_stdio_tools_publish_preview_and_finish_at_zero(self):
        params = StdioServerParameters(command=sys.executable,
                    args=[str(SCRIPTS / 'codex_ros_bridge.py')], env=dict(os.environ))
        async with stdio_client(params) as (reader, writer):
            async with ClientSession(reader, writer) as session:
                await session.initialize()
                tools = await session.list_tools()
                self.assertEqual({t.name for t in tools.tools},
                                 {'observe_scene', 'observe_rgbd', 'get_depth_region', 'set_target',
                                  'execute_steering', 'execute_velocity', 'stop'})
                result = await session.call_tool('set_target', {'prompt': 'chair'})
                self.assertFalse(result.isError)
                for _ in range(100):
                    result = await session.call_tool('observe_scene', {})
                    scene = json.loads(result.content[0].text)
                    if scene['valid']:
                        break
                    await asyncio.sleep(.05)
                self.assertTrue(scene['valid'], scene)
                self.assertAlmostEqual(scene['selected']['position'][0], 0., places=5)
                self.assertAlmostEqual(scene['selected']['position'][1], -1.9, places=5)
                result = await session.call_tool('execute_steering', dict(
                    snapshot_id=scene['snapshot_id'], vx=.1, vy=0., yaw_rate=0., duration=.2))
                self.assertFalse(result.isError, result)
                self.assertEqual(json.loads(result.content[0].text)['status'], 'duration_elapsed')
                await self.wait(lambda: self.preview and self.preview[-1].twist.linear.x == 0.)
                self.assertTrue(any(m.twist.linear.x == .1 for m in self.preview))
                self.assertFalse(self.commands)  # Preview never reaches controller topic.
                replay = await session.call_tool('execute_steering', dict(
                    snapshot_id=scene['snapshot_id'], vx=.1, vy=0., yaw_rate=0., duration=.2))
                self.assertTrue(replay.isError)
                await session.call_tool('stop', {})

    async def test_stdio_relative_velocity_three_seconds_without_target(self):
        self.visible = False
        params = StdioServerParameters(command=sys.executable,
                    args=[str(SCRIPTS / 'codex_ros_bridge.py'), '--execute'], env=dict(os.environ))
        async with stdio_client(params) as (reader, writer):
            async with ClientSession(reader, writer) as session:
                await session.initialize()
                for _ in range(100):
                    result = await session.call_tool('observe_scene', {})
                    scene = json.loads(result.content[0].text)
                    if scene['controller_ready']:
                        break
                    await asyncio.sleep(.05)
                self.assertFalse(scene['valid'])
                self.assertIsNone(scene['target'])
                self.assertTrue(scene['controller_ready'])
                before = time.monotonic()
                result = await session.call_tool('execute_velocity', dict(
                    vx=0., vy=.1, yaw_rate=0., duration=3.))
                self.assertFalse(result.isError, result)
                outcome = json.loads(result.content[0].text)
                self.assertFalse(outcome['dry_run'])
                self.assertFalse(outcome['requires_scene'])
                self.assertEqual(outcome['status'], 'duration_elapsed')
                self.assertGreaterEqual(time.monotonic()-before, 3.)
                self.assertTrue(any(m.twist.linear.y == .1 for m in self.commands))
                await self.wait(lambda: self.commands and self.commands[-1].twist.linear.y == 0.)

    async def test_rgbd_image_and_depth_over_mcp_without_sam3(self):
        self.visible = False
        self.camera_enabled = True
        params = StdioServerParameters(command=sys.executable,
                    args=[str(SCRIPTS / 'codex_ros_bridge.py')], env=dict(os.environ))
        async with stdio_client(params) as (reader, writer):
            async with ClientSession(reader, writer) as session:
                await session.initialize()
                for _ in range(100):
                    result = await session.call_tool('observe_rgbd', {'include_depth_preview': True})
                    if not result.isError:
                        break
                    await asyncio.sleep(.05)
                self.assertFalse(result.isError, result)
                metadata = json.loads(result.content[0].text)
                self.assertEqual([c.type for c in result.content], ['text', 'image', 'image'])
                rgb = cv2.imdecode(np.frombuffer(base64.b64decode(result.content[1].data), np.uint8), cv2.IMREAD_COLOR)
                self.assertEqual(rgb.shape, (6, 8, 3))
                result = await session.call_tool('get_depth_region', dict(
                    rgbd_snapshot_id=metadata['rgbd_snapshot_id'], x_min=2, y_min=1, x_max=6, y_max=5))
                self.assertFalse(result.isError, result)
                region = json.loads(result.content[0].text)
                self.assertEqual(region['depth_median_m'], 2.)
                np.testing.assert_allclose(region['point_optical_m'], [0, 0, 2])
                self.assertIn('heading_unavailable', region)  # TF absence does not prevent RGB-D use.
                self.assertFalse(self.commands)  # Observation never emits live movement.

    async def test_visual_velocity_uses_rgbd_freshness_without_sam3(self):
        self.camera_enabled = True
        self.visible = False
        node = RosBridge(dry_run=False)
        self.executor.add_node(node)
        try:
            await self.wait(lambda: node.controller_ready() and
                            all(node.rgbd.frames[key] for key in ('rgb', 'depth', 'info')))
            token, decision = node.rgbd.capture()
            # Simulate model reasoning taking five seconds while new frames keep arriving.
            for key in ('rgb', 'depth', 'info'):
                decision[key].header.stamp.sec -= 5
            decision['_received_at'] -= 5
            self.assertFalse(node.rgbd.metadata(token, decision)['fresh'])
            await asyncio.sleep(.1)
            with self.assertRaises(ValueError):
                await node.execute_velocity(.1, 0., 0., 1.1, token)
            result = await node.execute_velocity(.1, 0., 0., .15, token)
            self.assertEqual(result['status'], 'duration_elapsed')
            self.assertEqual(result['rgbd_snapshot_id'], token)
            with self.assertRaises(ValueError):
                await node.execute_velocity(.1, 0., 0., .1, token)
            self.assertIsNone(node.prompt)
            token, _ = node.rgbd.capture()
            task = asyncio.create_task(node.execute_velocity(.1, 0., 0., 1., token))
            await self.wait(lambda: node.supervisor.active)
            # A clock jump or image age over the freshness bound cancels visual actions.
            node.rgbd.ros_now = lambda: node.get_clock().now().nanoseconds + 2_000_000_000
            result = await asyncio.wait_for(task, 2)
            self.assertEqual(result['status'], 'perception_lost')
            with self.assertRaises(ValueError):
                await node.execute_velocity(.1, 0., 0., .1, token)
            # Relative commands remain independent of the unavailable camera data.
            result = await node.execute_velocity(0., .1, 0., .1)
            self.assertEqual(result['status'], 'duration_elapsed')
        finally:
            node.stop()
            self.executor.remove_node(node)
            node.destroy_node()

    async def test_stop_during_visual_revalidation_prevents_late_command(self):
        self.camera_enabled = True
        node = RosBridge(dry_run=False)
        self.executor.add_node(node)
        entered, release = threading.Event(), threading.Event()
        try:
            await self.wait(lambda: node.controller_ready() and
                            all(node.rgbd.frames[key] for key in ('rgb', 'depth', 'info')))
            token, _ = node.rgbd.capture()
            revalidate = node.rgbd.revalidate
            def slow_revalidation(token):
                messages = revalidate(token)
                entered.set()
                release.wait(timeout=2)
                return messages
            node.rgbd.revalidate = slow_revalidation
            task = asyncio.create_task(node.execute_velocity(.1, 0., 0., .2, token))
            await self.wait(entered.is_set)
            node.stop()
            release.set()
            with self.assertRaisesRegex(ValueError, 'Control state changed'):
                await task
            self.assertFalse(node.supervisor.active)
            self.assertFalse(any(m.twist.linear.x for m in self.commands))
        finally:
            release.set()
            node.stop()
            self.executor.remove_node(node)
            node.destroy_node()

    async def test_relative_velocity_cancellation_without_target(self):
        node = RosBridge(dry_run=False)
        self.executor.add_node(node)
        try:
            await self.wait(node.controller_ready)
            task = asyncio.create_task(node.execute_velocity(0., .1, 0., 3.))
            await self.wait(lambda: node.supervisor.active)
            task.cancel()
            with self.assertRaises(asyncio.CancelledError):
                await task
            self.assertFalse(node.supervisor.active)
            self.assertEqual(node.supervisor.velocity, (0, 0, 0))
        finally:
            node.stop()
            self.executor.remove_node(node)
            node.destroy_node()

    async def test_cancellation_perception_loss_and_controller_loss(self):
        node = RosBridge(dry_run=False)
        self.executor.add_node(node)
        try:
            node.set_target('chair')
            await self.wait(lambda: node.observe()['valid'] and node.controller_ready())
            async def begin():
                scene = node.observe()
                task = asyncio.create_task(node.execute(scene['snapshot_id'], .1, 0., 0., .8))
                await self.wait(lambda: node.supervisor.active)
                return task
            task = await begin()
            task.cancel()
            with self.assertRaises(asyncio.CancelledError):
                await task
            self.assertFalse(node.supervisor.active)
            self.assertEqual(node.supervisor.velocity, (0, 0, 0))
            task = await begin()
            self.visible = False
            result = await asyncio.wait_for(task, 2)
            self.assertEqual(result['status'], 'perception_lost')
            self.visible = True
            await self.wait(lambda: node.observe()['valid'])
            task = await begin()
            self.ready = False
            result = await asyncio.wait_for(task, 2)
            self.assertEqual(result['status'], 'controller_unavailable')
            await self.wait(lambda: self.commands and self.commands[-1].twist.linear.x == 0)
        finally:
            node.stop()
            self.executor.remove_node(node)
            node.destroy_node()


if __name__ == '__main__':
    if os.environ.get('ROS_DOMAIN_ID') != '173':
        raise SystemExit('Use ROS_DOMAIN_ID=173 to isolate synthetic fixtures from the robot')
    unittest.main()
