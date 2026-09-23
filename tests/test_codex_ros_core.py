import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from codex_ros_core import CommandSupervisor, heading_point


class SupervisorTests(unittest.TestCase):
    def setUp(self):
        self.now = 10.0
        self.core = CommandSupervisor(clock=lambda: self.now)

    def start(self, **overrides):
        args = dict(token=self.core.observe(), vx=0.1, vy=0., wz=0.2,
                    duration=0.5, scene_valid=True, ready=True)
        args.update(overrides)
        self.core.start(**args)

    def test_idle_execution_expiry_and_no_replay(self):
        self.assertEqual(self.core.tick(True, True), (0, 0, 0))
        self.start()
        self.assertEqual(self.core.tick(True, True), (0.1, 0, 0.2))
        self.now += 0.5
        self.assertEqual(self.core.tick(True, True), (0, 0, 0))
        self.assertEqual(self.core.reason, 'duration_elapsed')
        with self.assertRaises(ValueError):
            self.core.start(None, .1, 0, 0, .5, True, True)

    def test_delayed_reasoning_and_cancel_invalidate_snapshot(self):
        token = self.core.observe()
        self.now += 2
        with self.assertRaises(ValueError):
            self.core.start(token, .1, 0, 0, .5, True, True)
        token = self.core.observe()
        self.core.stop()
        with self.assertRaises(ValueError):
            self.core.start(token, .1, 0, 0, .5, True, True)

    def test_loss_and_manual_override_stop(self):
        for scene, ready, reason in [(False, True, 'perception_lost'), (True, False, 'controller_unavailable')]:
            self.start()
            self.assertEqual(self.core.tick(scene, ready), (0, 0, 0))
            self.assertEqual(self.core.reason, reason)

    def test_invalid_and_excessive_commands(self):
        for args in [dict(vx=float('nan')), dict(wz=float('inf')), dict(vx=.5, vy=.5),
                     dict(duration=-1), dict(duration=2), dict(wz=1.01), dict(ready=False),
                     dict(scene_valid=False)]:
            with self.subTest(args=args), self.assertRaises(ValueError):
                self.start(**args)
            self.assertEqual(self.core.tick(True, True), (0, 0, 0))

    def test_overlap_rejected(self):
        self.start()
        with self.assertRaises(ValueError):
            self.start()

    def test_velocity_runs_three_seconds_without_scene_and_expires(self):
        self.core.start_velocity(0, .1, 0, 3, True)
        self.assertEqual(self.core.tick(False, True), (0, .1, 0))
        self.now += 2.99
        self.assertEqual(self.core.tick(False, True), (0, .1, 0))
        self.now += .02
        self.assertEqual(self.core.tick(False, True), (0, 0, 0))
        self.assertEqual(self.core.reason, 'duration_elapsed')

    def test_velocity_requires_controller_and_obeys_limits(self):
        for vx, vy, wz, duration, ready in [(0, .1, 0, 3, False), (0, .1, 0, 3.1, True),
                (0, .51, 0, 1, True), (0, .1, float('nan'), 1, True), (0, .1, 0, 0, True)]:
            with self.assertRaises(ValueError):
                self.core.start_velocity(vx, vy, wz, duration, ready)
        self.core.start_velocity(0, .1, 0, 3, True)
        with self.assertRaises(ValueError):
            self.core.start_velocity(0, .1, 0, 3, True)
        self.assertEqual(self.core.tick(False, False), (0, 0, 0))
        self.assertEqual(self.core.reason, 'controller_unavailable')

    def test_yaw_rate_boundary_for_both_action_types(self):
        for guided in (False, True):
            for wz in (-1., -.5, .5, 1.):
                with self.subTest(guided=guided, wz=wz):
                    if guided:
                        self.start(vx=0, vy=0, wz=wz)
                    else:
                        self.core.start_velocity(0, 0, wz, .5, True)
                    self.assertEqual(self.core.tick(True, True), (0, 0, wz))
                    self.now += .5
                    self.assertEqual(self.core.tick(True, True), (0, 0, 0))
            for wz in (-1.0001, 1.0001):
                with self.subTest(guided=guided, wz=wz), self.assertRaises(ValueError):
                    if guided:
                        self.start(wz=wz)
                    else:
                        self.core.start_velocity(0, 0, wz, .5, True)
                self.assertEqual(self.core.tick(True, True), (0, 0, 0))

    def test_translation_speed_boundary_for_both_action_types(self):
        for guided in (False, True):
            for vx, vy in [(.5, 0), (-.5, 0), (0, .5), (0, -.5), (.3, .4)]:
                with self.subTest(guided=guided, vx=vx, vy=vy):
                    if guided:
                        self.start(vx=vx, vy=vy, wz=0)
                    else:
                        self.core.start_velocity(vx, vy, 0, .5, True)
                    self.assertEqual(self.core.tick(True, True), (vx, vy, 0))
                    self.now += .5
                    self.assertEqual(self.core.tick(True, True), (0, 0, 0))
            for vx, vy in [(.5001, 0), (0, -.5001), (.4, .4)]:
                with self.subTest(guided=guided, vx=vx, vy=vy), self.assertRaises(ValueError):
                    if guided:
                        self.start(vx=vx, vy=vy)
                    else:
                        self.core.start_velocity(vx, vy, 0, .5, True)
                self.assertEqual(self.core.tick(True, True), (0, 0, 0))

    def test_target_action_still_requires_scene_after_velocity_action(self):
        self.core.start_velocity(0, .1, 0, 3, True)
        self.core.stop()
        with self.assertRaises(ValueError):
            self.start(scene_valid=False)
        self.start()
        self.assertEqual(self.core.tick(False, True), (0, 0, 0))
        self.assertEqual(self.core.reason, 'perception_lost')

    def test_heading_frame_removes_yaw_but_retains_tilt(self):
        result = heading_point([1, 0, 0], [0, 0, math.sin(.7), math.cos(.7)])
        for a, b in zip(result, [1, 0, 0]):
            self.assertAlmostEqual(a, b)
        result = heading_point([1, 0, 0], [0, math.sin(.2), 0, math.cos(.2)])
        self.assertAlmostEqual(result[0], math.cos(.4))
        self.assertAlmostEqual(result[2], -math.sin(.4))


if __name__ == '__main__':
    unittest.main()
