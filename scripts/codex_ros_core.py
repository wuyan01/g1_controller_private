"""Bounded command lifecycle, independent of ROS and the MCP transport."""
import math
import time
import uuid


class CommandSupervisor:
    def __init__(self, clock=time.monotonic, max_speed=0.5, max_yaw_rate=1.0,
                 max_duration=1.0, max_scene_age=1.5, max_velocity_duration=3.0):
        self.clock = clock
        self.max_speed = max_speed
        self.max_yaw_rate = max_yaw_rate
        self.max_duration = max_duration
        self.max_scene_age = max_scene_age
        self.max_velocity_duration = max_velocity_duration
        self.requires_scene = True
        for value in (max_speed, max_yaw_rate, max_duration, max_scene_age, max_velocity_duration):
            if not math.isfinite(value) or value <= 0:
                raise ValueError('Limits must be finite and positive')
        self.token = None
        self.observed = 0.0
        self.deadline = 0.0
        self.velocity = (0.0, 0.0, 0.0)
        self.reason = 'idle'
        self.active = False

    def observe(self):
        self.token = uuid.uuid4().hex
        self.observed = self.clock()
        return self.token

    def stop(self, reason='stopped'):
        self.active = False
        self.velocity = (0.0, 0.0, 0.0)
        self.token = None
        self.reason = reason

    def start(self, token, vx, vy, wz, duration, scene_valid, ready):
        if self.active:
            raise ValueError('An action is already executing')
        if not self.token or token != self.token or self.clock()-self.observed > self.max_scene_age:
            raise ValueError('Observe the scene again: snapshot is expired or already consumed')
        self.token = None  # Even rejected attempts cannot replay an observation.
        self._begin(vx, vy, wz, duration, ready, scene_valid, True, self.max_duration)

    def start_velocity(self, vx, vy, wz, duration, ready):
        """Explicit relative motion: requires controller readiness, not a target."""
        if self.active:
            raise ValueError('An action is already executing')
        self.token = None
        self._begin(vx, vy, wz, duration, ready, True, False, self.max_velocity_duration)

    def _begin(self, vx, vy, wz, duration, ready, scene_valid, requires_scene, limit):
        if not all(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)
                   for v in (vx, vy, wz, duration)):
            raise ValueError('Commands must be finite numbers')
        if math.hypot(vx, vy) > self.max_speed or abs(wz) > self.max_yaw_rate:
            raise ValueError('Command exceeds velocity limits')
        if not 0 < duration <= limit:
            raise ValueError('Command exceeds duration limit')
        if not ready:
            raise ValueError('A ready controller is required')
        if requires_scene and not scene_valid:
            raise ValueError('Fresh target perception is required for target-guided steering')
        self.requires_scene = requires_scene
        self.velocity = (vx, vy, wz)
        self.deadline = self.clock() + duration
        self.active = True
        self.reason = 'executing'

    def tick(self, scene_valid, ready):
        if self.active:
            if self.requires_scene and not scene_valid:
                self.stop('perception_lost')
            elif not ready:
                self.stop('controller_unavailable')
            elif self.clock() >= self.deadline:
                self.stop('duration_elapsed')
        return self.velocity if self.active else (0.0, 0.0, 0.0)


def rotate_point(point, quaternion):
    """Rotate a point with a normalized quaternion."""
    x, y, z, w = quaternion
    norm = math.sqrt(x*x+y*y+z*z+w*w)
    if not math.isfinite(norm) or norm < 1e-9 or not all(map(math.isfinite, point)):
        raise ValueError('Invalid point or orientation')
    x, y, z, w = (v/norm for v in (x, y, z, w))
    px, py, pz = point
    wx = (1-2*(y*y+z*z))*px + 2*(x*y-z*w)*py + 2*(x*z+y*w)*pz
    wy = 2*(x*y+z*w)*px + (1-2*(x*x+z*z))*py + 2*(y*z-x*w)*pz
    wz = 2*(x*z-y*w)*px + 2*(y*z+x*w)*py + (1-2*(x*x+y*y))*pz
    return [wx, wy, wz]


def heading_point(point, quaternion):
    """Rotate a control-body-relative point into its yaw-only frame."""
    wx, wy, wz = rotate_point(point, quaternion)
    norm = math.sqrt(sum(v*v for v in quaternion))
    x, y, z, w = (v/norm for v in quaternion)
    yaw = math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))
    return [math.cos(yaw)*wx+math.sin(yaw)*wy, -math.sin(yaw)*wx+math.cos(yaw)*wy, wz]
