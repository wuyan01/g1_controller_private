# Codex ↔ ROS steering bridge

For the step-by-step **VELOCITY mode** workflow, start with
[the table-approach tutorial](codex_velocity_tutorial.md). The RoboGPC launch
instructions below remain an alternative; VELOCITY now accepts the same bridge.

The bridge exposes `observe_rgbd`, `get_depth_region`, `set_target`, `observe_scene`,
`execute_steering`, `execute_velocity`, and `stop`
through MCP stdio. A separate ROS executor publishes at 50 Hz. Codex calls are
outside the policy thread. Between bounded actions the command is zero; each
execution tool call waits until the command window has ended before returning.

## Current setup

The `g1_ros` MCP entry is registered with Codex using
`scripts/codex_ros_docker.sh`, **in preview mode**. The wrapper starts the server
inside `g1-robogpc-sim` using `/opt/g1-codex-venv/bin/python`.
Reload the MCP connection/restart Codex if the seven tools do not appear in the
current task. Only one bridge per ROS domain can run in this container.

The modified controller was built into `/tmp/g1-codex-install` inside the
container. This is a separate install; the normal workspace controller binary
has not been replaced. The temporary install is lost when the container is
recreated. To rebuild into that location:

```bash
# Inside g1-robogpc-sim
source /opt/ros/jazzy/setup.bash
source /root/Workspaces/g1_ros_ws/install/setup.bash
cd /root/Workspaces/g1_ros_ws
colcon build --build-base /tmp/g1-codex-build \
  --install-base /tmp/g1-codex-install --packages-select g1_controller \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## Preview test

For direct RGB-D perception without SAM3, see [on-demand RGB-D](codex_rgbd.md).
`execute_velocity` also accepts an optional `rgbd_snapshot_id`; supplying it
revalidates the retained decision image against fresh RGB-D and limits the action to one second. Omitting it keeps
relative motion independent of perception.

Start the camera simulator and SAM3/controller stack using the existing setup.
The bridge expects SAM3 localization in `pelvis`, plus timestamped TF from
`odom` to `torso_link` and from `torso_link` to `pelvis`. These frames can be
configured via ROS parameters `world_frame`, `control_frame`, and `base_frame`.
The policy uses **torso heading**; the bridge converts pelvis detections into
torso-relative coordinates and then removes torso roll/pitch using TF at image
capture time. The `heading` command frame is a semantic label, not a TF frame.

An example Codex prompt:

> Use g1_ros in preview mode. Set the target to monitor. Observe the scene;
> if the target is visible with fresh data, preview forward velocity 0.1 m/s
> for 0.3 seconds, then stop and report the result. Do not enable execution.

In the container, inspect the preview with:

```bash
source /opt/ros/jazzy/setup.bash
ros2 topic echo /codex/steering_preview
```

Without a fresh detection/TF, target-guided `execute_steering` actions are rejected.
`execute_velocity` supports relative motion without a target, SAM3, TF, or a
snapshot ID. For example, `vx=0, vy=0.1, yaw_rate=0, duration=3` commands slow
leftward motion for three seconds. Both tools require a ready controller in
execution mode. If reasoning about a target takes too long, observe again
before submitting a target-guided command.
No synthetic perception is injected into the normal ROS domain by the tests.

## Enable simulator motion

Use one controller process. Restart the steering controller with the newly
built overlay and ROS steering enabled (keep your normal camera/SAM3 options):

```bash
# Inside the container; simulator runs separately as usual.
source /opt/ros/jazzy/setup.bash
source /root/Workspaces/g1_ros_ws/install/setup.bash
source /tmp/g1-codex-install/setup.bash
ros2 launch g1_controller g1_controller_robogpc_steering.launch.py \
  simulation:=true steering_ros_enabled:=true camera:=true sam3:=true \
  sam3_python:=/opt/g1-sam3-venv/bin/python \
  sam3_checkpoint:=/opt/g1-models/sam3.pt sam3_prompt:=monitor
```

Enter FixStand with Start, allow the standing transition to finish, then enter
RoboGPC steering with LB held for two seconds + D-pad Up. The bridge cannot
switch the robot's FSM. `/codex/steering_ready` must be true and recent.

To change the Codex entry from preview to execution, run on the host and reload
the MCP connection (close the previous server first):

```bash
codex mcp add g1_ros -- \
  /home/wuyan/Workspaces/g1_ros_ws/src/g1_rsl/dependencies/g1_controller/scripts/codex_ros_docker.sh \
  --execute
```

Omit `--execute` to restore preview. With ROS steering disabled, the existing
joystick behavior is unchanged. With ROS steering enabled, moving either
steering stick beyond the deadzone latches joystick control; releasing it does
not resume Codex commands. Exit and re-enter steering mode to re-enable ROS.
The existing Back-to-Passive transition remains available.

## Command behavior

| Property | Initial value |
| --- | --- |
| Translational speed norm | At most 0.5 m/s |
| Yaw rate | At most 1.0 rad/s (VELOCITY policy range: -1.0 to +1.0) |
| Target-guided action duration | At most 1 second |
| Relative velocity action duration | At most 3 seconds |
| SAM3 snapshot / live RGB-D age | At most 1.5 seconds |
| RGB-D reasoning window | At most 60 seconds; fresh scene comparison before action |
| Controller readiness heartbeat age | Less than 250 ms |
| Controller command watchdog | 200 ms, monotonic clock |
| Command publishing | 50 Hz, steady clock |

Each observation ID is single-use and tied to image capture age. New targets,
stop, cancellation, expiry, and loss of perception/readiness invalidate the
pending target-guided action. Relative velocity actions do not depend on
perception; cancellation, expiry, and controller loss still stop them.
Commands use forward/left/CCW signs, m/s and rad/s. Concurrent
actions are rejected. The bridge publishes `TwistStamped` on
`/codex/steering_command` only with `--execute`, otherwise on the preview topic.
`/codex/bridge_status` reports state. A `Bool(true)` on `/codex/stop` cancels
the current action; this is a command cancellation, not a latched emergency stop.

In VELOCITY mode, zero is `[vx, vy, yaw_rate] = [0, 0, 0]` and commands enter
the observation at the next policy tick (configured at 50 Hz). In RoboGPC
steering mode, zero resets the desired heading and produces `[0, 0, 1, 0]`;
that stateful policy consumes new commands at its next replanning tick
(nominally 100 ms). Neither mode zeroes joint actions or forces a stand FSM.
Policy history and physical inertia mean motion does not stop instantly.

SAM3 chooses the highest-confidence matching detection. Instance IDs are not
persistent tracking identities. Use a scene with one unambiguous target for
the first trial. This bridge does not provide collision avoidance, task-success
verification, or a continuous visual servo controller. Tool success reports
the command lifecycle, not confirmed robot displacement. Closed-loop walking
and concurrent SAM3/policy GPU timing still require a simulator trial.

## Dependencies and verification

The bridge uses ROS Jazzy and the Python MCP SDK (tested with `mcp==1.30.0`).
For a new container, create an isolated environment without modifying SAM3:

```bash
python3 -m venv --without-pip --system-site-packages /opt/g1-codex-venv
python3 -m pip --python /opt/g1-codex-venv/bin/python install \
  -r scripts/requirements-codex-ros.txt
```

Run from this repository inside the container:

```bash
source /opt/ros/jazzy/setup.bash
python3 -m unittest discover -s tests -p test_codex_ros_core.py -v
ROS_DOMAIN_ID=173 /opt/g1-codex-venv/bin/python tests/test_codex_ros_bridge.py -v
g++ -std=c++17 -pthread -Iinclude tests/test_external_steering.cpp \
  -o /tmp/test_external_steering
/tmp/test_external_steering
```

The integration tests use actual DDS topics, synthetic perception/TF, and MCP
stdio in domain 173. They cover preview isolation, command expiry, replay
rejection, cancellation, target loss, and controller loss. The C++ test covers
watchdog expiry, mode reentry, joystick takeover, limits, and neutral heading.

Codex MCP connection reference:
https://learn.chatgpt.com/docs/extend/mcp?surface=cli
