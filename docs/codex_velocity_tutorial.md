# Walk close to a table using Codex and VELOCITY mode

This tutorial uses your existing Docker simulator and the existing
`config/policy/velocity` policy. It does not require the RoboGPC steering policy.

You type the task in Codex. Codex sees SAM3's table position, chooses a short
velocity command, waits for it to finish, observes again, and repeats. The ROS
bridge publishes the command at 50 Hz. Between actions, including while Codex
reasons, it publishes `[vx, vy, yaw_rate] = [0, 0, 0]`. The VELOCITY policy
continues running to produce joint commands.

```text
Your task in Codex: "Walk close to the table"
                       ↓
SAM3 position → Codex → MCP bridge → ROS velocity input → VELOCITY policy → G1
     ↑                                                                   |
     └──────────────── camera + depth + TF feedback ──────────────────────┘
```

You use the joystick to enter VELOCITY mode and to take control back. Codex
replaces the velocity input while it has control; it does not simulate button
presses or select robot modes.

## Relative motion without object tracking

For "walk left slowly for 3 seconds", use the new `execute_velocity` tool.
No target, snapshot ID, SAM3 detection, or camera TF is required. With the
controller ready and the bridge in execution mode, ask Codex:

> Use g1_ros execute_velocity to move left slowly for 3 seconds, with vx=0,
> vy=0.1, yaw_rate=0, duration=3. Do not set or track an object. Stop afterward.

This requests three continuous seconds of leftward velocity, followed by zero.
It does not split the action into perception-gated steps. The speed is a
command, not a promise of exact displacement. Controller readiness, joystick
takeover, stop/cancellation, and the command watchdog remain enforced.
Perception loss does not stop this explicitly tracking-free action.

Reload the MCP connection after updating the bridge so `execute_velocity`
appears in its tool list. The controller does not need restarting for this
Python-only update. The table-approach steps below use `execute_steering`, which
continues to require fresh target perception. For relative-only sessions,
SAM3 can be disabled with `sam3:=false` on the controller launch.

## 1. Know where commands run

- **Host** means your normal Linux terminal, outside Docker, where Codex runs.
- **Container** means a terminal entered with `docker exec -it g1-robogpc-sim bash`.
- **Codex prompt** means a normal chat message in Codex, not a shell command.

The container already contains the desk scene, SAM3 model and separate Python
environments. The updated controller is built in `/tmp/g1-codex-install`.
The `g1_ros` MCP connection is registered in preview mode initially.

## 2. Terminal A: run the camera simulator

If the desk simulator is already open, keep it and skip this step. Run only one
simulator. Otherwise open a host terminal and enter the container:

```bash
docker exec -it g1-robogpc-sim bash
```

Then, inside that terminal:

```bash
source /opt/ros/jazzy/setup.bash
source /root/Workspaces/g1_ros_ws/install/setup.bash
export LD_LIBRARY_PATH=/usr/local/lib:${LD_LIBRARY_PATH:-}
ros2 run g1_controller g1_mujoco_camera \
  -r g1 -s /opt/g1-scenes/desk/scene_29dof_desk.xml -i 0 -n lo
```

Leave it running. You should see the G1 and desk in the simulator. Use the
same joystick connection and simulator support/release procedure as your
existing manual VELOCITY tests.

## 3. Terminal B: launch VELOCITY, the camera interface, and SAM3

Stop any previous controller launch with Ctrl+C in its own terminal first.
Do not run two controllers against the same simulated robot.

Open another host terminal:

```bash
docker exec -it g1-robogpc-sim bash
```

Inside it:

```bash
source /opt/ros/jazzy/setup.bash
source /root/Workspaces/g1_ros_ws/install/setup.bash
source /tmp/g1-codex-install/setup.bash
export LD_LIBRARY_PATH=/usr/local/lib:${LD_LIBRARY_PATH:-}

ros2 launch g1_controller g1_controller_codex_velocity.launch.py \
  simulation:=true network:=lo rviz:=true \
  sam3_python:=/opt/g1-sam3-venv/bin/python \
  sam3_checkpoint:=/opt/g1-models/sam3.pt \
  sam3_prompt:=table
```

This new launch defaults to `steering_ros_enabled:=true`, `camera:=true`, and
`sam3:=true`. Despite the parameter name, it enables ROS commands for VELOCITY
mode here. It loads only Passive, FixStand, and Velocity. Wait for SAM3 model
loading to finish and check the camera/segmentation overlay in RViz.

## 4. Use the joystick to enter VELOCITY mode

1. Press **Start/Menu**: Passive → FixStand.
2. Wait at least **3 seconds** for the standing transition.
3. Center both sticks and press **A**: FixStand → Velocity.
4. Leave both sticks centered. Codex can now supply velocity commands.

Until an execution-enabled bridge submits an action, the velocity input is
zero. The policy still generates joint commands; zero velocity is not a
motor-off command.

If you move a stick, manual control latches. To hand control back, press **X**
to enter FixStand, wait 3 seconds, center the sticks, then press **A** again.
These X/A transitions belong to this dedicated launch profile.

## 5. In Codex: first verify perception without moving

Reload the `g1_ros` MCP connection, or restart Codex if the tools have not
appeared since registration. The connection launches the bridge itself;
do not start a second copy manually.

Send this as a Codex message:

> Use the g1_ros tools. Set the perception target to "table", then observe the
> scene. Do not execute any movement. Tell me whether the table is visible,
> its forward/left position and distance, whether the controller is ready,
> and whether the bridge is in preview mode.

Expected: a valid table position, `dry_run: true`, and a fresh observation.
The readiness field reports the actual controller heartbeat in both modes;
preview commands can be tested even when the controller is not ready.
If the table is not detected, inspect the overlay or try the description
"desk". Do not substitute the monitor when the intended target is the table.
For the first trial, use one visible table and a clear path.

## 6. On the host: switch the bridge from preview to execution

The initial installation intentionally sends commands only to a preview topic.
To let the bridge send commands to the controller, run on the **host**:

```bash
codex mcp add g1_ros -- \
  /home/wuyan/Workspaces/g1_ros_ws/src/g1_rsl/dependencies/g1_controller/scripts/codex_ros_docker.sh \
  --execute
```

Reload the MCP connection or restart Codex so the old preview process exits
and the new configuration takes effect. Changing this entry alone does not
change a server that is already running. A singleton lock prevents two bridge
processes from running in the same ROS domain.

Ask Codex to call `observe_scene` again, without moving. It must now report
`dry_run: false` and `controller_ready: true`. If readiness is false, check
step 4 and ensure Terminal B sourced the new temporary install.

## 7. In Codex: give the high-level task

For the first run, use an explicit interpretation of "close":

> Use g1_ros to walk close to the table. Treat "close" as about 0.8 metres of
> horizontal distance from the robot's torso to the detected table point.
> First verify dry_run is false and the controller is ready. Use forward speed
> at most 0.15 m/s and yaw rate at most 0.3 rad/s, with actions lasting at most
> 0.5 seconds. Observe again after every action, and refresh an expired
> observation before executing. Stop when within the target distance. If the
> target becomes ambiguous, perception is lost, or control is unavailable,
> stop and report the reason. Do not modify code, configuration, or robot mode.

Codex should set the SAM3 target, inspect the position, turn/advance in short
actions, and reassess. You will see pauses between movements because the
command is zero while it reasons. It should call `stop` when finished and
report what it observed. A command completing does not itself establish that
the robot moved or reached the table.

The position is SAM3's estimate from visible surface pixels. The 0.8 m target
is **not a measured clearance from the nearest table edge**. This prototype
does not perform obstacle avoidance. It requires a visible target, a clear
path, and supervision for the simulator trial.

After a successful first trial, a shorter prompt such as "Walk close to the
table using g1_ros and the same limits" can reuse this task context.

## 8. Stop or take back control

| Action | Result |
| --- | --- |
| Tell Codex "Stop" | Calls the stop tool once Codex processes the request |
| Move either steering stick | Immediately selects joystick input at the next policy tick; manual control stays latched |
| Press X | Leaves VELOCITY for FixStand |
| Press Back/View | Enters Passive (damping mode; not a standing hold) |
| X, wait 3 seconds, center sticks, A | Re-enters VELOCITY and allows ROS control again |

The bridge limits target-guided actions to 1 second and relative velocity
actions to 3 seconds, rejecting longer requests. If the bridge stops publishing, the controller selects zero
after its 200 ms watchdog expires. Physical stopping still depends on the
policy and simulated dynamics.

## Optional: watch commands in Terminal C

Open another container terminal and source ROS:

```bash
docker exec -it g1-robogpc-sim bash
source /opt/ros/jazzy/setup.bash
ros2 topic echo /codex/steering_command
```

`twist.linear.x` is forward m/s, `twist.linear.y` is leftward m/s, and
`twist.angular.z` is counterclockwise rad/s. All three return to zero between
actions. For preview mode, use `/codex/steering_preview` instead.
`/codex/bridge_status` reports cancellation/expiry reasons, and
`/codex/steering_ready` is the controller heartbeat.

## If something does not work

| Symptom | What to check |
| --- | --- |
| Codex cannot find g1_ros tools | Run `codex mcp get g1_ros` on the host; reload/restart Codex; ensure Docker is running |
| `dry_run: true` after enabling motion | The old preview process is still connected; reload the MCP connection |
| `controller_ready: false` | Enter VELOCITY with centered sticks; after manual takeover use X → wait → A; check the new controller install |
| `target_not_visible` | Check SAM3 overlay, target description, and camera view |
| `stale_perception` | SAM3 results exceeded 1.5 seconds of age; inspect GPU load and camera/SAM3 timing |
| TF error | The controller and robot-state publisher must be running; check `odom`, `pelvis`, `torso_link` |
| Snapshot expired | Ask Codex to observe again; commands deliberately do not reuse old decisions |
| Missing temporary install | Rebuild using the block below |
| Bridge lock error | Close the previous MCP connection before starting another bridge |

The launcher now cleans up its remote Docker process on normal exit or
termination. Older launcher versions could leave a preview bridge alive after
Codex disconnected, keeping `dry_run: true` and holding the singleton lock even
when the saved configuration had `--execute`. That requires removing the
verified orphaned bridge, not repeatedly changing the MCP configuration. Do not
kill the simulator/controller or delete the lock file to work around a live
lock holder. The host-side regression test is
`python3 tests/test_codex_ros_launcher.py -v` (requires Docker; uses domain 173).
| `Failed to find a free participant index for domain 0` | Older controller binary: the updated controller expands Unitree's loopback DDS search from slots 0–9 to 0–99. Source the rebuilt temporary install and restart the controller launch. SAM3/RViz staying alive does not mean the controller survived. |

Rebuild inside the container if needed (already done for this implementation):

```bash
source /opt/ros/jazzy/setup.bash
source /root/Workspaces/g1_ros_ws/install/setup.bash
cd /root/Workspaces/g1_ros_ws
colcon build --build-base /tmp/g1-codex-build \
  --install-base /tmp/g1-codex-install --packages-select g1_controller \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
```

Controller compilation and isolated ROS/MCP tests are verification of the
integration. An end-to-end table-approach walking trial has not yet been run.
