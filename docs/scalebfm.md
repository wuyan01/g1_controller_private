# ScaleBFM ONNX in g1_controller

This opt-in deployment adds the native ScaleBFM M and XL actor to the existing
ROS 2 G1 controller. Both models were exported locally to ONNX from the released
checkpoints. The upstream local exporter supplied TensorRT, not these ONNX
bundles. This deployment does not include the IsaacLab bounded wrist servo.

The implementation is installed in the existing `g1-robogpc-sim` container at
`/root/Workspaces/g1_ros_ws/src/g1_rsl/dependencies/g1_controller`, backed by the
host checkout `/home/wuyan/Workspaces/g1_ros_ws/src/g1_rsl/dependencies/g1_controller`.
The current build uses the separate `install_scalebfm_timing_v4_20260923` overlay. Existing launch
profiles and policy assets remain unchanged. Only a new FSM include in
`src/main.cpp` and an OpenSSL link dependency in `CMakeLists.txt` touch shared files.

## Current validation and limits

- M and XL each pass 24 PyTorch/ONNX/C++ numerical cases across all eight exported
  modes. Maximum PD target errors are 3.31e-6 rad and 2.35e-6 rad, respectively.
- Each model passes 563 standalone C++ mapping, history, frame, FK, reset, gain,
  integrity and output-guard checks. The complete ROS package builds on the
  existing Jazzy/x86_64 container using ONNX Runtime 1.22 CPU inference.
- M and XL each stood for 10 seconds in an offline copy of the target MuJoCo
  plant without a fall, nonfinite state, target guard or torque saturation.
- M passes the latest exact-desk tests at both .10 and .30 m/s: five seconds
  standing, four seconds forward, then three seconds stopping. At full forward
  it moves 1.031 m during the command and another .157 m while stopping; final
  stopped horizontal speed is .000295 m/s. There are no falls, actual joint-range
  violations, remaining guard failures or torque saturation. XL has not run this
  latest walking protocol.
- Two adapter checks incorrectly treated virtual PD target angles/rates as
  physical joint angles/speeds. Both have been corrected; see the sections below.

These are offline core/plant checks. **The complete ROS/DDS FSM transition,
real-time scheduling, free-space wrists, manipulation and physical hardware are
not qualified.** No physical robot or existing simulator was commanded during
implementation. The target plant is 29 DOF, 35.112142 kg, with no articulated
Dex3. IsaacLab desk qualification does not transfer to it.

Evidence in the source workspace:
`/home/wuyan/Documents/code/agentic-protomotions/outputs/scalebfm_g1_controller_20260923/`
contains `core_validation.json`, `ros_build.log`, `OFFLINE_VALIDATION.md`, and
per-trial reports with hashes, timing and complete control traces. The adjacent
`deployment/scalebfm_g1_controller/validation` directory contains the plant audit
and a reproducible offline harness.
The walking correction and its regression results are in
`outputs/scalebfm_rate_guard_20260923/`, including `VALIDATION.md`,
`core_validation.json`, `summary.json` and `ros_build.log`.
The latest timing correction, live fault analysis and v4 build evidence are in
`outputs/scalebfm_timing_guard_20260923/`.

## Launch a simulation trial

The simulator and controller are separate processes. Keep the existing manual
support/release procedure. Do not run a second controller against a running
experiment: this stack uses Unitree DDS domain **0**, independent of
`ROS_DOMAIN_ID`. Its existing main can shut down another low-command publisher.
The dedicated ScaleBFM launch accepts only `simulation:=true network:=lo`.

Terminal A, if the simulator is not already running:

```bash
docker exec -it g1-robogpc-sim bash
source /opt/ros/jazzy/setup.bash
source /root/Workspaces/g1_ros_ws/install/setup.bash
export LD_LIBRARY_PATH=/usr/local/lib:${LD_LIBRARY_PATH:-}
ros2 run g1_controller g1_mujoco_camera \
  -r g1 -s /opt/g1-scenes/desk/scene_29dof_desk.xml -i 0 -n lo
```

Terminal B, after ending any previous controller in its own terminal:

```bash
docker exec -it g1-robogpc-sim bash
source /opt/ros/jazzy/setup.bash
source /root/Workspaces/g1_ros_ws/install/setup.bash
source /root/Workspaces/g1_ros_ws/install_scalebfm_timing_v4_20260923/setup.bash
export LD_LIBRARY_PATH=/usr/local/lib:${LD_LIBRARY_PATH:-}
ros2 launch g1_controller g1_controller_scalebfm.launch.py \
  scalebfm_model:=m simulation:=true network:=lo \
  steering_ros_enabled:=false camera:=true rviz:=true sam3:=false
```

Use `scalebfm_model:=xl` for XL. Press Start/Menu for Passive -> FixStand,
wait at least three seconds with the existing simulator support, then center
the sticks and press A for ScaleBFM. X returns to FixStand; Back enters Passive.
The new FixStand profile uses the training nominal pose with the existing
FixStand gains; activation switches to the exact training policy gains.

Static PD standing alone tipped forward in the unsupported offline plant, with
both training and existing FixStand gains. The simulator's existing elastic
band assistance therefore matters for entry. The successful grounded trial
used actor standing before a policy-only reset; it does not validate the manual
FixStand transition. Enter only from a settled, near-ground pose. Entry checks
bound pose, velocity and geometric ankle height on a flat floor at odom z=0;
they cannot prove foot contact or unassisted balance.

Start with zero-command standing, then bounded low-speed navigation and stopping.
This actor's zero velocity holds its integrated root reference, so it is not
an immediate physical stop. Start new qualification with the existing support
procedure and use Back/Passive on a guard fault; do not treat the diagnostic
pass as permission for autonomous tabletop acquisition.

## Existing Codex ROS bridge

This ROS deployment can consume the existing `/codex/steering_command` protocol;
it is separate from the IsaacLab worker's `humanoid_skills` MCP server and
Humanoid Studio HTTP launcher. Their reset, scoring, recording and bounded
physics-pausing APIs are not part of this ROS FSM. Here physics runs continuously
while the bridge bounds command duration.

After manual standing/navigation validation, use the same controller launch with
`steering_ros_enabled:=true`. The existing `g1_ros` bridge can provide relative
velocity commands within **0.3 m/s XY norm and 0.6 rad/s yaw**. Larger commands
fault this controller. Its bridge defaults allow larger commands, so retain
these tighter ScaleBFM limits in the test instructions.

For target perception, configure the bridge's `control_frame` to `pelvis`:
ScaleBFM uses pelvis heading, whereas the existing bridge defaults to torso
heading. A host-side stdio launch for a new preview connection is:

```bash
/home/wuyan/Workspaces/g1_ros_ws/src/g1_rsl/dependencies/g1_controller/scripts/codex_ros_docker.sh \
  --ros-args -p control_frame:=pelvis
```

The existing wrapper forwards these ROS arguments. Use its `--execute` option
only when intentionally enabling the simulator trial. Keep one bridge in that
ROS domain and reuse the existing MCP setup instructions in `codex_ros.md`.
Moving a joystick latches manual takeover until X -> A re-entry. The command
watchdog becomes zero after 200 ms without updates; reference catch-up can still
move the body after that. No existing bridge registration was changed here.

## Controller contract

- Nine ONNX inputs include three measured-state/raw-action history frames and
  six future 14-body references. Quaternions are WXYZ. Gyro is pelvis-local.
  Joint mapping is by name, validated against the robot SDK's 29 motor IDs.
- The graph owns normalization, training FK features, mode masks and action
  scaling. Outputs are absolute PD positions; the adapter never scales twice.
  Only raw actor actions feed back into history. Reset repeats measured state,
  zeros action history and initializes references from measured FK.
- The FSM reads locked native LowState pelvis IMU and fresh sport-mode pelvis
  position. It bypasses the generic G1 articulation's torso-orientation override
  and the existing ROS IMU quaternion-order bug. External localization frames
  are explicitly unsupported until their IMU-heading alignment is implemented.
- Policy inference is nominally 50 Hz. The existing FSM publishes held PD targets at 1 kHz;
  the photo-desk model runs 1 kHz physics (the older plain G1 test used 500 Hz).
  Computation over 20 ms, policy sample gaps over 40 ms, accumulated wake delay
  over 50 ms in a rolling second, stale state over 200 ms or target age over
  100 ms faults to Passive. Isolated wake delays over 5 ms produce notices.
  Each next wake is scheduled 20 ms after the actual start, so delayed ticks
  never cause compressed catch-up inference. History contains actual samples;
  reference integration retains the trained 20 ms step.
  This is a guarded ordinary thread, not a hard real-time scheduling guarantee.
- The core limits XY commands to .3 m/s, yaw to .6 rad/s, reference acceleration
  vector magnitude to 1 m/s² and current root lead to .18 m. It rejects nonfinite
  sensors/actions, measured joint-limit violations (0.05 rad simulator tolerance),
  or target lead over 1.5 rad. Faults latch until
  re-entry/reset. A PD equilibrium target may lie outside the physical joint
  range; native actuator torque limits are enforced by the simulator at its
  actuation rate. The adapter does not clip those targets at the policy rate.
  PD target slew is diagnostic only (`target_rate_notice_rad_s`, default 30);
  it is not a measured joint speed and does not trigger a transition to Passive.
- Training Kp/Kd are applied exactly. Optional core waist compensation exists for
  explicitly different waist gains; this FSM does not enable it. Non-waist gain
  substitutions are rejected. Native actuator torque limits remain active,
  including the target plant's 88 Nm hip-roll limit versus metadata's 139 Nm.
- Current runtime reference modes are **0: heading navigation** and **2: wrist
  poses**. Other exported modes, arbitrary pelvis/body target APIs, trajectories,
  motion playback and finger commands have no implemented ROS command interface.

Native wrist mode is disabled in both profiles (`enable_wrist_targets: false`).
The launch argument `enable_wrist_targets:=true` creates a private temporary
copy of the selected profile with only that setting enabled; it leaves the
saved profiles and default behavior unchanged. When enabled for a separate
free-space qualification, it accepts
`/scalebfm/wrist_targets` as a fresh `geometry_msgs/PoseArray` in `odom`, exactly
two poses ordered left/right **wrist_yaw_link**, not palms/fingertips. ROS
quaternions use named XYZW fields, converted to WXYZ internally. Goals must be
finite, unit-quaternion, and within 1 m of the pelvis. Goals persist until
`/scalebfm/clear_wrist_targets` (`std_msgs/Empty`) or FSM re-entry. Invalid input
is rejected explicitly. This is native actor tracking; no bounded wrist servo
or manipulation success is claimed.

### Preview and test native wrist tracking

Keep the existing simulator running. Close the previous controller in its own
terminal, then launch from a shell inside `g1-robogpc-sim`:

```bash
source /opt/ros/jazzy/setup.bash
source /root/Workspaces/g1_ros_ws/install/setup.bash
source /root/Workspaces/g1_ros_ws/install_scalebfm_timing_v4_20260923/setup.bash
export LD_LIBRARY_PATH=/usr/local/lib:${LD_LIBRARY_PATH:-}
ros2 launch g1_controller g1_controller_scalebfm.launch.py \
  scalebfm_model:=m simulation:=true network:=lo \
  steering_ros_enabled:=false enable_wrist_targets:=true \
  camera:=true rviz:=true sam3:=false
```

Use the established FixStand -> A entry procedure. Confirm the startup log says
`wrist_targets=true`, settle in ScaleBFM with the support released, and center
the sticks. Begin in open space away from the desk. A wrist command switches the
same policy from mode 0 to **mode 2: pelvis plus both wrists**. Even a measured-pose
hold changes the policy's conditioning. The ROS command interface does not yet
expose a wrist-only mode selector.

In a second container shell, source the same three setup files, then:

```bash
cd /root/Workspaces/g1_ros_ws/src/g1_rsl/dependencies/g1_controller

# Read current wrist poses and preview the right-wrist +2 cm world-Z target.
python3 scripts/scalebfm_wrist_test.py

# First test switching to mode 2 while holding both measured wrist poses.
python3 scripts/scalebfm_wrist_test.py --execute --dz 0

# After the hold is stable, test a small right-wrist lift and return.
python3 scripts/scalebfm_wrist_test.py --execute --dz 0.02
```

Preview publishes no commands. Each execution starts from newly measured
poses: hold one second, ramp outward over two seconds, hold two seconds, return
over two seconds, hold one second, then request clearing the targets. The other
wrist and both orientations stay fixed in `odom`. Offsets are in odom metres
and limited to 5 cm; `--side left` selects the left wrist. Keep joystick and ROS
steering inactive throughout the test. These are native actor requests, with
no inverse-kinematics or bounded-servo correction.

The helper reads the controller's active configuration, uses named joint
positions and correctly ordered **sport-mode pelvis odometry**, and computes
wrist poses with the same exported kinematic metadata as the C++ adapter. It
does not use the incorrectly ordered generic ROS IMU orientation. It checks
state freshness, stationary entry and bounded body motion, prints both wrist
position errors, and saves measured poses, orientation errors, targets and
model identity to a fresh `/tmp/scalebfm_wrist_test_*.json` report. A successful
script exit means the sequence completed; the interface has no acceptance
acknowledgment and the report does not certify tracking or manipulation.

After any publication attempt, completion, exception or Ctrl-C requests clearing
the targets. Delivery cannot be guaranteed after process/transport failure;
targets otherwise persist. To explicitly return to pelvis navigation:

```bash
ros2 topic pub --once /scalebfm/clear_wrist_targets std_msgs/msg/Empty '{}'
```

X exits ScaleBFM to FixStand. Inspect tracking and body stability in the initial
hold before progressing to the offset test; use free-space results to decide
whether more demanding reaches should be attempted. This setup has interface
and geometry checks, but a live wrist experiment has not been run by the installer.

## Artifacts and rebuild

Installed bundles are under
`config/policy/scalebfm_{m,xl}/exported/{policy.onnx,deployment.yaml,manifest.json,parity.json}`.
The manifest records checkpoint, mode-table, FK, exporter and ONNX SHA-256 hashes,
schema, dependencies, reference settings and parity evidence. The controller
checks the model hash and exact tensor interface before activating and logs
model/checkpoint identity, gains-related settings and timing.

From the source workspace on the host, reinstall reviewed ScaleBFM changes:

```bash
cd /home/wuyan/Documents/code/agentic-protomotions
rats/env_rats/bin/python deployment/scalebfm_g1_controller/install.py \
  --controller-root /home/wuyan/Workspaces/g1_ros_ws/src/g1_rsl/dependencies/g1_controller \
  --update
```

`--update` replaces the dedicated ScaleBFM files; omit it for a first install or
use `--dry-run` to inspect. It preserves other profiles and unrelated shared-file
edits. ONNX models are generated local artifacts and are not committed by this
installer. To regenerate them, see `standalone/scalebfm/ONNX.md` in the source
workspace. Build inside the container:

```bash
source /opt/ros/jazzy/setup.bash
source /root/Workspaces/g1_ros_ws/install/setup.bash
cd /root/Workspaces/g1_ros_ws
colcon --log-base log_scalebfm_timing_v4_20260923 build \
  --packages-select g1_controller --build-base build_scalebfm_timing_v4_20260923 \
  --install-base install_scalebfm_timing_v4_20260923 --parallel-workers 1 \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DG1_BUILD_MUJOCO_CAMERA=OFF
```

This builds the controller only; the existing camera simulator in the base
workspace remains usable. `ros2 launch g1_controller
g1_controller_scalebfm.launch.py --show-args` checks the launch description
without starting a controller. ONNX execution needs Eigen3, yaml-cpp, OpenSSL
and ONNX Runtime; all are already present in the tested container. ARM/Jetson
compilation and latency have not been checked.

## Correction for the ankle PD-target fault

Adapter revision `pd_effort_target_v2` corrects the former
`PD target exceeds mechanical range` check. The upstream actor produces virtual
PD equilibrium positions: effort is `Kp * (q_des - q) - Kd * dq`. Applying a
physical joint-angle limit to `q_des` rejected valid effort requests. Physical
angle checks apply to measured `q`; native motor torque limits apply to effort.

The preserved baseline reproduced a right-ankle target of 0.532757 rad against
an upper physical limit of 0.5236 rad, while its measured angle was -0.143475 rad.
Requested torque was 19.3544 Nm, below the native 50 Nm limit. This reproduces
the faulty predicate; the user's exact entry state was not recorded by v1.
Revision v2 passed through the original actor targets and retained finite-state,
measured-position, target-lead/rate, entry, timing and fall checks. Revision v3
also corrects the rate check, as described below.
It neither widens the physical joint limits nor changes actuator gains.

Startup logs include the adapter revision. Entry logs capture pelvis pose and
right ankle state; first-tick and periodic logs include the requested ankle PD
target and estimated, unsaturated effort. Core guard failures include the
offending joint's state, target, rate and estimated effort. Estimated effort
must not be presented as a measured motor torque.

The user's photo-desk model uses 1 ms physics and `implicitfast`, while the
earlier plain G1 tests used 2 ms and Euler integration. The correction's
validation therefore includes a separately copied photo-desk model with its
original physics and actuator settings. No running simulator or ROS bridge is
replaced or commanded by those offline tests. Evidence is stored under
`outputs/scalebfm_g1_guard_fix_20260923` in the source workspace.

The corrected core passes 179 checks per model, 16 entry checks, and all 24
numerical parity cases per model. The previously rejected offline sequence now
completes 350 ticks with unchanged targets/trajectory before the old fault.
With matching MuJoCo 3.3.7, exact desk standing passes 500 ticks / 10 seconds
from both 0.76 m and 0.793 m initial pelvis heights. Measured joint ranges,
native torque limits, finite state and fall criteria all pass. These tests do
not replay the user's exact pre-entry state or the complete ROS/DDS transition.

To try the corrected build, keep the simulator and use a new controller terminal
with `install_scalebfm_timing_v4_20260923/setup.bash` sourced, as above. The
latest build includes this correction and prints `adapter=bounded_timing_v4`.
Keep
`steering_ros_enabled:=false` for the first zero-command standing retry.

## Walking and PD-target rate

The first correction retained an added 30 rad/s limit on PD target changes. A
later joystick walking trial exceeded it and switched to Passive. This was also
an inappropriate proxy for physical motion: the requested ankle effort was
dropping from about 19 Nm to 1.1 Nm while measured speed was only 2.42 rad/s.
There is no matching hard PD-target rate cutoff in the upstream controller.

Revision `native_pd_navigation_v3` records and reports these target changes
without interrupting the actor or modifying its targets/action history. All
measured-angle, finite-state, target-lead, timing, entry and fall checks remain.
Native actuator effort saturation remains active. The legacy
`max_target_slew_rad_s` configuration is accepted with an explicit notice as a
diagnostic threshold; new profiles use `target_rate_notice_rad_s`.

Reference velocity changes now use a vector-norm acceleration bound. Clipping
world X and Y separately previously gave diagonal headings up to sqrt(2) times
the intended acceleration and a sideways component for a forward command.
The correction preserves the heading-relative direction and acceleration limit.

Fault diagnostics retain the last 100 policy ticks in memory and write a unique
file in `/tmp` only when a fault occurs. The error log prints its path. It
records state, commands, references, policy inputs, PD requests, timing and
model identity for replay; normal control does not write this trace to disk.
Walking regression evidence is under `outputs/scalebfm_rate_guard_20260923` in
the source workspace. Full ROS scheduling and the user's exact unrecorded
joystick stream remain separate from these deterministic offline trials.

With the exact desk plant and matching MuJoCo 3.3.7, the old rate guard rejects a
full-forward trial while the robot is upright (pelvis .769 m, tilt .055 rad,
requested ankle effort -7.58 Nm). Replaying the damping-only Passive response
from that state causes a fall after .428 s. Removing only the rate abort keeps
the complete pre-fault trajectory identical and completes the trial. The final
candidate, including the acceleration correction, walks 1.031 m during four
seconds at .30 m/s, then stops stably. Maximum tilt is .115 rad and minimum
pelvis height is .753 m. A .10 m/s trial also passes. These bounded tests use an
approximation of the reported entry heading/position, not the full unrecorded
live state. The full-speed test averages .225 m/s in its last commanded second
and travels another .157 m during stopping; velocity tracking and instant
stopping are not established by this result.

## Bounded scheduler jitter

The next live trial successfully walked forward, backward and sideways, then
revision v3 switched to Passive because its policy thread woke more than 5 ms
late. The saved trace shows 29.795 ms between policy starts, with the preceding
computation taking 3.062 ms. The last command had XY norm .30 m/s and zero yaw;
the last measured pelvis height was .769 m and tilt .075 rad. The failed tick
was rejected before sensor capture, so its zero-filled command fields do not
represent a user stop command or a new measured state.

Revision `bounded_timing_v4` tolerates isolated wake delays within the bounds
listed above. It keeps the prior held PD target until one fresh inference
finishes, then resumes the nominal cadence from the actual tick start. It does
not duplicate history entries, extrapolate observations, or run catch-up
inference. Long gaps, sustained slowdown, computation overruns, stale inputs
and stale targets still fault. The 40 ms gap and rolling 50 ms delay budget
define a bounded simulator test envelope; this is not a hard real-time or
hardware qualification.

Startup and fault snapshots record these timing limits. Timing notices and
faults report measured gaps, wake delay and the accumulated delay budget.
Fault traces distinguish attempts rejected before capture from sampled ticks.
The original live snapshot and analysis are retained under
`outputs/scalebfm_timing_guard_20260923/` in the source workspace. No existing
simulator, ROS bridge or controller is restarted by this update.

The scheduler passes 1,370 deterministic checks, including the observed 29.795
ms interval, sustained sub-notice jitter, exact deadline boundaries, reset and
no-catch-up behavior. A non-actuating M inference probe runs 1,500 ticks over
30.156 seconds on one recorded live input: maximum interval 22.679 ms, maximum
work 10.627 ms, and maximum rolling wake delay 8.670 ms. Recomputed targets
match the recorded targets exactly. This probe has no ROS/DDS traffic and does
not replay changing robot history or reproduce the original OS scheduling delay.

Two exact-desk offline M trials inject isolated 30 and 40 ms sample intervals
during .30 m/s forward and sideways commands. Extra physics steps hold the prior
PD target before a single fresh sample. Both complete 600 policy ticks and
12.030 seconds of physics without falls, finite-state or guard failures; progress
along the command is 1.026 m forward and 1.050 m sideways. These tests use the
previously validated open-floor heading of -2.415 rad, not the full live entry
state. Full ROS scheduling, arbitrary jitter sequences and indefinite walking
remain unqualified.
