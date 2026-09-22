# RoboGPC steering policy deployment

This bundle combines the trained steering actor, diffusion policy, and RoboGPC
decoder in one stateful ONNX graph. It executes **five latent targets per plan**
and decodes each target against fresh measured state on every control tick
(configured at **50 Hz**). The actor
receives joystick velocity and facing commands. There is no text input or
critic in the exported policy.

The deployment uses **DDIM=20 and five-step execution**, matching the packaged
steering checkpoint's training settings. The selected sampler and checkpoint
provenance are recorded in `exported/policy.json`.

The policy, deployment YAML, controller profile, and launch file use separate
`physhuman_robogpc_steering` names. The original Physhuman, one-step RoboGPC,
and five-step unsteered policies remain available.

## Start in the prepared simulator container

Start the flat-ground simulator in one terminal inside `g1-robogpc-sim`:

```bash
export LD_LIBRARY_PATH=/usr/local/lib:${LD_LIBRARY_PATH:-}
cd /root/Workspaces/g1_ws/unitree_mujoco/simulate/build
./unitree_mujoco -r g1 -s scene_29dof.xml -i 0 -n lo
```

Start one controller in another terminal inside the same container:

```bash
source /root/Workspaces/g1_ros_ws/install/setup.bash
export LD_LIBRARY_PATH=/usr/local/lib:${LD_LIBRARY_PATH:-}
ros2 launch g1_controller g1_controller_robogpc_steering.launch.py rviz:=false
```

This launch file selects `config_robogpc_steering.yaml`, whose steering policy
uses `config/policy/physhuman_robogpc_steering/params/deploy.yaml`.
The new profile selects `policy_provider: cuda`. Its CUDA runner disables TF32
matrix multiplication so the continuous latents match the FP32 export before
the decoder rounds them to discrete codes. CUDA initialization failures are
reported instead of silently changing providers. Existing profiles retain
their automatic provider selection.

### Xbox controls

| Control | Behavior |
| --- | --- |
| Menu / Start | Enter FixStand from Passive; allow the standing transition to finish |
| Hold LB for at least two seconds, then tap D-pad Up | Enter RoboGPC steering from FixStand or Velocity |
| Left stick forward/back | Command forward/backward velocity |
| Left stick left/right | Command lateral velocity |
| Right stick left/right | Turn the desired heading |
| Release right stick | Hold the requested heading |
| View / Back | Return to Passive |

Velocity is expressed in the robot's heading frame. The deployment limits the
left-stick velocity vector to **1 m/s** and desired-heading rotation to
**1 rad/s**. The stick deadzone is **0.1**. On policy entry, the heading target
resets to the current robot heading. The controller converts the target to a
local unit facing vector before inference. Limits and deadzone are configured
under `observations.steering_command.params` in this bundle's `deploy.yaml`.

Replanning happens every five control ticks after bootstrap, so new commands
are used at the next planning call, nominally every **100 ms**. The decoder
still consumes the latest measured state on each control tick, with a configured
**20 ms** period. Replanning latency can exceed this period; see the validation
report for measurements on the deployment hardware.

### RViz velocity arrows

Launch with `rviz:=true` and enter steering mode. The existing **MarkerArray**
display on `/g1_controller/velocity_markers` shows:

- **Green:** the current steering velocity command, after joystick deadzone
  and speed limits.
- **Blue:** measured horizontal velocity from the configured odometry source
  (MuJoCo sport-mode odometry with the simulation launch defaults).

Both arrows use the `odom` frame and a scale of **1 metre per 1 m/s**. They sit
at slightly different heights above the torso so equal velocities remain
visible. They update on policy ticks and expire 0.3 seconds after updates stop.
The green arrow shows the current requested velocity; a five-step plan uses a
new command at its next replanning tick. `zero_out_linvel` affects only policy
observations and does not zero the blue arrow.

For an existing RViz window, enable **MarkerArray**, select the topic above,
and use **odom** as the fixed frame. The measured arrow depends on the selected
odometry source providing velocity; no velocity is estimated by the marker code.

## Export

From the ProtoMotions repository root, with PyTorch, NumPy, PyYAML, ONNX, and
ONNX Runtime available:

```bash
PYTHONPATH=/tmp/robogpc-onnx-deps:. OMP_NUM_THREADS=1 \
env_isaaclab/bin/python deployment/export_physhuman_steering_onnx.py \
  --source-config /home/wuyan/Workspaces/g1_ros_ws/src/g1_rsl/dependencies/g1_controller/config/policy/physhuman/params/deploy.yaml \
  --steering-checkpoint checkpoints/robogpc_uniphys_20k5/steering.ckpt \
  --checkpoint checkpoints/robogpc_uniphys_20k5/diffusion.ckpt \
  --tracker-checkpoint checkpoints/robogpc_uniphys_20k5/tracker.pt \
  --sampling-timesteps 20 \
  --output-dir output/onnx/physhuman_robogpc_steering
```

The exporter validates the steering checkpoint's frozen planner, tracker,
normalizers, temporal alignment, and actor contract before export. Its output
contains:

- `exported/policy.onnx`: complete actor, diffusion, and decoder graph used by
  the controller's `onnx_stateful` runner.
- `exported/steering_actor.onnx`: standalone actor for diagnostics. It returns
  raw sampled noise and the actor mean; it does not produce robot actions.
- `exported/policy.json`: checkpoint provenance, sampler override, tensor
  contract, and validation results.
- `params/deploy.yaml`: matching observation layout, command settings, and
  unchanged source motor mapping, action scales, offsets, and PD gains.
- `exported/validation/`: input/output replay arrays for controller-runner
  verification.
- Controller integration patches and this README at the bundle root.

Install the whole bundle into its own controller policy directory. Use the
steering launch profile and matching YAML together: the one-step and unsteered
five-step YAML files have different observation layouts.

## ONNX input and output contract

All tensors use a fixed batch size of one.

| Input | Shape | Type | Meaning |
| --- | --- | --- | --- |
| `obs` | `[1, 593]` | float32 | 480 history channels, four command channels, 109 epsilon channels |
| `plan_cache_flat` | `[1, 160]` | float32 | 80 executed latent values followed by 80 planned latent values |
| `plan_idx` | `[1]` | int64 | Bootstrap count or execution phase |
| `plan_valid` | `[1]` | float32 | Zero resets the policy; positive indicates initialized state |

The observation history is term-major, with five frames per state term, oldest
first. Controller scales are reversed in the graph.

| Zero-based slice | Term | Treatment |
| --- | --- | --- |
| `0:15` | `base_lin_vel` | Ignored by the trained RoboGPC state |
| `15:30` | `base_ang_vel` | Undo controller scale 0.2 |
| `30:45` | `projected_gravity` | Preserve local gravity |
| `45:190` | `joint_pos_rel` | Relative joint positions |
| `190:335` | `joint_vel_rel` | Undo controller scale 0.05 |
| `335:480` | `last_action` | Previous raw policy commands |
| `480:484` | `steering_command` | Heading-local velocity x/y, unit facing x/y |
| `484:593` | `noise` | One **unclipped** standard-normal vector of 109 values |

The actor receives 545 normalized history channels, 93 normalized current
measured state channels, and four command channels, totaling 642. It samples
`raw_noise = mean + exp(clamped_log_std) * epsilon`. The same raw noise vector
initializes every future token. The diffusion sampler then applies its own
noise clipping. Clipping epsilon before adding the actor mean changes the
trained policy, so this YAML sets `noise.params.clip_enabled: false`.

Despite five-step execution, `noise.params.num_future_steps` is **1**, with
109 scale entries. The actor expands the shared noise inside the ONNX graph.

| Output | Shape | Type |
| --- | --- | --- |
| `actions` | `[1, 29]` | float32 |
| `plan_cache_flat_out` | `[1, 160]` | float32 |
| `plan_idx_out` | `[1]` | int64 |
| `plan_valid_out` | `[1]` | float32 |

Actions are raw RoboGPC commands in the checkpoint's native joint order. The
controller applies the existing joint mapping, action scales, offsets, and PD
gains. Carry the three state outputs into the next call after executing its
action. The next measured state's `last_action` must contain that raw command.

The standalone actor accepts `state_history[1,5,109]` in physical units,
`command[1,4]`, and `epsilon[1,109]`; its `raw_noise` and `mean` outputs are both
`[1,109]`. Each 109-channel history token contains latent16, gravity3,
angular velocity3, joint position29, joint velocity29, and previous action29,
in that order. No critic, terrain map, or internal random generator is exported.

## State, execution, and reset

Initialize cache, index, and validity to zero. Calls 0–4 decode a standing
reference tracker latent against current measurements to establish executed
latent history. Calls 5, 10, 15, … run the steering actor and diffusion planner.
The other calls decode the next cached latent using the newest measured state.
An ONNX `If` branch skips both actor and diffusion during bootstrap and cached
execution. The cache contains latents, not future joint actions.

Reset observation history, heading target, and runner state together on policy
entry. Setting `plan_valid=0` discards stale cached values and restarts the
five-call bootstrap, including when the old cache contains NaNs. If an emitted
action was not executed, discard that cache update and reset before resuming.

The controller integration adds the steering command observation and explicit
noise clipping switch. Its default clipping behavior remains compatible with
the existing policies. The separate profile removes the legacy base-velocity
command generator because its joystick mapping differs from the steering
actor's command layout.

## Verification

Focused tests cover observation packing, unclipped epsilon, five-call
bootstrap, command/noise use only on replanning calls, fresh-state decoding,
reset, source preservation, ONNX branch placement, and sequential ONNX Runtime
parity. Export validation also compares the real actor against the native
training implementation and the composite graph against native shared-noise
diffusion plus decoder across three replanning calls.

Read the generated `exported/policy.json` and controller-runner validation
report for measured errors and timing. Numerical parity does not establish
closed-loop steering behavior or that replanning fits the 20 ms control period.
Initial GPU calls can also take longer while kernels initialize. The CPU path
is useful for numerical checks, but its measured replanning time is about
65 ms on this deployment machine.

The installed ONNX Runtime 1.22.0 stateful runner passed 34 CUDA calls with
TF32 disabled on an RTX PRO 6000 Blackwell. Maximum CPU/CUDA action error was
`1.91e-6`, with exact reset replay. Replanning median was **13.5 ms** and cached
decoding median **0.41 ms**. Its first action took **276 ms** and first replan
**44.7 ms**; these are measured offline timings, not a control-loop deadline
guarantee. No closed-loop simulator or robot trial was run.
