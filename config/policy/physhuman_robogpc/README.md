# ROBOGPC diffusion policy for Physhuman's stateful ONNX runner

This export creates a separate **`physhuman_robogpc`** policy directory for
`StatefulOrtRunner`. It combines the diffusion base policy and ROBOGPC decoder,
uses `exec_steps=1` and 10 DDIM denoising steps, and has no text conditioning or PPO
steering. The original `physhuman` model and deployment configuration stay intact.

## Integration in this controller checkout

The controller now registers **Physhuman_RoboGPC (ID 7)** separately from the
original Physhuman mode. From FixStand or Velocity, hold **LB for 2 seconds**
and press **D-pad Up** to enter. **Back/Select** exits to Passive;
**LB held for 2 seconds + A** returns to Velocity.

The runner reset patch has been applied to this checkout. Rebuild the ROS
controller and source its installation to include the new state and reset hook.
The model uses DDIM=10 and exec_steps=1; original Physhuman files are unchanged.

## Export and install the separate configuration

From the ProtoMotions repository root, use an environment with PyTorch, NumPy,
PyYAML, ONNX, and ONNX Runtime:

```bash
env_isaaclab/bin/python deployment/export_physhuman_onnx.py \
  --source-config /home/wuyan/Workspaces/g1_ros_ws/src/g1_rsl/dependencies/g1_controller/config/policy/physhuman/params/deploy.yaml \
  --sampling-timesteps 10 \
  --output-dir output/onnx/physhuman_robogpc
```

The default checkpoints are `checkpoints/robogpc_uniphys_20k5/diffusion.ckpt` and
`tracker.pt`. Override them with `--checkpoint` and `--tracker-checkpoint`;
`--sampling-timesteps` controls the default 10 DDIM denoising steps. Export needs no
simulator or ROBOGPC checkout.

The generated directory contains:

```text
physhuman_robogpc/
  exported/policy.onnx
  exported/policy.json
  exported/validation/tick_*.npz
  params/deploy.yaml
  physhuman_runner_reset.patch
  README.md
```

Deploy this as a **new sibling** directory named `physhuman_robogpc` under the
controller's `config/policy/`. Bind a separate controller configuration to
`config/policy/physhuman_robogpc`; retain the existing `physhuman` binding. Export
does not activate an FSM state or run the controller.

The new YAML preserves the source joint mapping, action scales and offsets,
stiffness, damping, and 50 Hz control rate. It selects `onnx_stateful`, removes
the text observation, provides one 109-channel noise token, disables the source
soft gate, and uses a separate action log. The exporter checks the joint order,
default pose, and timing against the checkpoints.

## Dedicated controller launch profile

See [controller integration results](controller_integration.md) for the applied
changes, successful build/CPU checks, and current GPU runtime limitations.

The dedicated `g1_controller_robogpc.launch.py` profile selects
`config/config_robogpc.yaml` and enables only Passive, FixStand, Velocity, and
Physhuman_RoboGPC. This avoids loading the absent older Physhuman model files.
The original launch still defaults to `config/config.yaml`.

Build and source the controller in the project's ROS environment:

```bash
cd /root/Workspaces/g1_ros_ws  # workspace path inside the project's ROS container
colcon build --packages-up-to g1_controller
source install/setup.bash
ros2 launch g1_controller g1_controller_robogpc.launch.py rviz:=false
```

The dedicated launch defaults to simulation and network interface `lo`, matching
the existing simulation launch. It also accepts `simulation`, `network`,
`pose_source_odom`, and `rviz`. The generic simulation launch accepts a
`config_file` argument (relative to the package's `config/` directory, or an
absolute path).

## Observation and cache contract

All batch sizes are fixed to one. The graph inputs are:

| Input | Shape | Type | Meaning |
| --- | --- | --- | --- |
| `obs` | `[1, 589]` | float32 | Term-major measured history followed by sampled noise |
| `plan_cache_flat` | `[1, 80]` | float32 | Five continuous 16-channel executed latents, oldest first |
| `plan_idx` | `[1]` | int64 | Completed bootstrap count, saturated at five |
| `plan_valid` | `[1]` | float32 | Zero starts fresh bootstrap; positive means initialized |

The supplied configuration produces these zero-based slices in `obs`. Each
measurement term contains five frames, oldest to newest; the six terms occupy
480 channels and the final noise token occupies 109:

| Slice | Term | Treatment inside the graph |
| --- | --- | --- |
| `0:15` | `base_lin_vel` | Ignored; the trained ROBOGPC state excludes linear velocity |
| `15:30` | `base_ang_vel` | Divide by the controller's `0.2` scale |
| `30:45` | `projected_gravity` | Preserve local-frame gravity vectors |
| `45:190` | `joint_pos_rel` | Preserve relative joint positions |
| `190:335` | `joint_vel_rel` | Divide by the controller's `0.05` scale |
| `335:480` | `last_action` | Preserve previous raw policy commands |
| `480:589` | `noise` | One externally sampled 109-channel future token |

There is no text embedding. The wrapper reconstructs raw state93 in this order:
gravity3, angular-velocity3, relative-position29, relative-velocity29,
previous-action29. It pairs those measured states with the cached latent16
values, then applies the saved diffusion and decoder normalizers. Do not change
observation scales or add normalization outside the generated contract. Noise
comes from the controller's Gaussian observation term; its clipping settings
remain in `deploy.yaml`. The ONNX model has no internal random generator.

The graph outputs are:

| Output | Shape | Type |
| --- | --- | --- |
| `actions` | `[1, 29]` | float32 |
| `plan_cache_flat_out` | `[1, 80]` | float32 |
| `plan_idx_out` | `[1]` | int64 |
| `plan_valid_out` | `[1]` | float32 |

`actions` contains raw ROBOGPC policy commands in the checkpoint's native joint
order. The controller applies its preserved joint map, action scales, offsets,
and PD settings. Cache output includes this call's continuous latent; carry the
three state outputs into the next inference **after executing the action**. At
that next call, the newest measured state's previous-action channels must equal
the executed raw command. These cache tensors hold latent history, not a chunk
of future actions.

## Startup and reset

Initialize cache, index, and valid flag to zero. The first **five physical control
calls** use a static standing-reference tracker latent and decode it against the
fresh measured state. The sixth call starts diffusion using five genuine
latent/result pairs. Repeated measurements used by the observation manager at
startup are flushed before diffusion begins; no missing latent history is
invented for autonomous inference.

The standing latent is computed once from the saved tracker encoder and its
normalizer using the native constant default-pose reference. Against five
recorded native tracker bootstrap steps, its maximum latent difference was
`7.15e-7`, with identical rounded FSQ codes. This verifies encoder equivalence;
it does not establish standing performance on a robot.

**The separate reset patch is required for correct policy re-entry.** The existing
runner retains cache state across environment resets. The supplied
`physhuman_runner_reset.patch` adds an algorithm reset hook and clears the
runner's action, latent cache, index, valid flag, and related runtime state when
the environment resets. Without that integration, re-entry can combine stale
latents with newly initialized observation history and skip bootstrap.

The patch is **applied in this controller checkout**. The separate patch file
is retained for reference; do not apply it a second time. A controller rebuild
is required. The observation history and runner state now reset together. If an
emitted action was not executed, discard its cache update and
restart bootstrap before resuming inference.

## Validation scope

Export runs the ONNX checker and CPU ONNX Runtime comparisons with the PyTorch
wrapper and native decoder/prior paths. The checks cover nine sequential calls,
the five-call bootstrap, transition to diffusion, tensor names/shapes/dtypes,
and discarding stale cache when `plan_valid=0`. Results and maximum numerical
errors are recorded in `exported/policy.json`; replay arrays are saved under
`exported/validation/` for testing another ONNX Runtime build.

The standalone harness `deployment/validate_physhuman_onnx.cpp` tests the
controller's bundled **ONNX Runtime 1.22.0 C++ CPU library**, including sequential
calls and exact reset replay. Refer to the exported `policy.json` for validation
results and latency measurements for the current 10-step model. Accelerator
latency and 50 Hz operation have not been validated. Changing
`--sampling-timesteps` changes the sampler and needs its own control validation.

These checks establish the inference and cache interface. They do not include
closed-loop robot validation, and the export does not execute the controller.
