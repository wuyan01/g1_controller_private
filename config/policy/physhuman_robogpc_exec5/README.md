# RoboGPC ONNX policy with five-step execution

This variant exports the diffusion base policy and RoboGPC decoder with
**`exec_steps=5` and DDIM=10**. It plans five continuous latents at a time and
executes one per control call. Each latent is decoded against the latest
measured state, so intermediate calls still produce fresh actions. There is no
text embedding or PPO steering.

The separate `physhuman_robogpc_exec5` bundle uses the existing `onnx_stateful`
runner. Export does not activate a controller mode or replace the existing
`physhuman` or one-step `physhuman_robogpc` policy.

## Export

From the ProtoMotions repository root, use an environment with PyTorch, NumPy,
PyYAML, ONNX, and ONNX Runtime:

```bash
env_isaaclab/bin/python deployment/export_physhuman_onnx.py \
  --source-config /home/wuyan/Workspaces/g1_ros_ws/src/g1_rsl/dependencies/g1_controller/config/policy/physhuman/params/deploy.yaml \
  --exec-steps 5 \
  --sampling-timesteps 10 \
  --output-dir output/onnx/physhuman_robogpc_exec5
```

The default checkpoints are
`checkpoints/robogpc_uniphys_20k5/diffusion.ckpt` and
`checkpoints/robogpc_uniphys_20k5/tracker.pt`. Override them with `--checkpoint`
and `--tracker-checkpoint`.

The bundle contains `exported/policy.onnx`, `exported/policy.json`,
`params/deploy.yaml`, validation replay arrays, the runner reset patch, and a
README. The generated deployment YAML preserves the source joint mapping,
action scales, offsets, PD settings, and 50 Hz control rate. It configures five
109-channel noise tokens and removes the text observation.

If installing this export, copy the entire bundle into a new sibling policy
directory named `physhuman_robogpc_exec5` and bind a separate controller profile
to it. Use this bundle's `deploy.yaml`: the one-step configuration has different
observation and cache dimensions. The runner reads the graph's static tensor
shapes; the existing runner reset integration also applies to this variant.

## Input and output contract

All batches are fixed to one. Inputs retain the stateful runner's existing names:

| Input | Shape | Type | Meaning |
| --- | --- | --- | --- |
| `obs` | `[1, 1025]` | float32 | Five-frame measured history, followed by five noise tokens |
| `plan_cache_flat` | `[1, 160]` | float32 | Executed latent history, followed by the current latent plan |
| `plan_idx` | `[1]` | int64 | Bootstrap count or position within the current plan |
| `plan_valid` | `[1]` | float32 | Zero resets state; positive means initialized |

The six measurement terms occupy 480 channels. Each term contains five frames,
oldest first; the layout is term-major, rather than frame-major.

| Zero-based slice | Term | Treatment inside the graph |
| --- | --- | --- |
| `0:15` | `base_lin_vel` | Ignored; the trained state excludes linear velocity |
| `15:30` | `base_ang_vel` | Divide by the controller's `0.2` scale |
| `30:45` | `projected_gravity` | Preserve local-frame gravity vectors |
| `45:190` | `joint_pos_rel` | Preserve relative joint positions |
| `190:335` | `joint_vel_rel` | Divide by the controller's `0.05` scale |
| `335:480` | `last_action` | Preserve previous raw policy commands |
| `480:1025` | `noise` | Five consecutive 109-channel future tokens |

The graph reconstructs each 93-channel measured state in this order: gravity3,
angular-velocity3, relative-position29, relative-velocity29, previous-action29.
It pairs the state history with five executed 16-channel continuous latents and
applies the checkpoint normalizers. Noise comes from the controller's Gaussian
observation term; use the generated YAML's sampling and clipping settings. The
graph has no internal random generator and no text input.

The flattened cache is ordered as follows:

| Cache slice | Meaning |
| --- | --- |
| `0:80` | Five executed 16-channel latents, oldest first |
| `80:160` | Five planned 16-channel latents, in execution order |

Outputs are:

| Output | Shape | Type |
| --- | --- | --- |
| `actions` | `[1, 29]` | float32 |
| `plan_cache_flat_out` | `[1, 160]` | float32 |
| `plan_idx_out` | `[1]` | int64 |
| `plan_valid_out` | `[1]` | float32 |

`actions` contains one raw RoboGPC command in the checkpoint's native joint
order. The controller applies the preserved joint map, action scales, offsets,
and PD settings. Carry the three state outputs into the next call after
executing the action. At that next call, the latest measured state's
previous-action channels must contain the executed raw command.

## Execution and reset

Initialize the cache, index, and valid flag to zero. The first five physical
control calls use the standing-reference tracker latent, decoded against fresh
measurements, to establish executed latent history. With zero-based call
indices, inference then proceeds as follows:

| Calls | Behavior |
| --- | --- |
| `0–4` | Bootstrap; outgoing index advances from `1` through `5` |
| `5` | Plan five latents and decode the first; outgoing index is `6` |
| `6–9` | Decode the remaining latents against fresh states; outgoing indices are `7, 8, 9, 5` |
| `10, 15, …` | Replan, repeating the five-call execution cycle |

Thus, after bootstrap, planning runs every 100 ms at the configured 50 Hz
control rate. The cache holds **latents**, not future actions. An ONNX `If`
branch skips diffusion on bootstrap and intermediate execution calls.

DDIM=10 is the number of denoising steps per token. The native pyramid schedule
staggers five future tokens, giving 14 sampler rows per five-token plan.
Changing execution length therefore changes both the plan horizon and the work
performed on a planning call.

Reset the observation history and runner state together when entering the
policy. The supplied `physhuman_runner_reset.patch` provides the runner reset
hook; it is required if that integration is not already present. Setting
`plan_valid=0` discards stale cache and restarts bootstrap. If an emitted action
was not executed, discard its cache update and restart bootstrap before
resuming inference.

## Validation

Refer to the generated `exported/policy.json` for the exported tensor contract,
checkpoint provenance, and validation results. Replay arrays under
`exported/validation/` support checks against the deployment's ONNX Runtime.
Use results from this five-step export when assessing numerical parity or
latency; measurements for the one-step model do not establish this variant's
performance. Inference validation does not establish closed-loop robot
behavior or that planning calls meet the controller's 20 ms deadline.
