# Steering controller integration

New launch: `g1_controller_robogpc_steering.launch.py`.
New profile: `config/config_robogpc_steering.yaml`.
Policy directory: `config/policy/physhuman_robogpc_steering`.

The profile keeps Passive, FixStand and Velocity and adds its own
`Physhuman_RoboGPC_Steering` state. Enter with LB held two seconds + D-pad Up
from FixStand or Velocity; View/Back returns to Passive.

## Command term

```yaml
steering_command:
  params:
    max_speed: 1.0
    max_yaw_rate: 1.0
    deadzone: 0.1
  clip: null
  scale: [1.0, 1.0, 1.0, 1.0]
  history_length: 1
```

Output is raw heading-local `[vx, vy, facing_x, facing_y]`. Left stick sets
translation relative to the robot: up is forward; left is left. Its radial
magnitude maps to 0–1 m/s, capped also on diagonals. Right stick horizontal
integrates a world heading target at up to 1 rad/s; left means counterclockwise.
Release the right stick to retain the target heading. Target heading resets to
the measured IMU yaw on mode entry, before observation history is filled.
The command advances exactly once per physical policy tick, including ticks
executing cached plan latents. Reading observations cannot advance heading.

The native training command range is 0–4 m/s; deployment starts with a 1 m/s stick limit,
matching the existing profile's maximum forward command. It can be changed in
this policy's deploy.yaml without re-exporting the model.

## Execution provider

The steering deploy.yaml selects `policy_provider: cuda`. The stateful runner
uses strict FP32 (`use_tf32=0`) to preserve the latent codes passed to the FSQ
decoder. An unavailable CUDA provider raises an initialization error. Existing
profiles without this setting retain automatic provider selection.

## Actor noise

Use `noise.params.clip_enabled: false`, `state_dim: 109`, and
`num_future_steps: 1` for an untruncated standard Gaussian sample. The learned
actor computes its Gaussian noise and the ONNX prior performs the final clip.
The new option defaults to true for existing policies, preserving their noise.

## Validation

The standalone `test_steering_command.cpp` passed with C++17 and
`-Wall -Wextra -Werror`. It checks held world heading, mode reentry, wrap across
plus/minus pi, left/right signs, deadzone and diagonal maximum speed.
The real StatefulOrtRunner passed 34 calls against an independent CPU reference
using the final DDIM20 ONNX graph; reset replay and reference action errors were
zero. The production ManagerBasedRLEnv/ObservationManager passed a mock-state
check of observation593, heading update/reset behavior, unclipped Gaussian
noise109, and backward compatibility with observation589 and clipped noise.
The explicit CUDA runner with TF32 disabled also passed 34 calls with a maximum
CPU-reference action error of 1.91e-6 and exact reset replay. Median replan time
was 13.5 ms; initial calls exceeded the 20 ms period.
No ROS, DDS, simulator control, or motor commands were started by these tests.
Results are in exported/validation/runtime_validation.json and the adjacent logs.

After installation, rebuild the controller inside the existing container:

```bash
source /opt/ros/jazzy/setup.bash
export LD_LIBRARY_PATH=/usr/local/lib:${LD_LIBRARY_PATH:-}
cd /root/Workspaces/g1_ros_ws
colcon build --packages-up-to g1_controller
```

