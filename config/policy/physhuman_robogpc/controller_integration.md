# RoboGPC controller integration

Applied to `/home/wuyan/Workspaces/g1_ros_ws/src/g1_rsl/dependencies/g1_controller`.

## Configuration and controls

- `config/config_robogpc.yaml` enables Passive, FixStand, Velocity, and Physhuman_RoboGPC (ID 7).
- `g1_controller_robogpc.launch.py` selects that profile; simulation and network `lo` are the defaults.
- From FixStand or Velocity, hold LB for two seconds and press D-pad Up to enter RoboGPC.
- Back/Select returns to Passive. LB for two seconds plus A returns to Velocity.
- The policy combines the diffusion base policy and RoboGPC decoder with DDIM=10, `exec_steps=1`, and no text input.
- Each entry resets action/history/latent state and repeats the five-step bootstrap.

The original Physhuman policy directory is unchanged. The default controller profile retains its existing modes and adds RoboGPC. Its three missing older Physhuman models are avoided by the dedicated profile. The default launch continues to select `config.yaml`.

The controller accepts the ROS parameter `config_file`, either a filename relative to the installed package's `config/` directory or an absolute path. The terrain-point count now starts at zero when terrain sensing is disabled.

## Build and launch

Inside the project's ROS environment, from its workspace root:

```bash
colcon build --packages-up-to g1_controller
source install/setup.bash
ros2 launch g1_controller g1_controller_robogpc.launch.py rviz:=false
```

The validation build used an isolated output directory, so rebuild the active ROS workspace before using this launch. No controller loop or robot communication was started during integration.

## Verification

- ROS Jazzy build: `g1_description` and `g1_controller` passed.
- Launch files parsed, and ROS launch argument inspection passed.
- Actual C++ config loader: relative/absolute RoboGPC selection, original default profile, transitions, and installed policy paths passed.
- Actual `StatefulOrtRunner`: 16 CPU inference calls returned finite 29-element actions. Virtual reset cleared its action buffer, and the full replay after reset matched exactly (maximum absolute error 0).
- CPU verification used the unchanged bundled ONNX Runtime 1.22.0 core in an isolated directory without GPU provider plugins. No deployed runtime libraries were changed.
- Original Physhuman YAML and exported DDIM10 model hashes were verified.

## Remaining runtime limitations

This host has no working NVIDIA driver (`nvidia-smi` cannot communicate with it). Normal provider initialization in the test container crashed inside the bundled `libonnxruntime_providers_cuda.so` while loading TensorRT. GPU execution remains unvalidated; a working GPU runtime is needed to assess that path. This crash occurs before model inference.

The earlier single-thread C++ CPU measurement was 22.7 ms median and 28.6 ms maximum, exceeding the 20 ms policy period. The CPU smoke test establishes interface/reset correctness, not 50 Hz operation or closed-loop behavior.

The existing controller's `--help` path loads the selected YAML but aborts during shutdown: its global `Keyboard` leaves a joinable thread in its destructor. The isolated config-loader check passed without this unrelated shutdown path. That existing behavior was not changed.

See `exported/validation/controller_integration.json` and the adjacent controller build/CPU logs for recorded results.
