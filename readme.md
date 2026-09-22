# C++ Deployment Overlays for RSL's G1s

[![Linux platform](https://img.shields.io/badge/platform-linux--64-orange.svg)](https://releases.ubuntu.com/20.04/)


**Maintainers:** Chenhao Li  
**Affiliation:** Robotic Systems Lab, ETH Zurich  
**Contact:** chenhli@leggedrobotics.com


## Usage

### Configuration Files

- **Config File**: Place your configuration files under the `configs` folder. These files define various settings and parameters required for the deployment.
  
Example:

```
config/policy/velocity/v0/params/deploy.yaml
```

- **Policy Files**: Place your policy files under the `policies` folder. These files specify the policies that control the behavior of the robot during deployment.

Example:

```
config/policy/velocity/v0/exported/policy.onnx
```


## Camera in simulation and on the robot

All controller launch profiles accept `camera:=true` (the default) and
`rviz:=true`. `simulation` selects the backend, and RViz uses the same **Camera**
group in both modes: RGB image and colored points are enabled; depth image is
available by checking **Camera Depth**.

| Output | Shared ROS topic |
| --- | --- |
| RGB (`rgb8`) | `/camera/color/image_raw` |
| RGB calibration | `/camera/color/camera_info` |
| Aligned depth (`16UC1`, millimeters; zero invalid) | `/camera/aligned_depth_to_color/image_raw` |
| Aligned depth calibration | `/camera/aligned_depth_to_color/camera_info` |
| Colored point cloud | `/camera/depth/color/points` |

Consumers should use sensor-data QoS (best effort, volatile), and use each
message's `header.frame_id` with TF. Both backends attach beneath the existing
`camera_link` mount published by `g1_description`. Images and clouds share ROS
wall-clock time with the existing controller; this change does not enable
`use_sim_time` or add a `/clock` publisher.

### Build for MuJoCo

The stock `unitree_mujoco` executable does not publish camera images. Build the
optional **`g1_mujoco_camera`** executable and use it in place of the stock one.
It retains the existing simulator UI, joystick, scenes, and Unitree DDS bridge.
The camera renders the live MuJoCo scene; it does not reconstruct the world from
controller odometry. Vendor sources are left untouched: checked rendering hooks
are applied to copies in the build directory. Unsupported source layouts fail
at configure time.

Inside the project's ROS container:

```bash
cd /root/Workspaces/g1_ros_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to g1_controller --cmake-args \
  -DG1_BUILD_MUJOCO_CAMERA=ON \
  -DUNITREE_MUJOCO_DIR=/root/Workspaces/g1_ws/unitree_mujoco
source install/setup.bash

# Terminal 1: use this instead of ./unitree_mujoco; do not run both simulators.
ros2 run g1_controller g1_mujoco_camera -r g1 -s scene_29dof.xml -i 0 -n lo

# Terminal 2, in the same container with install/setup.bash sourced:
ros2 launch g1_controller g1_controller_robogpc_steering.launch.py \
  simulation:=true camera:=true rviz:=true
```

Other controller profiles work identically. The simulator reads its original
`simulate/config.yaml` and scene assets from `UNITREE_MUJOCO_DIR`, which must
remain available at runtime. It requires a working OpenGL display, as does the
stock simulator. Rendering runs on the UI thread outside the physics mutex,
with a short locked scene snapshot. RGB-D is 640×480 at up to 30 Hz, with an
ideal 58-degree vertical field of view and 0.01–10 m clipping range. Intrinsics
are published in `CameraInfo`; this is an ideal sensor, without RealSense noise
or lens distortion. The simulated optical frame is co-located with
`camera_link`; hardware supplies its own factory sensor extrinsics.

The simulator resolves `torso_link` to `camera_color_optical_frame` through TF,
so the existing camera mount is shared with deployment. It waits for that TF
and subscribers before rendering. Launching only the controller with a stock
simulator will not produce camera images. `camera:=false` skips camera TF/driver
startup; also use `rviz:=false` or disable the RViz Camera group if unwanted.

### Deploy with the physical camera

Install/source `realsense2_camera`, connect the G1's RealSense USB camera to the
machine running the camera node, and use the same controller profile with
`simulation:=false`. For the existing on-robot LiDAR/localization setup:

```bash
ros2 launch g1_controller g1_controller_opc.launch.py \
  camera:=true rviz:=true network:=eno1
```

The OPC profile already defaults to `simulation:=false`. Any RoboGPC profile
also accepts `simulation:=false`, with the appropriate network and odometry
arguments for the deployment. RealSense publishes RGB, aligned depth, sensor
TF, and colored points at the same topic names. Do not start another camera
driver publishing these topics/frames. Simulator dependencies are optional on
the physical robot (`G1_BUILD_MUJOCO_CAMERA` defaults to `OFF`).

To run only the camera backend alongside an existing controller/description:

```bash
ros2 launch g1_controller g1_camera.launch.py simulation:=false
```

To verify simulation output without starting the controller:

```bash
python3 src/g1_rsl/dependencies/g1_controller/tests/camera_smoke.py \
  install/g1_controller/lib/g1_controller/g1_mujoco_camera --rviz
```

This uses isolated ROS and Unitree domain 87, checks synchronized RGB/depth/
calibration/cloud, depth units, point projection, color packing, and RViz
subscriptions, and shuts down its test processes. Real hardware image capture
must still be checked on a connected G1 camera.

## GPU Inference

The inference process on GPU provides significant speedup compared to CPU inference.  
To enable GPU inference, ensure that your environment is set up with the necessary CUDA, cuDNN, and (optionally) TensorRT libraries.

### Add NVIDIA’s CUDA repo

Set your Ubuntu release and architecture:

```bash
# Examples:
# Ubuntu 20.04 on x86_64
export UBUNTU_VERSION=2004
export ARCH=x86_64
# Ubuntu 22.04 on ARM64 (Jetson)
# export UBUNTU_VERSION=2204
# export ARCH=arm64
```

Add the NVIDIA CUDA repo:

```bash
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu${UBUNTU_VERSION}/${ARCH}/cuda-keyring_1.1-1_all.deb
sudo dpkg -i cuda-keyring_1.1-1_all.deb
sudo apt-get update
```

### Install cuDNN and TensorRT

```bash
# cuDNN 9 for CUDA 12
sudo apt-get install -y cudnn9-cuda-12

# TensorRT runtime libraries
sudo apt-get install -y tensorrt-libs
```

## Python Implementation

Check out our Python implementation of the G1 robot deployment:

- [g1_rsl_python](https://github.com/leggedrobotics/g1_rsl_python): Python implementation of the G1 robot deployment.

## Change text command
ros2 topic pub /text_command std_msgs/msg/String "data: 'stand'" --once

In the `Physhuman` experiment, after switching into it with joystick `B`, you can also press joystick `Y` to directly change the text command to `stand`.

## Physhuman RoboGPC diffusion policy

`Physhuman_RoboGPC` is a separate RLBase mode (ID 7) using
`config/policy/physhuman_robogpc`. It runs the diffusion base policy plus the
RoboGPC decoder with **DDIM=10** and **exec_steps=1**, without text conditioning.
The original Physhuman mode and bindings are preserved.

- From **FixStand** or **Velocity**, hold **LB for 2 seconds**, then press
  **D-pad Up** to enter.
- Press **Back/Select** to return to Passive, or hold **LB for 2 seconds** and
  press **A** to return to Velocity.
- Each entry clears the stateful cache and performs five standing-reference
  tracker steps before starting diffusion from measured history.

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

The stateful runner reset hook is integrated in the controller source. Rebuild
`g1_controller` and source the resulting ROS installation before using the new
mode. Build in the project's ROS environment; the host checkout alone does not
provide ROS build dependencies. This change does not start the controller.

The ONNX/C++ CPU interface and reset replay passed validation. The recorded
single-thread CPU latency for DDIM=10 was 22.7 ms median (28.6 ms maximum), above
the 20 ms control period; validate timing on the deployment runtime.
See [the policy guide](config/policy/physhuman_robogpc/README.md) for tensor and
bootstrap details.

## Physhuman Goal Reaching

The `Physhuman_goal` experiment is entered with joystick `LB` held for 2 seconds plus `Y`. It listens to RViz goal messages on `/goal_pose` (`geometry_msgs/msg/PoseStamped`) and feeds the policy the planar goal offset in the robot yaw-aligned body frame. Every time the state is entered, any previous external goal is cleared and the stored goal is reset to odom XY `(0, 0)` until a fresh goal arrives.

To visualize the selected goal in RViz, add a `Marker` display with topic `/goal_marker`. The RViz goal tool arrow is only the one-shot input command, so it can disappear after publishing; `/goal_marker` is the persistent controller marker.

You can also publish a goal directly from the terminal:

```bash
ros2 topic pub /goal_pose geometry_msgs/msg/PoseStamped "{
  header: {frame_id: 'odom'},
  pose: {
    position: {x: 1.5, y: 0.0, z: 0.76},
    orientation: {w: 1.0}
  }
}" --once
```

The controller logs planar distance to the goal while `Physhuman_goal` is active. It starts with the idle text command `stand`; when a real `/goal_pose` arrives, it switches to the configured active command such as `walk`. Once the distance stays below the configured threshold, it changes back to `stand` so the policy can stop. The defaults live in `config/policy/physhuman_goal/params/deploy.yaml` under `goal_monitor`.

Text commands for this mode still use:

```bash
ros2 topic pub /text_command std_msgs/msg/String "data: 'walk'" --once
```

In `Physhuman_goal`, publishing a non-idle text command also updates the active motion style for the current goal, or queues it for the next goal if the robot is currently idle:

```bash
ros2 topic pub /text_command std_msgs/msg/String "data: 'jog'" --once
```


## Citation
If you find this repository useful for your research, please consider citing:

```
@article{li2025robotic,
  title={Robotic world model: A neural network simulator for robust policy optimization in robotics},
  author={Li, Chenhao and Krause, Andreas and Hutter, Marco},
  journal={arXiv preprint arXiv:2501.10100},
  year={2025}
}
```
