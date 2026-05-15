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
