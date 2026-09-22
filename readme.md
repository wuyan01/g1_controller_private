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

## SAM 3 perception

The optional Python node consumes the same camera topic in simulation and on the
physical G1. It runs independently from the controller and defaults to **2 Hz**,
keeping only the latest pending image. Enable it with `sam3:=true`; the shared
controller launch then also enables **SAM3 Overlay** in RViz automatically.
SAM 3 is disabled by default.

The current `g1-robogpc-sim` container has been provisioned with:

- Interpreter: `/opt/g1-sam3-venv/bin/python` (isolated dependencies, reusing the
  existing PyTorch 2.10/CUDA 12.8 installation).
- Checkpoint: `/opt/g1-models/sam3.pt`, copied from the supplied host Hugging Face
  cache. The host checkpoint is unchanged.
- Standard ROS `vision_msgs` messages.

With `g1_mujoco_camera` running in another terminal, start the controller:

```bash
source /root/Workspaces/g1_ros_ws/install/setup.bash
ros2 launch g1_controller g1_controller_robogpc_steering.launch.py \
  simulation:=true camera:=true rviz:=true sam3:=true \
  sam3_python:=/opt/g1-sam3-venv/bin/python \
  sam3_checkpoint:=/opt/g1-models/sam3.pt \
  sam3_prompt:=floor
```

The default MuJoCo scene shows a checkerboard floor, so `floor` is a useful first
prompt. A reusable chair scene is provided in `config/mujoco/chair.xml`. Generate
the full scene inside the container (the current container already has it):

```bash
ros2 run g1_controller create_chair_scene.py \
  --mujoco-dir /root/Workspaces/g1_ws/unitree_mujoco \
  --output-dir /opt/g1-scenes/chair
```

Stop the previous simulator, then start the generated scene:

```bash
ros2 run g1_controller g1_mujoco_camera -r g1 \
  -s /opt/g1-scenes/chair/scene_29dof_chair.xml -i 0 -n lo
```

Use `sam3_prompt:=chair` in the controller command above, or publish `chair` on
`/sam3/prompt` if perception is already running. The chair is a fixed perception
prop, 1.4 m behind the robot (world X = -1.4 m); it adds no joints and preserves the robot's state
layout. The generator accepts `--chair-x` and `--chair-y` to reposition it,
keeps the vendor checkout untouched, and resolves the existing robot mesh paths.
The `/opt/g1-scenes` output must be regenerated after recreating the container.

The chair starts outside the forward-facing camera view. Turn the humanoid
around to bring it into view. For the detection smoke test, generate a separate
scene with `--chair-x 1.4` so the chair is visible immediately.

Use the same SAM 3
arguments with `simulation:=false` and the appropriate deployment network and
odometry settings. Physical-camera segmentation has not yet been validated on
the robot.

If the controller/camera are already running, start perception alone:

```bash
ros2 launch g1_controller g1_sam3.launch.py \
  sam3_python:=/opt/g1-sam3-venv/bin/python \
  sam3_checkpoint:=/opt/g1-models/sam3.pt sam3_prompt:=floor
```

In an already-open RViz, check **Camera → SAM3 Overlay**, or add an Image display
on `/sam3/overlay` with best-effort reliability. Change the prompt at runtime:

```bash
ros2 topic pub --once /sam3/prompt std_msgs/msg/String "data: 'chair'"
# Empty text pauses inference and clears results on the next camera frame:
ros2 topic pub --once /sam3/prompt std_msgs/msg/String "data: ''"
```

| Topic | ROS type | Meaning |
| --- | --- | --- |
| `/camera/color/image_raw` | `sensor_msgs/Image` | RGB input, configurable with `sam3_image_topic` |
| `/sam3/prompt` | `std_msgs/String` | Updated text prompt |
| `/sam3/overlay` | `sensor_msgs/Image` (`rgb8`) | Colored masks, boxes, and confidence labels |
| `/sam3/instance_labels` | `sensor_msgs/Image` (`16UC1`) | 0 = background; positive values identify instances |
| `/sam3/detections` | `vision_msgs/Detection2DArray` | Pixel boxes, prompt as class, confidence, and instance IDs |
| `/sam3/status` | `std_msgs/String` | JSON state, processing duration, source timestamp, and errors |

All three segmentation outputs preserve the source image's timestamp and frame
ID. Subscribe with sensor-data QoS (best effort, volatile, depth 1). Status uses
reliable, transient-local QoS. Empty detections publish an empty array, all-zero
labels, and the original RGB image. Instance IDs restart per frame and match
`Detection2D.id`; they are **not tracking IDs**. Higher-confidence masks win
where masks overlap in the single label image. Boxes/detections retain all
selected instances, including an instance fully occluded in that label image.
Depth-based localization is enabled by default with SAM 3; it does not issue
robot motion commands.

Additional launch settings: `sam3_rate:=2.0`, `sam3_threshold:=0.5`, and
`sam3_device:=cuda` (also accepts `cuda:1` or `cpu`). `sam3_prompt` is the initial
prompt; use the topic to update it. Other settings require restarting the node.
An empty `sam3_checkpoint` uses Hugging Face, requiring checkpoint access and a
cached model or network. Use an explicit local checkpoint for offline operation.
Startup failures terminate the node with an error instead of running an
uninitialized model. The Python launcher prioritizes PyTorch's bundled NVIDIA
libraries for this process, avoiding the cuBLAS/cuBLASLt mismatch encountered
with the container's system CUDA toolkit.

### Object position relative to the pelvis

`sam3:=true` also starts `sam3_localization.py` by default. It combines each
instance label with aligned depth and calibration from the **same capture**
(with up to 20 ms timestamp tolerance for depth/calibration), then uses TF at
the RGB timestamp to express the result in `pelvis`. Delayed SAM 3 outputs are
matched against a bounded 120-frame depth/calibration cache. There is no
fallback to the latest TF when the capture-time transform is unavailable.

Read the highest-confidence valid object's position:

```bash
ros2 topic echo /sam3/target_position --qos-reliability best_effort
```

The message is `geometry_msgs/PointStamped`, with `header.frame_id: pelvis` and
`point: {x, y, z}` in **meters**: +X forward, +Y left, +Z up relative to the pelvis
at capture time. The controller/robot-state publisher must supply the waist
joint transforms connecting `pelvis` to the camera. A torso-mounted camera's
extrinsics alone are insufficient if those joints are moving.

| Topic | Type | Meaning |
| --- | --- | --- |
| `/sam3/target_position` | `geometry_msgs/PointStamped` | Highest-confidence localized match |
| `/sam3/target_visible` | `std_msgs/Bool` | Whether a valid target is currently available |
| `/sam3/detections_3d` | `vision_msgs/Detection3DArray` | All localized instances, matching the 2D IDs and prompt |
| `/sam3/object_markers` | `visualization_msgs/MarkerArray` | Green position markers in RViz |
| `/sam3/localization_status` | `std_msgs/String` | JSON validity and reason for unavailable localization |

In each 3D detection, `results[0].pose.pose.position` is the estimated position.
The estimator erodes mask boundaries, rejects invalid depth and depth outliers,
back-projects valid pixels with camera intrinsics/distortion, and takes the
coordinate-wise median of the transformed visible-surface points. It supports
`16UC1` millimeter depth and `32FC1` meter depth. This is a **visible-surface
position**, not an inferred full-object center. The 3D box represents 5th–95th
percentile extents of that observed surface, aligned to the target frame;
object orientation is not estimated.

No target point is published when localization fails. The 3D array becomes
empty, markers clear, and `target_visible` becomes false if the object leaves
the view, the prompt changes, depth/TF is unavailable, or no fresh result arrives
for 3 seconds. Consumers must check visibility and timestamps instead of
reusing the last point indefinitely. Because the chair starts behind the robot,
it has no camera-derived position until the robot turns to see it. This node
does not maintain an object map while the chair is out of view.

RViz automatically enables **SAM3 Positions** when launched with `sam3:=true`.
Use `sam3_target_frame:=torso_link` for a different output frame, or
`sam3_localize:=false` for segmentation only. The localization node's inputs
are the shared aligned-depth topics, and can be remapped when running it
separately. Both simulation and RealSense use the same code.

### Recreate the SAM 3 environment

The paths under `/opt` persist in this container, but must be provisioned again
if the container is recreated. Inside a network-enabled ROS Jazzy container,
as root, with compatible GPU PyTorch/torchvision already installed:

```bash
apt-get install ros-jazzy-vision-msgs ros-jazzy-cv-bridge python3-venv
cd /root/Workspaces/g1_ros_ws
source /opt/ros/jazzy/setup.bash
python3 -m venv --system-site-packages /opt/g1-sam3-venv
/opt/g1-sam3-venv/bin/python -m pip install \
  -r src/g1_rsl/dependencies/g1_controller/scripts/requirements-sam3.txt
# Make your approved sam3.pt available and pass its container-local path.
colcon build --packages-select g1_controller
```

The requirements pin the official SAM 3 source revision used here. For a
network-isolated simulation container, prepare wheels outside the container
and install them offline into the environment. The node does not change the
controller's C++ inference libraries. Measure controller timing when sharing a
GPU with perception; the default rate is an inference cap, not a latency guarantee.

### Validation

With ROS sourced, run the scheduling/output tests without model weights:

```bash
ROS_DOMAIN_ID=88 python3 -m unittest discover -s tests -p "test_sam3*.py" -v
```

For real checkpoint inference through the live MuJoCo camera and RViz:

```bash
python3 tests/sam3_smoke.py --checkpoint /opt/g1-models/sam3.pt
```

The latter starts only simulator/perception/visualization processes on isolated
ROS domain 89 and Unitree domain 90, checks headers, instance IDs, a nonempty floor mask, RViz
subscription, live depth localization, and live prompt clearing, then stops its processes.
The controller-free smoke test uses `torso_link` because it has no waist joint
publisher; unit tests separately verify pelvis transforms at the capture timestamp. It saves an
overlay preview to `/tmp/g1-sam3-live-overlay.png` inside the container.

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
