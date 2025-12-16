# C++ Deployment Overlays for RSL's G1s

[![Linux platform](https://img.shields.io/badge/platform-linux--64-orange.svg)](https://releases.ubuntu.com/20.04/)


**Maintainers:** Chenhao Li  
**Affiliation:** Robotic Systems Lab, ETH Zurich  
**Contact:** chenhli@leggedrobotics.com

## Installation

Create a workspace.

```bash
mkdir -p ~/Workspaces/g1_ws
cd ~/Workspaces/g1_ws
```

Install dependencies.

```bash
sudo apt update
sudo apt install -y libyaml-cpp-dev libboost-all-dev libeigen3-dev libspdlog-dev libglfw3-dev libxinerama-dev libxcursor-dev libxi-dev libfmt-dev
```

Install [ROS Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu) if you haven't already.

```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt install curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt install ros-noetic-desktop-full
source /opt/ros/noetic/setup.bash
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential python3-catkin-tools
sudo rosdep init
rosdep update
```

Install [Unitree C++ SDK](https://github.com/unitreerobotics/unitree_sdk2).

```bash
git clone https://github.com/unitreerobotics/unitree_sdk2.git
cd unitree_sdk2
mkdir build && cd build
cmake .. -DBUILD_EXAMPLES=OFF # Install on the /usr/local directory
sudo make install
```

Download Mujoco for sim-to-sim visualization.

```bash
cd ~/Workspaces/g1_ws
export MUJOCO_VERSION=3.3.7
wget https://github.com/google-deepmind/mujoco/releases/download/${MUJOCO_VERSION}/mujoco-${MUJOCO_VERSION}-linux-x86_64.tar.gz
tar -xvzf mujoco-${MUJOCO_VERSION}-linux-x86_64.tar.gz
rm mujoco-${MUJOCO_VERSION}-linux-x86_64.tar.gz
```

Install [Unitree Mujoco plugin](https://github.com/unitreerobotics/unitree_mujoco) to deploy the robot in Mujoco simulation.

```bash
cd ~/Workspaces/g1_ws
git clone https://github.com/unitreerobotics/unitree_mujoco.git
cd unitree_mujoco/simulate
ln -s ~/Workspaces/g1_ws/mujoco-${MUJOCO_VERSION} mujoco
mkdir build && cd build
cmake ..
make -j4
```

Adapt the configuration file at `/simulate/config.yaml` for the G1 robot.
- Set `robot` to `g1`.
- Set `robot_scene` to `scene_29dof.xml`.
- Set `domain_id` to 0.
- Set `use_joystick` to 1.
- Set `enable_elastic_hand` to 1.

Unitree Mujoco supports Xbox and Switch gamepad layout for joystick control.
You can add the following lines to the `SetupJoystick` method of class `UnitreeSdk2Bridge` in `simulate_python/unitree_sdk2py_bridge.py` to enable Logitech Gamepad F710 support:

```python
        elif js_type == "logitech":
            self.axis_id = {
                "LX": 0,  # Left stick axis x
                "LY": 1,  # Left stick axis y
                "RX": 3,  # Right stick axis x
                "RY": 4,  # Right stick axis y
                "LT": 2,  # Left trigger
                "RT": 5,  # Right trigger
                "DX": 6,  # Directional pad x
                "DY": 7,  # Directional pad y
            }
            
            self.button_id = {
                "X": 2,
                "Y": 3,
                "B": 1,
                "A": 0,
                "LB": 4,
                "RB": 5,
                "SELECT": 6,
                "START": 7,
            }
```
Remember to recompile the Unitree Mujoco plugin after making these changes.


Compile the robot controller.

```bash
mkdir -p ~/Workspaces/catkin_ws/src
cd ~/Workspaces/catkin_ws/src
git clone git@github.com:leggedrobotics/g1_rsl_cpp.git
catkin build g1_controller
source ~/Workspaces/catkin_ws/devel/setup.bash
```


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

### Identifying the Network Interface

For sim-to-sim, start simulation.
The network interface is set to `lo` (loopback) by default, which means the robot controller will communicate with the Mujoco simulation locally.

```bash
cd ~/Workspaces/g1_ws/unitree_mujoco/simulate/build
./unitree_mujoco
```

For sim-to-real, you need to specify the network interface that connects the robot to the network.
To identify the network interface that connects the robot to the network, run the following command:

```bash
ifconfig
```

Look for the interface name that is connected to the robot (e.g., `enp0s31f6`).



### Deployment

Once you have identified the network interface and prepared the configuration file, you can deploy the system using the following command:

Start the robot controller in a separate terminal.

```bash
cd ~/Workspaces/catkin_ws
roslaunch g1_controller g1_controller.launch network:=<network_interface>
# 1. press [Start] to set the robot to stand up.
# 2. Click the Mujoco window, and then press 8 to make the robot feet touch the ground.
# 3. Press [A] to run the policy.
# 4. Click the Mujoco window, and then press 9 to disable the elastic band.
# 5. Press [Select] to set the robot to passive mode.
```

#### Example

To deploy using the `enp0s31f6` network interface, run:

```bash
roslaunch g1_controller g1_controller.launch network:=enp0s31f6
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

## Livox

Install [Livox-LiDAR-SDK2](https://github.com/leggedrobotics/Livox-LiDAR-SDK2).

```bash
cd ~/Workspaces/g1_ws
git clone git@github.com:leggedrobotics/Livox-LiDAR-SDK2.git
cd Livox-LiDAR-SDK2
mkdir build && cd build
cmake .. && make -j
sudo make install
```

Install [Livox ROS Driver](https://github.com/Livox-SDK/livox_ros_driver) and [Livox ROS Driver 2](https://github.com/leggedrobotics/livox_ros_driver2/tree/unitree_g1).

```bash
cd ~/Workspaces/catkin_ws
git clone https://github.com/Livox-SDK/livox_ros_driver.git
git clone git@github.com:leggedrobotics/livox_ros_driver2.git
cd livox_ros_driver2
git checkout unitree_g1
catkin build livox_ros_driver livox_ros_driver2
source ~/Workspaces/catkin_ws/devel/setup.bash
```

## FAST_LIO

Install [FAST_LIO_LOCALIZATION](https://github.com/deepglint/FAST_LIO_LOCALIZATION_HUMANOID/tree/main).

```bash
sudo apt install -y libblas-dev liblapack-dev libopenblas-dev
git clone https://github.com/deepglint/FAST_LIO_LOCALIZATION_HUMANOID.git
catkin build open3d_loc fast_lio
source ~/Workspaces/catkin_ws/devel/setup.bash
```

## RealSense

Install [realsense-ros](https://github.com/realsenseai/realsense-ros/tree/ros1-legacy).

```bash
sudo apt-get install ros-$ROS_DISTRO-realsense2-camera
cd ~/Workspaces/catkin_ws
git clone https://github.com/IntelRealSense/realsense-ros.git
cd realsense-ros/
git checkout `git tag | sort -V | grep -P "^2.\d+\.\d+" | tail -1`
catkin build realsense2_camera realsense2_description
source ~/Workspaces/catkin_ws/devel/setup.bash
```


## Python Implementation

Check out our Python implementation of the G1 robot deployment:

- [g1_rsl_python](https://github.com/leggedrobotics/g1_rsl_python): Python implementation of the G1 robot deployment.


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
