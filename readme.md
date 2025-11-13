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
sudo apt install curl # if you haven't already installed curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt install ros-noetic-desktop-full
source /opt/ros/noetic/setup.bash
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
sudo apt install python3-rosdep
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

Install Mujoco for sim-to-sim visualization.

```bash
cd ~/Workspaces/g1_ws
wget https://github.com/google-deepmind/mujoco/releases/download/3.2.7/mujoco-3.2.7-linux-x86_64.tar.gz
tar -xvzf mujoco-3.2.7-linux-x86_64.tar.gz
rm mujoco-3.2.7-linux-x86_64.tar.gz
cd mujoco-3.2.7
mkdir build && cd build
cmake ..
make -j4
sudo make install
```

Install [Unitree Mujoco plugin](https://github.com/unitreerobotics/unitree_mujoco) to deploy the robot in Mujoco simulation.

```bash
cd ~/Workspaces/g1_ws
git clone https://github.com/unitreerobotics/unitree_mujoco.git
cd unitree_mujoco/simulate
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
You can add the following lines to the `UnitreeSdk2Bridge::SetupJoystick` method in `simulate/src/unitree_sdk2_bridge/unitree_sdk2_bridge.cc` to enable Logitech Gamepad F710 support:

```cpp
    else if (js_type == "logitech")
    {
        js_id_.axis["LX"] = 0; // Left stick axis x
        js_id_.axis["LY"] = 1; // Left stick axis y
        js_id_.axis["RX"] = 3; // Right stick axis x
        js_id_.axis["RY"] = 4; // Right stick axis y
        js_id_.axis["LT"] = 2; // Left trigger
        js_id_.axis["RT"] = 5; // Right trigger
        js_id_.axis["DX"] = 6; // Directional pad x
        js_id_.axis["DY"] = 7; // Directional pad y

        js_id_.button["X"] = 2;
        js_id_.button["Y"] = 3;
        js_id_.button["B"] = 1;
        js_id_.button["A"] = 0;
        js_id_.button["LB"] = 4;
        js_id_.button["RB"] = 5;
        js_id_.button["BACK"] = 6;
        js_id_.button["START"] = 7;
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
rosrun g1_controller g1_ctrl --network <network_interface>
# 1. press [Start] to set the robot to stand up.
# 2. Click the Mujoco window, and then press 8 to make the robot feet touch the ground.
# 3. Press [A] to run the policy.
# 4. Click the Mujoco window, and then press 9 to disable the elastic band.
# 5. Press [Select] to set the robot to passive mode.
```

#### Example

To deploy using the `enp0s31f6` network interface, run:

```bash
rosrun g1_controller g1_ctrl --network enp0s31f6
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
