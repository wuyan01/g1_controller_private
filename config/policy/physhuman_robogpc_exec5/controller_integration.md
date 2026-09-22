# Five-step RoboGPC controller profile

These files are staged for installation into the current `g1_controller`
repository. Staging does not change the external controller or its running
processes.

| Staged file | Destination relative to `g1_controller` |
| --- | --- |
| `config/config_robogpc_exec5.yaml` | `config/config_robogpc_exec5.yaml` |
| `launch/g1_controller_robogpc_exec5.launch.py` | `launch/g1_controller_robogpc_exec5.launch.py` |

The source repository is
`/home/wuyan/Workspaces/g1_ros_ws/src/g1_rsl/dependencies/g1_controller`.

## Changes from the existing one-step profile

The exact comparison is in `controller_profile_comparison.patch`. It contains
three changes:

1. The new YAML's header names the five-step launch file.
2. `FSM.Physhuman_RoboGPC.policy_dir` points to
   `config/policy/physhuman_robogpc_exec5`.
3. The new launch file passes `config_robogpc_exec5.yaml` to the existing
   simulation launch file.

All mode names, IDs, joystick bindings, joint settings, and launch arguments
remain identical. The original one-step files stay in place. Python syntax was
checked with `ast.parse`; parsed YAML was compared to the original and differs
only in the policy directory.

## Installation and build

After export validation passes, install the model bundle as the new policy
directory `config/policy/physhuman_robogpc_exec5`. Include its `exported/` and
`params/` directories, README, and metadata. Copy the two staged profile files to
their destinations above. Do not copy the staging `integration/` directory into
itself.

The current controller already has the required stateful-runner reset hook.
The runner allocates its cache from the ONNX input shape, and the observation
manager computes the noise width as `state_dim * num_future_steps`. This variant
therefore needs no additional C++ changes. Its matching deployment YAML supplies
five noise tokens and the model declares its 160-channel cache.

The package's CMake install rules copy the `config` and `launch` directories.
Run the incremental build inside the existing `g1-robogpc-sim` container so ROS
can find the newly added files:

```bash
source /opt/ros/jazzy/setup.bash
export LD_LIBRARY_PATH=/usr/local/lib:${LD_LIBRARY_PATH:-}
cd /root/Workspaces/g1_ros_ws
colcon build --packages-up-to g1_controller
source install/setup.bash
```

After stopping any existing controller process, launch the five-step profile in
the same container as MuJoCo:

```bash
ros2 launch g1_controller g1_controller_robogpc_exec5.launch.py \
  simulation:=true network:=lo rviz:=false
```

Xbox controls are unchanged: Menu/Start enters FixStand from Passive; after the
stand transition, hold LB for two seconds and tap D-pad Up to enter RoboGPC.
View/Back returns to Passive. This profile selects the five-step policy;
`g1_controller_robogpc.launch.py` continues to select the one-step policy.
