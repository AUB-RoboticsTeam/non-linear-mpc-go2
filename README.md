# Nonlinear MPC for Unitree Go2

ROS 2 Jazzy workspace for nonlinear model predictive control (NMPC), whole-body control (WBC), state estimation, and simulation of the Unitree Go2 quadruped.

The controller uses OCS2 SQP-MPC to optimize body motion and contact forces, then a weighted or hierarchical WBC converts the plan into joint commands. The same control stack can run against MuJoCo or Gazebo Sim through `ros2_control`.

> [!WARNING]
> This is research software under active validation. Begin in simulation, keep the controller inactive during startup, and activate it only after checking joint, IMU, and contact interfaces. Real-hardware use has not been fully validated.

## Features

- ROS 2 Jazzy and Ubuntu 24.04
- Unitree Go2 centroidal-dynamics NMPC with OCS2
- Pinocchio rigid-body dynamics and kinematics
- Weighted and hierarchical whole-body controllers
- Kalman-filter state estimation from joints, IMU, and foot contacts
- MuJoCo and Gazebo Sim backends
- Runtime gait selection and `/cmd_vel` motion commands
- Elevation mapping and perceptive-terrain processing pipeline

## Control architecture

```text
joint, IMU and contact measurements
                |
                v
        state estimation
                |
                v
       OCS2 nonlinear MPC
    desired motion + contact forces
                |
                v
     whole-body control (WBC)
       joint torque commands
                |
                v
          ros2_control
          /           \
     MuJoCo         Gazebo Sim
```

## Workspace layout

```text
src/
|- leggedcontrol_go2/
|  |- legged_controllers/      ROS 2 controller, gait and command tools
|  |- legged_interface/        OCS2 optimal-control problem
|  |- legged_estimation/       Floating-base state estimation
|  |- legged_wbc/              Whole-body control
|  |- legged_common/           Shared interfaces and definitions
|  |- legged_examples/         Go2 description, simulation and hardware packages
|  `- src/ocs2/                Vendored OCS2 ROS 2 source
|- unitree_go2_ros2_jazzy/     Go2 model and MuJoCo ros2_control bridge
`- elevation_mapping_cupy_ros2_ws/
                               GPU elevation-mapping integration
```

The main Go2 parameters are in:

```text
src/leggedcontrol_go2/legged_controllers/config/go2/task.info
src/leggedcontrol_go2/legged_controllers/config/go2/reference.info
src/leggedcontrol_go2/legged_controllers/config/go2/gait.info
```

## Prerequisites

- Ubuntu 24.04
- ROS 2 Jazzy
- Gazebo Harmonic and `gz_ros2_control` for Gazebo simulation
- MuJoCo 3.x, GLFW, and `ros2_control` packages for MuJoCo simulation
- A CUDA-capable setup for the optional CuPy elevation-mapping pipeline

Install package dependencies after cloning:

```bash
sudo apt update
sudo apt install python3-rosdep libglfw3-dev libeigen3-dev

sudo rosdep init 2>/dev/null || true
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

For a binary MuJoCo release, point CMake to its extracted directory:

```bash
export MUJOCO_DIR=/absolute/path/to/mujoco-3.x.x
```

## Build

```bash
git clone https://github.com/AUB-RoboticsTeam/non-linear-mpc-go2.git
cd non-linear-mpc-go2

source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

After rebuilding a controller or simulator plugin, fully restart the simulator so it loads the new shared library.

## Important launch commands

These are the primary launch commands for hardware and simulation.

### Go2 hardware

```bash
ros2 launch legged_unitree_hw go2.launch.py \
  network_interface:=eno1 \
  load_mpc_controller:=true \
  start_controller:=false \
  release_mode_on_arm:=true \
  max_torque:=30.0 \
  arm_ramp_duration:=5.0
```

### MuJoCo elevation world

```bash
ros2 launch legged_unitree_sim go2_mpc_mujoco_elevation.launch.py \
  rviz:=true \
  start_controller:=true \
  start_joint_state_broadcaster:=true \
  start_command_console:=false
```

### Gazebo

```bash
ros2 launch legged_unitree_sim go2_mpc_gz.launch.py \
  rviz:=true \
  start_joint_state_broadcaster:=true \
  start_controller:=false \
  start_command_console:=false
```

### Command GUI and joystick

Start the command GUI in a separate terminal:

```bash
ros2 launch legged_controllers command_gui.launch.py
```

To arm using the joystick, hold **R1**.

## Quick start: MuJoCo

Start the simulator and load the controller in an inactive state:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch legged_unitree_sim go2_mpc_mujoco.launch.py \
  start_command_console:=false
```

In a second terminal, activate the controller once the robot and interfaces are ready:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 control list_controllers
ros2 control switch_controllers --activate legged_controller
```

Command a 0.30 m body height, select standing trot, and walk forward:

```bash
ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.30}, angular: {z: 0.0}}"

ros2 topic pub -1 /legged_robot_gait_command std_msgs/msg/String \
  "{data: standing_trot}"

ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.20, y: 0.0, z: 0.0}, angular: {z: 0.0}}" -r 10
```

Stop motion and return to stance:

```bash
ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {z: 0.0}}"

ros2 topic pub -1 /legged_robot_gait_command std_msgs/msg/String \
  "{data: stance}"
```

`linear.z` is the desired body height and is latched when positive. Gait selection and velocity commands are independent.

## Quick start: Gazebo Sim

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch legged_unitree_sim go2_mpc_gz.launch.py \
  start_command_console:=false
```

Gazebo currently defaults to the simulator ground-truth (`cheater`) estimator for a stable MPC pipeline, so activate:

```bash
ros2 control switch_controllers --activate legged_cheater_controller
```

To develop against the normal IMU/contact estimator instead:

```bash
ros2 launch legged_unitree_sim go2_mpc_gz.launch.py \
  cheater:=false start_command_console:=false
ros2 control switch_controllers --activate legged_controller
```

The MuJoCo launch uses the normal `legged_controller` by default. Never use the cheater controller on real hardware.

## Terrain perception

Launch the MuJoCo elevation/building world and perception front end:

```bash
ros2 launch legged_unitree_sim go2_mpc_mujoco_elevation.launch.py \
  start_command_console:=false
```

Validate the elevation-map input while the simulator and mapper are running:

```bash
ros2 run legged_controllers check_perception_pipeline.py --duration 5
```

The current pipeline provides 2.5-D elevation-map classification and planar regions. Dense 3-D signed-distance-field collision constraints are not yet connected to the Go2 NMPC.

## Safety and current limitations

- Stable standing and locomotion still require runtime validation and tuning.
- The controller is not a self-righting controller.
- Gazebo and MuJoCo contact dynamics may require backend-specific tuning.
- Verify the joint order `LF, LH, RF, RH`, axis signs, IMU convention, foot mapping, and effort limits before tuning MPC weights.
- Use `simulation_mode:=false` for any real-hardware integration.

For the controller design, ROS 1/ROS 2 migration notes, simulator interface details, and debugging checklist, see [the detailed MPC guide](README_MPC_ROS2.md).

## Acknowledgements and licensing

This workspace builds on [OCS2](https://github.com/leggedrobotics/ocs2), [legged_control](https://github.com/qiayuanliao/legged_control), [elevation_mapping_cupy_ros2](https://github.com/amilearning/elevation_mapping_cupy_ros2), and the [Unitree Go2 ROS 2 Jazzy](https://github.com/RobInLabUJI/unitree_go2_ros2_jazzy) ecosystem. Upstream components retain their respective licenses; consult the license files in each source directory before redistribution or deployment.
