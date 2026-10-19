# ROS 2 Jazzy Dependencies

This workspace needs the ROS 2 OCS2 stack from source. The upstream `leggedrobotics/ocs2` `ros2` branch targets Ubuntu 24.04 + ROS 2 Jazzy and contains the packages used by this repository:

- `ocs2_core`
- `ocs2_mpc`
- `ocs2_sqp`
- `ocs2_legged_robot`
- `ocs2_legged_robot_ros`
- `ocs2_ros_interfaces`
- `ocs2_msgs`
- `ocs2_centroidal_model`
- `ocs2_pinocchio_interface`
- `ocs2_self_collision`
- `ocs2_self_collision_visualization`

## Source Repositories

Fetch the required OCS2 repositories with:

```bash
vcs import . < dependencies.repos
```

This imports:

- `src/ocs2` from `https://github.com/leggedrobotics/ocs2.git`, branch `ros2`
- `src/ocs2_robotic_assets` from `https://github.com/leggedrobotics/ocs2_robotic_assets.git`, branch `ros2`

## Local Dependency Audit

Already present in this environment:

- ROS 2 Jazzy
- `colcon`
- `rosdep`
- `ros2_control`
- `ros2_controllers`
- `controller_interface`
- `hardware_interface`
- `realtime_tools`
- `gz_ros2_control`
- `ros_gz`
- `ros_gz_sim`
- `ros_gz_bridge`
- `pinocchio`
- `hpp-fcl`
- standard ROS messages and TF packages
- Unitree SDK2 in `/usr/local` (`unitree_sdk2`, `ddsc`, `ddscxx`)

Missing or not confirmed:

- `ros-jazzy-grid-map`

The upstream OCS2 Jazzy instructions also list these apt dependencies:

```bash
sudo apt update
sudo apt install -y \
  build-essential cmake git \
  python3-colcon-common-extensions python3-rosdep \
  python3-dev pybind11-dev \
  libeigen3-dev libboost-all-dev libglpk-dev \
  libgmp-dev libmpfr-dev libcgal-dev libopencv-dev libpcl-dev \
  liburdfdom-dev \
  ros-jazzy-eigen3-cmake-module \
  ros-jazzy-hpp-fcl \
  ros-jazzy-grid-map \
  ros-jazzy-xacro ros-jazzy-robot-state-publisher ros-jazzy-rviz2
```

## Build Order

After importing dependencies and installing missing apt packages:

```bash
source /opt/ros/jazzy/setup.bash
rosdep update
rosdep install --from-paths src legged_common legged_interface legged_wbc legged_estimation legged_controllers \
  --ignore-src -r -y --skip-keys pinocchio
colcon build --packages-up-to ocs2_legged_robot_ros ocs2_self_collision_visualization
```

Once OCS2 builds, continue with this repository's migrated packages:

```bash
colcon build --packages-select \
  legged_common legged_hw legged_estimation legged_controllers \
  legged_control legged_unitree_description legged_unitree_hw legged_unitree_sim qpoases_catkin \
  legged_interface legged_wbc
```
