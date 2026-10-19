# ROS 2 Jazzy Migration Notes

This repository started as a ROS 1 catkin workspace. The ROS 2 migration should be done in two layers:

1. Move package metadata and build files from `catkin` to `ament_cmake`.
2. Rewrite the ROS-facing APIs from ROS 1 / `ros_control` / Gazebo Classic to ROS 2 / `ros2_control` / Gazebo Sim.

The optimal-control and WBC code can stay mostly intact, but the controller, hardware, estimation publishers, launch files, and Gazebo integration need API changes.

## Local Status

The workspace now vendors the OCS2 ROS 2 dependency stack through `dependencies.repos`. The selected OCS2 packages build locally on Jazzy, including:

- `ocs2_ros_interfaces`
- `ocs2_legged_robot`
- `ocs2_legged_robot_ros`
- `ocs2_self_collision_visualization`

The following local packages have been migrated to `ament_cmake` and build with `colcon`:

- `legged_control`
- `legged_unitree_description`
- `qpoases_catkin`
- `legged_interface`
- `legged_wbc`
- `legged_common`
- `legged_hw`
- `legged_estimation`
- `legged_controllers` target trajectory publisher and ROS 2 controller plugin
- `legged_unitree_hw` ROS 2 control hardware plugin for Unitree Go2 DDS low-level state/command
- `legged_unitree_sim` Gazebo Sim launch/config package using the local OCS2-compatible Unitree description

`legged_controllers` now exports `legged/LeggedController` and `legged/LeggedCheaterController` as ROS 2 `controller_interface::ControllerInterface` plugins. The plugin expects ROS 2 control interfaces for hybrid joint commands, joint states, contact states, and the Unitree IMU state. The Unitree hardware plugin now provides this interface shape for Go2 DDS; Gazebo Sim still needs a matching `gz_ros2_control` setup.

`legged_unitree_hw` now exports `legged_unitree_hw/UnitreeHW` as a ROS 2 `hardware_interface::SystemInterface` plugin. It reads Unitree Go2 DDS `rt/lowstate`, writes `rt/lowcmd`, exposes joint/contact/IMU state interfaces to `ros2_control`, and subscribes to `/cmd_vel` for the existing contact-threshold behavior.

The Gazebo Sim migration uses `RobInLabUJI/unitree_go2_ros2_jazzy` as a reference for Gazebo Sim world, spawn, bridge, and sensor plugin patterns. The CHAMP controller packages from that repository are intentionally not used. The local sim path keeps this repository's OCS2 joint naming (`LF_HAA`, `LF_HFE`, etc.) instead of the reference repository's lowercase CHAMP joint naming.

## Real Topic Surface

These are the existing ROS 1 topics that must keep their behavioral meaning after the ROS 2 port:

| Topic | Direction | Current type | Owner |
| --- | --- | --- | --- |
| `/cmd_vel` | subscribe | `geometry_msgs/Twist` | `legged_target_trajectories_publisher`, `legged_unitree_hw` |
| `/move_base_simple/goal` | subscribe | `geometry_msgs/PoseStamped` | `legged_target_trajectories_publisher` |
| `/ground_truth/state` | subscribe | `nav_msgs/Odometry` | cheater estimator |
| `/tracking_camera/odom/sample` | subscribe | `nav_msgs/Odometry` | Kalman estimator |
| `legged_robot_mpc_observation` | publish/subscribe | `ocs2_msgs/mpc_observation` | controller and target publisher |
| `odom` | publish | `nav_msgs/Odometry` | state estimator |
| `pose` | publish | `geometry_msgs/PoseWithCovarianceStamped` | state estimator |
| `/joy` | publish | `sensor_msgs/Joy` | Unitree hardware, not yet re-enabled for Go2 DDS |
| `motor_temperatures` | publish | `std_msgs/Float64MultiArray` | Unitree hardware |
| `kp_cmd` | publish | `std_msgs/Float64MultiArray` | Unitree hardware |

ROS 2 message headers are namespace changes only for these standard messages, for example `geometry_msgs::msg::Twist`. OCS2 message and helper namespaces depend on the ROS 2 OCS2 port used.

## Package Migration Inventory

### Build-system only / low ROS surface

- `legged_interface`: mostly OCS2/pinocchio logic. Needs `ament_cmake`, `ament_target_dependencies`, and ROS 2 OCS2 package availability.
- `legged_wbc`: mostly OCS2/pinocchio/qpOASES logic. Needs `ament_cmake`; tests should move from `catkin_add_gtest` to `ament_add_gtest`.
- `qpoases_catkin`: should either become a normal `ament_cmake` vendor/wrapper package or be replaced by a system/rosdep qpOASES dependency.
- `legged_unitree_description`: can become a pure `ament_cmake` resource package. ROS 1 `.launch` files need ROS 2 Python launch equivalents.
- `legged_unitree_sim`: added as a ROS 2 Gazebo Sim launch/config package. It spawns `legged_unitree_description/urdf/robot_gz.xacro`, starts `gz_ros2_control`, bridges `/clock`, `/imu/data`, and `/ground_truth/state`, and loads generic effort-control test controllers.

### ROS-facing code rewrite required

- `legged_common`: migrated to a ROS 2 interface contract package. It defines hybrid joint command/state interface names and contact state interface names for `ros2_control`.
- `legged_hw`: migrated from ROS 1 `RobotHW` to a ROS 2 `hardware_interface::SystemInterface` base library.
- `legged_unitree_hw`: migrated from the ROS 1 hardware abstraction to a ROS 2 `hardware_interface::SystemInterface` plugin for the Go2 DDS SDK. Runtime validation still needs real robot/network testing and a ROS 2 launch/control configuration.
- `legged_controllers`: migrated from ROS 1 `controller_interface::MultiInterfaceController` to ROS 2 `controller_interface::ControllerInterface`. It uses individual command/state interface claims and keeps OCS2 subscriptions/publishers on a private `rclcpp::Node`.
- `legged_estimation`: migrated to `rclcpp`, ROS 2 message namespaces, `rclcpp::Time`, `rclcpp::Duration`, ROS 2 subscriptions, TF2, and ROS 2 realtime publisher APIs.
- `legged_gazebo`: Gazebo Classic `gazebo_ros_control::RobotHWSim`. Do not port line-by-line. The first Gazebo Sim path is now in `legged_unitree_sim` using `gz_ros2_control`.

## Launch Migration

ROS 1 XML launch files to replace with ROS 2 Python launch:

- `legged_controllers/launch/load_controller.launch`
- `legged_controllers/launch/joy_teleop.launch`
- `legged_examples/legged_unitree/legged_unitree_description/launch/empty_world.launch`
- `legged_examples/legged_unitree/legged_unitree_hw/launch/*.launch`

Controller loading should use ROS 2 commands like:

```bash
ros2 control load_controller --set-state active controllers/legged_controller
```

The exact controller name should match the ROS 2 controller plugin and YAML structure chosen during the controller port.

## Gazebo Sim Migration Boundary

Do not port `legged_gazebo` line-by-line. It is tied to Gazebo Classic types, Classic contact manager APIs, and `gazebo_ros_control`.

For Gazebo Sim:

- Use the external GO2 Gazebo Sim repository as a reference source for Gazebo Sim world, spawn, bridge, and sensor plugin patterns.
- Keep the local OCS2-compatible Unitree description and joint names.
- Use `gz_ros2_control` instead of a Classic `RobotHWSim`.
- Current Gazebo Sim config exposes effort commands for generic Gazebo testing. The OCS2 controller still needs a Gazebo-side hybrid-command adapter or controller change because it commands `position`, `velocity`, `kp`, `kd`, and `feedforward`, while `gz_ros2_control/GazeboSimSystem` consumes standard Gazebo effort/position/velocity command interfaces.
- Recreate contact states as ROS 2 state interfaces or topics, then connect them to the controller. IMU is now also exported as a ros2_control sensor in the GO2 sim xacro, and remains bridged on `/imu/data` for other consumers.

## Recommended Next Order

1. Install or vendor the ROS 2 OCS2 stack and confirm package names for `ocs2_ros_interfaces`, `ocs2_legged_robot`, `ocs2_legged_robot_ros`, and `ocs2_self_collision`.
2. Convert `qpoases_catkin`, `legged_interface`, `legged_wbc`, and `legged_unitree_description` to `ament_cmake`.
3. Redesign `legged_common`/`legged_hw` around ROS 2 control interfaces.
4. Port `legged_estimation` and `legged_target_trajectories_publisher` to `rclcpp`.
5. Add ROS 2 launch/control configuration for the Unitree hardware plugin and validate it against the real robot network.
6. Add the Gazebo Sim adapter pieces that are still missing for the OCS2 controller: contact state interfaces and hybrid-command-to-effort behavior.
