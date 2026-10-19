# Go2 MPC Locomotion on ROS 2 Jazzy

This workspace is a ROS 2 Jazzy migration of the ROS 1
[`leggedcontrol_go2`](https://github.com/WeixianLin-cc/leggedcontrol_go2)
locomotion stack, with support for both Gazebo Sim and MuJoCo.

The goal is to preserve the original control architecture and command workflow:

1. Start the simulator or real robot hardware.
2. Load the legged controller without activating it.
3. Activate the controller explicitly.
4. Send a desired body height with `/cmd_vel`.
5. Select a gait independently.
6. Send planar velocity commands to move in that gait.

The current migration is under active validation. MuJoCo and Gazebo use the
same MPC, state estimator, Pinocchio model, and WBC. Only the simulator-facing
hardware adapters are different.

## Workspace layout

The workspace contains two main source repositories:

```text
src/
├── leggedcontrol_go2/
│   ├── legged_controllers/       ROS 2 controller, target publisher and gait tools
│   ├── legged_interface/         OCS2 optimal-control problem definition
│   ├── legged_estimation/        Base position and velocity estimation
│   ├── legged_wbc/               Whole-body controller
│   ├── legged_common/            Joint/contact interface definitions
│   └── legged_examples/
│       └── legged_unitree/
│           ├── legged_unitree_description/
│           ├── legged_unitree_sim/
│           └── legged_unitree_hw/
│
└── unitree_go2_ros2_jazzy/
    └── mujoco_ros2_control/      MuJoCo ros2_control integration
```

`leggedcontrol_go2` contains the locomotion algorithms. The second repository
provides the MuJoCo-to-`ros2_control` bridge used by this workspace.

## What is `legged_controller`?

`legged_controller` is the ROS 2 `controller_interface` plugin that runs the
complete locomotion control pipeline. It is not a simple joint controller.

Its plugin type is:

```text
legged/LeggedController
```

Activating it starts the active control path:

```text
joint + IMU + contact measurements
              │
              ▼
       state estimation
              │
              ▼
       OCS2 nonlinear MPC
              │
       desired robot motion
       and contact forces
              │
              ▼
     whole-body control (WBC)
              │
       joint feed-forward torque
       + low-gain velocity damping
              │
              ▼
       ros2_control effort
              │
       ┌──────┴──────┐
       ▼             ▼
   Gazebo Sim      MuJoCo
```

Therefore, activating `legged_controller` does activate the MPC-based
locomotion controller. MPC is an internal part of this controller.

The controller immediately produces balancing torques after activation even
before a `/cmd_vel` message is sent. This is expected: it initializes the MPC
with the measured state and the default `STANCE` contact mode, then WBC tries
to support and stabilize the robot.

Activation must not cause one leg to kick violently. A previous ROS 2 migration
bug used joint order `LF, RF, LH, RH`, while the URDF and Pinocchio model use
`LF, LH, RF, RH`. This sent left-hind commands to the right-front leg. The
controller, launch configuration, and YAML now consistently use:

```text
LF_HAA, LF_HFE, LF_KFE,
LH_HAA, LH_HFE, LH_KFE,
RF_HAA, RF_HFE, RF_KFE,
RH_HAA, RH_HFE, RH_KFE
```

## MPC: what it does

MPC means Model Predictive Control. The controller repeatedly predicts the
robot motion over a finite time horizon and optimizes:

- torso position, orientation and velocity;
- foot contact forces;
- leg motion;
- contact-mode constraints;
- friction-cone constraints;
- joint and self-collision costs.

The OCS2 problem is constructed in `legged_interface`. Its main Go2 settings
are in:

```text
legged_controllers/config/go2/task.info
legged_controllers/config/go2/reference.info
legged_controllers/config/go2/gait.info
```

Important settings include:

- MPC horizon and update frequency;
- state and input cost matrices;
- nominal body height and joint pose;
- friction coefficients;
- swing-foot height and timing;
- torque limits;
- WBC task weights;
- estimator noise parameters.

MPC does not directly send motor commands. It returns optimized desired states,
foot forces, and joint velocities. WBC converts those results into commands
that the simulated or real joints can execute.

## Pinocchio: where it is used

Yes, this package uses Pinocchio extensively.

Pinocchio loads the generated Go2 URDF and provides the rigid-body model used
by the controller. It is used for:

- forward kinematics;
- foot positions and velocities;
- frame Jacobians;
- centroidal dynamics;
- mass and inertia calculations;
- mapping between floating-base and centroidal states;
- WBC dynamics and contact constraints;
- state-estimator foot kinematics;
- self-collision geometry.

The primary construction path is:

```text
LeggedController
  └── LeggedInterface
        ├── PinocchioInterface
        ├── CentroidalModelInfo
        ├── foot kinematics
        └── OCS2 optimal-control problem
```

The URDF used by Pinocchio must have exactly the same joint names, joint order,
geometry, inertial parameters, and frame conventions as the simulator model.
A mismatch can make a mathematically valid MPC/WBC output physically wrong.

## WBC: what it does

WBC means Whole-Body Control. This workspace currently defaults to
`WeightedWbc`.

MPC works at the motion/contact-force level. WBC solves the lower-level
whole-body problem using the measured robot state and the current MPC result.
It computes:

- floating-base acceleration;
- joint acceleration;
- contact forces consistent with the active gait;
- joint torques.

It also enforces or tracks:

- rigid-body dynamics;
- stance-foot constraints;
- swing-foot motion;
- friction cones;
- torque limits;
- base acceleration objectives.

The implementation is under:

```text
legged_wbc/src/WbcBase.cpp
legged_wbc/src/WeightedWbc.cpp
legged_wbc/src/HierarchicalWbc.cpp
```

The default weighted controller can be changed at launch with:

```bash
wbc_type:=weighted
```

or:

```bash
wbc_type:=hierarchical
```

The weighted controller should remain the reference until the complete
locomotion pipeline is stable.

## State estimation and sensors

The normal `legged_controller` uses the linear Kalman-filter estimator. It
combines:

- joint position and velocity;
- IMU orientation;
- IMU angular velocity;
- IMU linear acceleration;
- four foot-contact measurements;
- Pinocchio foot kinematics.

It estimates floating-base position and velocity and publishes odometry.

The `legged_cheater_controller` instead reads simulator ground truth. It is
useful for separating controller/model problems from estimator problems, but
it must never be used on real hardware.

Recommended debugging order:

1. Stabilize `legged_cheater_controller` in simulation.
2. Stabilize `legged_controller` with simulated IMU and contacts.
3. Validate the same normal estimator and controller on hardware.

## Hybrid joint command behavior

The ROS 1 simulator exported a custom hybrid joint interface. For every joint,
the controller supplied:

```text
desired position
desired velocity
kp
kd
feed-forward torque
```

The ROS 1 `LeggedHWSim` converted this to:

```text
effort =
  kp * (desired_position - measured_position)
  + kd * (desired_velocity - measured_velocity)
  + feed_forward_torque
```

Standard Gazebo Sim and MuJoCo `ros2_control` systems expose an effort command
instead of the ROS 1 custom hybrid interface. In `simulation_mode`, the ROS 2
`LeggedController` performs the same conversion and writes the resulting
effort.

The migrated controller currently uses the original low-gain behavior:

```text
kp = 0
kd = 3
```

This means WBC torque is the main control input and joint-velocity damping
reduces shocks and oscillation.

Before activation, MuJoCo applies zero effort. The robot is allowed to settle
under gravity, joint damping, and ground contact, matching the original ROS 1
simulator behavior.

## Gait commands and velocity commands are separate

This is an important property inherited from the ROS 1 package.

A gait command selects the contact schedule:

```text
stance
trot
standing_trot
flying_trot
pace
standing_pace
dynamic_walk
static_walk
amble
...
```

`/cmd_vel` selects torso height and velocity:

```text
linear.x   forward/backward velocity
linear.y   lateral velocity
linear.z   absolute desired body height
angular.z  yaw rate
```

Positive `linear.z` is latched. After commanding a height of `0.30`, later
velocity messages with `linear.z: 0.0` keep the 0.30 m target.

With a dynamic gait selected and zero `linear.x/y`, the robot executes that
contact schedule in place. With nonzero `x/y`, it moves in the requested
direction.

The controller already starts with the `STANCE` contact mode. A `stance`
message is not required to make the robot stand.

## ROS 1 versus this ROS 2 migration

The algorithmic structure is intentionally retained:

| Area | Original ROS 1 | Current ROS 2 |
|---|---|---|
| Middleware | `roscpp` | `rclcpp` |
| Control framework | `ros_control` | `ros2_control` |
| Controller lifecycle | `load` + switch service | inactive load + lifecycle activation |
| Gazebo | Gazebo Classic | Gazebo Sim |
| Gazebo control plugin | custom `LeggedHWSim` | `gz_ros2_control` plus effort conversion |
| MuJoCo | not part of original workflow | custom `mujoco_ros2_control` integration |
| MPC | OCS2 SQP MPC | OCS2 SQP MPC ported to ROS 2 |
| Rigid-body model | Pinocchio | Pinocchio |
| WBC | weighted/hierarchical WBC | weighted/hierarchical WBC |
| Estimation | IMU, joints and contacts | same inputs through ROS 2 state interfaces |
| Gait input | interactive ROS 1 node | ROS 2 gait bridge and CLI |
| Motion input | `/cmd_vel` and simple goal | same command topics |
| Sim joint command | native custom hybrid interface | hybrid equation converted to effort |

The largest migration risks are not the MPC equations themselves. They are the
interfaces around them:

- joint ordering;
- foot/contact ordering;
- quaternion and IMU frame conventions;
- joint-axis signs;
- simulator inertial parameters;
- contact-force signs and thresholds;
- controller activation handover;
- update period and simulation time;
- effort limits.

## Build

```bash
cd ~/Downloads/go2222
source /opt/ros/jazzy/setup.bash
source install/setup.bash

colcon build --symlink-install
source install/setup.bash
```

After rebuilding a controller or hardware plugin, completely restart the
simulator. A running process keeps the old shared library loaded.

## Safe MuJoCo launch sequence

Terminal 1:

```bash
cd ~/Downloads/go2222
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch legged_unitree_sim go2_mpc_mujoco.launch.py \
  start_command_console:=false
```

Wait until the controller is configured but inactive.

Terminal 2:

```bash
cd ~/Downloads/go2222
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 control list_controllers
ros2 control switch_controllers --activate legged_controller
ros2 control list_controllers
```

Activate it once. Immediately after activation, observe the initial stance for
several seconds before sending commands.

Stand at 0.30 m:

```bash
ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.30}, angular: {z: 0.0}}"
```

Start with the more conservative standing trot:

```bash
ros2 topic pub -1 /legged_robot_gait_command std_msgs/msg/String \
  "{data: standing_trot}"
```

Walk forward while keeping the latched height:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.20, y: 0.0, z: 0.0}, angular: {z: 0.0}}" -r 10
```

Stop planar movement without changing the gait or height:

```bash
ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {z: 0.0}}"
```

Return to full stance:

```bash
ros2 topic pub -1 /legged_robot_gait_command std_msgs/msg/String \
  "{data: stance}"
```

## Gazebo Sim

Use the same controller and command sequence, replacing the simulator launch:

```bash
ros2 launch legged_unitree_sim go2_mpc_gz.launch.py \
  start_command_console:=false
```

The MPC, Pinocchio model, WBC, gait files, and command topics are shared with
MuJoCo. Gazebo currently defaults to its ground-truth estimator for stability,
so activate the cheater controller:

```bash
ros2 control switch_controllers --activate legged_cheater_controller
```

Contact state interfaces are selected by the simulator launch. Gazebo uses
the boolean interfaces `LF_FOOT/contact`, `LH_FOOT/contact`,
`RF_FOOT/contact`, and `RH_FOOT/contact`. MuJoCo instead uses the four
`*_foot_fts/force.z` force-sensor interfaces. Do not pass the MuJoCo
`contact_sensor_names` list to the Gazebo launch; otherwise activation fails
with an unavailable `FL_foot_fts/force.z` interface.

To test Gazebo with the normal IMU/contact estimator, launch explicitly with
`cheater:=false` and activate the normal controller:

```bash
ros2 launch legged_unitree_sim go2_mpc_gz.launch.py \
  cheater:=false start_command_console:=false

ros2 control switch_controllers --activate legged_controller
```

## MuJoCo elevation/building world

The Gazebo `elevation.sdf` static environment is converted to native MJCF as
`elevation_scene.xml`. Launch the Go2, LiDAR, controller manager, and MPC stack
in that environment with:

```bash
ros2 launch legged_unitree_sim go2_mpc_mujoco_elevation.launch.py \
  start_command_console:=false
```

The controller remains inactive by default:

```bash
ros2 control switch_controllers --activate legged_controller
```

If `elevation.sdf` changes, regenerate the MJCF environment using
`legged_unitree_description/scripts/sdf_static_world_to_mjcf.py`, then rebuild
`legged_unitree_description`.

## Paper-compatible terrain perception

The elevation launch starts the perception front end from Grandia et al.,
“Perceptive Locomotion through Nonlinear Model Predictive Control” by default.
The data path is:

```text
/unitree_lidar -> elevation_mapping_cupy -> /elevation_map_raw
               -> perceptive_terrain_processor
               -> /perceptive_terrain_map + /planar_terrain
               -> terrain_foothold_planner
```

The mapper uses the paper's 4 cm resolution and 20 Hz update rate. The terrain
processor performs minimum-contour-height inpainting, a 3x3 median filter and
PCA classification, connected-component plane extraction, global plane
validation/RANSAC, contour extraction with holes and a one-cell safety margin,
and smooth torso-height reference generation. The configured paper thresholds
are:

- local PCA roughness at most 0.020 m and `normal.z >= 0.82`;
- planar-region roughness at most 0.025 m and `normal.z >= 0.87`;
- at least four cells per planar region.

Validate only the perception input while the simulator and elevation mapper are
running:

```bash
ros2 run legged_controllers check_perception_pipeline.py --duration 5
```

A passing result confirms the 4 cm geometry, at least 18 Hz effective output,
all required classified layers, and valid planar-region polygons. The foothold
planner defaults to `/perceptive_terrain_map` and intentionally rejects a raw
map, preventing unclassified terrain from bypassing this stage.

This completes the paper's 2.5-D elevation-map classification and planar-region
input. The paper's dense 3-D signed-distance field and its direct collision
constraints inside the Go2 NMPC are a separate integration stage; the
perceptive ANYmal implementation is present in the OCS2 source tree but its
`grid_map_sdf`, convex-plane-decomposition, and message dependencies are not
yet connected to `legged_controller`.

## Current limitations

- Stable standing and locomotion still require runtime tuning and validation.
- The controller is a locomotion controller, not a self-righting controller.
  It may rise from a symmetric crouch with usable foot contacts, but it cannot
  reliably recover from lying on its side or back.
- MuJoCo currently reports a shutdown-only exception when forcibly interrupted;
  this is separate from normal physics/control operation.
- Gazebo and MuJoCo contact models are not identical, so final contact and
  friction tuning may need backend-specific values.
- Real Go2 hardware validation must use `simulation_mode:=false`.

## Debugging an unstable activation

Do not tune MPC weights first. Check interface correctness in this order:

1. Joint names and order are `LF, LH, RF, RH`.
2. Measured joint signs match Pinocchio when moving one joint at a time.
3. The stationary IMU quaternion represents a level base.
4. Stationary accelerometer output has the convention expected by the estimator.
5. Foot sensors map `LF, RF, LH, RH` to the correct physical feet.
6. Contact flags are true for loaded feet and false for airborne feet.
7. All MPC, WBC, and effort outputs are finite.
8. Effort limits agree between URDF, WBC, Gazebo, and MuJoCo.
9. Only then adjust MPC/WBC weights, damping, friction, and contact thresholds.

Controller diagnostics print observed and desired body pose, planned contacts,
vertical contact forces, selected joint states, and torque range. These values
should be captured immediately before and after activation when investigating
remaining instability.
