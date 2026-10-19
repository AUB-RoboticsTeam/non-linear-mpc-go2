# Perceptive Go2 locomotion

This package adapts the architecture in Grandia et al., *Perceptive
Locomotion through Nonlinear Model Predictive Control*, to the ROS 2 Go2
controller and MuJoCo simulation.

## Implemented data flow

```text
Unitree L1 -> Elevation Mapping CuPy -> steppability/plane checks
           -> nominal foothold + velocity correction
           -> convex inner region + terrain normal + swing clearance
           -> OCS2 nonlinear MPC -> weighted whole-body QP -> joint torque
```

`terrain_foothold_planner.py` rejects unobserved, rough, steep, high-cost and
kinematically unreachable candidates. Around every nominal touchdown it grows
a four-sided convex inner approximation, checking the complete physical foot
support patch after every expansion. It publishes the polygon counter-clockwise
in `mpc_world`.

`FootholdPlan.msg` carries both the soft nominal touchdown and the selected
plane:

- lift-off and nominal touchdown positions;
- minimum terrain-clearing swing apex;
- plane origin and surface normal;
- counter-clockwise convex polygon;
- independent validity flags for point and region.

The OCS2 cost implements the paper's contact inequality

```text
A_t p_foot(x) + b_t >= 0
```

with a relaxed barrier. The exact foot position remains an MPC decision. The
nominal point has weight 30 in x/y and biases the solution without pinning it
to one map cell. Region constraints begin only in the stance following the
corresponding swing and are never applied to a foot still supporting the body.

Other integrated behavior:

- configuration-dependent full centroidal dynamics via Pinocchio;
- Raibert/inverted-pendulum velocity-error foothold correction;
- terrain-derived body height, roll and pitch reference;
- apex selection from the highest mapped point along the complete swing;
- region freezing through the active swing/stance plan;
- friction, joint, torque and self-collision constraints already present in
  the OCS2/WBC stack;
- measured-contact fusion in WBC for early and late touchdown;
- stance/zero-velocity fail-safe when mapping, touchdown or TF validation
  fails.

## Build

```bash
cd ~/Downloads/go2222
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select \
  legged_terrain_msgs legged_interface legged_controllers legged_unitree_sim
source install/setup.bash
```

## Integrated launch

The elevation-world launch now starts MuJoCo, RViz when requested, Elevation
Mapping using the workspace's GPU-compatible wrapper, and the perceptive
planner:

```bash
ros2 launch legged_unitree_sim go2_mpc_mujoco_elevation.launch.py \
  rviz:=true \
  start_controller:=true \
  start_joint_state_broadcaster:=true
```

Disable individual components when debugging:

```bash
start_elevation_mapping:=false
start_perceptive_planner:=false
```

The planner starts passively. It will not move the robot merely because a map
exists.

## One final stair trial

Do not run another foothold planner at the same time.

1. Wait until `/elevation_map_raw`, `/legged_robot_mpc_observation` and
   `/legged_robot/planned_mode` are active.
2. Stand at 30 cm:

   ```bash
   ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist \
     "{linear: {x: 0.0, y: 0.0, z: 0.30}, angular: {z: 0.0}}"
   ```

3. Inspect without altering the controller:

   ```bash
   ros2 run legged_controllers terrain_foothold_planner.py \
     --dry-run --forward 0.14 --search-radius 0.12 --timeout 8
   ```

   Every required foot must report `region_vertices=4`.

4. Start the slow contact schedule:

   ```bash
   ros2 topic pub -1 /legged_robot_gait_command std_msgs/msg/String \
     "{data: stair_crawl}"
   ```

5. Command forward motion. The passive continuous planner replans at
   all-feet-stance boundaries:

   ```bash
   ros2 topic pub -r 2 /cmd_vel geometry_msgs/msg/Twist \
     "{linear: {x: 0.03, y: 0.0, z: 0.30}, angular: {z: 0.0}}"
   ```

Stop with Ctrl-C, then command zero velocity and `stance`.

For a single-process autonomous validation, launch with
`start_perceptive_planner:=false` and run:

```bash
ros2 run legged_controllers terrain_foothold_planner.py \
  --autonomous --forward 0.14 --search-radius 0.12 \
  --autonomous-speed 0.03
```

This command installs the first safe plan before starting `stair_crawl`,
validates every touchdown, and returns to stance on completion or failure.

## Deliberate differences from the ANYmal experiment

This is a Go2 adaptation, not a binary copy of the ANYmal controller:

- Go2 geometry, mass, joint limits and actuator limits are used.
- The existing centroidal OCS2 state/input layout is retained rather than
  replacing the entire ROS 2 controller ABI with the paper's loop-shaped
  48-state ANYmal interface.
- Convex regions are conservative four-sided inner approximations. This avoids
  unsafe convex hulls across stair edges and holes.
- Elevation Mapping CuPy supplies normal and traversability layers, replacing
  the paper's CPU RANSAC front end.
- Swing-foot terrain clearance is enforced through the existing normal-motion
  constraint and terrain-adapted apex. A global voxel SDF for knee-terrain
  collision is not fabricated from unavailable map layers; robot
  self-collision remains enforced.

These differences preserve the paper's planning/optimization boundary while
remaining consistent with the working Go2 ROS 2 and MuJoCo interfaces.
