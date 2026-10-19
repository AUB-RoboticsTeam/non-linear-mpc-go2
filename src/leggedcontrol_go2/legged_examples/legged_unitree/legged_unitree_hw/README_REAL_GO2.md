# Real Unitree Go2 MPC bring-up

This package connects the ROS 2 `legged_controller` to the Go2 low-level DDS
topics (`rt/lowstate` and `rt/lowcmd`). It uses the same MPC, WBC, gait command,
target publisher, and `/cmd_vel` interface as the MuJoCo launch, but replaces the
simulator ros2_control system with `legged_unitree_hw/UnitreeHW`.

The launch keeps ROS traffic in the controller-manager process on Fast DDS and
preloads Unitree's matching `/usr/local/lib/libddsc*` pair. This avoids mixing
ROS Jazzy's CycloneDDS C ABI with Unitree's separately installed CycloneDDS C++
wrapper, which otherwise crashes the receive thread.

## Safety design

- Motor output is disarmed after launch, even when the hardware component is active.
- The Go2 motor order (FR, FL, RR, RL) is explicitly mapped to the controller order
  (LF, LH, RF, RH).
- MPC arming requires a completed low-level stand-up, fresh `rt/lowstate`, and
  a finite command from an active controller.
- Unitree high-level motion mode is checked before arming. It is released only when
  `release_mode_on_arm:=true` was explicitly passed at launch.
- Torque, kp, and kd are clamped. Commands ramp in over `arm_ramp_duration`.
- A dedicated hardware thread owns `rt/lowcmd` at `command_stream_rate` (500 Hz
  by default). MPC computation and ROS visualization can no longer interrupt
  the standing or damping stream.
- `controller_command_timeout` (50 ms by default) rejects stale MPC commands.
  If a command stalls near the nominal stance, the hardware returns to its
  autonomous stand hold; otherwise it enters damping.
- Real-hardware launches disable the heavy optimized-trajectory visualization
  and 500 Hz MPC-observation publisher. Simulation launches retain them.
- Stale state, non-finite commands, controller deactivation, and hardware
  shutdown force a safe fallback or disarm output.
- Real controller activation is rejected below 0.18 m body height or beyond 0.60 rad
  roll/pitch. Low-level output is also stopped if the optimized pose, force, or
  torque leaves its real-hardware safety envelope.

The `/unitree_hw/stand_up` service interpolates the measured folded pose to a
slightly taller pose (`HAA=0`, `HFE=0.60`, `KFE=-1.20`) and then holds it.
The targets are configurable with `stand_hfe_target` and `stand_kfe_target`.
Simulation did not need this transition because MuJoCo already spawned near its
nominal pose. `/unitree_hw/damp` continuously
publishes velocity damping. A normal disarm sends one passive command and stops
publishing; none of these operations restores Unitree sport mode.

## Build

```bash
cd ~/Downloads/go2222
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to legged_unitree_hw legged_controllers --symlink-install
source install/setup.bash
```

## Stage 1: state-only verification

Suspend the robot with all feet clear of the floor, keep the remote emergency stop
ready, and connect the computer directly to the Go2 Ethernet network. Find the wired
interface with `ip -br link`; replace `YOUR_ETHERNET_INTERFACE` below.

```bash
ros2 launch legged_unitree_hw go2.launch.py \
  network_interface:=YOUR_ETHERNET_INTERFACE \
  load_mpc_controller:=false \
  start_controller:=false \
  release_mode_on_arm:=false \
  max_torque:=5.0 \
  arm_ramp_duration:=5.0
```

This launch must report fresh Unitree state and `DISARMED`. It must not release sport
mode or command a motor. In another sourced terminal:

```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 control list_controllers
ros2 topic echo /unitree_hw/commands_enabled std_msgs/msg/Bool \
  --qos-durability transient_local --qos-reliability reliable --once
ros2 topic hz /joint_states
ros2 topic echo /joint_states sensor_msgs/msg/JointState --once
ros2 control list_hardware_interfaces
```

Expected controller state is only `joint_state_broadcaster active`; command state
must be `false`. Check in RViz that each
physical leg corresponds to the same named LF/RF/LH/RH joint before proceeding.

## Stage 2: suspended low-torque verification

This locomotion MPC is not a self-righting controller. The hardware layer now
provides the missing folded-to-nominal stand transition. Keep the harness carrying
part of the load, put the feet on clear level ground, and keep the remote emergency
stop in hand throughout this stage.

Then restart with explicit permission to release Unitree's high-level mode:

```bash
ros2 launch legged_unitree_hw go2.launch.py \
  network_interface:=YOUR_ETHERNET_INTERFACE \
  load_mpc_controller:=true \
  start_controller:=false \
  release_mode_on_arm:=true \
  max_torque:=5.0 \
  arm_ramp_duration:=5.0
```

Start the low-level stand transition before activating MPC:

```bash
ros2 service call /unitree_hw/stand_up std_srvs/srv/Trigger "{}"
ros2 topic echo /unitree_hw/stand_state std_msgs/msg/String \
  --qos-durability transient_local --once
ros2 topic echo /unitree_hw/stand_ready std_msgs/msg/Bool \
  --qos-durability transient_local --once
```

Do not proceed until the state is `ready` and the robot is stable. The hardware
will continue holding the nominal pose. Then activate MPC while its motor path is
still gated, select stance, and give it one second to produce a finite policy:

```bash
ros2 control switch_controllers --activate legged_controller
ros2 topic pub -1 /legged_robot_gait_command std_msgs/msg/String "{data: stance}"
ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.34}, angular: {z: 0.0}}"
ros2 control list_controllers
```

Only after `legged_controller` is active and its diagnostics remain finite, arm.
The stand service already performed the required Unitree
`MotionSwitcher::ReleaseMode()` transition. This call hands output from the
standing hold to MPC with a gain/torque ramp:

```bash
ros2 service call /unitree_hw/enable_commands std_srvs/srv/SetBool "{data: true}"
ros2 topic echo /unitree_hw/commands_enabled --once
```

At the conservative settings, verify only that every leg reacts in the correct
direction, there is no single-leg jump, and no stale-state or non-finite-command
fault occurs. Then enter continuous damping (use the physical emergency stop first
if motion is unsafe):

```bash
ros2 service call /unitree_hw/damp std_srvs/srv/Trigger "{}"
```

Do not put the robot on the floor for this 5 Nm mapping test.

## Stage 3: supported stand

With a harness carrying part of the body weight, restart with the normal 30 Nm limit
and a slow ramp. Repeat controller activation, stance, height command, and explicit arm:

```bash
ros2 launch legged_unitree_hw go2.launch.py \
  network_interface:=YOUR_ETHERNET_INTERFACE \
  start_controller:=false \
  release_mode_on_arm:=true \
  max_torque:=30.0 \
  arm_ramp_duration:=3.0
```

Do not test locomotion until the robot holds stance without oscillation, incorrect
joint motion, estimator divergence, watchdog faults, or torque saturation.

## Stage 4: gait and velocity

Keep the harness attached. Select a gait first, then publish a small command continuously:

```bash
ros2 topic pub -1 /legged_robot_gait_command std_msgs/msg/String "{data: trot}"
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.05, y: 0.0, z: 0.34}, angular: {z: 0.0}}"
```

Stop that publisher with Ctrl-C and immediately send zero velocity:

```bash
ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.34}, angular: {z: 0.0}}"
ros2 topic pub -1 /legged_robot_gait_command std_msgs/msg/String "{data: stance}"
```

Increase speed in small increments only after the previous command is repeatable.
The command GUI detects the real-hardware services. Its `STAND` button performs
stand-up, waits for convergence, activates `legged_controller` in stance, and
requests the smooth MPC handoff. In simulation it retains the original stance and
height behavior:

```bash
ros2 launch legged_controllers command_gui.launch.py
```

The same GUI accepts the Go2 handheld remote at the same time. The hardware
interface converts the remote bytes in `rt/lowstate` to the standard ROS topic
`/unitree_hw/joystick` (`sensor_msgs/msg/Joy`). After `STAND` completes and MPC
is armed:

- GUI `W` / `S`: forward / backward; `A` / `D`: strafe; `Q` / `E`: yaw.
- Hold remote `R1` to give the sticks temporary control. Left-stick vertical
  drives forward/backward, left-stick horizontal strafes, and right-stick
  horizontal yaws.
- Release `R1` to command zero velocity and return control to the GUI.
- GUI `SPEED -` / `SPEED +`, or remote `R1` + D-pad down/up, changes all three
  speed limits. Individual forward, lateral, and yaw sliders remain available.
- A missing remote packet for 0.3 seconds commands zero while remote control is
  active. The remote does not stand, arm, choose a gait, or damp the robot;
  those deliberate actions stay in the GUI.

Confirm the bridge before arming:

```bash
ros2 topic info /unitree_hw/joystick
ros2 topic echo /unitree_hw/joystick --once
```

## Emergency software stop

Use the physical emergency stop first. The GUI's emergency stop calls continuous
hardware damping. The equivalent command is:

```bash
ros2 service call /unitree_hw/damp std_srvs/srv/Trigger "{}"
```

Controller deactivation also disarms:

```bash
ros2 control switch_controllers --deactivate legged_controller
```
