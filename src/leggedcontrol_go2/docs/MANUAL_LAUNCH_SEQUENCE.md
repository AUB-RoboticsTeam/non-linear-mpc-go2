# Manual launch sequence (ROS 1 parity)

This is the ROS 2 equivalent of the original ROS 1 `legged_control` quick-start
sequence, run as separate manual steps instead of one all-in-one launch file.
Use this when debugging -- it lets you stop between "controller loaded" and
"controller active" and inspect state, exactly like the ROS 1 workflow did.

The all-in-one launches (`go2_mpc_gz.launch.py`, `go2_mpc_mujoco.launch.py`)
automate steps 1-4 with timers. Use those once things are stable; use the
manual sequence below while they aren't.

ROS 1 for reference:

```bash
export ROBOT_TYPE=a1
roslaunch legged_unitree_description empty_world.launch      # or legged_unitree_hw legged_unitree_hw.launch
roslaunch legged_controllers load_controller.launch cheater:=false
rosservice call /controller_manager/switch_controller "start_controllers: ['controllers/legged_controller']
stop_controllers: ['']
strictness: 0
start_asap: false
timeout: 0.0"
# then type a gait name in the load_controller.launch terminal, and drive with /cmd_vel
```

## 1. Simulation or hardware

**Gazebo:**
```bash
ros2 launch legged_unitree_description empty_world.launch.py robot_type:=go2 sim_backend:=gazebo
```

**MuJoCo:**
```bash
ros2 launch legged_unitree_description empty_world.launch.py robot_type:=go2 sim_backend:=mujoco
```

**Hardware** (never use `cheater:=true` past this point on real hardware):
```bash
ros2 launch legged_unitree_hw legged_unitree_hw.launch.py robot_type:=go2
```

Gazebo and MuJoCo both host `controller_manager` in-process (via
`gz_ros2_control` / `mujoco_ros2_control`), same as ROS 1's
`gazebo_ros_control` plugin did -- no separate `ros2_control_node` needed for
sim. Hardware launches its own `ros2_control_node` since there's no sim
plugin to host it.

## 2. Load the controller (inactive)

```bash
ros2 launch legged_controllers load_controller.launch.py robot_type:=go2 cheater:=false \
    simulation_mode:=true   # simulation_mode:=false on hardware
```

`cheater:=true` only works in sim (it reads ground-truth state instead of the
estimator, same restriction as ROS1's `legged_cheater_controller`).

`start_controller` now defaults to `false`, matching ROS1's
`controller_manager load` verb: this spawns the controller and loads it
(state = inactive), but does not activate it. `joint_state_broadcaster`, the
gait command bridge, and `legged_target_trajectories_publisher` all still
start immediately, same as ROS1.

## 3. Activate the controller

ROS 2's `SwitchController` service renamed the ROS1 fields
(`start_controllers`→`activate_controllers`,
`stop_controllers`→`deactivate_controllers`, `start_asap`→`activate_asap`):

```bash
ros2 service call /controller_manager/switch_controller \
    controller_manager_msgs/srv/SwitchController \
    "{activate_controllers: ['legged_controller'], deactivate_controllers: [], strictness: 2, activate_asap: false, timeout: {sec: 0, nanosec: 0}}"
```

(`strictness: 2` = `STRICT`, same numeric value as ROS1's `STRICT`.) Swap
`legged_controller` for `legged_cheater_controller` if you loaded with
`cheater:=true` -- **sim only, never on hardware.**

Equivalent via the `ros2 control` CLI (like ROS1's `rqt_controller_manager`):

```bash
ros2 control switch_controllers --activate legged_controller
ros2 control list_controllers          # check state
```

## 4. Set the gait

ROS1 typed the gait name directly into the `load_controller.launch` terminal
(`legged_robot_gait_command` read stdin). Here that's a separate console
talking to the gait bridge that `load_controller.launch.py` already started:

```bash
ros2 run legged_controllers legged_gait_command_cli.py
gait> list
gait> stance
```

(`legged_cheater_console.py` also exists and additionally wraps step 3's
service call plus Gazebo pause/pose-reset helpers -- it's Gazebo-specific and
not needed for the manual/MuJoCo/hardware flow above.)

## 5. Drive with /cmd_vel

Same mapping as ROS1 -- `linear.x`/`linear.y`/`angular.z` drive the walking
target, `linear.z` is what stands the robot up from its default joint
configuration (see `TargetTrajectoriesPublisher::cmdVelCallback`, unchanged
from ROS1):

```bash
# stand up
ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist "{linear: {z: 0.3}}"

# walk forward
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}}" -r 10

# or use the interactive console / teleop instead of one-shot topic pub
ros2 run legged_controllers legged_cheater_console.py   # sim only
ros2 launch legged_controllers joy_teleop.launch.py      # gamepad, all backends
```

`linear.z` is an absolute body-height command. A positive value is latched, so
later planar commands with `linear.z: 0.0` keep the requested height:

```bash
# stand to 0.30 m and keep that height
ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist "{linear: {z: 0.30}}"

# select a gait independently (zero x/y means gait in place)
ros2 topic pub -1 /legged_robot_gait_command std_msgs/msg/String "{data: trot}"

# move forward in the selected gait while retaining 0.30 m height
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.30}}" -r 10
```

As in ROS 1, gait selection and the torso target are separate. The controller
starts with a `STANCE` contact schedule; do not send `stance` merely to make it
stand up. Select `stance`, `trot`, `standing_trot`, `dynamic_walk`, or
`static_walk` according to the contact pattern you want. A dynamic gait with
zero planar velocity runs in place.

`/move_base_simple/goal` (`geometry_msgs/msg/PoseStamped`) still works too.

## Simulation command adapter

The ROS 1 `LeggedHWSim` converted the hybrid command to effort as
`kp*(q_des-q) + kd*(v_des-v) + feedforward`. In ROS 2, `simulation_mode`
performs the same conversion inside `LeggedController` and sends the resulting
standard effort command to either `gz_ros2_control` or `mujoco_ros2_control`.
Real hardware must use `simulation_mode:=false` so it receives the native
hybrid command interfaces.
