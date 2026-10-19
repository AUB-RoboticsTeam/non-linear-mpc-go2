# Vendored package

This package is vendored from the upstream `mujoco_ros2_control` project
(unmodified, core `mujoco_ros2_control` subfolder only; the `mujoco_ros2_control_demos`
package was dropped):

- Upstream: https://github.com/sangteak601/mujoco_ros2_control
- Commit: 8ddb0b624e6f2546d368990ea178f66feb1bc2ff

It provides the `mujoco_ros2_control/MujocoSystem` `ros2_control` hardware
plugin and the `mujoco_ros2_control` node used to simulate the Go2 with
MuJoCo, as an alternative to `gz_ros2_control` (Gazebo). See LICENSE for the
original MIT license.

## Building

MuJoCo itself is not distributed through rosdep/apt. Install it and point
CMake at it before building this package:

```bash
export MUJOCO_DIR=/path/to/mujoco-3.x.x   # extracted MuJoCo release
sudo apt install libglfw3-dev libeigen3-dev
```
