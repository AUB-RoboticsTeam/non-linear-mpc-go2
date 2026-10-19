// Copyright (c) 2025 Sangtaek Lee
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#include <chrono>
#include <thread>

#include "mujoco/mujoco.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include "mujoco_ros2_control/mujoco_cameras.hpp"
#include "mujoco_ros2_control/mujoco_lidar.hpp"
#include "mujoco_ros2_control/mujoco_rendering.hpp"
#include "mujoco_ros2_control/mujoco_ros2_control.hpp"

// MuJoCo data structures
mjModel *mujoco_model = nullptr;
mjData *mujoco_data = nullptr;

// main function
int main(int argc, const char **argv)
{
  rclcpp::init(argc, argv);
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared(
    "mujoco_ros2_control_node",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  RCLCPP_INFO_STREAM(node->get_logger(), "Initializing mujoco_ros2_control node...");
  auto model_path = node->get_parameter("mujoco_model_path").as_string();

  // load and compile model
  char error[1000] = "Could not load binary model";
  if (
    std::strlen(model_path.c_str()) > 4 &&
    !std::strcmp(model_path.c_str() + std::strlen(model_path.c_str()) - 4, ".mjb"))
  {
    mujoco_model = mj_loadModel(model_path.c_str(), 0);
  }
  else
  {
    mujoco_model = mj_loadXML(model_path.c_str(), 0, error, 1000);
  }
  if (!mujoco_model)
  {
    mju_error("Load model error: %s", error);
  }

  RCLCPP_INFO_STREAM(node->get_logger(), "Mujoco model has been successfully loaded !");
  // make data
  mujoco_data = mj_makeData(mujoco_model);

  int base_qpos_address = -1;
  for (int joint_id = 0; joint_id < mujoco_model->njnt; ++joint_id)
  {
    if (mujoco_model->jnt_type[joint_id] == mjJNT_FREE)
    {
      base_qpos_address = mujoco_model->jnt_qposadr[joint_id];
      break;
    }
  }

  // Optionally override the floating-base translation before ros2_control
  // records its initial state. This lets different scenes choose a safe spawn
  // without duplicating the robot MJCF.
  if (
    node->has_parameter("initial_base_x") &&
    node->has_parameter("initial_base_y") &&
    node->has_parameter("initial_base_z"))
  {
    if (base_qpos_address >= 0)
    {
      mujoco_data->qpos[base_qpos_address] = node->get_parameter("initial_base_x").as_double();
      mujoco_data->qpos[base_qpos_address + 1] =
        node->get_parameter("initial_base_y").as_double();
      mujoco_data->qpos[base_qpos_address + 2] =
        node->get_parameter("initial_base_z").as_double();
      mj_forward(mujoco_model, mujoco_data);
      RCLCPP_INFO(
        node->get_logger(), "Initial base position: [%.3f, %.3f, %.3f]",
        mujoco_data->qpos[base_qpos_address], mujoco_data->qpos[base_qpos_address + 1],
        mujoco_data->qpos[base_qpos_address + 2]);
    }
  }

  // initialize mujoco control
  auto mujoco_control = mujoco_ros2_control::MujocoRos2Control(node, mujoco_model, mujoco_data);

  mujoco_control.init();
  RCLCPP_INFO_STREAM(
    node->get_logger(), "Mujoco ros2 controller has been successfully initialized !");

  // initialize mujoco visualization environment for rendering and cameras
  if (!glfwInit())
  {
    mju_error("Could not initialize GLFW");
  }
  auto rendering = mujoco_ros2_control::MujocoRendering::get_instance();
  rendering->init(mujoco_model, mujoco_data);
  RCLCPP_INFO_STREAM(node->get_logger(), "Mujoco rendering has been successfully initialized !");

  auto cameras = std::make_unique<mujoco_ros2_control::MujocoCameras>(node);
  cameras->init(mujoco_model);
  auto lidar = std::make_unique<mujoco_ros2_control::MujocoLidar>(node);
  lidar->init(mujoco_model);
  auto tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(node);

  // run main loop, target real-time simulation and 60 fps rendering with cameras around 6 hz
  //
  // glfwSwapInterval(0) (see mujoco_rendering.cpp) disables vsync, so
  // glfwSwapBuffers below does not block -- nothing paces this loop to wall
  // time on its own. Without explicit pacing MuJoCo just steps as fast as
  // the CPU allows (observed 15-25x real time), which desyncs the sim/ROS
  // clock from wall-clock time that OCS2's MPC solve actually consumes:
  // policies come back stale relative to the (much further advanced) "now",
  // and the robot goes unstable. Pace explicitly against a steady_clock
  // reference instead of relying on vsync.
  mjtNum last_cam_update = mujoco_data->time;
  mjtNum last_tf_update = -1.0;
  auto wall_clock_start = std::chrono::steady_clock::now();
  mjtNum sim_time_start = mujoco_data->time;
  while (rclcpp::ok() && !rendering->is_close_flag_raised())
  {
    // advance interactive simulation for 1/60 sec
    mjtNum simstart = mujoco_data->time;
    while (mujoco_data->time - simstart < 1.0 / 60.0)
    {
      mujoco_control.update();
      if (
        base_qpos_address >= 0 &&
        (last_tf_update < 0.0 || mujoco_data->time - last_tf_update >= 0.01))
      {
        // Stamp directly from MuJoCo simulation time.  Using node->now() here
        // can briefly (or permanently, if use_sim_time was not propagated)
        // stamp odom->base with wall time while robot_state_publisher stamps
        // base->link transforms with /clock.  TF then has a connected graph
        // whose edges can never be evaluated at the same instant.
        const rclcpp::Time sensor_stamp(
          static_cast<int64_t>(mujoco_data->time * 1e9), RCL_ROS_TIME);
        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = sensor_stamp;
        transform.header.frame_id = "odom";
        transform.child_frame_id = "base";
        transform.transform.translation.x = mujoco_data->qpos[base_qpos_address];
        transform.transform.translation.y = mujoco_data->qpos[base_qpos_address + 1];
        transform.transform.translation.z = mujoco_data->qpos[base_qpos_address + 2];
        transform.transform.rotation.w = mujoco_data->qpos[base_qpos_address + 3];
        transform.transform.rotation.x = mujoco_data->qpos[base_qpos_address + 4];
        transform.transform.rotation.y = mujoco_data->qpos[base_qpos_address + 5];
        transform.transform.rotation.z = mujoco_data->qpos[base_qpos_address + 6];
        tf_broadcaster->sendTransform(transform);
        last_tf_update = mujoco_data->time;
        // The LiDAR internally limits itself to 10 Hz. Calling it here makes
        // every emitted scan share this transform's exact timestamp.
        lidar->update(mujoco_model, mujoco_data, sensor_stamp);
      }
    }

    // Sleep off any surplus: if simulated time has gotten ahead of wall-clock
    // time since we started, wait for real time to catch up before rendering
    // the next frame and continuing. If the CPU is too slow to keep up,
    // elapsed_wall > elapsed_sim and we just proceed without sleeping.
    const double elapsed_sim = mujoco_data->time - sim_time_start;
    const double elapsed_wall = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - wall_clock_start).count();
    const double surplus = elapsed_sim - elapsed_wall;
    if (surplus > 0)
    {
      std::this_thread::sleep_for(std::chrono::duration<double>(surplus));
    }

    rendering->update();

    // Updating cameras at ~6 Hz
    // TODO(eholum): Break control and rendering into separate processes
    if (simstart - last_cam_update > 1.0 / 6.0)
    {
      cameras->update(mujoco_model, mujoco_data);
      last_cam_update = simstart;
    }
  }

  rendering->close();
  cameras->close();

  // free MuJoCo model and data
  mj_deleteData(mujoco_data);
  mj_deleteModel(mujoco_model);

  return 1;
}
