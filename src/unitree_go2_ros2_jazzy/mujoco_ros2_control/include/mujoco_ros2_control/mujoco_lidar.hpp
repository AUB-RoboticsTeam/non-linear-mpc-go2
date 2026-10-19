// Copyright (c) 2026
// SPDX-License-Identifier: MIT
#pragma once

#include <random>
#include <string>
#include <vector>
#include "mujoco/mujoco.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace mujoco_ros2_control
{
class MujocoLidar
{
public:
  explicit MujocoLidar(rclcpp::Node::SharedPtr node);
  void init(const mjModel *model);
  void update(const mjModel *model, mjData *data, const rclcpp::Time &stamp);

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  int site_id_{-1};
  int body_id_{-1};
  double last_update_time_{-1.0};
  int horizontal_samples_{600};
  int vertical_samples_{30};
  double min_horizontal_angle_{-3.14159};
  double max_horizontal_angle_{3.14159};
  double min_vertical_angle_{-0.785398};
  double max_vertical_angle_{0.785398};
  double min_range_{0.8};
  double max_range_{30.0};
  double noise_stddev_{0.008};
  double update_rate_{10.0};
  std::string frame_id_{"lidar_l1_link"};
  std::mt19937 random_engine_{0x4c314c31};
  std::normal_distribution<double> noise_{0.0, 1.0};
  std::vector<mjtNum> local_directions_;
  std::vector<mjtNum> world_directions_;
  std::vector<mjtNum> ranges_;
  std::vector<int> geom_ids_;
};
}  // namespace mujoco_ros2_control
