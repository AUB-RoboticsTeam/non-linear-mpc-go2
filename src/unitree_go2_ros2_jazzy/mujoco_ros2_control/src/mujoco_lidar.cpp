// Copyright (c) 2026
// SPDX-License-Identifier: MIT
#include "mujoco_ros2_control/mujoco_lidar.hpp"

#include <cmath>
#include <limits>
#include <utility>
#include "sensor_msgs/point_cloud2_iterator.hpp"

namespace mujoco_ros2_control
{
MujocoLidar::MujocoLidar(rclcpp::Node::SharedPtr node) : node_(std::move(node)) {}

void MujocoLidar::init(const mjModel *model)
{
  site_id_ = mj_name2id(model, mjOBJ_SITE, "lidar_l1_site");
  if (site_id_ < 0)
  {
    RCLCPP_WARN(node_->get_logger(), "No lidar_l1_site in MJCF; LiDAR is disabled");
    return;
  }
  body_id_ = model->site_bodyid[site_id_];
  publisher_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/unitree_lidar", 1);
  const int ray_count = horizontal_samples_ * vertical_samples_;
  local_directions_.resize(3 * ray_count);
  world_directions_.resize(3 * ray_count);
  ranges_.resize(ray_count);
  geom_ids_.resize(ray_count);

  for (int vertical = 0; vertical < vertical_samples_; ++vertical)
  {
    const double vf = vertical_samples_ == 1 ? 0.5 :
      static_cast<double>(vertical) / (vertical_samples_ - 1);
    const double elevation = min_vertical_angle_ + vf *
      (max_vertical_angle_ - min_vertical_angle_);
    const double cos_elevation = std::cos(elevation);
    for (int horizontal = 0; horizontal < horizontal_samples_; ++horizontal)
    {
      const double hf = horizontal_samples_ == 1 ? 0.5 :
        static_cast<double>(horizontal) / (horizontal_samples_ - 1);
      const double azimuth = min_horizontal_angle_ + hf *
        (max_horizontal_angle_ - min_horizontal_angle_);
      const int ray = vertical * horizontal_samples_ + horizontal;
      local_directions_[3 * ray] = cos_elevation * std::cos(azimuth);
      local_directions_[3 * ray + 1] = cos_elevation * std::sin(azimuth);
      local_directions_[3 * ray + 2] = std::sin(elevation);
    }
  }
  RCLCPP_INFO(node_->get_logger(), "Unitree L1: %dx%d rays at %.1f Hz, %.1f..%.1f m",
    horizontal_samples_, vertical_samples_, update_rate_, min_range_, max_range_);
}

void MujocoLidar::update(
  const mjModel *model, mjData *data, const rclcpp::Time &stamp)
{
  if (!publisher_ || update_rate_ <= 0.0 ||
    (last_update_time_ >= 0.0 && data->time - last_update_time_ < 1.0 / update_rate_))
  {
    return;
  }
  last_update_time_ = data->time;

  sensor_msgs::msg::PointCloud2 cloud;
  // Use the exact timestamp of the odom->base transform published in the
  // same simulation cycle. A newer cloud stamp makes RViz wait for a future
  // transform and causes visible blinking.
  cloud.header.stamp = stamp;
  cloud.header.frame_id = frame_id_;
  cloud.height = static_cast<uint32_t>(vertical_samples_);
  cloud.width = static_cast<uint32_t>(horizontal_samples_);
  cloud.is_bigendian = false;
  cloud.is_dense = false;
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(static_cast<std::size_t>(horizontal_samples_) * vertical_samples_);
  sensor_msgs::PointCloud2Iterator<float> out_x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> out_y(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> out_z(cloud, "z");

  const mjtNum *origin = data->site_xpos + 3 * site_id_;
  const mjtNum *rotation = data->site_xmat + 9 * site_id_;
  const mjtByte geom_groups[mjNGROUP] = {1, 1, 0, 1, 1, 1};
  const float nan = std::numeric_limits<float>::quiet_NaN();
  const int ray_count = horizontal_samples_ * vertical_samples_;

  for (int ray = 0; ray < ray_count; ++ray)
  {
    mju_mulMatVec3(
      world_directions_.data() + 3 * ray, rotation,
      local_directions_.data() + 3 * ray);
  }

  mj_multiRay(
    model, data, origin, world_directions_.data(), geom_groups, 1, body_id_,
    geom_ids_.data(), ranges_.data(), nullptr, ray_count, max_range_);

  for (int ray = 0; ray < ray_count; ++ray)
  {
    double range = ranges_[ray];
    if (geom_ids_[ray] >= 0) range += noise_stddev_ * noise_(random_engine_);
    if (geom_ids_[ray] < 0 || range < min_range_ || range > max_range_)
    {
      *out_x = nan; *out_y = nan; *out_z = nan;
    }
    else
    {
      *out_x = static_cast<float>(range * local_directions_[3 * ray]);
      *out_y = static_cast<float>(range * local_directions_[3 * ray + 1]);
      *out_z = static_cast<float>(range * local_directions_[3 * ray + 2]);
    }
    ++out_x; ++out_y; ++out_z;
  }
  publisher_->publish(cloud);
}
}  // namespace mujoco_ros2_control
