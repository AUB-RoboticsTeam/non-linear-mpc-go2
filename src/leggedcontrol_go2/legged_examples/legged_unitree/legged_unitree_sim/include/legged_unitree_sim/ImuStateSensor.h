//
// Created by codex.
// ROS 2 IMU state bridge for Gazebo IMU topics.
//

#pragma once

#include <map>
#include <string>
#include <unordered_map>
#include <vector>

#include <gz_ros2_control/gz_ros2_control/gz_system_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

namespace legged {
namespace sim {

class ImuStateSensor : public gz_ros2_control::GazeboSimSystemInterface {
 public:
  using CallbackReturn = ::hardware_interface::CallbackReturn;
  using return_type = ::hardware_interface::return_type;

  CallbackReturn on_init(const ::hardware_interface::HardwareComponentInterfaceParams& params) override;
  std::vector<::hardware_interface::StateInterface::ConstSharedPtr> on_export_state_interfaces() override;
  std::vector<::hardware_interface::CommandInterface::SharedPtr> on_export_command_interfaces() override;

  bool initSim(rclcpp::Node::SharedPtr& model_nh, std::map<std::string, gz::sim::Entity>& joints,
               const ::hardware_interface::HardwareInfo& hardware_info, gz::sim::EntityComponentManager& ecm,
               unsigned int update_rate) override;
  return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;
  return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

 private:
  void subscribeTopic();
  void imuCallback(const sensor_msgs::msg::Imu::ConstSharedPtr& msg);

  std::string sensorName_;
  std::string topicName_;
  std::unordered_map<std::string, double> stateValues_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_;
};

}  // namespace sim
}  // namespace legged
