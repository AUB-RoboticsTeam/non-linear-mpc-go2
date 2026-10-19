//
// Created by codex.
// ROS 2 force/torque state bridge for Gazebo contact topics.
//

#pragma once

#include <array>
#include <map>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include <gz_ros2_control/gz_ros2_control/gz_system_interface.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <rclcpp/rclcpp.hpp>
#include <ros_gz_interfaces/msg/contacts.hpp>

#include <gz/sim/EntityComponentManager.hh>

namespace legged {
namespace sim {

class FootContactSensor : public gz_ros2_control::GazeboSimSystemInterface {
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
  void subscribeTopics();

  struct WrenchState {
    std::array<double, 6> exported{};
    std::array<double, 6> pending{};
  };

  std::vector<std::string> sensorNames_;
  std::unordered_map<std::string, std::string> topics_;
  std::unordered_map<std::string, std::string> collisions_;
  std::map<std::string, WrenchState> wrenchStates_;
  std::mutex wrenchMutex_;
  std::vector<rclcpp::Subscription<ros_gz_interfaces::msg::Contacts>::SharedPtr> subscriptions_;
};

}  // namespace sim
}  // namespace legged
