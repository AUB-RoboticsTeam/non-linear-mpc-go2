//
// Created by codex.
//

#include "legged_unitree_sim/ImuStateSensor.h"

#include <pluginlib/class_list_macros.hpp>

namespace legged {
namespace sim {
ImuStateSensor::CallbackReturn ImuStateSensor::on_init(const ::hardware_interface::HardwareComponentInterfaceParams& params) {
  if (::hardware_interface::SystemInterface::on_init(params) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  if (get_hardware_info().sensors.empty()) {
    RCLCPP_ERROR(get_logger(), "IMU state bridge expects one sensor entry.");
    return CallbackReturn::ERROR;
  }

  const auto& sensor = get_hardware_info().sensors.front();
  sensorName_ = sensor.name;
  const auto topicParameter = sensor.parameters.find("topic");
  topicName_ = topicParameter != sensor.parameters.end() && !topicParameter->second.empty()
                   ? topicParameter->second
                   : "/imu/data";

  const std::vector<std::string> interfaces = {"orientation.x",      "orientation.y",      "orientation.z",      "orientation.w",
                                              "angular_velocity.x", "angular_velocity.y", "angular_velocity.z", "linear_acceleration.x",
                                              "linear_acceleration.y", "linear_acceleration.z"};
  for (const auto& iface : interfaces) {
    stateValues_[iface] = 0.0;
  }

  if (sensor.state_interfaces.size() != interfaces.size()) {
    RCLCPP_ERROR(get_logger(), "IMU sensor %s must expose %zu state interfaces.", sensor.name.c_str(), interfaces.size());
    return CallbackReturn::ERROR;
  }

  for (size_t i = 0; i < interfaces.size(); ++i) {
    if (sensor.state_interfaces[i].name != interfaces[i]) {
      RCLCPP_ERROR(get_logger(), "IMU sensor %s interface mismatch at index %zu: expected %s", sensor.name.c_str(), i,
                   interfaces[i].c_str());
      return CallbackReturn::ERROR;
    }
  }

  // gz_ros2_control can call initSim() before on_init() for a separately
  // declared sensor component, so subscribe after either callback once both
  // the ROS node and parsed topic are available.
  subscribeTopic();
  RCLCPP_INFO(get_logger(), "Initialized IMU bridge for sensor '%s' from '%s'.", sensorName_.c_str(), topicName_.c_str());
  return CallbackReturn::SUCCESS;
}

std::vector<::hardware_interface::StateInterface::ConstSharedPtr> ImuStateSensor::on_export_state_interfaces() {
  std::vector<::hardware_interface::StateInterface::ConstSharedPtr> state_interfaces;
  state_interfaces.reserve(stateValues_.size());
  for (auto& [name, value] : stateValues_) {
    state_interfaces.push_back(std::make_shared<::hardware_interface::StateInterface>(sensorName_, name, &value));
  }
  return state_interfaces;
}

std::vector<::hardware_interface::CommandInterface::SharedPtr> ImuStateSensor::on_export_command_interfaces() { return {}; }

bool ImuStateSensor::initSim(rclcpp::Node::SharedPtr& model_nh, std::map<std::string, gz::sim::Entity>& /*joints*/,
                             const ::hardware_interface::HardwareInfo& /*hardware_info*/, gz::sim::EntityComponentManager& /*ecm*/,
                             unsigned int /*update_rate*/) {
  nh_ = model_nh;
  subscribeTopic();
  return true;
}

void ImuStateSensor::subscribeTopic() {
  if (!nh_ || topicName_.empty() || subscription_) {
    return;
  }
  subscription_ = nh_->create_subscription<sensor_msgs::msg::Imu>(
      topicName_, rclcpp::QoS(10).reliable(), [this](const sensor_msgs::msg::Imu::ConstSharedPtr msg) { imuCallback(msg); });
  RCLCPP_INFO(get_logger(), "Subscribed IMU bridge '%s' to topic '%s'", sensorName_.c_str(), topicName_.c_str());
}

void ImuStateSensor::imuCallback(const sensor_msgs::msg::Imu::ConstSharedPtr& msg) {
  if (!msg) {
    return;
  }
  stateValues_["orientation.x"] = msg->orientation.x;
  stateValues_["orientation.y"] = msg->orientation.y;
  stateValues_["orientation.z"] = msg->orientation.z;
  stateValues_["orientation.w"] = msg->orientation.w;
  stateValues_["angular_velocity.x"] = msg->angular_velocity.x;
  stateValues_["angular_velocity.y"] = msg->angular_velocity.y;
  stateValues_["angular_velocity.z"] = msg->angular_velocity.z;
  stateValues_["linear_acceleration.x"] = msg->linear_acceleration.x;
  stateValues_["linear_acceleration.y"] = msg->linear_acceleration.y;
  stateValues_["linear_acceleration.z"] = msg->linear_acceleration.z;
}

ImuStateSensor::return_type ImuStateSensor::read(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
  return return_type::OK;
}

ImuStateSensor::return_type ImuStateSensor::write(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
  return return_type::OK;
}

}  // namespace sim
}  // namespace legged

PLUGINLIB_EXPORT_CLASS(legged::sim::ImuStateSensor, gz_ros2_control::GazeboSimSystemInterface)
