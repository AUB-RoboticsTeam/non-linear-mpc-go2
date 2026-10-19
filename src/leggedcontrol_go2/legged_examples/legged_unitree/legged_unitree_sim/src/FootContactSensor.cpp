//
// Created by codex.
//

#include "legged_unitree_sim/FootContactSensor.h"

#include <array>
#include <algorithm>

#include <pluginlib/class_list_macros.hpp>

namespace legged {
namespace sim {
namespace {
std::string topicFromComponent(const ::hardware_interface::ComponentInfo& component) {
  const auto it = component.parameters.find("topic");
  if (it != component.parameters.end() && !it->second.empty()) {
    return it->second;
  }
  return "/" + component.name + "/contact";
}

std::string collisionFromComponent(const ::hardware_interface::ComponentInfo& component) {
  const auto it = component.parameters.find("collision");
  return it == component.parameters.end() ? std::string{} : it->second;
}

constexpr std::array<const char*, 6> kWrenchInterfaces{
    "force.x", "force.y", "force.z", "torque.x", "torque.y", "torque.z"};
}  // namespace

FootContactSensor::CallbackReturn FootContactSensor::on_init(
    const ::hardware_interface::HardwareComponentInterfaceParams& params) {
  if (::hardware_interface::SystemInterface::on_init(params) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  sensorNames_.clear();
  topics_.clear();
  collisions_.clear();
  wrenchStates_.clear();
  subscriptions_.clear();

  for (const auto& sensor : get_hardware_info().sensors) {
    if (sensor.state_interfaces.size() != kWrenchInterfaces.size()) {
      RCLCPP_ERROR(get_logger(), "Sensor %s must expose all six force/torque state interfaces.", sensor.name.c_str());
      return CallbackReturn::ERROR;
    }
    for (const auto* interfaceName : kWrenchInterfaces) {
      const auto found = std::find_if(sensor.state_interfaces.begin(), sensor.state_interfaces.end(),
                                      [interfaceName](const auto& interface) { return interface.name == interfaceName; });
      if (found == sensor.state_interfaces.end()) {
        RCLCPP_ERROR(get_logger(), "Sensor %s is missing state interface '%s'.", sensor.name.c_str(), interfaceName);
        return CallbackReturn::ERROR;
      }
    }

    sensorNames_.push_back(sensor.name);
    topics_[sensor.name] = topicFromComponent(sensor);
    collisions_[sensor.name] = collisionFromComponent(sensor);
    wrenchStates_[sensor.name] = WrenchState{};
  }

  // gz_ros2_control calls initSim() before ros2_control calls on_init() for
  // separately declared sensor components. Subscribe only after both the ROS
  // node and the parsed sensor names are available.
  subscribeTopics();

  RCLCPP_INFO(get_logger(), "Initialized foot contact state bridge with %zu sensors.", sensorNames_.size());
  return CallbackReturn::SUCCESS;
}

std::vector<::hardware_interface::StateInterface::ConstSharedPtr> FootContactSensor::on_export_state_interfaces() {
  std::vector<::hardware_interface::StateInterface::ConstSharedPtr> state_interfaces;
  state_interfaces.reserve(sensorNames_.size() * kWrenchInterfaces.size());

  for (const auto& sensorName : sensorNames_) {
    auto& wrench = wrenchStates_.at(sensorName).exported;
    for (size_t i = 0; i < kWrenchInterfaces.size(); ++i) {
      state_interfaces.push_back(
          std::make_shared<::hardware_interface::StateInterface>(sensorName, kWrenchInterfaces[i], &wrench[i]));
    }
  }

  return state_interfaces;
}

std::vector<::hardware_interface::CommandInterface::SharedPtr> FootContactSensor::on_export_command_interfaces() {
  return {};
}

bool FootContactSensor::initSim(rclcpp::Node::SharedPtr& model_nh, std::map<std::string, gz::sim::Entity>& /*joints*/,
                                const ::hardware_interface::HardwareInfo& /*hardware_info*/,
                                gz::sim::EntityComponentManager& /*ecm*/, unsigned int /*update_rate*/) {
  nh_ = model_nh;
  subscribeTopics();
  return true;
}

void FootContactSensor::subscribeTopics() {
  if (!nh_ || sensorNames_.empty() || !subscriptions_.empty()) {
    return;
  }
  subscriptions_.reserve(sensorNames_.size());

  for (const auto& sensorName : sensorNames_) {
    const auto topic = topics_.at(sensorName);
    const auto collision = collisions_.at(sensorName);
    subscriptions_.push_back(nh_->create_subscription<ros_gz_interfaces::msg::Contacts>(
        topic, rclcpp::QoS(10).reliable(),
        [this, sensorName, collision](const ros_gz_interfaces::msg::Contacts::ConstSharedPtr msg) {
          std::array<double, 6> total{};
          if (msg) {
            for (const auto& contact : msg->contacts) {
              const bool footIsBody1 = !collision.empty() && contact.collision1.name.find(collision) != std::string::npos;
              const bool footIsBody2 = !collision.empty() && contact.collision2.name.find(collision) != std::string::npos;
              if (!footIsBody1 && !footIsBody2) {
                continue;
              }
              for (const auto& jointWrench : contact.wrenches) {
                const auto& wrench = footIsBody2 ? jointWrench.body_2_wrench : jointWrench.body_1_wrench;
                total[0] += wrench.force.x;
                total[1] += wrench.force.y;
                total[2] += wrench.force.z;
                total[3] += wrench.torque.x;
                total[4] += wrench.torque.y;
                total[5] += wrench.torque.z;
              }
            }
          }
          std::lock_guard<std::mutex> lock(wrenchMutex_);
          wrenchStates_.at(sensorName).pending = total;
        }));
    RCLCPP_INFO(get_logger(), "Subscribed foot wrench '%s' to Gazebo contacts '%s'.", sensorName.c_str(), topic.c_str());
  }
}

FootContactSensor::return_type FootContactSensor::read(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
  std::lock_guard<std::mutex> lock(wrenchMutex_);
  for (auto& [name, state] : wrenchStates_) {
    (void)name;
    state.exported = state.pending;
  }
  return return_type::OK;
}

FootContactSensor::return_type FootContactSensor::write(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
  return return_type::OK;
}

}  // namespace sim
}  // namespace legged

PLUGINLIB_EXPORT_CLASS(legged::sim::FootContactSensor, gz_ros2_control::GazeboSimSystemInterface)
