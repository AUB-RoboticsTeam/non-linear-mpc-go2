//
// Created by qiayuan on 1/24/22.
//

#include "legged_hw/LeggedHW.h"

#include <algorithm>
#include <string>

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/logging.hpp>

namespace legged {
namespace {
std::string key(const std::string& component, const std::string& interfaceName) {
  return component + "/" + interfaceName;
}
}  // namespace

LeggedHW::CallbackReturn LeggedHW::on_init(const ::hardware_interface::HardwareComponentInterfaceParams& params) {
  if (::hardware_interface::SystemInterface::on_init(params) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  jointNames_.clear();
  contactNames_.clear();

  for (const auto& joint : params.hardware_info.joints) {
    if (!validateJointInterfaces(joint)) {
      RCLCPP_ERROR(get_logger(), "Joint %s does not expose the required hybrid command/state interfaces", joint.name.c_str());
      return CallbackReturn::ERROR;
    }
    jointNames_.push_back(joint.name);
  }

  for (const auto& gpio : params.hardware_info.gpios) {
    if (hasInterface(gpio.state_interfaces, legged::hardware_interface::HW_IF_CONTACT)) {
      contactNames_.push_back(gpio.name);
    }
  }

  for (const auto& sensor : params.hardware_info.sensors) {
    if (hasInterface(sensor.state_interfaces, legged::hardware_interface::HW_IF_CONTACT)) {
      contactNames_.push_back(sensor.name);
    }
  }

  RCLCPP_INFO(get_logger(), "Initialized legged hardware %s with %zu joints and %zu contact interfaces", get_name().c_str(),
              jointNames_.size(), contactNames_.size());
  return CallbackReturn::SUCCESS;
}

LeggedHW::return_type LeggedHW::read(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
  return return_type::OK;
}

LeggedHW::return_type LeggedHW::write(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
  return return_type::OK;
}

double LeggedHW::getJointCommand(const std::string& jointName, const std::string& interfaceName) const {
  return get_command<double>(key(jointName, interfaceName));
}

void LeggedHW::setJointState(const std::string& jointName, const std::string& interfaceName, double value) {
  set_state(key(jointName, interfaceName), value);
}

void LeggedHW::setContactState(const std::string& contactName, bool isContact) {
  set_state(key(contactName, legged::hardware_interface::HW_IF_CONTACT), isContact ? 1.0 : 0.0);
}

bool LeggedHW::hasInterface(const std::vector<::hardware_interface::InterfaceInfo>& interfaces, const std::string& name) const {
  return std::any_of(interfaces.begin(), interfaces.end(), [&](const auto& interface) { return interface.name == name; });
}

bool LeggedHW::validateJointInterfaces(const ::hardware_interface::ComponentInfo& joint) const {
  for (const auto interfaceName : legged::hardware_interface::HYBRID_JOINT_STATE_INTERFACES) {
    if (!hasInterface(joint.state_interfaces, std::string{interfaceName})) {
      return false;
    }
  }
  for (const auto interfaceName : legged::hardware_interface::HYBRID_JOINT_COMMAND_INTERFACES) {
    if (!hasInterface(joint.command_interfaces, std::string{interfaceName})) {
      return false;
    }
  }
  return true;
}

}  // namespace legged
