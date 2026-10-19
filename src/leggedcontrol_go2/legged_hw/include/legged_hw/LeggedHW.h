//
// Created by qiayuan on 6/24/22.
// ROS 2 migration base for ros2_control hardware plugins.
//

#pragma once

#include <string>
#include <vector>

#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>

#include <legged_common/hardware_interface/ContactSensorInterface.h>
#include <legged_common/hardware_interface/HybridJointInterface.h>

namespace legged {

class LeggedHW : public ::hardware_interface::SystemInterface {
 public:
  using CallbackReturn = ::hardware_interface::CallbackReturn;
  using return_type = ::hardware_interface::return_type;

  CallbackReturn on_init(const ::hardware_interface::HardwareComponentInterfaceParams& params) override;

  return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;
  return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

 protected:
  const std::vector<std::string>& jointNames() const { return jointNames_; }
  const std::vector<std::string>& contactNames() const { return contactNames_; }

  double getJointCommand(const std::string& jointName, const std::string& interfaceName) const;
  void setJointState(const std::string& jointName, const std::string& interfaceName, double value);
  void setContactState(const std::string& contactName, bool isContact);

 private:
  bool hasInterface(const std::vector<::hardware_interface::InterfaceInfo>& interfaces, const std::string& name) const;
  bool validateJointInterfaces(const ::hardware_interface::ComponentInfo& joint) const;

  std::vector<std::string> jointNames_;
  std::vector<std::string> contactNames_;
};

}  // namespace legged
