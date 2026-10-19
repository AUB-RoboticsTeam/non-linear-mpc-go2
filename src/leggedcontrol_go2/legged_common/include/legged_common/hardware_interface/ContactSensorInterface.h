//
// Created by qiayuan on 2021/11/5.
// ROS 2 migration: interface-name contract for contact states.
//
#pragma once

#include <string>
#include <string_view>

namespace legged::hardware_interface {

inline constexpr char HW_IF_CONTACT[] = "contact";

struct ContactSensorState {
  double contact = 0.0;
};

inline bool isContact(double value) {
  return value > 0.5;
}

inline std::string contactInterfaceKey(std::string_view name) {
  return std::string{name} + "/" + HW_IF_CONTACT;
}

}  // namespace legged::hardware_interface
