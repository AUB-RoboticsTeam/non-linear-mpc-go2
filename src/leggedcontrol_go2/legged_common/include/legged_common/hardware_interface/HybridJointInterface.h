//
// Created by qiayuan on 2021/11/5.
// ROS 2 migration: interface-name contract for ros2_control.
//
#pragma once

#include <array>
#include <string>
#include <string_view>

#include <hardware_interface/types/hardware_interface_type_values.hpp>

namespace legged::hardware_interface {

inline constexpr char HW_IF_KP[] = "kp";
inline constexpr char HW_IF_KD[] = "kd";
inline constexpr char HW_IF_FEEDFORWARD[] = "feedforward";

inline constexpr std::array<std::string_view, 5> HYBRID_JOINT_COMMAND_INTERFACES{
    ::hardware_interface::HW_IF_POSITION,
    ::hardware_interface::HW_IF_VELOCITY,
    HW_IF_KP,
    HW_IF_KD,
    HW_IF_FEEDFORWARD,
};

inline constexpr std::array<std::string_view, 3> HYBRID_JOINT_STATE_INTERFACES{
    ::hardware_interface::HW_IF_POSITION,
    ::hardware_interface::HW_IF_VELOCITY,
    ::hardware_interface::HW_IF_EFFORT,
};

struct HybridJointCommand {
  double position = 0.0;
  double velocity = 0.0;
  double kp = 0.0;
  double kd = 0.0;
  double feedforward = 0.0;
};

struct HybridJointState {
  double position = 0.0;
  double velocity = 0.0;
  double effort = 0.0;
};

inline std::string interfaceKey(std::string_view name, std::string_view interface_name) {
  return std::string{name} + "/" + std::string{interface_name};
}

}  // namespace legged::hardware_interface
