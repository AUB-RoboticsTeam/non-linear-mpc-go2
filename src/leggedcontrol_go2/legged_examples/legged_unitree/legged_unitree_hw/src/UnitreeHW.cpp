//
// Created by qiayuan on 1/24/22.
// ROS 2 migration for Unitree Go2 DDS low-level hardware.
//

#include "legged_unitree_hw/UnitreeHW.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <unordered_map>

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <legged_common/hardware_interface/ContactSensorInterface.h>
#include <legged_common/hardware_interface/HybridJointInterface.h>
#include <pluginlib/class_list_macros.hpp>

namespace legged {
namespace {
constexpr auto TOPIC_LOWCMD = "rt/lowcmd";
constexpr auto TOPIC_LOWSTATE = "rt/lowstate";
constexpr auto IMU_NAME = "unitree_imu";
constexpr int64_t JOYSTICK_PUBLISH_PERIOD_NS = 20000000; // 50 Hz

int getIntParam(const ::hardware_interface::HardwareInfo &info,
                const std::string &name, int defaultValue) {
  const auto it = info.hardware_parameters.find(name);
  if (it == info.hardware_parameters.end() || it->second.empty()) {
    return defaultValue;
  }
  return std::stoi(it->second);
}

double getDoubleParam(const ::hardware_interface::HardwareInfo &info,
                      const std::string &name, double defaultValue) {
  const auto it = info.hardware_parameters.find(name);
  if (it == info.hardware_parameters.end() || it->second.empty()) {
    return defaultValue;
  }
  return std::stod(it->second);
}

bool getBoolParam(const ::hardware_interface::HardwareInfo &info,
                  const std::string &name, bool defaultValue) {
  const auto it = info.hardware_parameters.find(name);
  if (it == info.hardware_parameters.end() || it->second.empty()) {
    return defaultValue;
  }
  const auto &value = it->second;
  return value == "1" || value == "true" || value == "True" || value == "yes" ||
         value == "on";
}

int64_t steadyNowNs() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

bool validUnitreeNetworkInterface(const std::string &name, std::string &error) {
  if (name.empty()) {
    error = "network_interface is empty; pass the wired Go2 Ethernet interface "
            "explicitly";
    return false;
  }
  if (name.size() >= IFNAMSIZ) {
    error = "network interface name is too long";
    return false;
  }
  const int socketFd = ::socket(AF_INET, SOCK_DGRAM, 0);
  if (socketFd < 0) {
    error = "failed to open socket while validating network interface";
    return false;
  }
  struct ifreq request {};
  std::strncpy(request.ifr_name, name.c_str(), IFNAMSIZ - 1);
  const int result = ::ioctl(socketFd, SIOCGIFFLAGS, &request);
  ::close(socketFd);
  if (result != 0) {
    error = "network interface '" + name + "' does not exist";
    return false;
  }
  if ((request.ifr_flags & IFF_UP) == 0) {
    error = "network interface '" + name + "' is down";
    return false;
  }
  if ((request.ifr_flags & IFF_MULTICAST) == 0) {
    error = "network interface '" + name + "' is not multicast-capable";
    return false;
  }
  return true;
}

std::string getStringParam(const ::hardware_interface::HardwareInfo &info,
                           const std::string &name,
                           const std::string &defaultValue) {
  const auto it = info.hardware_parameters.find(name);
  if (it == info.hardware_parameters.end()) {
    return defaultValue;
  }
  return it->second;
}
} // namespace

LeggedHW::CallbackReturn UnitreeHW::on_init(
    const ::hardware_interface::HardwareComponentInterfaceParams &params) {
  if (LeggedHW::on_init(params) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  powerLimit_ = getIntParam(params.hardware_info, "power_limit", powerLimit_);
  contactThreshold_ =
      getIntParam(params.hardware_info, "contact_threshold", contactThreshold_);
  originalContactThreshold_ = contactThreshold_;
  domainId_ = getIntParam(params.hardware_info, "domain_id", domainId_);
  networkInterface_ = getStringParam(params.hardware_info, "network_interface",
                                     networkInterface_);
  stateTimeout_ =
      getDoubleParam(params.hardware_info, "state_timeout", stateTimeout_);
  controllerCommandTimeout_ = getDoubleParam(
      params.hardware_info, "controller_command_timeout",
      controllerCommandTimeout_);
  commandStreamRate_ = getDoubleParam(
      params.hardware_info, "command_stream_rate", commandStreamRate_);
  startupTimeout_ =
      getDoubleParam(params.hardware_info, "startup_timeout", startupTimeout_);
  armRampDuration_ = getDoubleParam(params.hardware_info, "arm_ramp_duration",
                                    armRampDuration_);
  maxTorque_ = getDoubleParam(params.hardware_info, "max_torque", maxTorque_);
  maxKp_ = getDoubleParam(params.hardware_info, "max_kp", maxKp_);
  maxKd_ = getDoubleParam(params.hardware_info, "max_kd", maxKd_);
  standDuration_ =
      getDoubleParam(params.hardware_info, "stand_duration", standDuration_);
  standKp_ = getDoubleParam(params.hardware_info, "stand_kp", standKp_);
  standHoldKp_ =
      getDoubleParam(params.hardware_info, "stand_hold_kp", standHoldKp_);
  standKd_ = getDoubleParam(params.hardware_info, "stand_kd", standKd_);
  standCrouchHfeTarget_ = getDoubleParam(
      params.hardware_info, "stand_crouch_hfe_target", standCrouchHfeTarget_);
  standCrouchKfeTarget_ = getDoubleParam(
      params.hardware_info, "stand_crouch_kfe_target", standCrouchKfeTarget_);
  standCrouchFraction_ = getDoubleParam(
      params.hardware_info, "stand_crouch_fraction", standCrouchFraction_);
  standHfeTarget_ = getDoubleParam(params.hardware_info, "stand_hfe_target",
                                   standHfeTarget_);
  standKfeTarget_ = getDoubleParam(params.hardware_info, "stand_kfe_target",
                                   standKfeTarget_);
  standPositionTolerance_ = getDoubleParam(
      params.hardware_info, "stand_position_tolerance", standPositionTolerance_);
  standVelocityTolerance_ = getDoubleParam(
      params.hardware_info, "stand_velocity_tolerance", standVelocityTolerance_);
  standSettleDuration_ = getDoubleParam(
      params.hardware_info, "stand_settle_duration", standSettleDuration_);
  dampingKd_ =
      getDoubleParam(params.hardware_info, "damping_kd", dampingKd_);
  releaseModeOnArm_ = getBoolParam(params.hardware_info, "release_mode_on_arm",
                                   releaseModeOnArm_);
  requireMotionModeCheck_ =
      getBoolParam(params.hardware_info, "require_motion_mode_check",
                   requireMotionModeCheck_);

  if (stateTimeout_ <= 0.0 || controllerCommandTimeout_ <= 0.0 ||
      commandStreamRate_ < 100.0 || commandStreamRate_ > 1000.0) {
    RCLCPP_ERROR(
        get_logger(),
        "Invalid watchdog/output settings: state_timeout=%.3f s, "
        "controller_command_timeout=%.3f s, command_stream_rate=%.1f Hz",
        stateTimeout_, controllerCommandTimeout_, commandStreamRate_);
    return CallbackReturn::ERROR;
  }

  std::string networkError;
  if (!validUnitreeNetworkInterface(networkInterface_, networkError)) {
    RCLCPP_ERROR(get_logger(), "%s", networkError.c_str());
    return CallbackReturn::ERROR;
  }

  if (jointNames().size() != 12) {
    RCLCPP_ERROR(get_logger(),
                 "Unitree Go2 hardware requires exactly 12 joints, got %zu",
                 jointNames().size());
    return CallbackReturn::ERROR;
  }
  for (const auto &jointName : jointNames()) {
    if (motorIndex(jointName) < 0) {
      RCLCPP_ERROR(get_logger(),
                   "No Unitree motor index is defined for joint '%s'",
                   jointName.c_str());
      return CallbackReturn::ERROR;
    }
  }

  try {
    unitree::robot::ChannelFactory::Instance()->Init(domainId_,
                                                     networkInterface_);
    initLowCmd();
  } catch (const std::exception &e) {
    RCLCPP_ERROR(get_logger(), "Failed to initialize Unitree DDS hardware: %s",
                 e.what());
    return CallbackReturn::ERROR;
  }

  const auto node = get_node();
  tempPublisher_ = node->create_publisher<std_msgs::msg::Float64MultiArray>(
      "motor_temperatures", 10);
  kpPublisher_ =
      node->create_publisher<std_msgs::msg::Float64MultiArray>("kp_cmd", 10);
  commandStatePublisher_ = node->create_publisher<std_msgs::msg::Bool>(
      "/unitree_hw/commands_enabled",
      rclcpp::QoS(1).reliable().transient_local());
  standReadyPublisher_ = node->create_publisher<std_msgs::msg::Bool>(
      "/unitree_hw/stand_ready",
      rclcpp::QoS(1).reliable().transient_local());
  standStatePublisher_ = node->create_publisher<std_msgs::msg::String>(
      "/unitree_hw/stand_state",
      rclcpp::QoS(1).reliable().transient_local());
  joystickPublisher_ = node->create_publisher<sensor_msgs::msg::Joy>(
      "/unitree_hw/joystick", rclcpp::SensorDataQoS());
  commandEnableService_ = node->create_service<std_srvs::srv::SetBool>(
      "/unitree_hw/enable_commands",
      std::bind(&UnitreeHW::commandEnableCallback, this, std::placeholders::_1,
                std::placeholders::_2));
  standUpService_ = node->create_service<std_srvs::srv::Trigger>(
      "/unitree_hw/stand_up",
      std::bind(&UnitreeHW::standUpCallback, this, std::placeholders::_1,
                std::placeholders::_2));
  dampService_ = node->create_service<std_srvs::srv::Trigger>(
      "/unitree_hw/damp",
      std::bind(&UnitreeHW::dampCallback, this, std::placeholders::_1,
                std::placeholders::_2));
  cmdVelSubscriber_ = node->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 1,
      std::bind(&UnitreeHW::cmdVelCallback, this, std::placeholders::_1));
  publishCommandState();
  publishStandState("passive");
  RCLCPP_WARN(get_logger(),
              "Initialized Unitree Go2 hardware on DDS domain %d, interface "
              "'%s'. Motor commands are DISARMED; "
              "activate the controller, then call /unitree_hw/enable_commands "
              "explicitly.",
              domainId_, networkInterface_.c_str());
  return CallbackReturn::SUCCESS;
}

UnitreeHW::~UnitreeHW() { stopCommandThread(); }

LeggedHW::CallbackReturn
UnitreeHW::on_activate(const rclcpp_lifecycle::State & /*previous_state*/) {
  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::duration<double>(startupTimeout_);
  while (!lowStateFresh() && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  if (!lowStateFresh()) {
    RCLCPP_ERROR(get_logger(),
                 "No fresh rt/lowstate received within %.1f s; refusing "
                 "hardware activation",
                 startupTimeout_);
    return CallbackReturn::ERROR;
  }
  hardwareActive_.store(true, std::memory_order_release);
  commandsEnabled_.store(false, std::memory_order_release);
  commandReady_.store(false, std::memory_order_release);
  lastControllerCommandSteadyNs_.store(0, std::memory_order_release);
  outputMode_.store(OutputMode::PASSIVE, std::memory_order_release);
  startCommandThread();
  publishCommandState();
  publishStandState("passive");
  RCLCPP_INFO(get_logger(), "Unitree state feedback is live; low-level motor "
                            "output remains DISARMED");
  return CallbackReturn::SUCCESS;
}

LeggedHW::CallbackReturn
UnitreeHW::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) {
  hardwareActive_.store(false, std::memory_order_release);
  stopCommandThread();
  commandReady_.store(false, std::memory_order_release);
  disableCommands("hardware deactivated", true);
  return CallbackReturn::SUCCESS;
}

LeggedHW::CallbackReturn
UnitreeHW::on_shutdown(const rclcpp_lifecycle::State & /*previous_state*/) {
  hardwareActive_.store(false, std::memory_order_release);
  stopCommandThread();
  commandReady_.store(false, std::memory_order_release);
  disableCommands("hardware shutdown", true);
  return CallbackReturn::SUCCESS;
}

void UnitreeHW::cmdVelCallback(
    const geometry_msgs::msg::Twist::SharedPtr /*msg*/) {
  contactThreshold_ = originalContactThreshold_;
  cmdVelSubscriber_.reset();
  RCLCPP_INFO(get_logger(),
              "Received /cmd_vel; restored contact threshold to %d",
              contactThreshold_);
}

void UnitreeHW::initLowCmd() {
  lowCmd_.head()[0] = 0xFE;
  lowCmd_.head()[1] = 0xEF;
  lowCmd_.level_flag() = 0xFF;
  lowCmd_.gpio() = 0;

  for (auto &motorCommand : lowCmd_.motor_cmd()) {
    motorCommand.mode() = 0x01;
    motorCommand.q() = static_cast<float>(PosStopF);
    motorCommand.kp() = 0.0F;
    motorCommand.dq() = static_cast<float>(VelStopF);
    motorCommand.kd() = 0.0F;
    motorCommand.tau() = 0.0F;
  }

  lowCmdPublisher_ = std::make_shared<
      unitree::robot::ChannelPublisher<unitree_go::msg::dds_::LowCmd_>>(
      TOPIC_LOWCMD);
  lowCmdPublisher_->InitChannel();

  lowStateSubscriber_ = std::make_shared<
      unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::LowState_>>(
      TOPIC_LOWSTATE);
  lowStateSubscriber_->InitChannel(std::bind(&UnitreeHW::lowStateMessageHandler,
                                             this, std::placeholders::_1),
                                   1);
}

void UnitreeHW::lowStateMessageHandler(const void *message) {
  std::lock_guard<std::mutex> lock(lowStateMutex_);
  lowState_ = *static_cast<const unitree_go::msg::dds_::LowState_ *>(message);
  lastLowStateSteadyNs_.store(steadyNowNs(), std::memory_order_release);
  lowStateReceived_.store(true, std::memory_order_release);
}

LeggedHW::return_type UnitreeHW::read(const rclcpp::Time &time,
                                      const rclcpp::Duration & /*period*/) {
  unitree_go::msg::dds_::LowState_ lowState;
  {
    std::lock_guard<std::mutex> lock(lowStateMutex_);
    lowState = lowState_;
  }

  const auto &joints = jointNames();
  if (!lowStateFresh()) {
    // Returning ERROR makes controller_manager deactivate this hardware.  Set
    // the service-facing state first so a still-alive Trigger service cannot
    // accept stand-up after read/write cycles have already stopped.
    hardwareActive_.store(false, std::memory_order_release);
    commandReady_.store(false, std::memory_order_release);
    disableCommands("rt/lowstate watchdog expired", true);
    publishStandState("inactive_stale_state");
    RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "rt/lowstate is missing or stale; hardware marked inactive and motor "
        "output disabled");
    return return_type::ERROR;
  }

  for (size_t i = 0; i < joints.size(); ++i) {
    const int index = motorIndex(joints[i]);
    setJointState(joints[i], ::hardware_interface::HW_IF_POSITION,
                  lowState.motor_state()[index].q());
    setJointState(joints[i], ::hardware_interface::HW_IF_VELOCITY,
                  lowState.motor_state()[index].dq());
    setJointState(joints[i], ::hardware_interface::HW_IF_EFFORT,
                  lowState.motor_state()[index].tau_est());
    jointTemperatures_[i] = lowState.motor_state()[index].temperature();
  }

  for (const auto &contactName : contactNames()) {
    const int index = footForceIndex(contactName);
    if (index >= 0) {
      setContactState(contactName,
                      lowState.foot_force()[index] > contactThreshold_);
    }
  }

  setImuState("orientation.x", lowState.imu_state().quaternion()[1]);
  setImuState("orientation.y", lowState.imu_state().quaternion()[2]);
  setImuState("orientation.z", lowState.imu_state().quaternion()[3]);
  setImuState("orientation.w", lowState.imu_state().quaternion()[0]);
  setImuState("angular_velocity.x", lowState.imu_state().gyroscope()[0]);
  setImuState("angular_velocity.y", lowState.imu_state().gyroscope()[1]);
  setImuState("angular_velocity.z", lowState.imu_state().gyroscope()[2]);
  setImuState("linear_acceleration.x", lowState.imu_state().accelerometer()[0]);
  setImuState("linear_acceleration.y", lowState.imu_state().accelerometer()[1]);
  setImuState("linear_acceleration.z", lowState.imu_state().accelerometer()[2]);

  publishJoystick(time, lowState);
  updateTemp(time);
  return return_type::OK;
}

void UnitreeHW::publishJoystick(
    const rclcpp::Time &time,
    const unitree_go::msg::dds_::LowState_ &lowState) {
  if (!joystickPublisher_) {
    return;
  }
  const int64_t now = steadyNowNs();
  if (now - lastJoystickPublishSteadyNs_ < JOYSTICK_PUBLISH_PERIOD_NS) {
    return;
  }
  lastJoystickPublishSteadyNs_ = now;

  // Unitree's 40-byte wireless packet is laid out as:
  // [2-byte header][uint16 key mask][lx][rx][ry][L2 analog][ly]...
  // Publish a standard Joy message so the GUI never needs the vendor-only
  // unitree_go/WirelessController ROS type.
  const auto &remote = lowState.wireless_remote();
  auto readFloat = [&remote](size_t offset) {
    float value = 0.0F;
    std::memcpy(&value, remote.data() + offset, sizeof(value));
    return std::isfinite(value) ? value : 0.0F;
  };
  const uint16_t keys = static_cast<uint16_t>(remote[2]) |
                        (static_cast<uint16_t>(remote[3]) << 8U);

  sensor_msgs::msg::Joy message;
  message.header.stamp = time;
  message.header.frame_id = "unitree_remote";
  message.axes = {readFloat(4), readFloat(20), readFloat(8), readFloat(12)};
  message.buttons.resize(16);
  for (size_t bit = 0; bit < message.buttons.size(); ++bit) {
    message.buttons[bit] = (keys & (1U << bit)) != 0U ? 1 : 0;
  }
  joystickPublisher_->publish(message);
}

LeggedHW::return_type UnitreeHW::write(const rclcpp::Time & /*time*/,
                                       const rclcpp::Duration & /*period*/) {
  if (!lowCmdPublisher_) {
    return return_type::ERROR;
  }
  if (!hardwareActive_.load(std::memory_order_acquire)) {
    return return_type::OK;
  }

  // The controller-manager thread only deposits a complete command snapshot.
  // A dedicated hardware thread owns rt/lowcmd publication, so MPC solver,
  // visualization, executor, or DDS stalls cannot interrupt stand/damping
  // command refreshes.
  std::array<CachedJointCommand, 12> pending{};
  bool finite = true;
  const auto &joints = jointNames();
  for (size_t i = 0; i < joints.size(); ++i) {
    const auto &jointName = joints[i];
    const double q =
        getJointCommand(jointName, ::hardware_interface::HW_IF_POSITION);
    const double dq =
        getJointCommand(jointName, ::hardware_interface::HW_IF_VELOCITY);
    const double kp =
        getJointCommand(jointName, legged::hardware_interface::HW_IF_KP);
    const double kd =
        getJointCommand(jointName, legged::hardware_interface::HW_IF_KD);
    const double tau = getJointCommand(
        jointName, legged::hardware_interface::HW_IF_FEEDFORWARD);
    finite = finite && std::isfinite(q) && std::isfinite(dq) &&
             std::isfinite(kp) && std::isfinite(kd) && std::isfinite(tau);
    if (!finite) {
      break;
    }
    pending[i] = {q, dq, std::clamp(kp, 0.0, maxKp_),
                  std::clamp(kd, 0.0, maxKd_),
                  std::clamp(tau, -maxTorque_, maxTorque_)};
  }

  commandReady_.store(finite, std::memory_order_release);
  if (!finite) {
    if (commandsEnabled_.load(std::memory_order_acquire)) {
      commandsEnabled_.store(false, std::memory_order_release);
      outputMode_.store(OutputMode::DAMPING, std::memory_order_release);
      publishCommandState();
      publishStandState("damping_non_finite_command");
      RCLCPP_ERROR(
          get_logger(),
          "Non-finite hybrid joint command detected; switched to damping");
      return return_type::ERROR;
    }
    // ros2_control initializes unclaimed command interfaces to NaN.  That is
    // expected while the legged controller is inactive and output is disarmed.
    return return_type::OK;
  }

  {
    std::lock_guard<std::mutex> lock(mpcCommandMutex_);
    cachedMpcCommands_ = pending;
  }
  lastControllerCommandSteadyNs_.store(steadyNowNs(),
                                       std::memory_order_release);
  return return_type::OK;
}

void UnitreeHW::startCommandThread() {
  stopCommandThread();
  commandThreadRunning_.store(true, std::memory_order_release);
  commandThread_ = std::thread(&UnitreeHW::commandThreadLoop, this);
  RCLCPP_INFO(get_logger(),
              "Started independent Unitree command stream at %.1f Hz "
              "(controller watchdog %.3f s)",
              commandStreamRate_, controllerCommandTimeout_);
}

void UnitreeHW::stopCommandThread() {
  commandThreadRunning_.store(false, std::memory_order_release);
  if (commandThread_.joinable()) {
    commandThread_.join();
  }
}

void UnitreeHW::commandThreadLoop() {
  const auto period = std::chrono::duration<double>(1.0 / commandStreamRate_);
  auto nextWakeup = std::chrono::steady_clock::now();
  while (commandThreadRunning_.load(std::memory_order_acquire)) {
    nextWakeup +=
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(period);
    try {
      commandThreadIteration();
    } catch (const std::exception &exception) {
      commandsEnabled_.store(false, std::memory_order_release);
      outputMode_.store(OutputMode::DAMPING, std::memory_order_release);
      RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "Unitree command-stream exception; forcing damping: %s",
          exception.what());
    }

    const auto now = std::chrono::steady_clock::now();
    if (now > nextWakeup +
                  std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                      period)) {
      nextWakeup = now;
    }
    std::this_thread::sleep_until(nextWakeup);
  }
}

bool UnitreeHW::standPoseWithin(double positionLimit, double velocityLimit,
                                double &maxPositionError,
                                double &maxVelocity) {
  unitree_go::msg::dds_::LowState_ state;
  {
    std::lock_guard<std::mutex> lock(lowStateMutex_);
    state = lowState_;
  }
  maxPositionError = 0.0;
  maxVelocity = 0.0;
  for (const auto &jointName : jointNames()) {
    double target = 0.0;
    if (jointName.find("_HFE") != std::string::npos) {
      target = standHfeTarget_;
    } else if (jointName.find("_KFE") != std::string::npos) {
      target = standKfeTarget_;
    }
    const auto &motor = state.motor_state()[motorIndex(jointName)];
    maxPositionError = std::max(
        maxPositionError,
        std::abs(target - static_cast<double>(motor.q())));
    maxVelocity = std::max(
        maxVelocity, std::abs(static_cast<double>(motor.dq())));
  }
  return maxPositionError <= positionLimit && maxVelocity <= velocityLimit;
}

bool UnitreeHW::controllerCommandFresh() const {
  const int64_t stamp =
      lastControllerCommandSteadyNs_.load(std::memory_order_acquire);
  return commandReady_.load(std::memory_order_acquire) && stamp != 0 &&
         (steadyNowNs() - stamp) * 1e-9 <= controllerCommandTimeout_;
}

void UnitreeHW::commandThreadIteration() {
  if (!hardwareActive_.load(std::memory_order_acquire) || !lowCmdPublisher_) {
    return;
  }

  auto mode = outputMode_.load(std::memory_order_acquire);
  if (mode != OutputMode::PASSIVE && !lowStateFresh()) {
    if (mode != OutputMode::DAMPING ||
        commandsEnabled_.load(std::memory_order_acquire)) {
      commandsEnabled_.store(false, std::memory_order_release);
      commandReady_.store(false, std::memory_order_release);
      outputMode_.store(OutputMode::DAMPING, std::memory_order_release);
      publishCommandState();
      publishStandState("damping_stale_state");
      RCLCPP_ERROR(get_logger(),
                   "rt/lowstate watchdog expired in command thread; forcing "
                   "continuous damping");
    }
    writeDampingCommand();
    return;
  }

  switch (mode) {
  case OutputMode::PASSIVE:
    return;
  case OutputMode::DAMPING:
    writeDampingCommand();
    return;
  case OutputMode::STANDING_UP:
    writeStandCommand();
    return;
  case OutputMode::HOLDING_STAND: {
    double maxPositionError = 0.0;
    double maxVelocity = 0.0;
    if (!standPoseWithin(standPositionTolerance_, standVelocityTolerance_,
                         maxPositionError, maxVelocity)) {
      OutputMode expected = OutputMode::HOLDING_STAND;
      if (outputMode_.compare_exchange_strong(
              expected, OutputMode::STANDING_UP,
              std::memory_order_acq_rel)) {
        standConvergedSinceNs_.store(0, std::memory_order_release);
        publishStandState("reconverging");
        RCLCPP_WARN(get_logger(),
                    "Stand readiness revoked: joint error %.3f rad, "
                    "velocity %.3f rad/s",
                    maxPositionError, maxVelocity);
      }
    }
    writeStandCommand();
    return;
  }
  case OutputMode::MPC:
    break;
  }

  if (!commandsEnabled_.load(std::memory_order_acquire)) {
    outputMode_.store(OutputMode::STANDING_UP, std::memory_order_release);
    standConvergedSinceNs_.store(0, std::memory_order_release);
    writeStandCommand();
    return;
  }

  if (!controllerCommandFresh()) {
    double maxPositionError = 0.0;
    double maxVelocity = 0.0;
    const double fallbackPositionLimit =
        std::max(0.35, 2.0 * standPositionTolerance_);
    const bool safeToHold = standPoseWithin(
        fallbackPositionLimit, 1.0, maxPositionError, maxVelocity);
    commandsEnabled_.store(false, std::memory_order_release);
    commandReady_.store(false, std::memory_order_release);
    outputMode_.store(safeToHold ? OutputMode::STANDING_UP
                                 : OutputMode::DAMPING,
                      std::memory_order_release);
    standConvergedSinceNs_.store(0, std::memory_order_release);
    publishCommandState();
    publishStandState(safeToHold ? "mpc_stale_recovering_stand"
                                 : "damping_stale_mpc_command");
    RCLCPP_ERROR(
        get_logger(),
        "MPC command watchdog expired after %.3f s; %s (joint error %.3f "
        "rad, velocity %.3f rad/s)",
        controllerCommandTimeout_,
        safeToHold ? "restoring autonomous stand" : "forcing damping",
        maxPositionError, maxVelocity);
    if (safeToHold) {
      writeStandCommand();
    } else {
      writeDampingCommand();
    }
    return;
  }

  writeMpcCommand();
}

void UnitreeHW::writeMpcCommand() {
  std::array<CachedJointCommand, 12> snapshot{};
  {
    std::lock_guard<std::mutex> lock(mpcCommandMutex_);
    snapshot = cachedMpcCommands_;
  }

  double ramp = 1.0;
  if (armRampDuration_ > 0.0) {
    ramp = std::clamp(
        (steadyNowNs() - armStartSteadyNs_.load(std::memory_order_acquire)) *
            1e-9 / armRampDuration_,
        0.0, 1.0);
  }

  std::lock_guard<std::mutex> lock(lowCmdMutex_);
  const auto &joints = jointNames();
  for (size_t i = 0; i < joints.size(); ++i) {
    const auto &jointName = joints[i];
    const int index = motorIndex(jointName);
    auto &command = lowCmd_.motor_cmd()[index];
    const auto &mpc = snapshot[i];
    double standPosition = 0.0;
    if (jointName.find("_HFE") != std::string::npos) {
      standPosition = standHfeTarget_;
    } else if (jointName.find("_KFE") != std::string::npos) {
      standPosition = standKfeTarget_;
    }

    // Blend the complete hybrid command from the autonomous standing hold to
    // MPC.  The controller requests kp=0 for torque control, so stiffness must
    // decrease together with the feed-forward torque ramp.
    command.q() = static_cast<float>(
        standPosition + ramp * (mpc.q - standPosition));
    command.dq() = static_cast<float>(ramp * mpc.dq);
    command.kp() = static_cast<float>(
        standHoldKp_ + ramp * (mpc.kp - standHoldKp_));
    command.kd() =
        static_cast<float>(standKd_ + ramp * (mpc.kd - standKd_));
    command.tau() = static_cast<float>(ramp * mpc.tau);
  }

  if (outputMode_.load(std::memory_order_acquire) == OutputMode::MPC &&
      commandsEnabled_.load(std::memory_order_acquire)) {
    writeLowCmd();
  }
}

LeggedHW::return_type UnitreeHW::perform_command_mode_switch(
    const std::vector<std::string> & /*start_interfaces*/,
    const std::vector<std::string> &stop_interfaces) {
  if (!stop_interfaces.empty()) {
    const auto previousMode = outputMode_.load(std::memory_order_acquire);
    if (previousMode == OutputMode::HOLDING_STAND ||
        previousMode == OutputMode::STANDING_UP) {
      // Controller activation can fail after command interfaces have already
      // been claimed.  Releasing those interfaces must not remove the
      // hardware-owned standing command: doing so sends a passive packet and
      // drops the robot even though the stand-up itself was healthy.
      commandsEnabled_.store(false, std::memory_order_release);
      commandReady_.store(false, std::memory_order_release);
      publishCommandState();
      publishStandState(previousMode == OutputMode::HOLDING_STAND
                            ? "ready"
                            : "standing");
      RCLCPP_ERROR(
          get_logger(),
          "Controller interfaces were released before MPC arm; preserving "
          "the autonomous standing hold instead of disarming motor output");
      return return_type::OK;
    }
    const double secondsSinceArm =
        (steadyNowNs() - armStartSteadyNs_.load(std::memory_order_acquire)) *
        1e-9;
    if (previousMode == OutputMode::MPC &&
        secondsSinceArm <= armRampDuration_ + 1.0) {
      unitree_go::msg::dds_::LowState_ state;
      {
        std::lock_guard<std::mutex> stateLock(lowStateMutex_);
        state = lowState_;
      }
      double maxPositionError = 0.0;
      double maxVelocity = 0.0;
      for (const auto &jointName : jointNames()) {
        double target = 0.0;
        if (jointName.find("_HFE") != std::string::npos) {
          target = standHfeTarget_;
        } else if (jointName.find("_KFE") != std::string::npos) {
          target = standKfeTarget_;
        }
        const auto &motor = state.motor_state()[motorIndex(jointName)];
        maxPositionError = std::max(
            maxPositionError,
            std::abs(target - static_cast<double>(motor.q())));
        maxVelocity = std::max(
            maxVelocity, std::abs(static_cast<double>(motor.dq())));
      }
      const double fallbackPositionLimit =
          std::max(0.35, 2.0 * standPositionTolerance_);
      if (maxPositionError <= fallbackPositionLimit && maxVelocity <= 1.0) {
        commandsEnabled_.store(false, std::memory_order_release);
        commandReady_.store(false, std::memory_order_release);
        standConvergedSinceNs_.store(steadyNowNs(),
                                     std::memory_order_release);
        outputMode_.store(OutputMode::HOLDING_STAND,
                          std::memory_order_release);
        publishCommandState();
        publishStandState("ready");
        RCLCPP_ERROR(
            get_logger(),
            "MPC handoff failed during its ramp; restored standing hold "
            "instead of damping (joint error %.3f rad, velocity %.3f rad/s)",
            maxPositionError, maxVelocity);
        return return_type::OK;
      }
    }
    disableCommands("joint command interfaces released", true);
  }
  return return_type::OK;
}

bool UnitreeHW::lowStateFresh() const {
  if (!lowStateReceived_.load(std::memory_order_acquire)) {
    return false;
  }
  return (steadyNowNs() -
          lastLowStateSteadyNs_.load(std::memory_order_acquire)) *
             1e-9 <=
         stateTimeout_;
}

bool UnitreeHW::prepareMotionMode(std::string &error) {
  if (!requireMotionModeCheck_) {
    return true;
  }
  if (!motionSwitcherClient_) {
    try {
      motionSwitcherClient_ =
          std::make_shared<unitree::robot::b2::MotionSwitcherClient>();
      motionSwitcherClient_->SetTimeout(3.0F);
      motionSwitcherClient_->Init();
    } catch (const std::exception &exception) {
      error = std::string("failed to initialize motion-switcher client: ") +
              exception.what();
      return false;
    }
  }
  std::string form;
  std::string mode;
  int result = motionSwitcherClient_->CheckMode(form, mode);
  if (result != 0) {
    error =
        "Unitree motion-mode query failed with code " + std::to_string(result);
    return false;
  }
  if (mode.empty()) {
    return true;
  }
  if (!releaseModeOnArm_) {
    error = "Unitree mode '" + mode +
            "' is active; relaunch with release_mode_on_arm:=true while the "
            "robot is suspended";
    return false;
  }
  RCLCPP_WARN(
      get_logger(),
      "Releasing Unitree high-level motion mode '%s' before low-level arm",
      mode.c_str());
  for (int attempt = 0; attempt < 5 && !mode.empty(); ++attempt) {
    result = motionSwitcherClient_->ReleaseMode();
    if (result != 0) {
      error = "failed to release Unitree motion mode; code " +
              std::to_string(result);
      return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    result = motionSwitcherClient_->CheckMode(form, mode);
    if (result != 0) {
      error =
          "motion-mode verification failed with code " + std::to_string(result);
      return false;
    }
  }
  if (!mode.empty()) {
    error =
        "Unitree high-level motion mode remained active after release attempts";
    return false;
  }
  return true;
}

void UnitreeHW::commandEnableCallback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
  if (!request->data) {
    disableCommands("operator disarm request", true);
    response->success = true;
    response->message = "Unitree low-level motor output is DISARMED";
    return;
  }
  if (!hardwareActive_.load(std::memory_order_acquire)) {
    response->success = false;
    response->message = "hardware is not active";
    return;
  }
  if (!lowStateFresh()) {
    response->success = false;
    response->message = "rt/lowstate is missing or stale";
    return;
  }
  if (outputMode_.load(std::memory_order_acquire) !=
      OutputMode::HOLDING_STAND) {
    response->success = false;
    response->message =
        "stand-up is not complete; call /unitree_hw/stand_up first and wait "
        "for /unitree_hw/stand_ready=true";
    return;
  }
  {
    unitree_go::msg::dds_::LowState_ state;
    {
      std::lock_guard<std::mutex> lock(lowStateMutex_);
      state = lowState_;
    }
    double maxPositionError = 0.0;
    double maxVelocity = 0.0;
    for (const auto &jointName : jointNames()) {
      double target = 0.0;
      if (jointName.find("_HFE") != std::string::npos) {
        target = standHfeTarget_;
      } else if (jointName.find("_KFE") != std::string::npos) {
        target = standKfeTarget_;
      }
      const auto &motor = state.motor_state()[motorIndex(jointName)];
      maxPositionError =
          std::max(maxPositionError, std::abs(target - motor.q()));
      maxVelocity =
          std::max(maxVelocity, std::abs(static_cast<double>(motor.dq())));
    }
    if (maxPositionError > standPositionTolerance_ ||
        maxVelocity > standVelocityTolerance_) {
      response->success = false;
      response->message =
          "stand pose is no longer settled; position error=" +
          std::to_string(maxPositionError) + " rad, velocity=" +
          std::to_string(maxVelocity) + " rad/s";
      return;
    }
  }
  if (!controllerCommandFresh()) {
    response->success = false;
    response->message =
        "no finite controller command is fresh; activate legged_controller "
        "and wait for a continuously updating stance policy";
    return;
  }
  std::string error;
  if (!prepareMotionMode(error)) {
    response->success = false;
    response->message = error;
    return;
  }
  armStartSteadyNs_.store(steadyNowNs(), std::memory_order_release);
  commandsEnabled_.store(true, std::memory_order_release);
  outputMode_.store(OutputMode::MPC, std::memory_order_release);
  publishCommandState();
  publishStandState("mpc");
  RCLCPP_WARN(get_logger(),
              "UNITREE LOW-LEVEL MOTOR OUTPUT ARMED (%.1f s gain/torque ramp)",
              armRampDuration_);
  response->success = true;
  response->message = "Unitree low-level motor output ARMED";
}

void UnitreeHW::standUpCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  if (!hardwareActive_.load(std::memory_order_acquire)) {
    response->success = false;
    response->message = "hardware is not active";
    return;
  }
  if (!lowStateFresh()) {
    response->success = false;
    response->message = "rt/lowstate is missing or stale";
    return;
  }
  if (commandsEnabled_.load(std::memory_order_acquire) ||
      outputMode_.load(std::memory_order_acquire) == OutputMode::MPC) {
    response->success = false;
    response->message = "MPC output is active; disarm it before stand-up";
    return;
  }

  // Publish damping continuously before asking Unitree to release its
  // high-level service.  The high-level service ignores lowcmd while active;
  // once released, the next 500 Hz command is already safe damping rather
  // than leaving an output gap during the mode-switch verification.
  outputMode_.store(OutputMode::DAMPING, std::memory_order_release);
  publishStandState("switching_to_low_level");
  std::string error;
  if (!prepareMotionMode(error)) {
    response->success = false;
    response->message = error;
    publishStandState("damping");
    return;
  }

  unitree_go::msg::dds_::LowState_ state;
  {
    std::lock_guard<std::mutex> lock(lowStateMutex_);
    state = lowState_;
  }
  const auto &joints = jointNames();
  double maxInitialError = 0.0;
  for (size_t i = 0; i < joints.size(); ++i) {
    const double q = state.motor_state()[motorIndex(joints[i])].q();
    if (!std::isfinite(q) || std::abs(q) > 4.0) {
      response->success = false;
      response->message = "invalid measured joint position for " + joints[i];
      return;
    }
    standStartPositions_[i] = q;
    double target = 0.0;
    if (joints[i].find("_HFE") != std::string::npos) {
      target = standHfeTarget_;
    } else if (joints[i].find("_KFE") != std::string::npos) {
      target = standKfeTarget_;
    }
    maxInitialError = std::max(maxInitialError, std::abs(target - q));
  }

  commandsEnabled_.store(false, std::memory_order_release);
  standUseCrouchStage_ = maxInitialError > 0.35;
  standConvergedSinceNs_.store(0, std::memory_order_release);
  standStartSteadyNs_.store(steadyNowNs(), std::memory_order_release);
  outputMode_.store(OutputMode::STANDING_UP, std::memory_order_release);
  publishCommandState();
  publishStandState("standing_up");
  RCLCPP_WARN(get_logger(),
              "LOW-LEVEL STAND-UP STARTED: %.1f s %s trajectory, "
              "kp=%.1f -> hold kp=%.1f, kd=%.1f, HFE=%.2f, KFE=%.2f. "
              "Keep the robot supported and "
              "keep clear.",
              standDuration_, standUseCrouchStage_ ? "crouch-to-rise" : "direct",
              standKp_, standHoldKp_, standKd_, standHfeTarget_,
              standKfeTarget_);
  response->success = true;
  response->message = "stand-up started; monitor /unitree_hw/stand_state";
}

void UnitreeHW::dampCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  commandsEnabled_.store(false, std::memory_order_release);
  outputMode_.store(OutputMode::DAMPING, std::memory_order_release);
  publishCommandState();
  publishStandState("damping");
  RCLCPP_WARN(get_logger(), "DAMPING MODE REQUESTED (kd=%.1f)", dampingKd_);
  response->success = true;
  response->message = "continuous low-level damping enabled";
}

LeggedHW::return_type UnitreeHW::writeStandCommand() {
  const auto &joints = jointNames();
  const double elapsed =
      (steadyNowNs() - standStartSteadyNs_.load(std::memory_order_acquire)) *
      1e-9;
  const double phase =
      standDuration_ > 0.0 ? std::clamp(elapsed / standDuration_, 0.0, 1.0)
                           : 1.0;

  unitree_go::msg::dds_::LowState_ state;
  {
    std::lock_guard<std::mutex> stateLock(lowStateMutex_);
    state = lowState_;
  }

  double maxPositionError = 0.0;
  double maxVelocity = 0.0;
  std::lock_guard<std::mutex> commandLock(lowCmdMutex_);
  for (size_t i = 0; i < joints.size(); ++i) {
    const auto &jointName = joints[i];
    double finalTarget = 0.0;
    double crouchTarget = 0.0;
    if (jointName.find("_HFE") != std::string::npos) {
      finalTarget = standHfeTarget_;
      crouchTarget = standCrouchHfeTarget_;
    } else if (jointName.find("_KFE") != std::string::npos) {
      finalTarget = standKfeTarget_;
      crouchTarget = standCrouchKfeTarget_;
    }
    double q = finalTarget;
    if (standUseCrouchStage_) {
      const double split = std::clamp(standCrouchFraction_, 0.20, 0.75);
      if (phase < split) {
        const double stagePhase = std::clamp(phase / split, 0.0, 1.0);
        const double blend =
            stagePhase * stagePhase * (3.0 - 2.0 * stagePhase);
        q = standStartPositions_[i] +
            blend * (crouchTarget - standStartPositions_[i]);
      } else {
        const double stagePhase =
            std::clamp((phase - split) / (1.0 - split), 0.0, 1.0);
        const double blend =
            stagePhase * stagePhase * (3.0 - 2.0 * stagePhase);
        q = crouchTarget + blend * (finalTarget - crouchTarget);
      }
    } else {
      const double blend = phase * phase * (3.0 - 2.0 * phase);
      q = standStartPositions_[i] +
          blend * (finalTarget - standStartPositions_[i]);
    }
    const int index = motorIndex(jointName);
    auto &command = lowCmd_.motor_cmd()[index];
    command.q() = static_cast<float>(q);
    command.dq() = 0.0F;
    double commandKp = standKp_;
    if (jointName.find("_HFE") != std::string::npos ||
        jointName.find("_KFE") != std::string::npos) {
      const double gainPhase = std::clamp((phase - 0.80) / 0.20, 0.0, 1.0);
      const double gainBlend =
          gainPhase * gainPhase * (3.0 - 2.0 * gainPhase);
      commandKp = standKp_ + gainBlend * (standHoldKp_ - standKp_);
    }
    command.kp() =
        static_cast<float>(std::clamp(commandKp, 0.0, maxKp_));
    command.kd() = static_cast<float>(std::clamp(standKd_, 0.0, maxKd_));
    command.tau() = 0.0F;
    maxPositionError = std::max(
        maxPositionError,
        std::abs(finalTarget - state.motor_state()[index].q()));
    maxVelocity = std::max(
        maxVelocity,
        std::abs(static_cast<double>(state.motor_state()[index].dq())));
  }
  writeLowCmd();

  const bool withinTolerance =
      phase >= 1.0 && maxPositionError <= standPositionTolerance_ &&
      maxVelocity <= standVelocityTolerance_;
  const int64_t nowNs = steadyNowNs();
  if (withinTolerance &&
      standConvergedSinceNs_.load(std::memory_order_acquire) == 0) {
    standConvergedSinceNs_.store(nowNs, std::memory_order_release);
    publishStandState("settling");
  } else if (!withinTolerance) {
    standConvergedSinceNs_.store(0, std::memory_order_release);
  }
  const int64_t convergedSince =
      standConvergedSinceNs_.load(std::memory_order_acquire);
  const bool settled =
      convergedSince != 0 &&
      (nowNs - convergedSince) * 1e-9 >= standSettleDuration_;

  OutputMode expectedMode = OutputMode::STANDING_UP;
  if (settled && outputMode_.compare_exchange_strong(
                     expectedMode, OutputMode::HOLDING_STAND,
                     std::memory_order_acq_rel)) {
    publishStandState("ready");
    RCLCPP_INFO(get_logger(),
                "LOW-LEVEL STAND-UP READY (maximum joint error %.3f rad, "
                "velocity %.3f rad/s, settled %.2f s). "
                "The nominal pose is being held; MPC may now be activated.",
                maxPositionError, maxVelocity, standSettleDuration_);
  } else if (phase >= 1.0 &&
             outputMode_.load(std::memory_order_acquire) ==
                 OutputMode::STANDING_UP) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                         "Waiting for stand convergence; maximum joint error "
                         "is %.3f rad (limit %.3f), velocity %.3f rad/s "
                         "(limit %.3f)",
                         maxPositionError, standPositionTolerance_,
                         maxVelocity, standVelocityTolerance_);
  }
  return return_type::OK;
}

void UnitreeHW::writeDampingCommand() {
  std::lock_guard<std::mutex> lock(lowCmdMutex_);
  for (auto &command : lowCmd_.motor_cmd()) {
    command.q() = static_cast<float>(PosStopF);
    command.dq() = 0.0F;
    command.kp() = 0.0F;
    command.kd() = static_cast<float>(std::clamp(dampingKd_, 0.0, maxKd_));
    command.tau() = 0.0F;
  }
  writeLowCmd();
}

void UnitreeHW::writeLowCmd() {
  lowCmd_.crc() = crc32Core(reinterpret_cast<uint32_t *>(&lowCmd_),
                            (sizeof(unitree_go::msg::dds_::LowCmd_) >> 2) - 1);
  lowCmdPublisher_->Write(lowCmd_);
}

void UnitreeHW::disableCommands(const char *reason, bool publish_passive) {
  const auto previousMode =
      outputMode_.exchange(OutputMode::PASSIVE, std::memory_order_acq_rel);
  const bool wasEnabled =
      commandsEnabled_.exchange(false, std::memory_order_acq_rel);
  if (previousMode != OutputMode::PASSIVE && publish_passive) {
    publishPassiveCommand();
  }
  if (wasEnabled || previousMode != OutputMode::PASSIVE) {
    RCLCPP_WARN(get_logger(), "Unitree low-level motor output DISARMED: %s",
                reason);
  }
  publishCommandState();
  publishStandState("passive");
}

void UnitreeHW::publishPassiveCommand() {
  if (!lowCmdPublisher_) {
    return;
  }
  std::lock_guard<std::mutex> lock(lowCmdMutex_);
  for (auto &motorCommand : lowCmd_.motor_cmd()) {
    motorCommand.q() = static_cast<float>(PosStopF);
    motorCommand.kp() = 0.0F;
    motorCommand.dq() = static_cast<float>(VelStopF);
    motorCommand.kd() = 0.0F;
    motorCommand.tau() = 0.0F;
  }
  writeLowCmd();
}

void UnitreeHW::publishCommandState() {
  if (commandStatePublisher_) {
    std_msgs::msg::Bool message;
    message.data = commandsEnabled_.load(std::memory_order_acquire);
    commandStatePublisher_->publish(message);
  }
}

void UnitreeHW::publishStandState(const char *state) {
  if (standStatePublisher_) {
    std_msgs::msg::String message;
    message.data = state;
    standStatePublisher_->publish(message);
  }
  if (standReadyPublisher_) {
    std_msgs::msg::Bool message;
    const auto mode = outputMode_.load(std::memory_order_acquire);
    message.data = mode == OutputMode::HOLDING_STAND ||
                   mode == OutputMode::MPC;
    standReadyPublisher_->publish(message);
  }
}

void UnitreeHW::updateTemp(const rclcpp::Time & /*time*/) {
  constexpr int64_t kPublishPeriodNs = 20'000'000; // 50 Hz
  const int64_t nowSteadyNs = steadyNowNs();
  if (!tempPublisher_ ||
      (lastTempPublishSteadyNs_ != 0 &&
       nowSteadyNs - lastTempPublishSteadyNs_ < kPublishPeriodNs)) {
    return;
  }
  lastTempPublishSteadyNs_ = nowSteadyNs;

  std_msgs::msg::Float64MultiArray motorTemperatures;
  motorTemperatures.data.resize(jointTemperatures_.size());
  for (size_t i = 0; i < jointTemperatures_.size(); ++i) {
    motorTemperatures.data[i] = static_cast<double>(jointTemperatures_[i]);
  }
  tempPublisher_->publish(motorTemperatures);
}

uint32_t UnitreeHW::crc32Core(uint32_t *ptr, uint32_t len) {
  uint32_t crc32 = 0xFFFFFFFF;
  constexpr uint32_t polynomial = 0x04c11db7;

  for (uint32_t i = 0; i < len; ++i) {
    uint32_t xbit = 1U << 31;
    const uint32_t data = ptr[i];
    for (uint32_t bits = 0; bits < 32; ++bits) {
      if (crc32 & 0x80000000) {
        crc32 = (crc32 << 1) ^ polynomial;
      } else {
        crc32 <<= 1;
      }

      if (data & xbit) {
        crc32 ^= polynomial;
      }
      xbit >>= 1;
    }
  }

  return crc32;
}

int UnitreeHW::footForceIndex(const std::string &contactName) {
  if (contactName.find("RF") != std::string::npos) {
    return 0;
  }
  if (contactName.find("LF") != std::string::npos) {
    return 1;
  }
  if (contactName.find("RH") != std::string::npos) {
    return 2;
  }
  if (contactName.find("LH") != std::string::npos) {
    return 3;
  }
  return -1;
}

int UnitreeHW::motorIndex(const std::string &jointName) {
  // Unitree Go2 LowState/LowCmd order is FR, FL, RR, RL.  The controller's
  // canonical order is LF, LH, RF, RH, so direct array indexing is unsafe.
  static const std::unordered_map<std::string, int> indices{
      {"RF_HAA", 0}, {"RF_HFE", 1}, {"RF_KFE", 2},  {"LF_HAA", 3},
      {"LF_HFE", 4}, {"LF_KFE", 5}, {"RH_HAA", 6},  {"RH_HFE", 7},
      {"RH_KFE", 8}, {"LH_HAA", 9}, {"LH_HFE", 10}, {"LH_KFE", 11},
  };
  const auto it = indices.find(jointName);
  return it == indices.end() ? -1 : it->second;
}

std::string UnitreeHW::interfaceKey(const std::string &component,
                                    const std::string &interfaceName) {
  return component + "/" + interfaceName;
}

void UnitreeHW::setImuState(const std::string &interfaceName, double value) {
  set_state(interfaceKey(IMU_NAME, interfaceName), value);
}

} // namespace legged

PLUGINLIB_EXPORT_CLASS(legged::UnitreeHW, hardware_interface::SystemInterface)
