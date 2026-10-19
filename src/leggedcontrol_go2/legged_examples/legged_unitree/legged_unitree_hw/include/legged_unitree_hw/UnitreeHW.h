//
// Created by qiayuan on 1/24/22.
// ROS 2 migration for Unitree Go2 DDS low-level hardware.
//

#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <geometry_msgs/msg/twist.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <legged_hw/LeggedHW.h>

#include <unitree/common/thread/thread.hpp>
#include <unitree/common/time/time_tool.hpp>
#include <unitree/idl/go2/LowCmd_.hpp>
#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/robot/b2/motion_switcher/motion_switcher_client.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

namespace legged {

constexpr double PosStopF = 2.146E+9;
constexpr double VelStopF = 16000.0;

class UnitreeHW : public LeggedHW {
public:
  UnitreeHW() = default;
  ~UnitreeHW() override;

  CallbackReturn
  on_init(const ::hardware_interface::HardwareComponentInterfaceParams &params)
      override;
  CallbackReturn
  on_activate(const rclcpp_lifecycle::State &previous_state) override;
  CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &previous_state) override;
  CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State &previous_state) override;
  return_type read(const rclcpp::Time &time,
                   const rclcpp::Duration &period) override;
  return_type write(const rclcpp::Time &time,
                    const rclcpp::Duration &period) override;
  return_type perform_command_mode_switch(
      const std::vector<std::string> &start_interfaces,
      const std::vector<std::string> &stop_interfaces) override;

private:
  struct CachedJointCommand {
    double q{0.0};
    double dq{0.0};
    double kp{0.0};
    double kd{0.0};
    double tau{0.0};
  };

  enum class OutputMode : uint8_t {
    PASSIVE = 0,
    STANDING_UP = 1,
    HOLDING_STAND = 2,
    MPC = 3,
    DAMPING = 4,
  };

  void initLowCmd();
  void lowStateMessageHandler(const void *message);
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void updateTemp(const rclcpp::Time &time);
  void publishJoystick(const rclcpp::Time &time,
                       const unitree_go::msg::dds_::LowState_ &lowState);
  void publishCommandState();
  void publishStandState(const char *state);
  void publishPassiveCommand();
  void writeLowCmd();
  void writeDampingCommand();
  void writeMpcCommand();
  return_type writeStandCommand();
  void startCommandThread();
  void stopCommandThread();
  void commandThreadLoop();
  void commandThreadIteration();
  bool standPoseWithin(double positionLimit, double velocityLimit,
                       double &maxPositionError, double &maxVelocity);
  bool controllerCommandFresh() const;
  void disableCommands(const char *reason, bool publish_passive);
  bool lowStateFresh() const;
  bool prepareMotionMode(std::string &error);
  void commandEnableCallback(
      const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
      std::shared_ptr<std_srvs::srv::SetBool::Response> response);
  void standUpCallback(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void dampCallback(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  uint32_t crc32Core(uint32_t *ptr, uint32_t len);

  static int motorIndex(const std::string &jointName);
  static int footForceIndex(const std::string &contactName);
  static std::string interfaceKey(const std::string &component,
                                  const std::string &interfaceName);
  void setImuState(const std::string &interfaceName, double value);

  unitree_go::msg::dds_::LowCmd_ lowCmd_{};
  unitree_go::msg::dds_::LowState_ lowState_{};
  std::mutex lowStateMutex_;
  std::mutex lowCmdMutex_;
  std::mutex mpcCommandMutex_;
  std::array<CachedJointCommand, 12> cachedMpcCommands_{};
  std::thread commandThread_;

  unitree::robot::ChannelPublisherPtr<unitree_go::msg::dds_::LowCmd_>
      lowCmdPublisher_;
  unitree::robot::ChannelSubscriberPtr<unitree_go::msg::dds_::LowState_>
      lowStateSubscriber_;

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr tempPublisher_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr kpPublisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr commandStatePublisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr standReadyPublisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr standStatePublisher_;
  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr joystickPublisher_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmdVelSubscriber_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr commandEnableService_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr standUpService_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr dampService_;
  std::shared_ptr<unitree::robot::b2::MotionSwitcherClient>
      motionSwitcherClient_;

  std::array<int8_t, 12> jointTemperatures_{};
  int64_t lastTempPublishSteadyNs_{0};
  int64_t lastJoystickPublishSteadyNs_{0};

  std::atomic<bool> lowStateReceived_{false};
  std::atomic<bool> hardwareActive_{false};
  std::atomic<bool> commandsEnabled_{false};
  std::atomic<bool> commandReady_{false};
  std::atomic<int64_t> lastLowStateSteadyNs_{0};
  std::atomic<int64_t> armStartSteadyNs_{0};
  std::atomic<int64_t> lastControllerCommandSteadyNs_{0};
  std::atomic<int64_t> standStartSteadyNs_{0};
  std::atomic<int64_t> standConvergedSinceNs_{0};
  std::atomic<bool> commandThreadRunning_{false};
  std::atomic<OutputMode> outputMode_{OutputMode::PASSIVE};

  std::array<double, 12> standStartPositions_{};
  bool standUseCrouchStage_{false};

  int powerLimit_{4};
  int contactThreshold_{30};
  int originalContactThreshold_{30};
  int domainId_{0};
  std::string networkInterface_;
  double stateTimeout_{0.10};
  double controllerCommandTimeout_{0.05};
  double commandStreamRate_{500.0};
  double startupTimeout_{5.0};
  double armRampDuration_{2.0};
  double maxTorque_{30.0};
  double maxKp_{80.0};
  double maxKd_{10.0};
  double standDuration_{8.0};
  double standKp_{50.0};
  double standHoldKp_{70.0};
  double standKd_{5.0};
  double standCrouchHfeTarget_{0.90};
  double standCrouchKfeTarget_{-1.80};
  double standCrouchFraction_{0.45};
  double standHfeTarget_{0.60};
  double standKfeTarget_{-1.20};
  double standPositionTolerance_{0.18};
  double standVelocityTolerance_{0.35};
  double standSettleDuration_{2.0};
  double dampingKd_{3.0};
  bool releaseModeOnArm_{false};
  bool requireMotionModeCheck_{true};
};

} // namespace legged
