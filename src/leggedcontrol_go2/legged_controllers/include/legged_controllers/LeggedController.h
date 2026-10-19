//
// Created by qiayuan on 2022/6/24.
//

#pragma once

#include <atomic>
#include <memory>
#include <limits>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include <controller_interface/controller_interface.hpp>
#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <legged_common/hardware_interface/ContactSensorInterface.h>
#include <legged_common/hardware_interface/HybridJointInterface.h>
#include <legged_terrain_msgs/msg/foothold_plan.hpp>
#include <ocs2_centroidal_model/CentroidalModelRbdConversions.h>
#include <ocs2_core/misc/Benchmark.h>
#include <ocs2_legged_robot_ros/visualization/LeggedRobotVisualizer.h>
#include <ocs2_mpc/MPC_MRT_Interface.h>
#include <ocs2_msgs/msg/mpc_observation.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>

#include <legged_estimation/StateEstimateBase.h>
#include <legged_interface/LeggedInterface.h>
#include <legged_wbc/WbcBase.h>

#include "legged_controllers/SafetyChecker.h"
#include "legged_controllers/visualization/LeggedSelfCollisionVisualization.h"

namespace legged {
using namespace ocs2;
using namespace legged_robot;

class LeggedController : public controller_interface::ControllerInterface {
 public:
  LeggedController() = default;
  ~LeggedController() override;

  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;

 protected:
  virtual void updateStateEstimation(const rclcpp::Time& time, const rclcpp::Duration& period);

  virtual void setupLeggedInterface(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile,
                                    bool verbose);
  virtual void setupMpc();
  virtual void setupMrt();
  virtual void setupStateEstimate(const std::string& taskFile, bool verbose);

  double stateValue(const std::string& key) const;
  bool setCommandValue(const std::string& key, double value);
  void footholdPlanCallback(const legged_terrain_msgs::msg::FootholdPlan::ConstSharedPtr& msg);

  // Interface
  std::shared_ptr<LeggedInterface> leggedInterface_;
  std::shared_ptr<PinocchioEndEffectorKinematics> eeKinematicsPtr_;
  std::vector<std::string> jointNames_;
  std::vector<std::string> contactNames_;
  std::vector<std::string> contactSensorNames_;
  std::string imuName_;
  std::vector<std::string> commandInterfaceNames_;
  std::vector<std::string> stateInterfaceNames_;
  std::unordered_map<std::string, size_t> commandInterfaceIndex_;
  std::unordered_map<std::string, size_t> stateInterfaceIndex_;
  bool simulationMode_{false};
  double simulationKp_{0.0};
  double simulationKd_{3.0};
  bool useScheduledContactsForEstimator_{false};
  bool enableVisualization_{true};
  bool publishMpcObservation_{true};
  double mpcObservationPublishRate_{500.0};
  bool requireSafeRealActivationPose_{true};
  double minimumRealActivationHeight_{0.22};
  double maximumRealActivationTilt_{0.60};
  double maximumRealContactForce_{600.0};
  bool useForceTorqueContacts_{false};
  double contactForceThreshold_{20.0};
  contact_flag_t measuredContactFlags_{};
  std::atomic_bool terrainPlanActive_{false};

  // State Estimation
  SystemObservation currentObservation_;
  vector_t measuredRbdState_;
  std::shared_ptr<StateEstimateBase> stateEstimate_;
  std::shared_ptr<CentroidalModelRbdConversions> rbdConversions_;

  // Whole Body Control
  std::shared_ptr<WbcBase> wbc_;
  std::shared_ptr<SafetyChecker> safetyChecker_;

  // Nonlinear MPC
  std::shared_ptr<MPC_BASE> mpc_;
  std::shared_ptr<MPC_MRT_Interface> mpcMrtInterface_;

  // Visualization
  std::shared_ptr<LeggedRobotVisualizer> robotVisualizer_;
  std::shared_ptr<LeggedSelfCollisionVisualization> selfCollisionVisualization_;
  rclcpp_lifecycle::LifecyclePublisher<ocs2_msgs::msg::MpcObservation>::SharedPtr observationPublisher_;
  rclcpp::Subscription<legged_terrain_msgs::msg::FootholdPlan>::SharedPtr footholdPlanSubscription_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr plannedModePublisher_;

  std::thread rosSpinThread_;
  rclcpp::Node::SharedPtr rosNode_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> rosExecutor_;

 private:
  std::thread mpcThread_;
  std::atomic_bool controllerRunning_{}, mpcRunning_{};
  benchmark::RepeatedTimer mpcTimer_;
  benchmark::RepeatedTimer wbcTimer_;
  double lastDiagnosticsTime_{0.0};
  double lastMpcObservationPublishTime_{-1.0};
  size_t lastPublishedPlannedMode_{std::numeric_limits<size_t>::max()};
  size_t plannedModePublishCounter_{0};

  bool realCommandSafe(const vector_t& observedState, const vector_t& optimizedState, const vector_t& optimizedInput,
                       const vector_t& torque) const;
};

class LeggedCheaterController : public LeggedController {
 protected:
  void setupStateEstimate(const std::string& taskFile, bool verbose) override;
};

}  // namespace legged
