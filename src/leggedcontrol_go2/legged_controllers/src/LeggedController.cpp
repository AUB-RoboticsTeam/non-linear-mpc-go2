//
// Created by qiayuan on 2022/6/24.
//

#include <pinocchio/fwd.hpp>  // forward declarations must be included first.

#include "legged_controllers/LeggedController.h"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>

#include <angles/angles.h>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_core/thread_support/ExecuteAndSleep.h>
#include <ocs2_core/thread_support/SetThreadPriority.h>
#include <ocs2_legged_robot_ros/gait/GaitReceiver.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <ocs2_ros_interfaces/common/RosMsgConversions.h>
#include <ocs2_ros_interfaces/synchronized_module/RosReferenceManager.h>
#include <ocs2_sqp/SqpMpc.h>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>

#include <legged_estimation/FromTopiceEstimate.h>
#include <legged_estimation/LinearKalmanFilter.h>
#include <legged_wbc/HierarchicalWbc.h>
#include <legged_wbc/WeightedWbc.h>

namespace legged {
namespace {
constexpr auto kRobotName = "legged_robot";
constexpr auto kImuName = "unitree_imu";

std::string key(const std::string& prefix, const std::string& interfaceName) {
  return prefix + "/" + interfaceName;
}

std::vector<std::string> defaultJointNames() {
  // Must match Pinocchio's actuated-coordinate order from robot*.xacro.
  return {"LF_HAA", "LF_HFE", "LF_KFE", "LH_HAA", "LH_HFE", "LH_KFE",
          "RF_HAA", "RF_HFE", "RF_KFE", "RH_HAA", "RH_HFE", "RH_KFE"};
}

std::vector<std::string> imuStateInterfaceNames(const std::string& imuName) {
  return {key(imuName, "orientation.x"),         key(imuName, "orientation.y"),         key(imuName, "orientation.z"),
          key(imuName, "orientation.w"),         key(imuName, "angular_velocity.x"),    key(imuName, "angular_velocity.y"),
          key(imuName, "angular_velocity.z"),    key(imuName, "linear_acceleration.x"), key(imuName, "linear_acceleration.y"),
          key(imuName, "linear_acceleration.z")};
}

std::string lower(std::string value) {
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return value;
}
}  // namespace

controller_interface::CallbackReturn LeggedController::on_init() {
  auto node = get_node();
  node->declare_parameter<std::string>("urdfFile", "");
  node->declare_parameter<std::string>("taskFile", "");
  node->declare_parameter<std::string>("referenceFile", "");
  node->declare_parameter<std::vector<std::string>>("joints", defaultJointNames());
  node->declare_parameter<std::string>("imu_name", kImuName);
  node->declare_parameter<std::vector<std::string>>("contact_sensor_names", std::vector<std::string>{});
  node->declare_parameter<double>("contact_force_threshold", 20.0);
  node->declare_parameter<std::string>("wbc_type", "weighted");
  node->declare_parameter<bool>("cheater", false);
  node->declare_parameter<bool>("simulation_mode", false);
  node->declare_parameter<double>("simulation_kp", 0.0);
  node->declare_parameter<double>("simulation_kd", 3.0);
  node->declare_parameter<bool>("use_scheduled_contacts_for_estimator", false);
  node->declare_parameter<bool>("enable_visualization", true);
  node->declare_parameter<bool>("publish_mpc_observation", true);
  node->declare_parameter<double>("mpc_observation_publish_rate", 500.0);
  node->declare_parameter<bool>("publish_estimated_base_tf", true);
  node->declare_parameter<bool>("require_safe_real_activation_pose", true);
  node->declare_parameter<double>("minimum_real_activation_height", 0.22);
  node->declare_parameter<double>("maximum_real_activation_tilt", 0.60);
  node->declare_parameter<double>("maximum_real_contact_force", 600.0);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn LeggedController::on_configure(const rclcpp_lifecycle::State& /*previous_state*/) {
  auto node = get_node();

  const auto urdfFile = node->get_parameter("urdfFile").as_string();
  const auto taskFile = node->get_parameter("taskFile").as_string();
  const auto referenceFile = node->get_parameter("referenceFile").as_string();
  jointNames_ = node->get_parameter("joints").as_string_array();
  imuName_ = node->get_parameter("imu_name").as_string();
  contactSensorNames_ = node->get_parameter("contact_sensor_names").as_string_array();
  contactForceThreshold_ = node->get_parameter("contact_force_threshold").as_double();
  const auto wbcType = lower(node->get_parameter("wbc_type").as_string());
  simulationMode_ = node->get_parameter("simulation_mode").as_bool();
  simulationKp_ = node->get_parameter("simulation_kp").as_double();
  simulationKd_ = node->get_parameter("simulation_kd").as_double();
  useScheduledContactsForEstimator_ = node->get_parameter("use_scheduled_contacts_for_estimator").as_bool();
  enableVisualization_ = node->get_parameter("enable_visualization").as_bool();
  publishMpcObservation_ = node->get_parameter("publish_mpc_observation").as_bool();
  mpcObservationPublishRate_ = node->get_parameter("mpc_observation_publish_rate").as_double();
  if (mpcObservationPublishRate_ <= 0.0) {
    RCLCPP_ERROR(node->get_logger(), "mpc_observation_publish_rate must be positive.");
    return controller_interface::CallbackReturn::ERROR;
  }
  if (simulationKp_ < 0.0 || simulationKd_ < 0.0) {
    RCLCPP_ERROR(node->get_logger(), "simulation_kp and simulation_kd must be non-negative.");
    return controller_interface::CallbackReturn::ERROR;
  }
  RCLCPP_INFO(node->get_logger(), "Simulation joint tracking gains: kp=%.2f kd=%.2f", simulationKp_, simulationKd_);
  requireSafeRealActivationPose_ = node->get_parameter("require_safe_real_activation_pose").as_bool();
  minimumRealActivationHeight_ = node->get_parameter("minimum_real_activation_height").as_double();
  maximumRealActivationTilt_ = node->get_parameter("maximum_real_activation_tilt").as_double();
  maximumRealContactForce_ = node->get_parameter("maximum_real_contact_force").as_double();
  if (urdfFile.empty() || taskFile.empty() || referenceFile.empty()) {
    RCLCPP_ERROR(node->get_logger(), "Parameters urdfFile, taskFile, and referenceFile are required.");
    return controller_interface::CallbackReturn::ERROR;
  }
  if (jointNames_.empty()) {
    RCLCPP_ERROR(node->get_logger(), "Parameter joints must not be empty.");
    return controller_interface::CallbackReturn::ERROR;
  }
  rosNode_ = std::make_shared<rclcpp::Node>(std::string(node->get_name()) + "_ocs2");
  rosExecutor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
  rosExecutor_->add_node(rosNode_);
  rosSpinThread_ = std::thread([this]() { rosExecutor_->spin(); });

  bool verbose = false;
  loadData::loadCppDataType(taskFile, "legged_robot_interface.verbose", verbose);

  setupLeggedInterface(taskFile, urdfFile, referenceFile, verbose);
  setupMpc();
  setupMrt();

  CentroidalModelPinocchioMapping pinocchioMapping(leggedInterface_->getCentroidalModelInfo());
  eeKinematicsPtr_ = std::make_shared<PinocchioEndEffectorKinematics>(leggedInterface_->getPinocchioInterface(), pinocchioMapping,
                                                                      leggedInterface_->modelSettings().contactNames3DoF);
  if (enableVisualization_) {
    robotVisualizer_ = std::make_shared<LeggedRobotVisualizer>(
        leggedInterface_->getPinocchioInterface(),
        leggedInterface_->getCentroidalModelInfo(), *eeKinematicsPtr_,
        rosNode_);
    robotVisualizer_->publishBaseTf_ =
        node->get_parameter("publish_estimated_base_tf").as_bool();
    selfCollisionVisualization_ =
        std::make_shared<LeggedSelfCollisionVisualization>(
            leggedInterface_->getPinocchioInterface(),
            leggedInterface_->getGeometryInterface(), pinocchioMapping);
  } else {
    RCLCPP_WARN(node->get_logger(),
                "Controller visualization is disabled on the real-time path");
  }

  contactNames_ = leggedInterface_->modelSettings().contactNames3DoF;
  footholdPlanSubscription_ = rosNode_->create_subscription<legged_terrain_msgs::msg::FootholdPlan>(
      "/legged_robot/foothold_plan", rclcpp::QoS(1).reliable(),
      std::bind(&LeggedController::footholdPlanCallback, this, std::placeholders::_1));
  plannedModePublisher_ = rosNode_->create_publisher<std_msgs::msg::UInt8>("/legged_robot/planned_mode", 1);
  useForceTorqueContacts_ = !contactSensorNames_.empty();
  if (useForceTorqueContacts_ && contactSensorNames_.size() != contactNames_.size()) {
    RCLCPP_ERROR(node->get_logger(), "contact_sensor_names must be empty or have one entry per OCS2 contact.");
    return controller_interface::CallbackReturn::ERROR;
  }

  setupStateEstimate(taskFile, verbose);

  if (wbcType == "weighted") {
    wbc_ = std::make_shared<WeightedWbc>(leggedInterface_->getPinocchioInterface(), leggedInterface_->getCentroidalModelInfo(),
                                         *eeKinematicsPtr_);
  } else if (wbcType == "hoq" || wbcType == "hierarchical" || wbcType == "hierarchical_wbc") {
    wbc_ = std::make_shared<HierarchicalWbc>(leggedInterface_->getPinocchioInterface(), leggedInterface_->getCentroidalModelInfo(),
                                             *eeKinematicsPtr_);
  } else {
    RCLCPP_ERROR(node->get_logger(), "Unknown wbc_type '%s'. Use 'weighted' or 'hoq'.", wbcType.c_str());
    return controller_interface::CallbackReturn::ERROR;
  }
  wbc_->loadTasksSetting(taskFile, verbose);
  RCLCPP_INFO(node->get_logger(), "Using %s whole-body controller.", wbcType.c_str());
  safetyChecker_ = std::make_shared<SafetyChecker>(leggedInterface_->getCentroidalModelInfo());

  commandInterfaceNames_.clear();
  for (const auto& jointName : jointNames_) {
    if (simulationMode_) {
      commandInterfaceNames_.push_back(key(jointName, ::hardware_interface::HW_IF_EFFORT));
    } else {
      for (const auto interfaceName : legged::hardware_interface::HYBRID_JOINT_COMMAND_INTERFACES) {
        commandInterfaceNames_.push_back(key(jointName, std::string{interfaceName}));
      }
    }
  }

  stateInterfaceNames_.clear();
  for (const auto& jointName : jointNames_) {
    for (const auto interfaceName : legged::hardware_interface::HYBRID_JOINT_STATE_INTERFACES) {
      stateInterfaceNames_.push_back(key(jointName, std::string{interfaceName}));
    }
  }
  if (useForceTorqueContacts_) {
    for (const auto& contactSensorName : contactSensorNames_) {
      stateInterfaceNames_.push_back(key(contactSensorName, "force.z"));
    }
  } else {
    for (const auto& contactName : contactNames_) {
      stateInterfaceNames_.push_back(key(contactName, legged::hardware_interface::HW_IF_CONTACT));
    }
  }
  const auto imuInterfaces = imuStateInterfaceNames(imuName_);
  stateInterfaceNames_.insert(stateInterfaceNames_.end(), imuInterfaces.begin(), imuInterfaces.end());

  RCLCPP_INFO(node->get_logger(), "Configured legged controller with %zu joints and %zu contacts.", jointNames_.size(), contactNames_.size());
  return controller_interface::CallbackReturn::SUCCESS;
}

void LeggedController::footholdPlanCallback(const legged_terrain_msgs::msg::FootholdPlan::ConstSharedPtr& msg) {
  SwitchedModelReferenceManager::FootholdHeightPlan plan;
  plan.activationTime = currentObservation_.time;
  bool terrainPlanActive = false;

  for (size_t source = 0; source < msg->foot_names.size(); ++source) {
    const auto destination = std::find(contactNames_.begin(), contactNames_.end(), msg->foot_names[source]);
    if (destination == contactNames_.end()) {
      if (msg->valid[source]) {
        RCLCPP_WARN(rosNode_->get_logger(), "Ignoring foothold for unknown foot '%s'.", msg->foot_names[source].c_str());
      }
      continue;
    }
    const size_t leg = std::distance(contactNames_.begin(), destination);
    plan.liftOffPositions[leg] << msg->lift_off_positions[source].x, msg->lift_off_positions[source].y,
        msg->lift_off_positions[source].z;
    plan.touchDownPositions[leg] << msg->touch_down_positions[source].x, msg->touch_down_positions[source].y,
        msg->touch_down_positions[source].z;
    plan.minimumApexHeights[leg] = msg->minimum_apex_heights[source];
    plan.valid[leg] = msg->valid[source];
    plan.horizontalValid[leg] = msg->horizontal_valid[source];
    terrainPlanActive = terrainPlanActive || msg->valid[source] || msg->region_valid[source];
    if (msg->region_valid[source] && msg->convex_regions[source].points.size() >= 3) {
      auto& region = plan.regions[leg];
      region.planeOrigin << msg->plane_origins[source].x, msg->plane_origins[source].y,
          msg->plane_origins[source].z;
      region.planeNormal << msg->plane_normals[source].x, msg->plane_normals[source].y,
          msg->plane_normals[source].z;
      if (region.planeNormal.norm() < 1e-6) region.planeNormal = vector3_t::UnitZ();
      region.planeNormal.normalize();
      const auto& vertices = msg->convex_regions[source].points;
      region.A.resize(vertices.size(), 3);
      region.b.resize(vertices.size());
      bool finite = true;
      for (size_t vertex = 0; vertex < vertices.size(); ++vertex) {
        const auto& p0msg = vertices[vertex];
        const auto& p1msg = vertices[(vertex + 1) % vertices.size()];
        const vector3_t p0{p0msg.x, p0msg.y, p0msg.z};
        const vector3_t p1{p1msg.x, p1msg.y, p1msg.z};
        vector3_t inward = region.planeNormal.cross(p1 - p0);
        if (inward.norm() < 1e-6) {
          finite = false;
          break;
        }
        inward.normalize();
        region.A.row(vertex) = inward.transpose();
        region.b(vertex) = -inward.dot(p0);
      }
      region.valid = finite;
    }
  }

  leggedInterface_->getSwitchedModelReferenceManagerPtr()->setFootholdHeightPlan(plan);
  terrainPlanActive_.store(terrainPlanActive, std::memory_order_release);
  RCLCPP_INFO(rosNode_->get_logger(), "Accepted foothold height plan at MPC time %.3f.", plan.activationTime);
}

controller_interface::CallbackReturn LeggedController::on_activate(const rclcpp_lifecycle::State& /*previous_state*/) {
  commandInterfaceIndex_.clear();
  for (size_t i = 0; i < command_interfaces_.size(); ++i) {
    commandInterfaceIndex_[command_interfaces_[i].get_name()] = i;
  }
  stateInterfaceIndex_.clear();
  for (size_t i = 0; i < state_interfaces_.size(); ++i) {
    stateInterfaceIndex_[state_interfaces_[i].get_name()] = i;
  }

  currentObservation_.state.setZero(leggedInterface_->getCentroidalModelInfo().stateDim);
  updateStateEstimation(get_node()->get_clock()->now(), rclcpp::Duration::from_seconds(0.002));
  currentObservation_.input.setZero(leggedInterface_->getCentroidalModelInfo().inputDim);
  currentObservation_.mode = ModeNumber::STANCE;

  if (!simulationMode_ && requireSafeRealActivationPose_) {
    const vector_t basePose = centroidal_model::getBasePose(
        currentObservation_.state, leggedInterface_->getCentroidalModelInfo());
    if (!basePose.allFinite() || basePose(2) < minimumRealActivationHeight_ ||
        std::abs(basePose(4)) > maximumRealActivationTilt_ ||
        std::abs(basePose(5)) > maximumRealActivationTilt_) {
      RCLCPP_ERROR(
          get_node()->get_logger(),
          "Refusing real-hardware activation: pose is outside the locomotion MPC envelope "
          "(z=%.3f m, pitch=%.3f rad, roll=%.3f rad; required z>=%.3f and |tilt|<=%.3f). "
          "Stand the robot with Unitree high-level control before low-level takeover.",
          basePose(2), basePose(4), basePose(5), minimumRealActivationHeight_, maximumRealActivationTilt_);
      return controller_interface::CallbackReturn::FAILURE;
    }
  }

  TargetTrajectories targetTrajectories({currentObservation_.time}, {currentObservation_.state}, {currentObservation_.input});
  mpcMrtInterface_->setCurrentObservation(currentObservation_);
  mpcMrtInterface_->getReferenceManager().setTargetTrajectories(targetTrajectories);

  RCLCPP_INFO(get_node()->get_logger(), "Waiting for the initial policy ...");
  rclcpp::WallRate rate(leggedInterface_->mpcSettings().mrtDesiredFrequency_);
  while (!mpcMrtInterface_->initialPolicyReceived() && rclcpp::ok()) {
    mpcMrtInterface_->advanceMpc();
    rate.sleep();
  }
  RCLCPP_INFO(get_node()->get_logger(), "Initial policy has been received.");

  if (observationPublisher_) {
    observationPublisher_->on_activate();
  }
  mpcRunning_ = true;
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn LeggedController::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/) {
  mpcRunning_ = false;
  if (observationPublisher_) {
    observationPublisher_->on_deactivate();
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration LeggedController::command_interface_configuration() const {
  return {controller_interface::interface_configuration_type::INDIVIDUAL, commandInterfaceNames_};
}

controller_interface::InterfaceConfiguration LeggedController::state_interface_configuration() const {
  return {controller_interface::interface_configuration_type::INDIVIDUAL, stateInterfaceNames_};
}

controller_interface::return_type LeggedController::update(const rclcpp::Time& time, const rclcpp::Duration& period) {
  updateStateEstimation(time, period);

  mpcMrtInterface_->setCurrentObservation(currentObservation_);
  mpcMrtInterface_->updatePolicy();

  vector_t optimizedState, optimizedInput;
  size_t plannedMode = 0;
  mpcMrtInterface_->evaluatePolicy(currentObservation_.time, currentObservation_.state, optimizedState, optimizedInput, plannedMode);
  // Publish immediately on transitions and periodically at 20 Hz. Publishing
  // every real-time control cycle overwhelms Python terrain consumers while
  // they deserialize GridMap messages, causing either lost swing transitions
  // (small QoS depth) or a large stale-message backlog (large QoS depth).
  if (plannedMode != lastPublishedPlannedMode_ ||
      (++plannedModePublishCounter_ % 20) == 0) {
    std_msgs::msg::UInt8 plannedModeMessage;
    plannedModeMessage.data = static_cast<uint8_t>(plannedMode);
    plannedModePublisher_->publish(plannedModeMessage);
    lastPublishedPlannedMode_ = plannedMode;
  }
  currentObservation_.input = optimizedInput;

  // Event-based execution from Sec. VI-A of the paper. WBC must use contacts
  // that physically exist: regain a missing planned contact as a swing, and
  // accept an early contact only when the schedule expects it imminently.
  auto executionContacts = modeNumber2StanceLeg(plannedMode);
  vector_t executionInput = optimizedInput;
  // Preserve the proven WBC contact schedule for ordinary stance and gaits.
  // Paper Sec. VI-A event-based overrides are enabled only while an explicit
  // terrain plan is active.
  // Keep WBC synchronized with the proven MPC gait schedule.  The raw foot
  // force flags are not phase-debounced; using a single sample to remove a
  // scheduled support contact made the robot collapse at the first crawl
  // transition whenever any foothold plan was active.  Terrain foothold
  // height/XY/region references remain active in MPC.  Event-based contact
  // overrides must only be re-enabled after adding hysteresis and explicit
  // phase-window validation.
  constexpr bool kEnableTerrainContactOverrides = false;
  if (kEnableTerrainContactOverrides &&
      terrainPlanActive_.load(std::memory_order_acquire)) {
    const auto imminentContacts = modeNumber2StanceLeg(
        leggedInterface_->getSwitchedModelReferenceManagerPtr()->getModeSchedule().modeAtTime(
            currentObservation_.time + 0.12));
    for (size_t leg = 0; leg < executionContacts.size(); ++leg) {
      if (executionContacts[leg] && !measuredContactFlags_[leg]) {
        executionContacts[leg] = false;
      } else if (!executionContacts[leg] && measuredContactFlags_[leg] && imminentContacts[leg]) {
        executionContacts[leg] = true;
        // Maintain a small positive normal force until the next MPC plan starts
        // from the measured early-contact state.
        if (executionInput.size() > static_cast<Eigen::Index>(3 * leg + 2)) {
          executionInput(3 * leg + 2) = std::max(executionInput(3 * leg + 2), scalar_t(8.0));
        }
      }
    }
  }
  const size_t executionMode = stanceLeg2ModeNumber(executionContacts);

  std::array<bool, 4> swingReferenceActive{};
  std::array<Eigen::Vector2d, 4> swingReferencePosition{};
  std::array<Eigen::Vector2d, 4> swingReferenceVelocity{};
  const auto swingPlanner = leggedInterface_->getSwitchedModelReferenceManagerPtr()->getSwingTrajectoryPlanner();
  for (size_t leg = 0; leg < swingReferenceActive.size(); ++leg) {
    swingReferenceActive[leg] = swingPlanner->hasHorizontalReference(leg, currentObservation_.time);
    if (swingReferenceActive[leg]) {
      swingReferencePosition[leg] = swingPlanner->getXYpositionConstraint(leg, currentObservation_.time);
      swingReferenceVelocity[leg] = swingPlanner->getXYvelocityConstraint(leg, currentObservation_.time);
    }
  }
  wbc_->setSwingFootReferences(swingReferenceActive, swingReferencePosition, swingReferenceVelocity);

  wbcTimer_.startTimer();
  vector_t x = wbc_->update(optimizedState, executionInput, measuredRbdState_, executionMode, period.seconds());
  wbcTimer_.endTimer();

  const vector_t torque = x.tail(12);
  const vector_t posDes = centroidal_model::getJointAngles(optimizedState, leggedInterface_->getCentroidalModelInfo());
  const vector_t velDes = centroidal_model::getJointVelocities(optimizedInput, leggedInterface_->getCentroidalModelInfo());

  if (!optimizedState.allFinite() || !optimizedInput.allFinite() || !x.allFinite() ||
      !posDes.allFinite() || !velDes.allFinite() || !torque.allFinite()) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "[Legged Controller] MPC/WBC produced a non-finite command; refusing to send it.");
    mpcRunning_ = false;
    for (const auto& jointName : jointNames_) {
      if (simulationMode_) {
        setCommandValue(key(jointName, ::hardware_interface::HW_IF_EFFORT), 0.0);
      }
    }
    return controller_interface::return_type::ERROR;
  }

  if (!simulationMode_ &&
      !realCommandSafe(currentObservation_.state, optimizedState,
                       optimizedInput, torque)) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "[Legged Controller] Real-hardware command left the safety envelope; stopping before hardware output.");
    mpcRunning_ = false;
    return controller_interface::return_type::ERROR;
  }

  if (currentObservation_.time - lastDiagnosticsTime_ >= 1.0) {
    const vector_t observedPose = centroidal_model::getBasePose(currentObservation_.state, leggedInterface_->getCentroidalModelInfo());
    const vector_t desiredPose = centroidal_model::getBasePose(optimizedState, leggedInterface_->getCentroidalModelInfo());
    const vector_t observedJoints = centroidal_model::getJointAngles(currentObservation_.state, leggedInterface_->getCentroidalModelInfo());
    const double lfHfe = observedJoints.size() > 1 ? observedJoints(1) : 0.0;
    const double lfKfe = observedJoints.size() > 2 ? observedJoints(2) : 0.0;
    const double rfHfe = observedJoints.size() > 4 ? observedJoints(4) : 0.0;
    const double rfKfe = observedJoints.size() > 5 ? observedJoints(5) : 0.0;
    const auto plannedContactFlag = modeNumber2StanceLeg(plannedMode);
    RCLCPP_INFO(get_node()->get_logger(),
                "[Legged Controller] obs z=%.3f rpy=(%.3f %.3f %.3f), des z=%.3f rpy=(%.3f %.3f %.3f), mode=%zu, contact=%d%d%d%d, fz=[%.1f %.1f %.1f %.1f], qLF=(%.2f %.2f/%.2f %.2f), qRF=(%.2f %.2f/%.2f %.2f), torque=[%.2f %.2f]",
                observedPose(2), observedPose(5), observedPose(4), observedPose(3),
                desiredPose(2), desiredPose(5), desiredPose(4), desiredPose(3),
                plannedMode,
                plannedContactFlag.size() > 0 && plannedContactFlag[0],
                plannedContactFlag.size() > 1 && plannedContactFlag[1],
                plannedContactFlag.size() > 2 && plannedContactFlag[2],
                plannedContactFlag.size() > 3 && plannedContactFlag[3],
                optimizedInput.size() > 2 ? optimizedInput(2) : 0.0,
                optimizedInput.size() > 5 ? optimizedInput(5) : 0.0,
                optimizedInput.size() > 8 ? optimizedInput(8) : 0.0,
                optimizedInput.size() > 11 ? optimizedInput(11) : 0.0,
                lfHfe, lfKfe, posDes.size() > 1 ? posDes(1) : 0.0, posDes.size() > 2 ? posDes(2) : 0.0,
                rfHfe, rfKfe, posDes.size() > 4 ? posDes(4) : 0.0, posDes.size() > 5 ? posDes(5) : 0.0,
                torque.minCoeff(), torque.maxCoeff());
    lastDiagnosticsTime_ = currentObservation_.time;
  }

  if (!safetyChecker_->check(currentObservation_, optimizedState, optimizedInput)) {
    RCLCPP_ERROR(get_node()->get_logger(), "[Legged Controller] Safety check failed, stopping the controller.");
    mpcRunning_ = false;
    return controller_interface::return_type::ERROR;
  }

  for (size_t j = 0; j < leggedInterface_->getCentroidalModelInfo().actuatedDofNum; ++j) {
    const auto& jointName = jointNames_[j];
    if (simulationMode_) {
      const double jointPos = stateValue(key(jointName, ::hardware_interface::HW_IF_POSITION));
      const double jointVel = stateValue(key(jointName, ::hardware_interface::HW_IF_VELOCITY));
      const double hybridEffort =
          simulationKp_ * (posDes(j) - jointPos) + simulationKd_ * (velDes(j) - jointVel) + torque(j);
      setCommandValue(key(jointName, ::hardware_interface::HW_IF_EFFORT), hybridEffort);
    } else {
      setCommandValue(key(jointName, ::hardware_interface::HW_IF_POSITION), posDes(j));
      setCommandValue(key(jointName, ::hardware_interface::HW_IF_VELOCITY), velDes(j));
      setCommandValue(key(jointName, legged::hardware_interface::HW_IF_KP), 0.0);
      setCommandValue(key(jointName, legged::hardware_interface::HW_IF_KD), 3.0);
      setCommandValue(key(jointName, legged::hardware_interface::HW_IF_FEEDFORWARD), torque(j));
    }
  }

  if (robotVisualizer_) {
    robotVisualizer_->update(currentObservation_, mpcMrtInterface_->getPolicy(),
                             mpcMrtInterface_->getCommand());
  }
  if (selfCollisionVisualization_) {
    selfCollisionVisualization_->update(currentObservation_);
  }

  if (observationPublisher_ && observationPublisher_->is_activated() &&
      (lastMpcObservationPublishTime_ < 0.0 ||
       currentObservation_.time - lastMpcObservationPublishTime_ >=
           1.0 / mpcObservationPublishRate_)) {
    observationPublisher_->publish(ros_msg_conversions::createObservationMsg(currentObservation_));
    lastMpcObservationPublishTime_ = currentObservation_.time;
  }

  return controller_interface::return_type::OK;
}

void LeggedController::updateStateEstimation(const rclcpp::Time& time, const rclcpp::Duration& period) {
  vector_t jointPos(jointNames_.size()), jointVel(jointNames_.size());
  contact_flag_t contactFlag;
  Eigen::Quaternion<scalar_t> quat;
  vector3_t angularVel = vector3_t::Zero();
  vector3_t linearAccel = vector3_t::Zero();
  matrix3_t orientationCovariance = matrix3_t::Zero();
  matrix3_t angularVelCovariance = matrix3_t::Zero();
  matrix3_t linearAccelCovariance = matrix3_t::Zero();

  for (size_t i = 0; i < jointNames_.size(); ++i) {
    jointPos(i) = stateValue(key(jointNames_[i], ::hardware_interface::HW_IF_POSITION));
    jointVel(i) = stateValue(key(jointNames_[i], ::hardware_interface::HW_IF_VELOCITY));
  }

  for (size_t i = 0; i < contactFlag.size(); ++i) {
    if (i >= contactNames_.size()) {
      contactFlag[i] = false;
    } else if (useForceTorqueContacts_) {
      contactFlag[i] = std::abs(stateValue(key(contactSensorNames_[i], "force.z"))) > contactForceThreshold_;
    } else {
      contactFlag[i] = stateValue(key(contactNames_[i], legged::hardware_interface::HW_IF_CONTACT)) > 0.5;
    }
  }
  measuredContactFlags_ = contactFlag;

  quat.x() = stateValue(key(imuName_, "orientation.x"));
  quat.y() = stateValue(key(imuName_, "orientation.y"));
  quat.z() = stateValue(key(imuName_, "orientation.z"));
  quat.w() = stateValue(key(imuName_, "orientation.w"));
  angularVel.x() = stateValue(key(imuName_, "angular_velocity.x"));
  angularVel.y() = stateValue(key(imuName_, "angular_velocity.y"));
  angularVel.z() = stateValue(key(imuName_, "angular_velocity.z"));
  linearAccel.x() = stateValue(key(imuName_, "linear_acceleration.x"));
  linearAccel.y() = stateValue(key(imuName_, "linear_acceleration.y"));
  linearAccel.z() = stateValue(key(imuName_, "linear_acceleration.z"));

  auto estimatorContactFlag = contactFlag;
  if (useScheduledContactsForEstimator_) {
    estimatorContactFlag = modeNumber2StanceLeg(
        leggedInterface_->getSwitchedModelReferenceManagerPtr()->getModeSchedule().modeAtTime(currentObservation_.time));
  }

  stateEstimate_->updateJointStates(jointPos, jointVel);
  stateEstimate_->updateContact(estimatorContactFlag);
  stateEstimate_->updateImu(quat, angularVel, linearAccel, orientationCovariance, angularVelCovariance, linearAccelCovariance);
  measuredRbdState_ = stateEstimate_->update(time, period);
  currentObservation_.time += period.seconds();
  scalar_t yawLast = currentObservation_.state(9);
  currentObservation_.state = rbdConversions_->computeCentroidalStateFromRbdModel(measuredRbdState_);
  currentObservation_.state(9) = yawLast + angles::shortest_angular_distance(yawLast, currentObservation_.state(9));
  currentObservation_.mode = stateEstimate_->getMode();
}

double LeggedController::stateValue(const std::string& keyName) const {
  const auto it = stateInterfaceIndex_.find(keyName);
  if (it == stateInterfaceIndex_.end()) {
    throw std::runtime_error("Missing state interface: " + keyName);
  }
  return state_interfaces_[it->second].get_optional<double>().value_or(std::numeric_limits<double>::quiet_NaN());
}

bool LeggedController::setCommandValue(const std::string& keyName, double value) {
  const auto it = commandInterfaceIndex_.find(keyName);
  if (it == commandInterfaceIndex_.end()) {
    throw std::runtime_error("Missing command interface: " + keyName);
  }
  return command_interfaces_[it->second].set_value(value);
}

bool LeggedController::realCommandSafe(const vector_t& observedState, const vector_t& optimizedState,
                                       const vector_t& optimizedInput,
                                       const vector_t& torque) const {
  const vector_t observedPose = centroidal_model::getBasePose(
      observedState, leggedInterface_->getCentroidalModelInfo());
  const vector_t plannedPose = centroidal_model::getBasePose(
      optimizedState, leggedInterface_->getCentroidalModelInfo());
  const double maximumTorque = torque.size() > 0
                                   ? torque.cwiseAbs().maxCoeff()
                                   : std::numeric_limits<double>::infinity();
  if (!observedPose.allFinite() || !plannedPose.allFinite() ||
      observedPose(2) < 0.10 || observedPose(2) > 0.65 ||
      plannedPose(2) < 0.10 || plannedPose(2) > 0.65 ||
      std::abs(observedPose(4)) > maximumRealActivationTilt_ ||
      std::abs(observedPose(5)) > maximumRealActivationTilt_ ||
      std::abs(plannedPose(4)) > maximumRealActivationTilt_ ||
      std::abs(plannedPose(5)) > maximumRealActivationTilt_ ||
      maximumTorque > 40.0) {
    RCLCPP_ERROR(
        get_node()->get_logger(),
        "Real-command safety detail: observed=(finite=%d z=%.3f pitch=%.3f "
        "roll=%.3f), planned=(finite=%d z=%.3f pitch=%.3f roll=%.3f), "
        "max_torque=%.3f (limits: z=[0.10,0.65], |tilt|<=%.3f, "
        "torque<=40.0)",
        observedPose.allFinite(), observedPose(2), observedPose(4),
        observedPose(5), plannedPose.allFinite(), plannedPose(2),
        plannedPose(4), plannedPose(5), maximumTorque,
        maximumRealActivationTilt_);
    return false;
  }
  const size_t contactCount = std::min<size_t>(4, optimizedInput.size() / 3);
  for (size_t leg = 0; leg < contactCount; ++leg) {
    const double contactForce = optimizedInput.segment<3>(3 * leg).norm();
    if (contactForce > maximumRealContactForce_) {
      RCLCPP_ERROR(get_node()->get_logger(),
                   "Real-command safety detail: leg %zu contact force %.3f N "
                   "exceeds %.3f N",
                   leg, contactForce, maximumRealContactForce_);
      return false;
    }
  }
  return true;
}

LeggedController::~LeggedController() {
  controllerRunning_ = false;
  if (mpcThread_.joinable()) {
    mpcThread_.join();
  }
  if (rosExecutor_) {
    rosExecutor_->cancel();
  }
  if (rosSpinThread_.joinable()) {
    rosSpinThread_.join();
  }
  std::cerr << "########################################################################";
  std::cerr << "\n### MPC Benchmarking";
  std::cerr << "\n###   Maximum : " << mpcTimer_.getMaxIntervalInMilliseconds() << "[ms].";
  std::cerr << "\n###   Average : " << mpcTimer_.getAverageInMilliseconds() << "[ms]." << std::endl;
  std::cerr << "########################################################################";
  std::cerr << "\n### WBC Benchmarking";
  std::cerr << "\n###   Maximum : " << wbcTimer_.getMaxIntervalInMilliseconds() << "[ms].";
  std::cerr << "\n###   Average : " << wbcTimer_.getAverageInMilliseconds() << "[ms].";
}

void LeggedController::setupLeggedInterface(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile,
                                            bool verbose) {
  leggedInterface_ = std::make_shared<LeggedInterface>(taskFile, urdfFile, referenceFile);
  leggedInterface_->setupOptimalControlProblem(taskFile, urdfFile, referenceFile, verbose);
}

void LeggedController::setupMpc() {
  mpc_ = std::make_shared<SqpMpc>(leggedInterface_->mpcSettings(), leggedInterface_->sqpSettings(),
                                  leggedInterface_->getOptimalControlProblem(), leggedInterface_->getInitializer());
  rbdConversions_ = std::make_shared<CentroidalModelRbdConversions>(leggedInterface_->getPinocchioInterface(),
                                                                    leggedInterface_->getCentroidalModelInfo());

  auto node = rosNode_;
  auto gaitReceiverPtr =
      std::make_shared<GaitReceiver>(node, leggedInterface_->getSwitchedModelReferenceManagerPtr()->getGaitSchedule(), kRobotName);
  auto rosReferenceManagerPtr = std::make_shared<RosReferenceManager>(kRobotName, leggedInterface_->getReferenceManagerPtr());
  rosReferenceManagerPtr->subscribe(node);
  mpc_->getSolverPtr()->addSynchronizedModule(gaitReceiverPtr);
  mpc_->getSolverPtr()->setReferenceManager(rosReferenceManagerPtr);
  if (publishMpcObservation_) {
    observationPublisher_ =
        get_node()->create_publisher<ocs2_msgs::msg::MpcObservation>(
            std::string(kRobotName) + "_mpc_observation", 1);
  }
}

void LeggedController::setupMrt() {
  mpcMrtInterface_ = std::make_shared<MPC_MRT_Interface>(*mpc_);
  mpcMrtInterface_->initRollout(&leggedInterface_->getRollout());
  mpcTimer_.reset();

  controllerRunning_ = true;
  mpcThread_ = std::thread([&]() {
    while (controllerRunning_) {
      try {
        executeAndSleep(
            [&]() {
              if (mpcRunning_) {
                mpcTimer_.startTimer();
                mpcMrtInterface_->advanceMpc();
                mpcTimer_.endTimer();
              }
            },
            leggedInterface_->mpcSettings().mpcDesiredFrequency_);
      } catch (const std::exception& e) {
        controllerRunning_ = false;
        RCLCPP_ERROR(get_node()->get_logger(), "[Ocs2 MPC thread] Error: %s", e.what());
        mpcRunning_ = false;
      }
    }
  });
  setThreadPriority(leggedInterface_->sqpSettings().threadPriority, mpcThread_);
}

void LeggedController::setupStateEstimate(const std::string& taskFile, bool verbose) {
  stateEstimate_ = std::make_shared<KalmanFilterEstimate>(leggedInterface_->getPinocchioInterface(),
                                                          leggedInterface_->getCentroidalModelInfo(), *eeKinematicsPtr_, rosNode_);
  dynamic_cast<KalmanFilterEstimate&>(*stateEstimate_).loadSettings(taskFile, verbose);
  currentObservation_.time = 0;
}

void LeggedCheaterController::setupStateEstimate(const std::string& /*taskFile*/, bool /*verbose*/) {
  stateEstimate_ = std::make_shared<FromTopicStateEstimate>(leggedInterface_->getPinocchioInterface(),
                                                            leggedInterface_->getCentroidalModelInfo(), *eeKinematicsPtr_, rosNode_);
}

}  // namespace legged

PLUGINLIB_EXPORT_CLASS(legged::LeggedController, controller_interface::ControllerInterface)
PLUGINLIB_EXPORT_CLASS(legged::LeggedCheaterController, controller_interface::ControllerInterface)
