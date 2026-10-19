//
// Created by codex.
// Gazebo Sim model plugin that resets Unitree leg joints to the MPC nominal pose.
//

#pragma once

#include <cstdint>
#include <string>
#include <unordered_map>

#include <gz/msgs/boolean.pb.h>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>

namespace legged {
namespace sim {

class InitialJointPositionSetter : public gz::sim::System,
                                   public gz::sim::ISystemConfigure,
                                   public gz::sim::ISystemPreUpdate {
 public:
  void Configure(const gz::sim::Entity& entity, const std::shared_ptr<const sdf::Element>& sdf,
                 gz::sim::EntityComponentManager& ecm, gz::sim::EventManager& eventMgr) override;
  void PreUpdate(const gz::sim::UpdateInfo& info, gz::sim::EntityComponentManager& ecm) override;

 private:
  void loadJointPositions(const std::shared_ptr<const sdf::Element>& sdf);
  bool resetJoint(gz::sim::EntityComponentManager& ecm, const std::string& jointName, double position);
  void holdCallback(const gz::msgs::Boolean& msg);

  gz::sim::Model model_;
  std::unordered_map<std::string, double> jointPositions_;
  gz::transport::Node node_;
  std::string holdTopic_{"/legged_initial_pose_hold"};
  std::uint64_t resetSteps_{1};
  bool holdEnabled_{false};
  std::uint64_t updateCount_{0};
  bool loggedHolding_{false};
  bool loggedReleased_{false};
};

}  // namespace sim
}  // namespace legged
