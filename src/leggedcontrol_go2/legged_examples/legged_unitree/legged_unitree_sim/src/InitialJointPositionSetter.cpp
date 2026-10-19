//
// Created by codex.
//

#include "legged_unitree_sim/InitialJointPositionSetter.h"

#include <iostream>
#include <memory>
#include <string>

#include <gz/plugin/RegisterMore.hh>
#include <gz/sim/components/JointPositionReset.hh>

namespace legged {
namespace sim {

void InitialJointPositionSetter::Configure(const gz::sim::Entity& entity, const std::shared_ptr<const sdf::Element>& sdf,
                                           gz::sim::EntityComponentManager& ecm, gz::sim::EventManager& /*eventMgr*/) {
  model_ = gz::sim::Model(entity);
  loadJointPositions(sdf);
  node_.Subscribe(holdTopic_, &InitialJointPositionSetter::holdCallback, this);
  std::cerr << "[InitialJointPositionSetter] configured with " << jointPositions_.size()
            << " joint positions for " << resetSteps_
            << " reset step(s). Topic " << holdTopic_
            << " can disable the initializer." << std::endl;

  bool allResolved = true;
  for (const auto& [jointName, position] : jointPositions_) {
    allResolved = resetJoint(ecm, jointName, position) && allResolved;
  }
  if (allResolved) {
    std::cerr << "[InitialJointPositionSetter] applied initial joint pose during configure." << std::endl;
  }
}

void InitialJointPositionSetter::PreUpdate(const gz::sim::UpdateInfo& /*info*/, gz::sim::EntityComponentManager& ecm) {
  if (!holdEnabled_) {
    if (!loggedReleased_) {
      std::cerr << "[InitialJointPositionSetter] released initial joint hold." << std::endl;
      loggedReleased_ = true;
    }
    if (resetSteps_ > 0 && updateCount_ >= resetSteps_) {
      return;
    }
  }

  if (!loggedHolding_) {
    if (holdEnabled_) {
      std::cerr << "[InitialJointPositionSetter] applying initial joint pose hold." << std::endl;
    } else {
      std::cerr << "[InitialJointPositionSetter] applying one-shot initial joint pose reset." << std::endl;
    }
    loggedHolding_ = true;
  }

  bool allResolved = true;
  for (const auto& [jointName, position] : jointPositions_) {
    allResolved = resetJoint(ecm, jointName, position) && allResolved;
  }

  if (allResolved) {
    ++updateCount_;
  }
}

void InitialJointPositionSetter::loadJointPositions(const std::shared_ptr<const sdf::Element>& sdf) {
  jointPositions_ = {
      {"LF_HAA", 0.0}, {"LF_HFE", 0.72}, {"LF_KFE", -1.44},
      {"RF_HAA", 0.0}, {"RF_HFE", 0.72}, {"RF_KFE", -1.44},
      {"LH_HAA", 0.0}, {"LH_HFE", 0.72}, {"LH_KFE", -1.44},
      {"RH_HAA", 0.0}, {"RH_HFE", 0.72}, {"RH_KFE", -1.44},
  };

  if (!sdf) {
    return;
  }
  if (sdf->HasElement("hold_topic")) {
    holdTopic_ = sdf->Get<std::string>("hold_topic");
  }
  if (sdf->HasElement("reset_steps")) {
    resetSteps_ = sdf->Get<std::uint64_t>("reset_steps");
  }
  if (sdf->HasElement("initial_hold")) {
    holdEnabled_ = sdf->Get<bool>("initial_hold");
  }

  auto jointElem = sdf->FindElement("joint");
  while (jointElem) {
    const auto name = jointElem->Get<std::string>("name");
    const auto position = jointElem->Get<double>("position");
    jointPositions_[name] = position;
    jointElem = jointElem->GetNextElement("joint");
  }
}

bool InitialJointPositionSetter::resetJoint(gz::sim::EntityComponentManager& ecm, const std::string& jointName, double position) {
  const auto joint = model_.JointByName(ecm, jointName);
  if (joint == gz::sim::kNullEntity) {
    return false;
  }

  auto reset = ecm.Component<gz::sim::components::JointPositionReset>(joint);
  if (reset) {
    reset->Data() = {position};
  } else {
    ecm.CreateComponent(joint, gz::sim::components::JointPositionReset({position}));
  }
  return true;
}

void InitialJointPositionSetter::holdCallback(const gz::msgs::Boolean& msg) {
  holdEnabled_ = msg.data();
  loggedHolding_ = false;
  loggedReleased_ = false;
}

}  // namespace sim
}  // namespace legged

GZ_ADD_PLUGIN(legged::sim::InitialJointPositionSetter, gz::sim::System, gz::sim::ISystemConfigure, gz::sim::ISystemPreUpdate)
