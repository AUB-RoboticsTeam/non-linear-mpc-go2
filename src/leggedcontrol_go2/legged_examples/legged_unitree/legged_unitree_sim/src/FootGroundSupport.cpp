//
// Created by codex.
//

#include "legged_unitree_sim/FootGroundSupport.h"

#include <algorithm>
#include <cmath>
#include <iostream>

#include <gz/math/Vector3.hh>
#include <gz/plugin/RegisterMore.hh>

namespace legged {
namespace sim {

void FootGroundSupport::Configure(const gz::sim::Entity& entity, const std::shared_ptr<const sdf::Element>& sdf,
                                  gz::sim::EntityComponentManager& ecm, gz::sim::EventManager& /*eventMgr*/) {
  model_ = gz::sim::Model(entity);

  if (sdf) {
    if (sdf->HasElement("ground_z")) {
      groundZ_ = sdf->Get<double>("ground_z");
    }
    if (sdf->HasElement("foot_radius")) {
      footRadius_ = sdf->Get<double>("foot_radius");
    }
    if (sdf->HasElement("normal_stiffness")) {
      normalStiffness_ = sdf->Get<double>("normal_stiffness");
    }
    if (sdf->HasElement("normal_damping")) {
      normalDamping_ = sdf->Get<double>("normal_damping");
    }
    if (sdf->HasElement("tangential_damping")) {
      tangentialDamping_ = sdf->Get<double>("tangential_damping");
    }
    if (sdf->HasElement("friction_coefficient")) {
      frictionCoefficient_ = sdf->Get<double>("friction_coefficient");
    }
    if (sdf->HasElement("max_normal_force")) {
      maxNormalForce_ = sdf->Get<double>("max_normal_force");
    }
  }

  const std::vector<std::string> names{"LF_FOOT", "RF_FOOT", "LH_FOOT", "RH_FOOT"};
  feet_.clear();
  for (const auto& name : names) {
    const auto entityId = model_.LinkByName(ecm, name);
    if (entityId == gz::sim::kNullEntity) {
      std::cerr << "[FootGroundSupport] could not resolve foot link " << name << std::endl;
      continue;
    }
    gz::sim::Link link(entityId);
    link.EnableVelocityChecks(ecm, true);
    feet_.push_back({name, link});
  }

  std::cerr << "[FootGroundSupport] configured with " << feet_.size()
            << " feet, ground_z=" << groundZ_
            << ", foot_radius=" << footRadius_ << std::endl;
}

void FootGroundSupport::PreUpdate(const gz::sim::UpdateInfo& info, gz::sim::EntityComponentManager& ecm) {
  if (info.paused) {
    return;
  }

  for (const auto& foot : feet_) {
    const auto pose = foot.link.WorldPose(ecm);
    if (!pose) {
      continue;
    }

    const double penetration = groundZ_ + footRadius_ - pose->Pos().Z();
    if (penetration <= 0.0) {
      continue;
    }

    gz::math::Vector3d velocity = gz::math::Vector3d::Zero;
    if (const auto vel = foot.link.WorldLinearVelocity(ecm)) {
      velocity = *vel;
    }

    double normalForce = normalStiffness_ * penetration - normalDamping_ * std::min(0.0, velocity.Z());
    normalForce = std::clamp(normalForce, 0.0, maxNormalForce_);

    gz::math::Vector3d tangentialForce{-tangentialDamping_ * velocity.X(), -tangentialDamping_ * velocity.Y(), 0.0};
    const double tangentialLimit = frictionCoefficient_ * normalForce;
    const double tangentialNorm = std::hypot(tangentialForce.X(), tangentialForce.Y());
    if (tangentialNorm > tangentialLimit && tangentialNorm > 1e-9) {
      tangentialForce *= tangentialLimit / tangentialNorm;
    }

    foot.link.AddWorldForce(ecm, gz::math::Vector3d(tangentialForce.X(), tangentialForce.Y(), normalForce));
  }
}

}  // namespace sim
}  // namespace legged

GZ_ADD_PLUGIN(legged::sim::FootGroundSupport, gz::sim::System, gz::sim::ISystemConfigure, gz::sim::ISystemPreUpdate)
