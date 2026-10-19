//
// Created by codex.
//

#include "legged_unitree_sim/FootContactPublisher.h"

#include <algorithm>
#include <iostream>

#include <gz/sim/Model.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>

namespace legged {
namespace sim {

void FootContactPublisher::Configure(const gz::sim::Entity& /*entity*/, const std::shared_ptr<const sdf::Element>& sdf,
                                     gz::sim::EntityComponentManager& /*ecm*/, gz::sim::EventManager& /*eventMgr*/) {
  if (sdf) {
    if (sdf->HasElement("foot_radius")) {
      foot_radius_ = sdf->Get<double>("foot_radius");
    }
    if (sdf->HasElement("ground_z")) {
      ground_z_ = sdf->Get<double>("ground_z");
    }
    if (sdf->HasElement("contact_margin")) {
      contact_margin_ = sdf->Get<double>("contact_margin");
    }
  }

  std::cerr << "[FootContactPublisher] loading Gazebo foot contact publisher "
            << "foot_radius=" << foot_radius_ << " ground_z=" << ground_z_
            << " contact_margin=" << contact_margin_ << std::endl;

  for (auto& foot : feet_) {
    foot.publisher = node_.Advertise<gz::msgs::Boolean>(foot.topic);
    std::cerr << "[FootContactPublisher] advertising " << foot.topic
              << " for link " << foot.link_name << std::endl;
  }
}

void FootContactPublisher::resolveFootEntities(const gz::sim::EntityComponentManager& ecm) {
  for (auto& foot : feet_) {
    if (foot.entity != gz::sim::kNullEntity && ecm.HasEntity(foot.entity)) {
      continue;
    }

    foot.entity = gz::sim::kNullEntity;
    ecm.Each<gz::sim::components::Link, gz::sim::components::Name>(
        [&](const gz::sim::Entity& entity, const gz::sim::components::Link*, const gz::sim::components::Name* name) {
          if (name && name->Data() == foot.link_name) {
            foot.entity = entity;
            if (!foot.logged_resolved) {
              std::cerr << "[FootContactPublisher] resolved " << foot.link_name
                        << " as entity " << foot.entity << std::endl;
              foot.logged_resolved = true;
              foot.warned_unresolved = false;
            }
            return false;
          }
          return true;
        });
  }
}

bool FootContactPublisher::computeContact(const gz::sim::EntityComponentManager& ecm, const FootState& foot) const {
  const auto world_pose = ecm.Component<gz::sim::components::WorldPose>(foot.entity);
  if (world_pose) {
    const double foot_z = world_pose->Data().Pos().Z();
    return foot_z <= (ground_z_ + foot_radius_ + contact_margin_);
  }

  const auto pose = ecm.Component<gz::sim::components::Pose>(foot.entity);
  if (pose) {
    const double foot_z = pose->Data().Pos().Z();
    return foot_z <= (ground_z_ + foot_radius_ + contact_margin_);
  }

  return false;
}

void FootContactPublisher::publishFootState(FootState& foot, bool contact) {
  if (!foot.publisher) {
    return;
  }

  gz::msgs::Boolean msg;
  msg.set_data(contact);
  foot.publisher.Publish(msg);
  foot.last_state = contact;
}

void FootContactPublisher::PostUpdate(const gz::sim::UpdateInfo& info, const gz::sim::EntityComponentManager& ecm) {
  (void)info;
  ++update_count_;

  resolveFootEntities(ecm);

  for (auto& foot : feet_) {
    if (foot.entity == gz::sim::kNullEntity) {
      if (!foot.warned_unresolved && update_count_ > 100) {
        std::cerr << "[FootContactPublisher] foot link not found yet: " << foot.link_name
                  << "; publishing false on " << foot.topic << std::endl;
        foot.warned_unresolved = true;
      }
      publishFootState(foot, false);
      continue;
    }

    const bool contact = computeContact(ecm, foot);
    publishFootState(foot, contact);
  }

  logged_ready_ = true;
}

}  // namespace sim
}  // namespace legged

GZ_ADD_PLUGIN(legged::sim::FootContactPublisher, gz::sim::System, gz::sim::ISystemConfigure, gz::sim::ISystemPostUpdate)
