//
// Created by codex.
// Gazebo Sim foot contact publisher based on foot link height.
//

#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <string>

#include <gz/msgs/boolean.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>

namespace legged {
namespace sim {

class FootContactPublisher : public gz::sim::System,
                             public gz::sim::ISystemConfigure,
                             public gz::sim::ISystemPostUpdate {
 public:
  void Configure(const gz::sim::Entity& entity, const std::shared_ptr<const sdf::Element>& sdf,
                 gz::sim::EntityComponentManager& ecm, gz::sim::EventManager& eventMgr) override;
  void PostUpdate(const gz::sim::UpdateInfo& info, const gz::sim::EntityComponentManager& ecm) override;

 private:
  struct FootState {
    std::string link_name;
    std::string topic;
    gz::sim::Entity entity{gz::sim::kNullEntity};
    gz::transport::Node::Publisher publisher;
    bool last_state{false};
    bool logged_resolved{false};
    bool warned_unresolved{false};
  };

  void resolveFootEntities(const gz::sim::EntityComponentManager& ecm);
  bool computeContact(const gz::sim::EntityComponentManager& ecm, const FootState& foot) const;
  void publishFootState(FootState& foot, bool contact);

  std::array<FootState, 4> feet_{{
      FootState{"LF_FOOT", "/LF_FOOT/contact"},
      FootState{"LH_FOOT", "/LH_FOOT/contact"},
      FootState{"RF_FOOT", "/RF_FOOT/contact"},
      FootState{"RH_FOOT", "/RH_FOOT/contact"},
  }};
  gz::transport::Node node_;
  double foot_radius_{0.02};
  double ground_z_{0.0};
  double contact_margin_{0.002};
  bool logged_ready_{false};
  std::uint64_t update_count_{0};
};

}  // namespace sim
}  // namespace legged
