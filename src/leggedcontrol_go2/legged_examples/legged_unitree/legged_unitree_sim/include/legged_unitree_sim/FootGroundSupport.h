//
// Gazebo Sim fallback foot-ground support for Unitree feet.
//

#pragma once

#include <memory>
#include <string>
#include <vector>

#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>

namespace legged {
namespace sim {

class FootGroundSupport : public gz::sim::System,
                          public gz::sim::ISystemConfigure,
                          public gz::sim::ISystemPreUpdate {
 public:
  void Configure(const gz::sim::Entity& entity, const std::shared_ptr<const sdf::Element>& sdf,
                 gz::sim::EntityComponentManager& ecm, gz::sim::EventManager& eventMgr) override;
  void PreUpdate(const gz::sim::UpdateInfo& info, gz::sim::EntityComponentManager& ecm) override;

 private:
  struct Foot {
    std::string name;
    gz::sim::Link link;
  };

  gz::sim::Model model_;
  std::vector<Foot> feet_;
  double groundZ_{0.0};
  double footRadius_{0.02};
  double normalStiffness_{6000.0};
  double normalDamping_{180.0};
  double tangentialDamping_{20.0};
  double frictionCoefficient_{0.8};
  double maxNormalForce_{350.0};
};

}  // namespace sim
}  // namespace legged
