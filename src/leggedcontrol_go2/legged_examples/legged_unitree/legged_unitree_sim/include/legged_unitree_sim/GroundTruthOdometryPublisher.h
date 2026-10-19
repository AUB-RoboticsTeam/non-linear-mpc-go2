//
// Created by codex.
// Gazebo Sim ground-truth odometry publisher using the model world pose.
//

#pragma once

#include <chrono>
#include <memory>
#include <string>

#include <gz/math/Pose3.hh>
#include <gz/msgs/odometry.pb.h>
#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>

namespace legged {
namespace sim {

class GroundTruthOdometryPublisher : public gz::sim::System,
                                     public gz::sim::ISystemConfigure,
                                     public gz::sim::ISystemPostUpdate {
 public:
  void Configure(const gz::sim::Entity& entity, const std::shared_ptr<const sdf::Element>& sdf,
                 gz::sim::EntityComponentManager& ecm, gz::sim::EventManager& eventMgr) override;
  void PostUpdate(const gz::sim::UpdateInfo& info, const gz::sim::EntityComponentManager& ecm) override;

 private:
  gz::sim::Entity model_entity_{gz::sim::kNullEntity};
  gz::transport::Node node_;
  gz::transport::Node::Publisher publisher_;
  std::string topic_{"/ground_truth/state"};
  std::string frame_id_{"odom"};
  std::string child_frame_id_{"base"};
  gz::math::Pose3d last_pose_;
  std::chrono::steady_clock::duration last_time_{0};
  bool have_last_pose_{false};
};

}  // namespace sim
}  // namespace legged
