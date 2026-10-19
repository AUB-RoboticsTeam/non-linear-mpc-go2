//
// Created by codex.
//

#include "legged_unitree_sim/GroundTruthOdometryPublisher.h"

#include <cmath>
#include <iostream>

#include <gz/msgs/Utility.hh>
#include <gz/plugin/RegisterMore.hh>
#include <gz/sim/Util.hh>

namespace legged {
namespace sim {

namespace {
void setHeaderData(gz::msgs::Header* header, const std::string& key, const std::string& value) {
  auto* data = header->add_data();
  data->set_key(key);
  data->add_value(value);
}
}  // namespace

void GroundTruthOdometryPublisher::Configure(const gz::sim::Entity& entity, const std::shared_ptr<const sdf::Element>& sdf,
                                             gz::sim::EntityComponentManager& /*ecm*/,
                                             gz::sim::EventManager& /*eventMgr*/) {
  model_entity_ = entity;

  if (sdf) {
    if (sdf->HasElement("topic")) {
      topic_ = sdf->Get<std::string>("topic");
    }
    if (sdf->HasElement("frame_id")) {
      frame_id_ = sdf->Get<std::string>("frame_id");
    }
    if (sdf->HasElement("child_frame_id")) {
      child_frame_id_ = sdf->Get<std::string>("child_frame_id");
    }
  }

  publisher_ = node_.Advertise<gz::msgs::Odometry>(topic_);
  std::cerr << "[GroundTruthOdometryPublisher] publishing model world odometry on "
            << topic_ << " frame_id=" << frame_id_ << " child_frame_id=" << child_frame_id_ << std::endl;
}

void GroundTruthOdometryPublisher::PostUpdate(const gz::sim::UpdateInfo& info,
                                              const gz::sim::EntityComponentManager& ecm) {
  if (!publisher_ || model_entity_ == gz::sim::kNullEntity || !ecm.HasEntity(model_entity_)) {
    return;
  }

  const gz::math::Pose3d pose = gz::sim::worldPose(model_entity_, ecm);
  gz::msgs::Odometry msg;

  auto* stamp = msg.mutable_header()->mutable_stamp();
  const auto sec = std::chrono::duration_cast<std::chrono::seconds>(info.simTime);
  const auto nsec = std::chrono::duration_cast<std::chrono::nanoseconds>(info.simTime - sec);
  stamp->set_sec(sec.count());
  stamp->set_nsec(nsec.count());
  setHeaderData(msg.mutable_header(), "frame_id", frame_id_);
  setHeaderData(msg.mutable_header(), "child_frame_id", child_frame_id_);

  gz::msgs::Set(msg.mutable_pose(), pose);

  if (have_last_pose_) {
    const double dt = std::chrono::duration<double>(info.simTime - last_time_).count();
    if (dt > 0.0 && std::isfinite(dt)) {
      const gz::math::Vector3d linear = (pose.Pos() - last_pose_.Pos()) / dt;
      const gz::math::Quaterniond delta_rot = last_pose_.Rot().Inverse() * pose.Rot();
      const gz::math::Vector3d angular = delta_rot.Euler() / dt;
      gz::msgs::Set(msg.mutable_twist()->mutable_linear(), linear);
      gz::msgs::Set(msg.mutable_twist()->mutable_angular(), angular);
    }
  }

  publisher_.Publish(msg);
  last_pose_ = pose;
  last_time_ = info.simTime;
  have_last_pose_ = true;
}

}  // namespace sim
}  // namespace legged

GZ_ADD_PLUGIN(legged::sim::GroundTruthOdometryPublisher, gz::sim::System, gz::sim::ISystemConfigure,
              gz::sim::ISystemPostUpdate)
