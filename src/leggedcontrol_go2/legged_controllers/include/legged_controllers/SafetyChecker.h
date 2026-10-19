//
// Created by qiayuan on 2022/7/26.
//

#pragma once

#include <iostream>

#include <ocs2_centroidal_model/AccessHelperFunctions.h>

namespace legged {
using namespace ocs2;
using namespace centroidal_model;
class SafetyChecker {
 public:
  explicit SafetyChecker(const CentroidalModelInfo& info) : info_(info) {}

  bool check(const SystemObservation& observation, const vector_t& /*optimized_state*/, const vector_t& /*optimized_input*/) {
    return checkOrientation(observation);
  }

 protected:
  bool checkOrientation(const SystemObservation& observation) {
    vector_t pose = getBasePose(observation.state, info_);
    if (pose(5) > M_PI_2 || pose(5) < -M_PI_2) {
      std::cerr << "[SafetyChecker] Orientation safety check failed: "
                << "z=" << pose(2)
                << " yaw=" << pose(3)
                << " pitch=" << pose(4)
                << " roll=" << pose(5) << std::endl;
      return false;
    }
    return true;
  }

  const CentroidalModelInfo& info_;
};

}  // namespace legged
