/******************************************************************************
Copyright (c) 2021, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

 * Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

 * Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#pragma once

#include <array>
#include <Eigen/Core>
#include <mutex>
#include <string>

#include <ocs2_core/thread_support/Synchronized.h>
#include <ocs2_oc/synchronized_module/ReferenceManager.h>

#include <ocs2_legged_robot/gait/GaitSchedule.h>
#include <ocs2_legged_robot/gait/MotionPhaseDefinition.h>

#include "legged_interface/constraint/SwingTrajectoryPlanner.h"

namespace ocs2 {
namespace legged_robot {

/**
 * Manages the ModeSchedule and the TargetTrajectories for switched model.
 */
class SwitchedModelReferenceManager : public ReferenceManager {
 public:
  struct FootholdHeightPlan {
    struct ConvexRegion {
      matrix_t A;
      vector_t b;
      vector3_t planeOrigin{vector3_t::Zero()};
      vector3_t planeNormal{vector3_t::UnitZ()};
      bool valid{false};
    };
    std::array<vector3_t, 4> liftOffPositions{};
    std::array<vector3_t, 4> touchDownPositions{};
    std::array<scalar_t, 4> minimumApexHeights{};
    std::array<bool, 4> valid{};
    std::array<bool, 4> horizontalValid{};
    std::array<ConvexRegion, 4> regions{};
    scalar_t activationTime{0.0};
  };

  SwitchedModelReferenceManager(std::shared_ptr<GaitSchedule> gaitSchedulePtr, std::shared_ptr<SwingTrajectoryPlanner> swingTrajectoryPtr);

  ~SwitchedModelReferenceManager() override = default;

  void setModeSchedule(const ModeSchedule& modeSchedule) override;

  contact_flag_t getContactFlags(scalar_t time) const;

  const std::shared_ptr<GaitSchedule>& getGaitSchedule() { return gaitSchedulePtr_; }

  const std::shared_ptr<SwingTrajectoryPlanner>& getSwingTrajectoryPlanner() const { return swingTrajectoryPtr_; }

  /** Thread-safe input from the terrain planner. Invalid feet keep the flat-ground behavior. */
  void setFootholdHeightPlan(const FootholdHeightPlan& plan);

  /** Convex touchdown region active for a stance following the planned swing. */
  const FootholdHeightPlan::ConvexRegion* getFootholdRegion(size_t leg, scalar_t time) const;

 protected:
  void modifyReferences(scalar_t initTime, scalar_t finalTime, const vector_t& initState, TargetTrajectories& targetTrajectories,
                        ModeSchedule& modeSchedule) override;

  std::shared_ptr<GaitSchedule> gaitSchedulePtr_;
  std::shared_ptr<SwingTrajectoryPlanner> swingTrajectoryPtr_;

 private:
  mutable std::mutex footholdPlanMutex_;
  FootholdHeightPlan footholdPlan_;
  feet_array_t<std::vector<FootholdHeightPlan::ConvexRegion>> footholdRegionSequence_;
  scalar_array_t footholdRegionEventTimes_;
  std::string lastLoggedFootholdSignature_;
};

}  // namespace legged_robot
}  // namespace ocs2
