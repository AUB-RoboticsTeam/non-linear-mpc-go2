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

#include "legged_interface/SwitchedModelReferenceManager.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <Eigen/QR>
#include <iostream>
#include <sstream>

namespace ocs2 {
namespace legged_robot {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
SwitchedModelReferenceManager::SwitchedModelReferenceManager(std::shared_ptr<GaitSchedule> gaitSchedulePtr,
                                                             std::shared_ptr<SwingTrajectoryPlanner> swingTrajectoryPtr)
    : ReferenceManager(TargetTrajectories(), ModeSchedule()),
      gaitSchedulePtr_(std::move(gaitSchedulePtr)),
      swingTrajectoryPtr_(std::move(swingTrajectoryPtr)) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void SwitchedModelReferenceManager::setModeSchedule(const ModeSchedule& modeSchedule) {
  ReferenceManager::setModeSchedule(modeSchedule);
  gaitSchedulePtr_->setModeSchedule(modeSchedule);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
contact_flag_t SwitchedModelReferenceManager::getContactFlags(scalar_t time) const {
  return modeNumber2StanceLeg(this->getModeSchedule().modeAtTime(time));
}

void SwitchedModelReferenceManager::setFootholdHeightPlan(const FootholdHeightPlan& plan) {
  std::lock_guard<std::mutex> lock(footholdPlanMutex_);
  footholdPlan_ = plan;
}

const SwitchedModelReferenceManager::FootholdHeightPlan::ConvexRegion*
SwitchedModelReferenceManager::getFootholdRegion(size_t leg, scalar_t time) const {
  if (leg >= footholdRegionSequence_.size() || footholdRegionSequence_[leg].empty()) return nullptr;
  size_t phase = 0;
  while (phase < footholdRegionEventTimes_.size() && time >= footholdRegionEventTimes_[phase]) ++phase;
  phase = std::min(phase, footholdRegionSequence_[leg].size() - 1);
  const auto& region = footholdRegionSequence_[leg][phase];
  return region.valid ? &region : nullptr;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void SwitchedModelReferenceManager::modifyReferences(scalar_t initTime, scalar_t finalTime, const vector_t& initState,
                                                     TargetTrajectories& targetTrajectories, ModeSchedule& modeSchedule) {
  const auto timeHorizon = finalTime - initTime;
  modeSchedule = gaitSchedulePtr_->getModeSchedule(initTime - timeHorizon, finalTime + timeHorizon);

  FootholdHeightPlan plan;
  {
    std::lock_guard<std::mutex> lock(footholdPlanMutex_);
    plan = footholdPlan_;
  }

  if (std::none_of(plan.valid.begin(), plan.valid.end(), [](bool valid) { return valid; })) {
    footholdRegionEventTimes_ = modeSchedule.eventTimes;
    for (auto& sequence : footholdRegionSequence_) {
      sequence.assign(modeSchedule.modeSequence.size(), FootholdHeightPlan::ConvexRegion{});
    }
    swingTrajectoryPtr_->update(modeSchedule, 0.0);
    feet_array_t<std::vector<vector3_t>> positions;
    feet_array_t<std::vector<bool>> active;
    for (size_t leg = 0; leg < 4; ++leg) {
      positions[leg].assign(modeSchedule.modeSequence.size(), vector3_t::Zero());
      active[leg].assign(modeSchedule.modeSequence.size(), false);
    }
    swingTrajectoryPtr_->updateHorizontal(modeSchedule, positions, positions, active);
    return;
  }

  // Paper Sec. IV-E torso reference, made phase-aware.  A future touchdown
  // must not change the torso reference while that foot is still supporting
  // the robot on its current surface.  Doing so used to raise and tilt the
  // body as soon as a four-foot stair plan arrived, before the first swing.
  // Determine when each planned swing returns to stance, then switch that
  // leg's support point from lift-off to touchdown only at that event.
  feet_array_t<scalar_t> touchDownTimes;
  touchDownTimes.fill(std::numeric_limits<scalar_t>::infinity());
  for (size_t leg = 0; leg < 4; ++leg) {
    if (!plan.valid[leg]) continue;
    bool swingStarted = false;
    for (size_t phase = 0; phase < modeSchedule.modeSequence.size(); ++phase) {
      const auto contacts = modeNumber2StanceLeg(modeSchedule.modeSequence[phase]);
      const scalar_t phaseStart =
          phase == 0 ? -std::numeric_limits<scalar_t>::infinity() : modeSchedule.eventTimes[phase - 1];
      const scalar_t phaseEnd = phase < modeSchedule.eventTimes.size()
                                    ? modeSchedule.eventTimes[phase]
                                    : std::numeric_limits<scalar_t>::infinity();
      if (!swingStarted && !contacts[leg] && phaseEnd >= initTime) {
        swingStarted = true;
      } else if (swingStarted && contacts[leg]) {
        touchDownTimes[leg] = phaseStart;
        break;
      }
    }
  }

  size_t terrainSamples = 0;
  scalar_t initialSupportHeight = 0.0;
  for (size_t leg = 0; leg < 4; ++leg) {
    if (!plan.valid[leg]) continue;
    ++terrainSamples;
    initialSupportHeight += plan.liftOffPositions[leg].z();
  }
  if (terrainSamples >= 3 && !targetTrajectories.stateTrajectory.empty()) {
    initialSupportHeight /= static_cast<scalar_t>(terrainSamples);
    for (size_t k = 0; k < targetTrajectories.stateTrajectory.size(); ++k) {
      auto& targetState = targetTrajectories.stateTrajectory[k];
      if (targetState.size() < 12) continue;
      const scalar_t targetTime = k < targetTrajectories.timeTrajectory.size()
                                      ? targetTrajectories.timeTrajectory[k]
                                      : initTime;
      scalar_t supportHeight = 0.0;
      std::vector<vector3_t> supportPoints;
      supportPoints.reserve(terrainSamples);
      for (size_t leg = 0; leg < 4; ++leg) {
        if (!plan.valid[leg]) continue;
        const auto& point =
            targetTime >= touchDownTimes[leg] ? plan.touchDownPositions[leg] : plan.liftOffPositions[leg];
        supportHeight += point.z();
        supportPoints.push_back(point);
      }
      supportHeight /= static_cast<scalar_t>(terrainSamples);
      const scalar_t nominalClearance = targetState(8) - initialSupportHeight;
      targetState(8) = supportHeight + nominalClearance;

      // Least-squares z = ax + by + c through the selected support points.
      Eigen::MatrixXd H(supportPoints.size(), 3);
      Eigen::VectorXd z(supportPoints.size());
      for (size_t i = 0; i < supportPoints.size(); ++i) {
        H.row(i) << supportPoints[i].x(), supportPoints[i].y(), 1.0;
        z(i) = supportPoints[i].z();
      }
      const Eigen::Vector3d plane = H.colPivHouseholderQr().solve(z);
      vector3_t normal{-plane.x(), -plane.y(), 1.0};
      normal.normalize();
      const scalar_t yaw = targetState(9);
      const scalar_t nxBody = std::cos(yaw) * normal.x() + std::sin(yaw) * normal.y();
      const scalar_t nyBody = -std::sin(yaw) * normal.x() + std::cos(yaw) * normal.y();
      targetState(10) = std::clamp(std::atan2(nxBody, normal.z()), scalar_t(-0.35), scalar_t(0.35));
      targetState(11) = std::clamp(-std::atan2(nyBody, normal.z()), scalar_t(-0.35), scalar_t(0.35));
    }
  }

  const auto numPhases = modeSchedule.modeSequence.size();
  footholdRegionEventTimes_ = modeSchedule.eventTimes;
  feet_array_t<scalar_array_t> liftOffHeightSequence;
  feet_array_t<scalar_array_t> touchDownHeightSequence;
  feet_array_t<scalar_array_t> maxHeightSequence;
  feet_array_t<std::vector<vector3_t>> liftOffPositionSequence;
  feet_array_t<std::vector<vector3_t>> touchDownPositionSequence;
  feet_array_t<std::vector<bool>> horizontalActiveSequence;
  for (size_t leg = 0; leg < 4; ++leg) {
    footholdRegionSequence_[leg].assign(numPhases, FootholdHeightPlan::ConvexRegion{});
    liftOffHeightSequence[leg].assign(numPhases, 0.0);
    touchDownHeightSequence[leg].assign(numPhases, 0.0);
    maxHeightSequence[leg].assign(numPhases, 0.0);
    liftOffPositionSequence[leg].assign(numPhases, vector3_t::Zero());
    touchDownPositionSequence[leg].assign(numPhases, vector3_t::Zero());
    horizontalActiveSequence[leg].assign(numPhases, false);

    if (!plan.valid[leg]) {
      continue;
    }

    vector3_t supportPosition = plan.liftOffPositions[leg];
    bool plannedSwingStarted = false;
    bool plannedSwingCompleted = false;
    for (size_t phase = 0; phase < numPhases; ++phase) {
      const auto contacts = modeNumber2StanceLeg(modeSchedule.modeSequence[phase]);
      const bool inSwing = !contacts[leg];
      const scalar_t phaseEnd = phase < modeSchedule.eventTimes.size() ? modeSchedule.eventTimes[phase] : finalTime;

      // The foothold message can arrive while the robot is still in stance and
      // the crawl gait can be inserted immediately afterwards.  Therefore its
      // reception timestamp is not a reliable boundary in a subsequently
      // rebuilt mode schedule. Select the first swing that has not ended at
      // the beginning of the current MPC horizon instead.
      if (!plannedSwingStarted && inSwing && phaseEnd >= initTime) {
        plannedSwingStarted = true;
      }
      if (plannedSwingStarted && !plannedSwingCompleted && !inSwing) {
        plannedSwingCompleted = true;
        supportPosition = plan.touchDownPositions[leg];
      }

      if (plannedSwingStarted && !plannedSwingCompleted && inSwing) {
        liftOffHeightSequence[leg][phase] = plan.liftOffPositions[leg].z();
        touchDownHeightSequence[leg][phase] = plan.touchDownPositions[leg].z();
        // SwingTrajectoryPlanner adds its nominal swing height to this sequence.
        maxHeightSequence[leg][phase] = std::max(
            {plan.liftOffPositions[leg].z(), plan.touchDownPositions[leg].z(),
             plan.minimumApexHeights[leg] - swingTrajectoryPtr_->nominalSwingHeight()});
        liftOffPositionSequence[leg][phase] = plan.liftOffPositions[leg];
        touchDownPositionSequence[leg][phase] = plan.touchDownPositions[leg];
        horizontalActiveSequence[leg][phase] = plan.horizontalValid[leg];
      } else {
        liftOffHeightSequence[leg][phase] = supportPosition.z();
        touchDownHeightSequence[leg][phase] = supportPosition.z();
        maxHeightSequence[leg][phase] = supportPosition.z();
        liftOffPositionSequence[leg][phase] = supportPosition;
        touchDownPositionSequence[leg][phase] = supportPosition;
        // Eq. (21) is a contact-phase constraint. Activate the selected
        // region only after this plan's swing has completed; never pull a
        // stance foot toward a future tread before liftoff.
        if (plannedSwingCompleted && contacts[leg] && plan.regions[leg].valid) {
          footholdRegionSequence_[leg][phase] = plan.regions[leg];
        }
      }
    }
  }
  swingTrajectoryPtr_->update(modeSchedule, liftOffHeightSequence, touchDownHeightSequence, maxHeightSequence);
  swingTrajectoryPtr_->updateHorizontal(modeSchedule, liftOffPositionSequence, touchDownPositionSequence, horizontalActiveSequence);
  std::ostringstream signature;
  for (size_t leg = 0; leg < 4; ++leg) {
    signature << leg << ':';
    for (size_t phase = 0; phase < numPhases; ++phase) {
      if (horizontalActiveSequence[leg][phase]) signature << modeSchedule.modeSequence[phase] << ',';
    }
    signature << ';';
  }
  if (signature.str() != lastLoggedFootholdSignature_) {
    lastLoggedFootholdSignature_ = signature.str();
    std::cerr << "[FootholdReference] activation=" << plan.activationTime << " init=" << initTime
              << " phases=" << numPhases << '\n';
    for (size_t leg = 0; leg < 4; ++leg) {
      std::cerr << "  leg=" << leg << " valid=" << plan.valid[leg]
                << " horizontal=" << plan.horizontalValid[leg]
                << " lift=" << plan.liftOffPositions[leg].transpose()
                << " target=" << plan.touchDownPositions[leg].transpose() << " active_modes=";
      for (size_t phase = 0; phase < numPhases; ++phase) {
        if (horizontalActiveSequence[leg][phase]) std::cerr << modeSchedule.modeSequence[phase] << ',';
      }
      std::cerr << '\n';
    }
  }
}

}  // namespace legged_robot
}  // namespace ocs2
