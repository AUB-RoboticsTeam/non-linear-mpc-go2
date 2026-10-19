#pragma once

#include <ocs2_core/cost/StateCost.h>
#include <ocs2_core/penalties/penalties/RelaxedBarrierPenalty.h>
#include <ocs2_robotic_tools/end_effector/EndEffectorKinematics.h>

#include "legged_interface/SwitchedModelReferenceManager.h"

namespace ocs2 {
namespace legged_robot {

/**
 * Relaxed-barrier implementation of Eq. (21) in Grandia et al.:
 *                    A_t p_foot(x) + b_t >= 0.
 *
 * The terrain planner selects the discrete plane and supplies a convex inner
 * approximation.  The NMPC remains free to choose the exact contact location
 * inside that region.
 */
class FootPlacementRegionCost final : public StateCost {
 public:
  FootPlacementRegionCost(const EndEffectorKinematics<scalar_t>& kinematics,
                          const SwitchedModelReferenceManager& referenceManager, size_t leg,
                          RelaxedBarrierPenalty::Config barrierConfig);
  FootPlacementRegionCost(const FootPlacementRegionCost& other);
  FootPlacementRegionCost* clone() const override { return new FootPlacementRegionCost(*this); }

  bool isActive(scalar_t time) const override;
  scalar_t getValue(scalar_t time, const vector_t& state, const TargetTrajectories&,
                    const PreComputation&) const override;
  ScalarFunctionQuadraticApproximation getQuadraticApproximation(
      scalar_t time, const vector_t& state, const TargetTrajectories&, const PreComputation&) const override;

 private:
  std::unique_ptr<EndEffectorKinematics<scalar_t>> kinematics_;
  const SwitchedModelReferenceManager& referenceManager_;
  size_t leg_;
  std::unique_ptr<RelaxedBarrierPenalty> penalty_;
};

}  // namespace legged_robot
}  // namespace ocs2
