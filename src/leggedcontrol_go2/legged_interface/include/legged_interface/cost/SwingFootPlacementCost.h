#pragma once

#include <ocs2_core/cost/StateCost.h>
#include <ocs2_robotic_tools/end_effector/EndEffectorKinematics.h>

#include "legged_interface/SwitchedModelReferenceManager.h"

namespace ocs2 {
namespace legged_robot {

/** Gauss-Newton x/y tracking cost for a terrain-planned swing foot. */
class SwingFootPlacementCost final : public StateCost {
 public:
  SwingFootPlacementCost(const EndEffectorKinematics<scalar_t>& kinematics,
                         const SwitchedModelReferenceManager& referenceManager, size_t leg, Eigen::Vector2d weights);
  SwingFootPlacementCost(const SwingFootPlacementCost& other);
  SwingFootPlacementCost* clone() const override { return new SwingFootPlacementCost(*this); }

  bool isActive(scalar_t time) const override;
  scalar_t getValue(scalar_t time, const vector_t& state, const TargetTrajectories&, const PreComputation&) const override;
  ScalarFunctionQuadraticApproximation getQuadraticApproximation(scalar_t time, const vector_t& state,
      const TargetTrajectories&, const PreComputation&) const override;

 private:
  std::unique_ptr<EndEffectorKinematics<scalar_t>> kinematics_;
  const SwitchedModelReferenceManager& referenceManager_;
  size_t leg_;
  Eigen::Matrix2d Q_;
};

}  // namespace legged_robot
}  // namespace ocs2
