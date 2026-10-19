#include "legged_interface/cost/FootPlacementRegionCost.h"

namespace ocs2 {
namespace legged_robot {

FootPlacementRegionCost::FootPlacementRegionCost(
    const EndEffectorKinematics<scalar_t>& kinematics,
    const SwitchedModelReferenceManager& referenceManager, size_t leg,
    RelaxedBarrierPenalty::Config barrierConfig)
    : kinematics_(kinematics.clone()),
      referenceManager_(referenceManager),
      leg_(leg),
      penalty_(std::make_unique<RelaxedBarrierPenalty>(barrierConfig)) {}

FootPlacementRegionCost::FootPlacementRegionCost(const FootPlacementRegionCost& other)
    : kinematics_(other.kinematics_->clone()),
      referenceManager_(other.referenceManager_),
      leg_(other.leg_),
      penalty_(other.penalty_->clone()) {}

bool FootPlacementRegionCost::isActive(scalar_t time) const {
  return referenceManager_.getFootholdRegion(leg_, time) != nullptr;
}

scalar_t FootPlacementRegionCost::getValue(scalar_t time, const vector_t& state,
                                           const TargetTrajectories&, const PreComputation&) const {
  const auto* region = referenceManager_.getFootholdRegion(leg_, time);
  if (region == nullptr) return 0.0;
  const vector3_t position = kinematics_->getPosition(state).front();
  const vector_t h = region->A * position + region->b;
  scalar_t value = 0.0;
  for (Eigen::Index i = 0; i < h.size(); ++i) value += penalty_->getValue(time, h(i));
  return value;
}

ScalarFunctionQuadraticApproximation FootPlacementRegionCost::getQuadraticApproximation(
    scalar_t time, const vector_t& state, const TargetTrajectories&, const PreComputation&) const {
  ScalarFunctionQuadraticApproximation cost;
  cost.f = 0.0;
  cost.dfdx = vector_t::Zero(state.size());
  cost.dfdxx = matrix_t::Zero(state.size(), state.size());
  const auto* region = referenceManager_.getFootholdRegion(leg_, time);
  if (region == nullptr) return cost;

  const auto approximation = kinematics_->getPositionLinearApproximation(state).front();
  const vector_t h = region->A * approximation.f + region->b;
  const matrix_t dhdx = region->A * approximation.dfdx;
  for (Eigen::Index i = 0; i < h.size(); ++i) {
    const scalar_t first = penalty_->getDerivative(time, h(i));
    const scalar_t second = penalty_->getSecondDerivative(time, h(i));
    cost.f += penalty_->getValue(time, h(i));
    cost.dfdx.noalias() += first * dhdx.row(i).transpose();
    cost.dfdxx.noalias() += second * dhdx.row(i).transpose() * dhdx.row(i);
  }
  return cost;
}

}  // namespace legged_robot
}  // namespace ocs2
