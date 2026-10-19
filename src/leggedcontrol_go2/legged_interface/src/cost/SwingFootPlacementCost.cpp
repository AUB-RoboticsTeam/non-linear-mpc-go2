#include "legged_interface/cost/SwingFootPlacementCost.h"

namespace ocs2 {
namespace legged_robot {

SwingFootPlacementCost::SwingFootPlacementCost(const EndEffectorKinematics<scalar_t>& kinematics,
                                               const SwitchedModelReferenceManager& referenceManager,
                                               size_t leg, Eigen::Vector2d weights)
    : kinematics_(kinematics.clone()), referenceManager_(referenceManager), leg_(leg), Q_(weights.asDiagonal()) {}

SwingFootPlacementCost::SwingFootPlacementCost(const SwingFootPlacementCost& other)
    : kinematics_(other.kinematics_->clone()), referenceManager_(other.referenceManager_), leg_(other.leg_), Q_(other.Q_) {}

bool SwingFootPlacementCost::isActive(scalar_t time) const {
  // The horizontal-reference mask is generated from the exact mode schedule
  // handed to the optimizer and is true only during this leg's selected swing.
  // Do not gate it a second time with ReferenceManager::getModeSchedule(): that
  // synchronized schedule can lag a newly inserted gait by one MPC update and
  // used to suppress an otherwise valid foothold cost.
  return referenceManager_.getSwingTrajectoryPlanner()->hasHorizontalReference(leg_, time);
}

scalar_t SwingFootPlacementCost::getValue(scalar_t time, const vector_t& state, const TargetTrajectories&,
                                          const PreComputation&) const {
  const Eigen::Vector2d error = kinematics_->getPosition(state).front().head<2>() -
                          referenceManager_.getSwingTrajectoryPlanner()->getXYpositionConstraint(leg_, time);
  return 0.5 * error.dot(Q_ * error);
}

ScalarFunctionQuadraticApproximation SwingFootPlacementCost::getQuadraticApproximation(
    scalar_t time, const vector_t& state, const TargetTrajectories&, const PreComputation&) const {
  const auto approximation = kinematics_->getPositionLinearApproximation(state).front();
  const Eigen::Vector2d error = approximation.f.head<2>() -
                          referenceManager_.getSwingTrajectoryPlanner()->getXYpositionConstraint(leg_, time);
  const matrix_t jacobian = approximation.dfdx.topRows<2>();
  ScalarFunctionQuadraticApproximation cost;
  cost.f = 0.5 * error.dot(Q_ * error);
  cost.dfdx = jacobian.transpose() * Q_ * error;
  cost.dfdxx = jacobian.transpose() * Q_ * jacobian;
  return cost;
}

}  // namespace legged_robot
}  // namespace ocs2
