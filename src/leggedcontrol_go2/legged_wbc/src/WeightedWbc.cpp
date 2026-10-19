//
// Created by qiayuan on 22-12-23.
//

#include "legged_wbc/WeightedWbc.h"

#include <iostream>

#include <qpOASES.hpp>

namespace legged {

vector_t WeightedWbc::update(const vector_t& stateDesired, const vector_t& inputDesired, const vector_t& rbdStateMeasured, size_t mode,
                             scalar_t period) {
  WbcBase::update(stateDesired, inputDesired, rbdStateMeasured, mode, period);

  // Constraints
  Task constraints = formulateConstraints();
  const size_t numEqualityConstraints = constraints.b_.size();
  const size_t numInequalityConstraints = constraints.f_.size();
  const size_t numConstraints = numEqualityConstraints + numInequalityConstraints;
  const size_t numDecisionVars = getNumDecisionVars();

  // constraints.a_/b_ (floating-base EOM + zero-acceleration for feet the mode schedule
  // assumes are in rigid contact) are hard equalities; constraints.d_/f_ (torque limits,
  // friction cone) are hard inequalities. When the measured state is far from what the
  // contact schedule assumes -- e.g. a leg still resting near a joint limit after sagging
  // while the controller was inactive -- there can be no torque within limits that satisfies
  // the assumed-rigid-contact dynamics exactly, and qpOASES::init() reports the QP as
  // infeasible. The old fallback (sending an all-zero torque command) briefly removes all
  // ground-reaction tracking from a loaded standing robot, which is itself destabilizing and
  // was observed to cascade into a fall. Instead, add slack to the equality rows only --
  // heavily penalized and tightly bounded, so it stays ~0 whenever an exact solution exists,
  // but lets the solver return the least-violating torques instead of nothing when it
  // doesn't. Torque limits and the friction cone are left hard since those are genuine
  // actuator/physics bounds that must never be exceeded.
  const size_t numSlackVars = numEqualityConstraints;
  const size_t numVars = numDecisionVars + numSlackVars;

  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> A(numConstraints, numVars);
  A.setZero();
  A.block(0, 0, numEqualityConstraints, numDecisionVars) = constraints.a_;
  A.block(numEqualityConstraints, 0, numInequalityConstraints, numDecisionVars) = constraints.d_;
  A.block(0, numDecisionVars, numEqualityConstraints, numSlackVars) = -matrix_t::Identity(numEqualityConstraints, numSlackVars);

  vector_t lbA(numConstraints), ubA(numConstraints);  // clang-format off
  lbA << constraints.b_,
         -qpOASES::INFTY * vector_t::Ones(numInequalityConstraints);
  ubA << constraints.b_,
         constraints.f_;  // clang-format on

  // Cost
  Task weighedTask = formulateWeightedTasks(stateDesired, inputDesired, period);
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> H(numVars, numVars);
  H.setZero();
  H.block(0, 0, numDecisionVars, numDecisionVars) = weighedTask.a_.transpose() * weighedTask.a_;
  H.diagonal().head(numDecisionVars).array() += 1e-8;
  H.diagonal().tail(numSlackVars).array() += dynamicsSlackWeight_;
  vector_t g = vector_t::Zero(numVars);
  g.head(numDecisionVars) = -weighedTask.a_.transpose() * weighedTask.b_;

  // Decision variables stay unbounded; slack is capped so it can only rescue an otherwise
  // infeasible cycle, not silently substitute for real tracking.
  vector_t lb = vector_t::Constant(numVars, -qpOASES::INFTY);
  vector_t ub = vector_t::Constant(numVars, qpOASES::INFTY);
  lb.tail(numSlackVars).setConstant(-dynamicsSlackBound_);
  ub.tail(numSlackVars).setConstant(dynamicsSlackBound_);

  // Solve
  auto qpProblem = qpOASES::QProblem(numVars, numConstraints);
  qpOASES::Options options;
  options.setToMPC();
  options.printLevel = qpOASES::PL_LOW;
  options.enableEqualities = qpOASES::BT_TRUE;
  qpProblem.setOptions(options);
  int nWsr = 200;

  vector_t qpSol(numVars);
  qpSol.setZero();

  const auto initStatus = qpProblem.init(H.data(), g.data(), A.data(), lb.data(), ub.data(), lbA.data(), ubA.data(), nWsr);
  if (initStatus != qpOASES::SUCCESSFUL_RETURN) {
    std::cerr << "[WeightedWbc] qpOASES init failed with status " << qpOASES::getSimpleStatus(initStatus)
              << ". Sending zero WBC solution for this cycle." << std::endl;
    return vector_t::Zero(numDecisionVars);
  }

  const auto solutionStatus = qpProblem.getPrimalSolution(qpSol.data());
  if (solutionStatus != qpOASES::SUCCESSFUL_RETURN || !qpSol.allFinite()) {
    std::cerr << "[WeightedWbc] qpOASES solution invalid with status " << qpOASES::getSimpleStatus(solutionStatus)
              << ". Sending zero WBC solution for this cycle." << std::endl;
    return vector_t::Zero(numDecisionVars);
  }
  return qpSol.head(numDecisionVars);
}

Task WeightedWbc::formulateConstraints() {
  return formulateFloatingBaseEomTask() + formulateTorqueLimitsTask() + formulateFrictionConeTask() + formulateNoContactMotionTask();
}

Task WeightedWbc::formulateWeightedTasks(const vector_t& stateDesired, const vector_t& inputDesired, scalar_t period) {
  return formulateSwingLegTask() * weightSwingLeg_ + formulateBaseAccelTask(stateDesired, inputDesired, period) * weightBaseAccel_ +
         formulateContactForceTask(inputDesired) * weightContactForce_;
}

void WeightedWbc::loadTasksSetting(const std::string& taskFile, bool verbose) {
  WbcBase::loadTasksSetting(taskFile, verbose);

  boost::property_tree::ptree pt;
  boost::property_tree::read_info(taskFile, pt);
  std::string prefix = "weight.";
  if (verbose) {
    std::cerr << "\n #### WBC weight:";
    std::cerr << "\n #### =============================================================================\n";
  }
  loadData::loadPtreeValue(pt, weightSwingLeg_, prefix + "swingLeg", verbose);
  loadData::loadPtreeValue(pt, weightBaseAccel_, prefix + "baseAccel", verbose);
  loadData::loadPtreeValue(pt, weightContactForce_, prefix + "contactForce", verbose);

  // Optional; keeps the defaults set in WeightedWbc.h if the task file has no such section.
  loadData::loadPtreeValue(pt, dynamicsSlackWeight_, "dynamicsSlackTask.weight", verbose);
  loadData::loadPtreeValue(pt, dynamicsSlackBound_, "dynamicsSlackTask.bound", verbose);
}

}  // namespace legged
