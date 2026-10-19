//
// Created by qiayuan on 22-12-23.
//

#include "legged_wbc/WbcBase.h"

namespace legged {

class WeightedWbc : public WbcBase {
 public:
  using WbcBase::WbcBase;

  vector_t update(const vector_t& stateDesired, const vector_t& inputDesired, const vector_t& rbdStateMeasured, size_t mode,
                  scalar_t period) override;

  void loadTasksSetting(const std::string& taskFile, bool verbose) override;

 protected:
  virtual Task formulateConstraints();
  virtual Task formulateWeightedTasks(const vector_t& stateDesired, const vector_t& inputDesired, scalar_t period);

 private:
  scalar_t weightSwingLeg_, weightBaseAccel_, weightContactForce_;

  // Penalized slack on the floating-base-dynamics / rigid-contact equality rows only (torque
  // limits and the friction cone stay hard). See WeightedWbc::update() for why these two can
  // otherwise make the QP infeasible.
  scalar_t dynamicsSlackWeight_{1e4}, dynamicsSlackBound_{1e3};
};

}  // namespace legged