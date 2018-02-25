#pragma once

#include <memory>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/autodiff.h"
#include "drake/multibody/multibody_tree/force_element.h"

namespace drake {
namespace multibody {

template <typename T>
class PokingElement : public ForceElement<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PokingElement)

  explicit PokingElement(double force);

  T CalcPotentialEnergy(
      const MultibodyTreeContext<T>& context,
      const PositionKinematicsCache<T>& pc) const final {
    return 0.0;
  }

  T CalcConservativePower(
      const MultibodyTreeContext<T>& context,
      const PositionKinematicsCache<T>& pc,
      const VelocityKinematicsCache<T>& vc) const final {
    return 0.0;
  }

  T CalcNonConservativePower(
      const MultibodyTreeContext<T>& context,
      const PositionKinematicsCache<T>& pc,
      const VelocityKinematicsCache<T>& vc) const final {
    return 0.0;
  }

 protected:
  void DoCalcAndAddForceContribution(
      const MultibodyTreeContext<T>& context,
      const PositionKinematicsCache<T>& pc,
      const VelocityKinematicsCache<T>& vc,
      MultibodyForces<T>* forces) const override;

  std::unique_ptr<ForceElement<double>>
  DoCloneToScalar(const MultibodyTree<double>& tree_clone) const override;

  std::unique_ptr<ForceElement<AutoDiffXd>>
  DoCloneToScalar(const MultibodyTree<AutoDiffXd>& tree_clone) const override;

 private:
  double force_{0};
};

}  // namespace multibody
}  // namespace drake