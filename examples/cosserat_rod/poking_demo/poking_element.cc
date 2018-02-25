#include "drake/examples/cosserat_rod/poking_demo/poking_element.h"

#include "drake/common/default_scalars.h"
#include "drake/math/cross_product.h"
#include "drake/multibody/multibody_tree/body.h"
#include "drake/multibody/multibody_tree/multibody_tree.h"

namespace drake {
namespace multibody {

template <typename T>
PokingElement<T>::PokingElement(const double force) :
    force_(force) {
}

template <typename T>
void PokingElement<T>::DoCalcAndAddForceContribution(
    const MultibodyTreeContext<T>& context,
    const PositionKinematicsCache<T>& pc,
    const VelocityKinematicsCache<T>& vc,
    MultibodyForces<T>* forces) const {
  std::vector<SpatialForce<T>>& Fapplied_Bo_W_array =
      forces->mutable_body_forces();

  const int num_elements = forces->num_bodies() - 1;

  const T& time = context.get_time();
  const BodyNodeIndex mid_element(num_elements / 2);

  if (time > 3.0 && time < 3.1) {
    SpatialForce<T> F_poke(
        Vector3<T>::Zero(), -force_ * Vector3<T>::UnitY());
    Fapplied_Bo_W_array[mid_element] += F_poke;
  }

  if ( time > 6.0 && time < 6.1) {
    const BodyNodeIndex last_element(num_elements);
    SpatialForce<T> F_poke(
        Vector3<T>::Zero(), -force_ / 2.0 * Vector3<T>::UnitY());
    Fapplied_Bo_W_array[last_element] -= F_poke;
  }
}

template <typename T>
std::unique_ptr<ForceElement<double>>
PokingElement<T>::DoCloneToScalar(
    const MultibodyTree<double>& tree_clone) const {
  return std::make_unique<PokingElement<double>>(force_);
}

template <typename T>
std::unique_ptr<ForceElement<AutoDiffXd>>
PokingElement<T>::DoCloneToScalar(
    const MultibodyTree<AutoDiffXd>& tree_clone) const {
  return std::make_unique<PokingElement<AutoDiffXd>>(force_);
}

}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class drake::multibody::PokingElement)
