#include "drake/multibody/multibody_tree/articulated_body_cache.h"

#include "drake/common/eigen_autodiff_types.h"

namespace drake {
namespace multibody {

// Explicitly instantiates on the most common scalar types.
template class ArticulatedBodyCache<double>;
template class ArticulatedBodyCache<AutoDiffXd>;

}  // namespace multibody
}  // namespace drake
