#pragma once

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/multibody_tree/multibody_tree_indexes.h"
#include "drake/multibody/multibody_tree/multibody_tree_topology.h"


namespace drake {
namespace multibody {

/// This class is one of the cache entries in MultibodyTreeContext. It holds the
/// kinematics results of computations relating to the Articulated Body Model.


template <typename T>
class ArticulatedBodyCache {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(ArticulatedBodyCache)

  explicit ArticulatedBodyCache(const MultibodyTreeTopology& topology) :
      num_nodes_(topology.get_num_bodies()) {
    Allocate();
  }

  const Matrix6<T>& get_P_plus(BodyNodeIndex body_node_index) const {
    DRAKE_ASSERT(0 <= body_node_index && body_node_index < num_nodes_);
    return P_plus_pool_[body_node_index];
  }

  Matrix6<T>& get_mutable_P_plus(BodyNodeIndex body_node_index) {
    DRAKE_ASSERT(0 <= body_node_index && body_node_index < num_nodes_);
    return P_plus_pool_[body_node_index];
  }

  const Vector6<T>& get_z_plus(BodyNodeIndex body_node_index) const {
    DRAKE_ASSERT(0 <= body_node_index && body_node_index < num_nodes_);
    return z_plus_pool_[body_node_index];
  }

  Vector6<T>& get_mutable_z_plus(BodyNodeIndex body_node_index) {
    DRAKE_ASSERT(0 <= body_node_index && body_node_index < num_nodes_);
    return z_plus_pool_[body_node_index];
  }

  const Matrix6X<T>& get_g(BodyNodeIndex body_node_index) const {
    DRAKE_ASSERT(0 <= body_node_index && body_node_index < num_nodes_);
    return g_pool_[body_node_index];
  }

  Matrix6X<T>& get_mutable_g(BodyNodeIndex body_node_index) {
    DRAKE_ASSERT(0 <= body_node_index && body_node_index < num_nodes_);
    return g_pool_[body_node_index];
  }

 private:
  typedef std::vector<Matrix6<T>> Matrix6_PoolType;
  typedef std::vector<Matrix6X<T>> Matrix6X_PoolType;
  typedef std::vector<Vector6<T>> Vector6_PoolType;

  void Allocate() {
    P_plus_pool_.resize(static_cast<unsigned long>(num_nodes_));
    P_plus_pool_[world_index()] = Matrix6<T>::Zero();

    z_plus_pool_.resize(static_cast<unsigned long>(num_nodes_));
    z_plus_pool_[world_index()] = Vector6<T>::Zero();

    g_pool_.resize(static_cast<unsigned long>(num_nodes_));
    // TODO(bobbyluig): Kalman gain for world is not defined, set to NaN.
  }

  int num_nodes_{0};

  // Pool names are directly from [Jain 2010, Algorithm 7.2].
  Matrix6_PoolType P_plus_pool_{};
  Vector6_PoolType z_plus_pool_{};
  Matrix6X_PoolType g_pool_{};
};

}  // namespace multibody
}  // namespace drake
