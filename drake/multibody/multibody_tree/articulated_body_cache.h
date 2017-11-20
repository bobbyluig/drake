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

  const Matrix6<T>& get_P_B(BodyNodeIndex body_node_index) const {
    DRAKE_ASSERT(0 <= body_node_index && body_node_index < num_nodes_);
    return P_B_pool_[body_node_index];
  }

  Matrix6<T>& get_mutable_P_B(BodyNodeIndex body_node_index) {
    DRAKE_ASSERT(0 <= body_node_index && body_node_index < num_nodes_);
    return P_B_pool_[body_node_index];
  }

  const Vector6<T>& get_z_B(BodyNodeIndex body_node_index) const {
    DRAKE_ASSERT(0 <= body_node_index && body_node_index < num_nodes_);
    return z_B_pool_[body_node_index];
  }

  Vector6<T>& get_mutable_z_B(BodyNodeIndex body_node_index) {
    DRAKE_ASSERT(0 <= body_node_index && body_node_index < num_nodes_);
    return z_B_pool_[body_node_index];
  }

  const Matrix6X<T>& get_g_M(BodyNodeIndex body_node_index) const {
    DRAKE_ASSERT(0 <= body_node_index && body_node_index < num_nodes_);
    return g_M_pool_[body_node_index];
  }

  Matrix6X<T>& get_mutable_g_M(BodyNodeIndex body_node_index) {
    DRAKE_ASSERT(0 <= body_node_index && body_node_index < num_nodes_);
    return g_M_pool_[body_node_index];
  }

  const Vector6<T>& get_a_B(BodyNodeIndex body_node_index) const {
    DRAKE_ASSERT(0 <= body_node_index && body_node_index < num_nodes_);
    return a_B_pool_[body_node_index];
  }

  Vector6<T>& get_mutable_a_B(BodyNodeIndex body_node_index) {
    DRAKE_ASSERT(0 <= body_node_index && body_node_index < num_nodes_);
    return a_B_pool_[body_node_index];
  }

  const VectorX<T>& get_v_M(BodyNodeIndex body_node_index) const {
    DRAKE_ASSERT(0 <= body_node_index && body_node_index < num_nodes_);
    return v_M_pool_[body_node_index];
  }

  VectorX<T>& get_mutable_v_M(BodyNodeIndex body_node_index) {
    DRAKE_ASSERT(0 <= body_node_index && body_node_index < num_nodes_);
    return v_M_pool_[body_node_index];
  }

  const Vector6<T>& get_alpha_B(BodyNodeIndex body_node_index) const {
    DRAKE_ASSERT(0 <= body_node_index && body_node_index < num_nodes_);
    return alpha_B_pool_[body_node_index];
  }

  Vector6<T>& get_mutable_alpha_B(BodyNodeIndex body_node_index) {
    DRAKE_ASSERT(0 <= body_node_index && body_node_index < num_nodes_);
    return alpha_B_pool_[body_node_index];
  }

 private:
  typedef std::vector<Matrix6<T>> Matrix6_PoolType;
  typedef std::vector<Matrix6X<T>> Matrix6X_PoolType;
  typedef std::vector<Vector6<T>> Vector6_PoolType;
  typedef std::vector<VectorX<T>> VectorX_PoolType;

  void Allocate() {
    P_B_pool_.resize(static_cast<unsigned long>(num_nodes_));
    z_B_pool_.resize(static_cast<unsigned long>(num_nodes_));
    g_M_pool_.resize(static_cast<unsigned long>(num_nodes_));
    a_B_pool_.resize(static_cast<unsigned long>(num_nodes_));
    v_M_pool_.resize(static_cast<unsigned long>(num_nodes_));
    alpha_B_pool_.resize(static_cast<unsigned long>(num_nodes_));

    // a_B for world is zero.
    alpha_B_pool_[world_index()] = Vector6<T>::Zero();
  }

  int num_nodes_{0};

  // Pool names are directly from [Jain 2010, Algorithm 7.2].
  Matrix6_PoolType P_B_pool_{};
  Vector6_PoolType z_B_pool_{};
  Matrix6X_PoolType g_M_pool_{};
  Vector6_PoolType a_B_pool_{};
  VectorX_PoolType v_M_pool_{};
  Vector6_PoolType alpha_B_pool_{};
};

}  // namespace multibody
}  // namespace drake
