#pragma once

#include <memory>

#include "drake/examples/acrobot2/gen/acrobot_state_vector.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/primitives/affine_system.h"
#include "drake/multibody/rigid_body_tree.h"

namespace drake {
namespace examples {
namespace acrobot {

/// The Acrobot - a canonical underactuated system as described in <a
/// href="http://underactuated.mit.edu/underactuated.html?chapter=3">Chapter 3
/// of Underactuated Robotics</a>.
///
/// @tparam T The vector element type, which must be a valid Eigen scalar.
/// @param m1 Mass of link 1 (kg).
/// @param m2 Mass of link 2 (kg).
/// @param l1 Length of link 1 (m).
/// @param l2 Length of link 2 (m).
/// @param lc1 Vertical distance from shoulder joint to center of mass of
/// link 1 (m).
/// @param lc2 Vertical distance from elbow joint to center of mass of
/// link 2 (m).
/// @param Ic1 Inertia of link 1 about the center of mass of link 1
/// (kg*m^2).
/// @param Ic2 Inertia of link 2 about the center of mass of link 2
/// (kg*m^2).
/// @param b1 Damping coefficient of the shoulder joint (kg*m^2/s).
/// @param b2 Damping coefficient of the elbow joint (kg*m^2/s).
/// @param k The spring constant for ball collision (kg/s^2).
/// @param g Gravitational constant (m/s^2).
///

class AcrobotPlant : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(AcrobotPlant)

  explicit AcrobotPlant(
      const RigidBodyTree<double>& tree,
      double m1 = 1.0,
      double m2 = 1.0,
      double l1 = 1.0,
      double l2 = 2.0,
      double lc1 = 0.5,
      double lc2 = 1.0,
      double Ic1 = .083,
      double Ic2 = .33,
      double b1 = 0.1,
      double b2 = 0.1,
      double k = 25000,
      double g = 9.81
  );

  ///@{
  /// Manipulator equation of Acrobot: H * qdotdot + C = B*u.
  /// H[2x2] is the mass matrix.
  /// C[2x1] includes the Coriolis term, gravity term and the damping term, i.e.
  /// C[2x1] = Coriolis(q,v)*v + g(q) + [b1*theta1;b2*theta2]
  Vector2<double> VectorC(const AcrobotStateVector<double>& x) const;
  Matrix2<double> MatrixH(const AcrobotStateVector<double>& x) const;
  ///@}

  // getter for tree
  const RigidBodyTree<double>& tree() const { return tree_; }

  // getters for robot parameters
  double m1() const { return m1_; }
  double m2() const { return m2_; }
  double l1() const { return l1_; }
  double l2() const { return l2_; }
  double lc1() const { return lc1_; }
  double lc2() const { return lc2_; }
  double Ic1() const { return Ic1_; }
  double Ic2() const { return Ic2_; }
  double b1() const { return b1_; }
  double b2() const { return b2_; }
  double k() const { return k_; }
  double g() const { return g_; }

 private:
  void OutputState(
      const systems::Context<double>& context,
      AcrobotStateVector<double>* output
  ) const;

  void DoCalcTimeDerivatives(
      const systems::Context<double>& context,
      systems::ContinuousState<double>* derivatives
  ) const override;

  const RigidBodyTree<double>& tree_;

  // TODO(russt): Declare these as parameters in the context.
  const double m1_, m2_, l1_, l2_, lc1_, lc2_, Ic1_, Ic2_, b1_, b2_, k_, g_;

  // Quantities that occur often.
  const double I1_ = Ic1_ + m1_ * lc1_ * lc1_;
  const double I2_ = Ic2_ + m2_ * lc2_ * lc2_;
  const double m2l1lc2_ = m2_ * l1_ * lc2_;
};

}  // namespace acrobot
}  // namespace examples
}  // namespace drake
