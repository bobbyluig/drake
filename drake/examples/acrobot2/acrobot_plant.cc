#include "drake/examples/acrobot2/acrobot_plant.h"

#include <cmath>
#include <vector>

#include "drake/common/drake_throw.h"
#include "drake/common/eigen_autodiff_types.h"
#include "drake/systems/controllers/linear_quadratic_regulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/sensors/rotary_encoders.h"

using std::sin;
using std::cos;

namespace drake {
namespace examples {
namespace acrobot {

namespace {
constexpr int kNumDOF = 2;  // theta1 + theta2.
}

AcrobotPlant::AcrobotPlant(
    const RigidBodyTree<double>& tree,
    double m1, double m2, double l1, double l2,
    double lc1, double lc2, double Ic1, double Ic2,
    double b1, double b2, double k, double g
) : systems::LeafSystem<double>(),
    tree_(tree),
    m1_(m1),
    m2_(m2),
    l1_(l1),
    l2_(l2),
    lc1_(lc1),
    lc2_(lc2),
    Ic1_(Ic1),
    Ic2_(Ic2),
    b1_(b1),
    b2_(b2),
    k_(k),
    g_(g) {
  this->DeclareInputPort(systems::kVectorValued, 2);
  this->DeclareVectorOutputPort(&AcrobotPlant::OutputState);
  static_assert(AcrobotStateVectorIndices::kNumCoordinates == kNumDOF * 2, "");
  this->DeclareContinuousState(
      AcrobotStateVector<double>(),
      kNumDOF /* num_q */,
      kNumDOF /* num_v */,
      0 /* num_z */
  );
}

void AcrobotPlant::OutputState(const systems::Context<double>& context,
                                  AcrobotStateVector<double>* output) const {
  output->set_value(
      dynamic_cast<const AcrobotStateVector<double>&>(
          context.get_continuous_state_vector()
      ).get_value()
  );
}

Matrix2<double> AcrobotPlant::MatrixH(const AcrobotStateVector<double>& x) const {
  const double c2 = cos(x.theta2());

  const double h12 = I2_ + m2l1lc2_ * c2;
  Matrix2<double> H;
  H << I1_ + I2_ + m2_ * l1_ * l1_ + 2 * m2l1lc2_ * c2, h12, h12, I2_;
  return H;
}

Vector2<double> AcrobotPlant::VectorC(const AcrobotStateVector<double>& x) const {
  const double s1 = sin(x.theta1()), s2 = sin(x.theta2());
  const double s12 = sin(x.theta1() + x.theta2());

  Vector2<double> C;
  C << -2 * m2l1lc2_ * s2 * x.theta2dot() * x.theta1dot() +
      -m2l1lc2_ * s2 * x.theta2dot() * x.theta2dot(),
      m2l1lc2_ * s2 * x.theta1dot() * x.theta1dot();

  // Add in G terms.
  C(0) += g_ * m1_ * lc1_ * s1 + g_ * m2_ * (l1_ * s1 + lc2_ * s12);
  C(1) += g_ * m2_ * lc2_ * s12;

  // Damping terms.
  C(0) += b1_ * x.theta1dot();
  C(1) += b2_ * x.theta2dot();

  // Collision terms.
  auto& tree = const_cast<RigidBodyTree<double>&>(tree_);

  VectorX<double> q(2);
  q << x.theta1(), x.theta2();
  VectorX<double> v(2);
  v << x.theta1dot(), x.theta2dot();

  auto kinsol = tree.doKinematics(q, v);
  auto pairs = tree.ComputeMaximumDepthCollisionPoints(kinsol, true);

  for (const auto& pair : pairs) {
    // No need to compute unless contact exists.
    // Only handle contact of Acrobot and Ball.
    // Its safe to assume that elementA and elementB are in this order because
    // Acrobot must always be added first.
    if (pair.distance >= 0.0 ||
        pair.elementA->get_body()->get_model_name() != "Acrobot" ||
        pair.elementB->get_body()->get_model_name() != "Ball") {
      continue;
    }

    // Transform contact to world coordinates.
    const int body_a_index = pair.elementA->get_body()->get_body_index();
    const int body_b_index = pair.elementB->get_body()->get_body_index();
    const Vector3<double> pointA =
        kinsol.get_element(body_a_index).transform_to_world * pair.ptA;
    const Vector3<double> pointB =
        kinsol.get_element(body_b_index).transform_to_world * pair.ptB;

    // Get the radius and origin of Ball.
    auto wT = pair.elementB->getWorldTransform();
    Vector3<double> originB;
    originB << wT(0, 3), wT(1, 3), wT(2, 3);
    const Vector3<double> n = pointB - originB;
    const double radius = n.norm();

    // Obtain force vector.
    // Convert to 2D by only considering forces in the x-z plane.
    // Force is considered as an ideal spring compressed to pointA.
    const double d = (pointA - originB).norm();
    const Vector3<double> f = 0.5 * k_ * pow(radius - d, 2) * n.normalized();

    if (pair.elementA->get_body()->get_name() == "upper_link") {
      // Colliding with upper link.
      // Only need to compute torque effects on C(0).
      const double tau1 = (-pointA(2) * f(0)) - (-pointA(0) * f(2));
      C(0) += tau1;
    } else if (pair.elementA->get_body()->get_name() == "lower_link") {
      // Colliding with lower link.
      // Need to compute torque effects on C(0) and C(1).
      const double elbowX = l1_ * s1;
      const double elbowZ = -(l1_ * cos(x.theta1()));
      const double tau1 = (-pointA(2) * f(0)) - (-pointA(0) * f(2));
      const double tau2 =
          ((elbowZ - pointA(2)) * f(0)) - ((elbowX - pointA(0)) * f(2));
      C(0) += tau1;
      C(1) += tau2;
    }

    // std::cout << pair.elementA->get_body()->get_name() << std::endl;
  }

  return C;
}

// Compute the actual physics.
void AcrobotPlant::DoCalcTimeDerivatives(
    const systems::Context<double>& context,
    systems::ContinuousState<double>* derivatives
) const {
  const AcrobotStateVector<double>& x = dynamic_cast<const AcrobotStateVector<double>&>(
      context.get_continuous_state_vector()
  );
  auto tau = this->EvalVectorInput(context, 0)->get_value();

  Matrix2<double> H = MatrixH(x);
  Vector2<double> C = VectorC(x);
  Matrix2<double> B;  // input matrix
  B << 1, 0, 0, 1;

  Vector4<double> xdot;
  xdot << x.theta1dot(), x.theta2dot(), H.inverse() * (B * tau - C);
  derivatives->SetFromVector(xdot);
}

}  // namespace acrobot
}  // namespace examples
}  // namespace drake
