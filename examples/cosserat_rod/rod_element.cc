#include "drake/examples/cosserat_rod/rod_element.h"

#include "drake/common/default_scalars.h"
#include "drake/math/cross_product.h"
#include "drake/multibody/multibody_tree/body.h"
#include "drake/multibody/multibody_tree/multibody_tree.h"

namespace drake {
namespace multibody {

template <typename T>
RodElement<T>::RodElement(
    const Body<T>& body1, double length1,
    const Body<T>& body2, double length2,
    double B1, double B2, double C,
    double tau_bending, double tau_twisting) :
    body_i_index_(body1.get_index()), length1_(length1),
    body_ip_index_(body2.get_index()), length2_(length2),
    B1_(B1), B2_(B2), C_(C),
    tau_bending_(tau_bending), tau_twisting_(tau_twisting) {
  length_ih_ = (length1 + length2) / 2.0;
}

template <typename T>
void RodElement<T>::DoCalcAndAddForceContribution(
    const MultibodyTreeContext<T>& context,
    const PositionKinematicsCache<T>& pc,
    const VelocityKinematicsCache<T>& vc,
    MultibodyForces<T>* forces) const {
  using std::abs;
  using std::sin;
  using std::cos;
  using std::min;
  using std::max;
  using std::acos;
  const double kEpsilon = 10 * std::numeric_limits<double>::epsilon();

  const MultibodyTree<T>& model = this->get_parent_tree();

  const BodyNodeIndex node_i = model.get_body(body_i_index_).get_node_index();
  const BodyNodeIndex node_ip = model.get_body(body_ip_index_).get_node_index();

  const Matrix3<T> R_WQi = pc.get_X_WB(node_i).rotation();
  const Matrix3<T> R_WQip = pc.get_X_WB(node_ip).rotation();

  // Orientation of Qip in Qi.
  // This model assumes Qi = Qicm and Qip = Qipcm.
  const Matrix3<T> R_QiQip = R_WQi.transpose() * R_WQip;

  // Compute log(R_QiQip) to obtain an approximation for the curvature vector
  // between body1 and body2.
  const T cos_theta = (R_QiQip.trace() - 1.0) / 2.0;
  const T cos_theta_limited = max(-1.0, min(cos_theta, 1.0));
  const T theta = acos(cos_theta_limited);
  // Rotation unit vector in Rodriguez formula.
  Vector3<T> uhat;
  if (abs(theta) < kEpsilon) {
    uhat.setZero();
  } else {
    Matrix3<T> ux = R_QiQip - R_QiQip.transpose();
    // Assert ux is skew-symmetric.
    DRAKE_ASSERT((ux + ux.transpose()).norm() < kEpsilon);
    // Extract u from ux:
    uhat =
        1.0 / (2.0 * sin(theta)) * Vector3<T>(-ux(1, 2), ux(0, 2), -ux(0, 1));
  }

  // Mean integrated curvature at ih = i + 1/2
  // ktilde = ell * k. ktilde is demensionless.
  const Vector3<T> ktilde_Qih = theta * uhat;

  // Compute curvature vector approximation at ih = i + 1/2, expressed in Qih.
  const Vector3<T> k_Qih = ktilde_Qih / length_ih_;

  // Compute bending moments at the ih = i + 1/2 frame Qih, expressed in
  // Lagrangian frame Qih.
  const Vector3<T> M_Qih(
      B1_ * k_Qih(0),
      B2_ * k_Qih(1),
      C_ * k_Qih(2));

  // Compute the pose of frame Qih using Rodriguez formula to advance to sh,
  // the coordinate right in between Qi and Qip.
  const T sh = length1_ / length_ih_ / 2.0;
  const Matrix3<T> Ux = math::VectorToSkewSymmetric(sh * uhat);
  const Matrix3<T> R_QiQih =
      Matrix3<T>::Identity() + sin(theta) * Ux + (1.0 - cos(theta)) * Ux * Ux;

  // Orientation of Qih in the world frame:
  // Note: since R_QiQip = Exp(theta * uhat) then we get the same result for
  // the orientation of Qih computing it from either side. That is:
  // R_WQih = R_WQi * R_QiQih = R_WQip * R_QipQih
  const Matrix3<T> R_WQih = R_WQi * R_QiQih;

  // Bending moment expressed in the world frame.
  const Vector3<T> M_W = R_WQih * M_Qih;

  // Internal dissipation forces.
  // We approximate the time derivative of curvature in the Lagrangian frame Qih
  // as: Dt(k_Qih) = Dt(u/ell_ih) = w_QiQip_Qih / ell_ih
  const Vector3<T>& w_WQi = vc.get_V_WB(node_i).rotational();
  const Vector3<T>& w_WQip = vc.get_V_WB(node_ip).rotational();
  const Vector3<T> w_QiQip_Qih = R_WQih.transpose() * (w_WQip - w_WQi);
  const Vector3<T> kdot_Qih = w_QiQip_Qih / length_ih_;

  // Finally the dissipation moment is:
  const Vector3<T> Md_Qih(
      tau_bending_ * B1_ * kdot_Qih(0),
      tau_bending_ * B2_ * kdot_Qih(1),
      tau_twisting_ * C_ * kdot_Qih(2));
  const Vector3<T> Md_W = R_WQih * Md_Qih;

  // Get F_Bo_W_array.
  std::vector<SpatialForce<T>>& F_Bo_W_array = forces->mutable_body_forces();

  // Spatial force on Bi.
  const SpatialForce<T> F_Bi_W(M_W + Md_W, Vector3<T>::Zero());
  F_Bo_W_array.at(node_i) += F_Bi_W;

  // Action/reaction on Bip.
  F_Bo_W_array.at(node_ip) -= F_Bi_W;
}

template <typename T>
T RodElement<T>::CalcPotentialEnergy(
    const MultibodyTreeContext<T>& context,
    const PositionKinematicsCache<T>& pc) const {
  return 0.0;
}

template <typename T>
T RodElement<T>::CalcConservativePower(
    const MultibodyTreeContext<T>& context,
    const PositionKinematicsCache<T>& pc,
    const VelocityKinematicsCache<T>& vc) const {
  return 0.0;
}

template <typename T>
T RodElement<T>::CalcNonConservativePower(
    const MultibodyTreeContext<T>& context,
    const PositionKinematicsCache<T>& pc,
    const VelocityKinematicsCache<T>& vc) const {
  return 0.0;
}

template <typename T>
std::unique_ptr<ForceElement<double>>
RodElement<T>::DoCloneToScalar(
    const MultibodyTree<double>& tree_clone) const {
  const Body<double>& Bi = tree_clone.get_body(body_i_index_);
  const Body<double>& Bip = tree_clone.get_body(body_ip_index_);
  return std::make_unique<RodElement<double>>(
      Bi, length1_, Bip, length2_, B1_, B2_, C_, tau_bending_, tau_twisting_);
}

template <typename T>
std::unique_ptr<ForceElement<AutoDiffXd>>
RodElement<T>::DoCloneToScalar(
    const MultibodyTree<AutoDiffXd>& tree_clone) const {
  const Body<AutoDiffXd>& Bi = tree_clone.get_body(body_i_index_);
  const Body<AutoDiffXd>& Bip = tree_clone.get_body(body_ip_index_);
  return std::make_unique<RodElement<AutoDiffXd>>(
      Bi, length1_, Bip, length2_, B1_, B2_, C_, tau_bending_, tau_twisting_);
}

}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class drake::multibody::RodElement)