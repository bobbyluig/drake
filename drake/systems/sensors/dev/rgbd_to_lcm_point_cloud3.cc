#include "rgbd_to_lcm_point_cloud3.h"

namespace drake {
namespace systems {
namespace sensors {

using robotlocomotion::image_t;
using robotlocomotion::image_array_t;
using rendering::PoseVector;

RgbdToPointCloud3::RgbdToPointCloud3(const RgbdCamera3& camera) :
    camera_(camera) {
  depth_image_input_index_ =
      DeclareAbstractInputPort(Value<ImageDepth32F>()).get_index();

  pose_vector_input_index_ =
      DeclareVectorInputPort(PoseVector<double>()).get_index();

  point_cloud_output_index_ =
      DeclareAbstractOutputPort(
          &RgbdToPointCloud3::CalcPointCloudMessage).get_index();
}

const InputPortDescriptor<double>&
RgbdToPointCloud3::depth_image_input_port() const {
  return this->get_input_port(depth_image_input_index_);
}

const InputPortDescriptor<double>&
RgbdToPointCloud3::pose_vector_input_port() const {
  return this->get_input_port(pose_vector_input_index_);
}

const OutputPort<double>& RgbdToPointCloud3::point_cloud_output_port() const {
  return this->get_output_port(point_cloud_output_index_);
}

void RgbdToPointCloud3::CalcPointCloudMessage(
    const systems::Context<double>& context,
    bot_core::pointcloud_t* output) const {
  const AbstractValue* depth_image_value =
      this->EvalAbstractInput(context, depth_image_input_index_);
  const ImageDepth32F& depth_image =
      depth_image_value->GetValue<ImageDepth32F>();
  const PoseVector<double>* pose_vector =
      this->EvalVectorInput<PoseVector>(context, pose_vector_input_index_);

  const CameraInfo& camera_info = camera_.color_camera_info();
  const Eigen::Isometry3d X_WB = pose_vector->get_isometry();
  const Eigen::Isometry3d X_BC = camera_.color_camera_optical_pose();
  const Eigen::Isometry3f X_WC = (X_WB * X_BC).cast<float>();

  bot_core::pointcloud_t& message = *output;
  Eigen::Matrix3Xf point_cloud;
  RgbdCamera3::DepthImageToPointCloud(depth_image, camera_info, &point_cloud);

  message.frame_id = std::string(RigidBodyTreeConstants::kWorldName);
  message.points.clear();
  for (int i = 0; i < point_cloud.cols(); ++i) {
    const auto& point = point_cloud.col(i);
    if (!std::isnan(point(0)) && !std::isinf(point(0))) {
      Eigen::Vector3f point_W = X_WC * point;
      message.points.push_back(std::vector<float>{
          point_W(0), point_W(1), point_W(2)
      });
    }
  }
  message.n_points = (int32_t) message.points.size();
  message.n_channels = 0;
}

}  // namespace sensors
}  // namespace systems
}  // namespace drake