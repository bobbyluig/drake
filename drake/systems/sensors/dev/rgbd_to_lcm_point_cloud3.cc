#include "rgbd_to_lcm_point_cloud3.h"

namespace drake {
namespace systems {
namespace sensors {

using robotlocomotion::image_t;
using robotlocomotion::image_array_t;
using rendering::PoseVector;

RgbdToPointCloud3::RgbdToPointCloud3(
    const std::vector<const RgbdCamera3*>& cameras) : cameras_(cameras) {
  for (size_t i = 0; i < cameras_.size(); i++) {
    depth_image_input_indices_.push_back(
        DeclareAbstractInputPort(Value<ImageDepth32F>()).get_index());
    pose_vector_input_indices_.push_back(
        DeclareVectorInputPort(PoseVector<double>()).get_index());
  }

  point_cloud_output_index_ = DeclareAbstractOutputPort(
      &RgbdToPointCloud3::CalcPointCloudMessage).get_index();
}

const InputPortDescriptor<double>&
RgbdToPointCloud3::depth_image_input_port(int index) const {
  DRAKE_DEMAND((size_t) index < cameras_.size());
  return this->get_input_port(depth_image_input_indices_[index]);
}

const InputPortDescriptor<double>&
RgbdToPointCloud3::pose_vector_input_port(int index) const {
  DRAKE_DEMAND((size_t) index < cameras_.size());
  return this->get_input_port(pose_vector_input_indices_[index]);
}

const OutputPort<double>& RgbdToPointCloud3::point_cloud_output_port() const {
  return this->get_output_port(point_cloud_output_index_);
}

void RgbdToPointCloud3::CalcPointCloudMessage(
    const systems::Context<double>& context,
    bot_core::pointcloud_t* output) const {
  bot_core::pointcloud_t& message = *output;
  message.points.clear();

  for (size_t i = 0; i < cameras_.size(); i++) {
    const AbstractValue* depth_image_value = this->EvalAbstractInput(
        context, depth_image_input_indices_[i]);
    const ImageDepth32F& depth_image =
        depth_image_value->GetValue<ImageDepth32F>();
    const PoseVector<double>* pose_vector = this->EvalVectorInput<PoseVector>(
        context, pose_vector_input_indices_[i]);

    const RgbdCamera3* camera = cameras_[i];
    const CameraInfo& camera_info = camera->color_camera_info();
    const Eigen::Isometry3d X_WB = pose_vector->get_isometry();
    const Eigen::Isometry3d X_BC = camera->color_camera_optical_pose();
    const Eigen::Isometry3f X_WC = (X_WB * X_BC).cast<float>();

    Eigen::Matrix3Xf point_cloud;
    RgbdCamera3::DepthImageToPointCloud(depth_image, camera_info, &point_cloud);

    for (int k = 0; k < point_cloud.cols(); k++) {
      const auto& point = point_cloud.col(k);
      if (!std::isnan(point(0)) && !std::isinf(point(0))) {
        Eigen::Vector3f point_W = X_WC * point;
        message.points.push_back(std::vector<float>{
            point_W(0), point_W(1), point_W(2)
        });
      }
    }

  }

  message.n_points = (int32_t) message.points.size();
  message.frame_id = this->get_name();
  message.n_channels = 0;
}

}  // namespace sensors
}  // namespace systems
}  // namespace drake