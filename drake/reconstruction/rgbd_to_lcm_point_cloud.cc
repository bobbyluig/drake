#include "rgbd_to_lcm_point_cloud.h"

using robotlocomotion::image_t;
using robotlocomotion::image_array_t;

namespace drake {
namespace systems {
namespace sensors {

RgbdToPointCloud::RgbdToPointCloud(const CameraInfo& camera_info) :
    camera_info_(camera_info) {
  depth_image_input_index_ =
      DeclareAbstractInputPort(Value<ImageDepth32F>()).get_index();

  point_cloud_output_index_ =
      DeclareAbstractOutputPort(
          &RgbdToPointCloud::CalcPointCloudMessage).get_index();
}

const InputPortDescriptor<double>&
RgbdToPointCloud::depth_image_input_port() const {
  return this->get_input_port(depth_image_input_index_);
}

const OutputPort<double>& RgbdToPointCloud::point_cloud_output_port() const {
  return this->get_output_port(point_cloud_output_index_);
}

void RgbdToPointCloud::CalcPointCloudMessage(
    const systems::Context<double>& context,
    bot_core::pointcloud_t* output) const {
  const AbstractValue* depth_image_value =
      this->EvalAbstractInput(context, depth_image_input_index_);
  const ImageDepth32F& depth_image =
      depth_image_value->GetValue<ImageDepth32F>();

  bot_core::pointcloud_t& message = *output;
  Eigen::Matrix3Xf point_cloud;
  RgbdCamera::ConvertDepthImageToPointCloud(
      depth_image, camera_info_, &point_cloud);

  message.frame_id = "world";
  message.points.clear();
  for (int i = 0; i < point_cloud.cols(); ++i) {
    const auto point = point_cloud.col(i);
    if (!std::isnan(point(0)) && !std::isinf(point(0))) {
      message.points.push_back(
          std::vector<float>{point(0), point(1), point(2)});
    }
  }
  message.n_points = (int32_t) message.points.size();
  message.n_channels = 0;
}

}  // namespace sensors
}  // namespace systems
}  // namespace drake