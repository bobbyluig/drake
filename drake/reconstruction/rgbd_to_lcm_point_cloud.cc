#include "rgbd_to_lcm_point_cloud.h"

using robotlocomotion::image_t;
using robotlocomotion::image_array_t;

namespace drake {
namespace systems {
namespace sensors {
namespace {

template<PixelType kPixelType>
void Decompress(Image<kPixelType>& image, const image_t& msg) {
  DRAKE_DEMAND(msg.compression_method == image_t::COMPRESSION_METHOD_ZLIB);

  const size_t source_size = (const size_t) msg.size;
  size_t dest_size =
      (size_t) (image.width() * image.height() * image.kPixelSize);

  const int decompress_status = uncompress(
      reinterpret_cast<Bytef*>(image.at(0, 0)), &dest_size,
      reinterpret_cast<const uint8_t*>(msg.data.data()), source_size);

  DRAKE_DEMAND(decompress_status == Z_OK);
}
}

RgbdToPointCloud::RgbdToPointCloud(const CameraInfo& camera_info) :
    camera_info_(camera_info) {
  image_array_input_index_ =
      DeclareAbstractInputPort(Value<image_array_t>()).get_index();

  point_cloud_output_index_ =
      DeclareAbstractOutputPort(
          &RgbdToPointCloud::CalcPointCloudMessage).get_index();
}

const InputPortDescriptor<double>&
RgbdToPointCloud::image_array_input_port() const {
  return this->get_input_port(image_array_input_index_);
}

const OutputPort<double>& RgbdToPointCloud::point_cloud_output_port() const {
  return this->get_output_port(point_cloud_output_index_);
}

void RgbdToPointCloud::CalcPointCloudMessage(
    const systems::Context<double>& context,
    bot_core::pointcloud_t* output) const {
  const AbstractValue* image_array_value =
      this->EvalAbstractInput(context, image_array_input_index_);
  const image_array_t& image_array =
      image_array_value->GetValue<image_array_t>();

  const image_t& depth_image_t = image_array.images[1];
  const int width = depth_image_t.width;
  const int height = depth_image_t.height;
  ImageDepth32F depth_image(width, height);
  Decompress(depth_image, depth_image_t);

  bot_core::pointcloud_t& message = *output;
  Eigen::Matrix3Xf point_cloud;
  RgbdCamera::ConvertDepthImageToPointCloud(
      depth_image, camera_info_, &point_cloud);

  message.frame_id = "world";
  message.points.clear();
  message.n_points = (int32_t) point_cloud.cols();
  for (int i = 0; i < point_cloud.cols(); ++i) {
    const auto point = point_cloud.col(i);
    message.points.push_back(std::vector<float>{point(0), point(1), point(2)});
  }
  message.n_channels = 0;
}

}  // namespace sensors
}  // namespace systems
}  // namespace drake