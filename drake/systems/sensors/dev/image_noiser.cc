#include "image_noiser.h"

namespace drake {
namespace systems {
namespace sensors {

ImageNoiser::ImageNoiser(const CameraInfo& camera_info,
                         float mean,
                         float stddev)
    : camera_info_(camera_info), mean_(mean), stddev_(stddev) {
  depth_image_input_index_ = DeclareAbstractInputPort(
      Value<ImageDepth32F>()
  ).get_index();

  const int kImageWidth = camera_info_.width();
  const int kImageHeight = camera_info_.height();
  ImageDepth32F depth_image(kImageWidth, kImageHeight);
  depth_image_output_index_ = DeclareAbstractOutputPort(
      ImageDepth32F(depth_image), &ImageNoiser::ApplyGaussian
  ).get_index();

  int seed = std::chrono::system_clock::now().time_since_epoch().count();
  generator_ = std::default_random_engine(seed);
  distribution_ = std::normal_distribution<float>(mean_, stddev_);
}

void ImageNoiser::ApplyGaussian(const Context<double>& context,
                                ImageDepth32F* depth_image) const {
  const AbstractValue* input_value =
      this->EvalAbstractInput(context, depth_image_input_index_);
  const ImageDepth32F& input_image =
      input_value->GetValue<ImageDepth32F>();

  for (int x = 0; x < camera_info_.width(); x++) {
    for (int y = 0; y < camera_info_.height(); y++) {
      depth_image->at(x, y)[0] =
          input_image.at(x, y)[0] + distribution_(generator_);
    }
  }
}

const InputPortDescriptor<double>& ImageNoiser::depth_image_input_port() const {
  return get_input_port(depth_image_input_index_);
}

const OutputPort<double>& ImageNoiser::depth_image_output_port() const {
  return get_output_port(depth_image_output_index_);
}

}  // namespace sensors
}  // namespace systems
}  // namespace drake