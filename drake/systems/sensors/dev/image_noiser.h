#pragma once

#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/sensors/image.h"
#include "drake/systems/sensors/camera_info.h"

namespace drake {
namespace systems {
namespace sensors {

class ImageNoiser : public LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ImageNoiser)

  ImageNoiser(const CameraInfo& camera_info,
              float mean = 0.0,
              float stddev = 0.1);

  void ApplyGaussian(const Context<double>& context,
                     ImageDepth32F* depth_image) const;

  const InputPortDescriptor<double>& depth_image_input_port() const;

  const OutputPort<double>& depth_image_output_port() const;

 private:
  const CameraInfo& camera_info_;
  const float mean_;
  const float stddev_;

  mutable std::default_random_engine generator_{};
  mutable std::normal_distribution<float> distribution_{};

  int depth_image_input_index_{};
  int depth_image_output_index_{};
};

}  // namespace sensors
}  // namespace systems
}  // namespace drake

