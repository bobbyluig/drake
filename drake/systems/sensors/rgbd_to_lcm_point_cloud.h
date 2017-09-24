#pragma once

#include <zlib.h>

#include "bot_core/pointcloud_t.hpp"
#include "robotlocomotion/image_array_t.hpp"

#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/sensors/rgbd_camera.h"

namespace drake {
namespace systems {
namespace sensors {

class RgbdToPointCloud : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RgbdToPointCloud);

  RgbdToPointCloud(CameraInfo& camera_info);

  const InputPortDescriptor<double>& image_array_input_port() const;

  const OutputPort<double>& point_cloud_output_port() const;

 private:
  void CalcPointCloudMessage(const systems::Context<double>& context,
                             bot_core::pointcloud_t* output) const;

  const CameraInfo& camera_info_;

  int image_array_input_index_{};
  int point_cloud_output_index_{};
};

}  // namespace sensors
}  // namespace systems
}  // namespace drake
