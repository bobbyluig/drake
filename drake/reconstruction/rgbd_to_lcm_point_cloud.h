#pragma once

#include <Eigen/Dense>

#include "bot_core/pointcloud_t.hpp"
#include "robotlocomotion/image_array_t.hpp"

#include "drake/systems/framework/leaf_system.h"
#include "rgbd_camera_2.h"

namespace drake {
namespace systems {
namespace sensors {

class RgbdToPointCloud : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RgbdToPointCloud);

  RgbdToPointCloud(const RgbdCamera2& camera);

  const InputPortDescriptor<double>& depth_image_input_port() const;

  const InputPortDescriptor<double>& pose_vector_input_port() const;

  const OutputPort<double>& point_cloud_output_port() const;

 private:
  void CalcPointCloudMessage(const systems::Context<double>& context,
                             bot_core::pointcloud_t* output) const;

  const RgbdCamera2& camera_;

  int depth_image_input_index_{};
  int pose_vector_input_index_{};
  int point_cloud_output_index_{};
};

}  // namespace sensors
}  // namespace systems
}  // namespace drake
