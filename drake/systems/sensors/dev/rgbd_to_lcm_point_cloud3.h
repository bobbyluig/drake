#pragma once

#include <Eigen/Dense>

#include "bot_core/pointcloud_t.hpp"
#include "robotlocomotion/image_array_t.hpp"

#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/sensors/dev/rgbd_camera_3.h"

namespace drake {
namespace systems {
namespace sensors {

class RgbdToPointCloud3 : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RgbdToPointCloud3);

  RgbdToPointCloud3(
      const std::vector<std::shared_ptr<const RgbdCamera3>>& cameras);

  const InputPortDescriptor<double>& depth_image_input_port(int index) const;

  const InputPortDescriptor<double>& pose_vector_input_port(int index) const;

  const OutputPort<double>& point_cloud_output_port() const;

 private:
  void CalcPointCloudMessage(const systems::Context<double>& context,
                             bot_core::pointcloud_t* output) const;

  const std::vector<std::shared_ptr<const RgbdCamera3>>& cameras_;

  std::vector<int> depth_image_input_indices_{};
  std::vector<int> pose_vector_input_indices_{};
  int point_cloud_output_index_{};
};

}  // namespace sensors
}  // namespace systems
}  // namespace drake
