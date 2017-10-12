#pragma once

#include <limits>
#include <memory>
#include <string>

#include <Eigen/Dense>

#include "drake/common/drake_copyable.h"
#include "drake/multibody/rigid_body_frame.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/rendering/pose_vector.h"
#include "drake/systems/sensors/camera_info.h"
#include "drake/systems/sensors/image.h"
#include "drake/geometry/geometry_system.h"

namespace drake {
namespace systems {
namespace sensors {

using geometry::GeometrySystem;
using geometry::GeometryFrame;

class RgbdCamera3 : public LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RgbdCamera3)

  static void DepthImageToPointCloud(const ImageDepth32F& depth_image,
                                     const CameraInfo& camera_info,
                                     Eigen::Matrix3Xf* point_cloud);

  class InvalidDepth {
   public:
    DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(InvalidDepth)

    static constexpr float kTooFar{std::numeric_limits<float>::infinity()};
    static constexpr float kTooClose{0.f};
  };

  RgbdCamera3(const std::string& name,
              const GeometrySystem<double>& geometry,
              const Eigen::Vector3d& position,
              const Eigen::Vector3d& orientation,
              double depth_range_near = 0.5,
              double depth_range_far = 5.0,
              double fov_y = M_PI_4,
              bool show_window = true);

  ~RgbdCamera3();

  const CameraInfo& color_camera_info() const;

  const CameraInfo& depth_camera_info() const;

  const Eigen::Isometry3d& color_camera_optical_pose() const;

  const Eigen::Isometry3d& depth_camera_optical_pose() const;

  const GeometrySystem<double>& geometry() const;

  const InputPortDescriptor<double>& query_handle_input_port() const;

  const OutputPort<double>& color_image_output_port() const;

  const OutputPort<double>& depth_image_output_port() const;

  const OutputPort<double>& camera_base_pose_output_port() const;

 private:
  void Init(const std::string& name);

  void OutputColorImage(const Context<double>& context,
                        ImageRgba8U* color_image) const;
  void OutputDepthImage(const Context<double>& context,
                        ImageDepth32F* depth_image) const;
  void OutputPoseVector(const Context<double>& context,
                        rendering::PoseVector<double>* pose_vector) const;

  const InputPortDescriptor<double>* query_handle_port_{};
  const OutputPort<double>* color_image_port_{};
  const OutputPort<double>* depth_image_port_{};
  const OutputPort<double>* camera_base_pose_port_{};

  class Impl;
  std::unique_ptr<Impl> impl_;

  class ShapeToVtk;
};

}  // namespace sensors
}  // namespace systems
}  // namespace drake
