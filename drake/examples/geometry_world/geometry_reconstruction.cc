#include "drake/examples/geometry_world/solar_system.h"
#include "drake/geometry/geometry_visualization.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/lcmt_viewer_draw.hpp"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/rendering/pose_bundle_to_draw_message.h"
#include "drake/systems/sensors/dev/rgbd_camera_3.h"
#include "drake/systems/sensors/dev/rgbd_to_lcm_point_cloud3.h"
#include "external/lcmtypes_bot2_core/lcmtypes/bot_core/pointcloud_t.hpp"
#include "drake/systems/sensors/dev/image_noiser.h"

namespace drake {
namespace examples {
namespace solar_system {
namespace {

using geometry::GeometrySystem;
using geometry::SourceId;
using lcm::DrakeLcm;
using systems::rendering::PoseBundleToDrawMessage;
using systems::lcm::LcmPublisherSystem;
using systems::lcm::Serializer;
using systems::sensors::RgbdCamera3;
using systems::sensors::RgbdToPointCloud3;
using systems::sensors::ImageNoiser;

int do_main() {
  systems::DiagramBuilder<double> builder;

  auto geometry_system = builder.AddSystem<GeometrySystem<double>>();
  geometry_system->set_name("geometry_system");

  auto solar_system = builder.AddSystem<SolarSystem>(geometry_system);
  solar_system->set_name("SolarSystem");

  DrakeLcm lcm;
  PoseBundleToDrawMessage* converter =
      builder.template AddSystem<PoseBundleToDrawMessage>();
  LcmPublisherSystem* publisher =
      builder.template AddSystem<LcmPublisherSystem>(
          "DRAKE_VIEWER_DRAW",
          std::make_unique<Serializer<drake::lcmt_viewer_draw>>(), &lcm);
  publisher->set_publish_period(1 / 60.0);

  builder.Connect(
      solar_system->get_geometry_id_output_port(),
      geometry_system->get_source_frame_id_port(solar_system->source_id()));
  builder.Connect(
      solar_system->get_geometry_pose_output_port(),
      geometry_system->get_source_pose_port(solar_system->source_id()));

  builder.Connect(geometry_system->get_pose_bundle_output_port(),
                  converter->get_input_port(0));
  builder.Connect(*converter, *publisher);

  // Cameras
  double fov_y = M_PI_4;
  double depth_range_near = 0.5;
  double depth_range_far = 20;

  Eigen::Vector3d pos1 = Eigen::Vector3d(-10, 0, 0);
  Eigen::Vector3d rpy1 = Eigen::Vector3d(0, 0, 0);
  auto camera1 = builder.AddSystem<RgbdCamera3>(
      "camera1", *geometry_system, pos1, rpy1,
      depth_range_near, depth_range_far, fov_y
  );

  /*
  Eigen::Vector3d pos2 = Eigen::Vector3d(10, 0, 0);
  Eigen::Vector3d rpy2 = Eigen::Vector3d(0, 0, M_PI);
  auto camera2 = builder.AddSystem<RgbdCamera3>(
      "camera2", *geometry_system, pos2, rpy2,
      depth_range_near, depth_range_far, fov_y
  );
   */

  std::vector<const RgbdCamera3*> cameras = {camera1};

  auto rgbd_to_point_cloud = builder.AddSystem<RgbdToPointCloud3>(cameras);
  auto point_cloud_lcm_publisher = builder.AddSystem(
      LcmPublisherSystem::Make<bot_core::pointcloud_t>(
          "DRAKE_POINTCLOUD_RGBD", &lcm
      )
  );

  auto noiser1 = builder.AddSystem<ImageNoiser>(
      camera1->color_camera_info(), 0.0, 0.02
  );

  /// Connect query to camera.
  builder.Connect(
      geometry_system->get_query_output_port(),
      camera1->query_handle_input_port()
  );

  /*
  builder.Connect(
      geometry_system->get_query_output_port(),
      camera2->query_handle_input_port()
  );
   */

  // Connect camera to point cloud.
  builder.Connect(
      camera1->depth_image_output_port(),
      noiser1->depth_image_input_port()
  );
  builder.Connect(
      noiser1->depth_image_output_port(),
      rgbd_to_point_cloud->depth_image_input_port(0)
  );
  builder.Connect(
      camera1->camera_base_pose_output_port(),
      rgbd_to_point_cloud->pose_vector_input_port(0)
  );

  /*
  builder.Connect(
      camera2->depth_image_output_port(),
      rgbd_to_point_cloud->depth_image_input_port(1)
  );
  builder.Connect(
      camera2->camera_base_pose_output_port(),
      rgbd_to_point_cloud->pose_vector_input_port(1)
  );
   */

  // Connect point cloud to publisher.
  builder.Connect(
      rgbd_to_point_cloud->point_cloud_output_port(),
      point_cloud_lcm_publisher->get_input_port(0)
  );

  // Set publish rates
  point_cloud_lcm_publisher->set_publish_period(1.0 / 30.0);

  // Last thing before building the diagram; dispatch the message to load
  // geometry.
  geometry::DispatchLoadMessage(*geometry_system);

  auto diagram = builder.Build();

  systems::Simulator<double> simulator(*diagram);
  simulator.get_mutable_integrator()->set_maximum_step_size(0.002);
  simulator.set_publish_every_time_step(false);
  simulator.set_target_realtime_rate(1);
  simulator.Initialize();
  simulator.StepTo(100);

  return 0;
}

}  // namespace
}  // namespace solar_system
}  // namespace examples
}  // namespace drake

int main() { return drake::examples::solar_system::do_main(); }
