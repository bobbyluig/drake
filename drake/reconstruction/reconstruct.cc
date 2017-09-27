#include <cmath>
#include <iomanip>
#include <limits>
#include <memory>

#include <gflags/gflags.h>

#include "drake/common/find_resource.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/sensors/image_to_lcm_image_array_t.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/reconstruction/rgbd_to_lcm_point_cloud.h"
#include "drake/reconstruction/rgbd_camera_2.h"

namespace drake {
namespace reconstruction {
namespace {

using systems::RigidBodyPlant;
using systems::DrakeVisualizer;
using systems::DiagramBuilder;
using systems::sensors::RgbdCamera2;
using systems::sensors::ImageToLcmImageArrayT;
using systems::sensors::RgbdToPointCloud;
using systems::lcm::LcmPublisherSystem;

DEFINE_double(realtime_rate, 1, "Playback speed relative to real-time.");

static const char* modelUrdfPath =
    "drake/reconstruction/ten_segment_2D.urdf";

int do_main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  auto tree_ptr = std::make_unique<RigidBodyTree<double>>();
  parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
      FindResourceOrThrow(modelUrdfPath),
      multibody::joints::kFixed, tree_ptr.get()
  );

  DiagramBuilder<double> builder;
  auto plant = builder.AddSystem<RigidBodyPlant<double>>(move(tree_ptr));
  plant->set_name("plant");

  lcm::DrakeLcm lcm;
  const auto& tree = plant->get_rigid_body_tree();
  auto publisher = builder.AddSystem<DrakeVisualizer>(tree, &lcm);
  builder.Connect(plant->get_output_port(0), publisher->get_input_port(0));

  Eigen::Vector3d pos = Eigen::Vector3d(10, 10, -5);
  Eigen::Vector3d rpy = Eigen::Vector3d(0, 0, -(M_PI_2 + M_PI_4));
  double fov_y = M_PI_4;
  double depth_range_near = 0.5;
  double depth_range_far = 20;
  auto camera = builder.AddSystem<RgbdCamera2>(
      "camera", tree, pos, rpy, depth_range_near, depth_range_far, fov_y
  );

  auto rgbd_to_point_cloud = builder.AddSystem<RgbdToPointCloud>(*camera);
  auto point_cloud_lcm_publisher = builder.AddSystem(
      LcmPublisherSystem::Make<bot_core::pointcloud_t>(
          "DRAKE_POINTCLOUD_RGBD", &lcm));

  // Connect plant to camera.
  builder.Connect(plant->get_output_port(0), camera->state_input_port());

  // Connect camera to point cloud.
  builder.Connect(
      camera->depth_image_output_port(),
      rgbd_to_point_cloud->depth_image_input_port()
  );

  builder.Connect(
      camera->camera_base_pose_output_port(),
      rgbd_to_point_cloud->pose_vector_input_port()
  );

  // Connect point cloud to publisher.
  builder.Connect(
      rgbd_to_point_cloud->point_cloud_output_port(),
      point_cloud_lcm_publisher->get_input_port(0)
  );

  // Set publish rates
  publisher->set_publish_period(1.0 / 30.0);
  point_cloud_lcm_publisher->set_publish_period(1.0 / 30.0);

  auto diagram = builder.Build();
  systems::Simulator<double> simulator(*diagram);
  auto context = simulator.get_mutable_context();
  auto& plant_context = diagram->GetMutableSubsystemContext(*plant, context);

  DRAKE_DEMAND(plant->get_num_positions() == plant->get_num_positions());
  for (int i = 0; i < plant->get_num_positions(); i++) {
    plant->set_position(&plant_context, i, 0.2);
    plant->set_velocity(&plant_context, i, 0);
  }

  simulator.set_target_realtime_rate(FLAGS_realtime_rate);
  simulator.set_publish_every_time_step(false);
  simulator.Initialize();
  simulator.StepTo(100);
  return 0;
}

} // namespace
} // namespace reconstruction
} // namespace drake

int main(int argc, char* argv[]) {
  return drake::reconstruction::do_main(argc, argv);
}


