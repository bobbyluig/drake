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
#include "drake/systems/sensors/rgbd_camera.h"
#include "drake/systems/sensors/image_to_lcm_image_array_t.h"
#include "drake/systems/lcm/lcm_publisher_system.h"

namespace drake {
namespace examples {
namespace reconstruction {
namespace {

using systems::RigidBodyPlant;
using systems::DrakeVisualizer;
using systems::DiagramBuilder;
using systems::sensors::RgbdCamera;
using systems::sensors::ImageToLcmImageArrayT;
using systems::lcm::LcmPublisherSystem;

DEFINE_double(realtime_rate, 1, "Playback speed relative to real-time.");

static const char* modelUrdfPath =
    "drake/examples/reconstruction/ten_segment_2D.urdf";

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
  publisher->set_publish_period(0.01667);
  builder.Connect(plant->get_output_port(0), publisher->get_input_port(0));

  Eigen::Vector3d pos = Eigen::Vector3d(0, 5, 0);
  Eigen::Vector3d rpy = Eigen::Vector3d(0, 0, -M_PI_2);
  double fov_y = M_PI_4;
  double depth_range_near = 0.5;
  double depth_range_far = 20;
  auto camera = builder.AddSystem<RgbdCamera>(
      "camera", tree, pos, rpy, depth_range_near, depth_range_far, fov_y
  );

  auto image_to_lcm_image_array = builder.AddSystem<ImageToLcmImageArrayT>(
          "color", "depth", "label");
  image_to_lcm_image_array->set_name("converter");

  auto image_array_lcm_publisher = builder.AddSystem(
      LcmPublisherSystem::Make<robotlocomotion::image_array_t>("images", &lcm)
  );
  image_array_lcm_publisher->set_name("publisher");
  image_array_lcm_publisher->set_publish_period(0.01667);

  // Connect plant to camera.
  builder.Connect(plant->get_output_port(0), camera->state_input_port());

  // Connect camera to LCM array.
  builder.Connect(
      camera->color_image_output_port(),
      image_to_lcm_image_array->color_image_input_port()
  );

  builder.Connect(
      camera->depth_image_output_port(),
      image_to_lcm_image_array->depth_image_input_port()
  );

  builder.Connect(
      camera->label_image_output_port(),
      image_to_lcm_image_array->label_image_input_port()
  );

  // Connect LCM array to LCM publisher.
  builder.Connect(
      image_to_lcm_image_array->image_array_t_msg_output_port(),
      image_array_lcm_publisher->get_input_port(0)
  );

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
  simulator.set_publish_at_initialization(true);
  simulator.set_publish_every_time_step(false);
  simulator.Initialize();
  simulator.StepTo(100);
  return 0;
}

} // namespace
} // namespace reconstruction
} // namespace systems
} // namespace drake

int main(int argc, char* argv[]) {
  return drake::examples::reconstruction::do_main(argc, argv);
}


