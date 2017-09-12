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

namespace drake {
namespace examples {
namespace reconstruction {
namespace {

using systems::RigidBodyPlant;
using systems::DrakeVisualizer;
using systems::DiagramBuilder;

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
  builder.Connect(plant->get_output_port(0), publisher->get_input_port(0));
  auto diagram = builder.Build();

  systems::Simulator<double> simulator(*diagram);
  auto context = simulator.get_mutable_context();
  auto& plant_context = diagram->GetMutableSubsystemContext(*plant, context);

  DRAKE_DEMAND(plant->get_num_positions() == plant->get_num_positions());
  for (int i = 0; i < plant->get_num_positions(); i++) {
    plant->set_position(&plant_context, i, 0.1);
    plant->set_velocity(&plant_context, i, 0);
  }

  simulator.set_target_realtime_rate(FLAGS_realtime_rate);
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


