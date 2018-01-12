#include <memory>
#include <fstream>

#include <gflags/gflags.h>

#include "drake/common/find_resource.h"
#include "drake/examples/acrobot/acrobot_plant.h"
#include "drake/examples/acrobot/gen/acrobot_state_vector.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/signal_logger.h"

namespace drake {
namespace examples {
namespace acrobot {
namespace {

// Simple example which simulates the (passive) Acrobot.  Run drake-visualizer
// to see the animated result.

DEFINE_double(realtime_factor, 1.0,
              "Playback speed.  See documentation for "
              "Simulator::set_target_realtime_rate() for details.");

int do_main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  lcm::DrakeLcm lcm;
  auto tree = std::make_unique<RigidBodyTree<double>>();
  parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
      FindResourceOrThrow("drake/examples/acrobot/Acrobot.urdf"),
      multibody::joints::kFixed, tree.get());

  systems::DiagramBuilder<double> builder;
  auto acrobot = builder.AddSystem<AcrobotPlant>(
      1.0, 1.0,     /* masses */
      1.0, 2.0,     /* lengths */
      0.5, 1.0,     /* com's */
      0.083, 0.33,  /* masses */
      0.0, 0.0,     /* damping */
      9.81);
  acrobot->set_name("acrobot");
  auto publisher = builder.AddSystem<systems::DrakeVisualizer>(*tree, &lcm);
  builder.Connect(acrobot->get_output_port(0), publisher->get_input_port(0));

  auto energy_logger = builder.AddSystem<systems::SignalLogger<double>>(2);
  energy_logger->set_name("Energy Logger");
  builder.Connect(acrobot->get_energy_port(),
                  energy_logger->get_input_port(0));
  auto diagram = builder.Build();

  systems::Simulator<double> simulator(*diagram);
  systems::Context<double>& acrobot_context =
      diagram->GetMutableSubsystemContext(*acrobot,
                                          &simulator.get_mutable_context());

  double tau = 0;
  acrobot_context.FixInputPort(0, Eigen::Matrix<double, 1, 1>::Constant(tau));

  // Set an initial condition that is sufficiently far from the downright fixed
  // point.
  AcrobotStateVector<double>* x0 = dynamic_cast<AcrobotStateVector<double>*>(
      &acrobot_context.get_mutable_continuous_state_vector());
  DRAKE_DEMAND(x0 != nullptr);
  x0->set_theta1(1.0);
  x0->set_theta2(1.0);
  x0->set_theta1dot(0.0);
  x0->set_theta2dot(0.0);

  simulator.set_target_realtime_rate(FLAGS_realtime_factor);
  simulator.Initialize();
  simulator.StepTo(10);

  // Write to file logged data.
  std::ofstream file("energy.dat");
  MatrixX<double> time_data(energy_logger->data().cols(),
                            energy_logger->data().rows() + 1);
  const int nsteps = energy_logger->sample_times().size();
  time_data.block(0, 0, nsteps, 1) = energy_logger->sample_times();
  time_data.block(0, 1, nsteps, 2) = energy_logger->data().transpose();
  file << time_data;
  file.close();

  return 0;
}

}  // namespace
}  // namespace acrobot
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  return drake::examples::acrobot::do_main(argc, argv);
}
