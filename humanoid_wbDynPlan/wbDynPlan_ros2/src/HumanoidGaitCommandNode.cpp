/* Computational Legged Robots: Planning and Control  */

#include <rclcpp/rclcpp.hpp>

#include "wbDynPlan_ros2/gait/GaitKeyboardPublisher.h"

using namespace ocs2;
using namespace ocs2::humanoid;

int main(int argc, char* argv[]) {
  std::vector<std::string> programArgs;
  programArgs = rclcpp::remove_ros_arguments(argc, argv);
  if (programArgs.size() < 3) {
    throw std::runtime_error("No robot name, config folder, target command file, or description name specified. Aborting.");
  }

  const std::string robotName(argv[1]);
  const std::string gaitCommandFile(argv[2]);
  std::cerr << "Loading gait file: " << gaitCommandFile << std::endl;

  rclcpp::init(argc, argv);
  auto nodeHandle = std::make_shared<rclcpp::Node>(robotName + "_mpc_mode_schedule");

  GaitKeyboardPublisher gaitCommand(nodeHandle, gaitCommandFile, robotName, true);

  while (rclcpp::ok()) {
    gaitCommand.getKeyboardCommand();
  }

  // Successful exit
  return 0;
}
