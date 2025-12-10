/* Computational Legged Robots: Planning and Control  */

#include <string>

#include <rclcpp/rclcpp.hpp>

#include <ocs2_core/misc/LoadData.h>
#include "wbDynPlan_ros2/ros_comm/VelocityCommandKeyboardPublisher.h"

using namespace ocs2;
using namespace ocs2::humanoid;

namespace ocs2 {}  // namespace ocs2

int main(int argc, char* argv[]) {
  std::vector<std::string> programArgs;
  programArgs = rclcpp::remove_ros_arguments(argc, argv);
  if (programArgs.size() < 5) {
    throw std::runtime_error("No robot name, config folder, target command file, or description name specified. Aborting.");
  }

  const std::string robotName(argv[1]);
  const std::string taskFile(argv[2]);
  const std::string referenceFile(argv[3]);
  const std::string urdfFile(argv[4]);
  const std::string gaitFile(argv[5]);

  rclcpp::init(argc, argv);

  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>(robotName + "_keyboard_velocity_publisher");

  // goalPose: [deltaX, deltaY, deltaZ, deltaYaw]
  scalar_array_t relativeBaseLimit{0.5, 0.3, 0.4, 0.5};
  scalar_t defaultBaseHeight = 0.7;

  loadData::loadCppDataType(referenceFile, "maxDisplacementVelocityX", relativeBaseLimit[0]);
  loadData::loadCppDataType(referenceFile, "maxDisplacementVelocityY", relativeBaseLimit[1]);
  loadData::loadCppDataType(referenceFile, "maxDeltaPelvisHeight", relativeBaseLimit[2]);
  loadData::loadCppDataType(referenceFile, "maxRotationVelocity", relativeBaseLimit[3]);
  loadData::loadCppDataType(referenceFile, "defaultBaseHeight", defaultBaseHeight);

  VelocityCommandKeyboardPublisher targetVelCommand(node, "humanoid", relativeBaseLimit, defaultBaseHeight);

  const std::string commandMsg = "Enter v_x [m/s], v_y [m/s], delta_height [m], ang_vel_z [rad/s] of the PELVIS, separated by spaces";
  targetVelCommand.publishKeyboardCommand(commandMsg);

  // Successful exit
  return 0;
}
