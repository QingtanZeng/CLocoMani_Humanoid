/* Computational Legged Robots: Planning and Control  */

#include "humanoid_common_mpc_ros2/ros_comm/VelocityCommandKeyboardPublisher.h"

#include <ocs2_core/misc/CommandLine.h>
#include <ocs2_core/misc/Display.h>

namespace ocs2::humanoid {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VelocityCommandKeyboardPublisher::VelocityCommandKeyboardPublisher(rclcpp::Node::SharedPtr nodeHandle,
                                                                   const std::string& topicPrefix,
                                                                   const scalar_array_t& targetCommandLimits,
                                                                   scalar_t defaultBaseHeight)
    : targetCommandLimits_(Eigen::Map<const vector_t>(targetCommandLimits.data(), targetCommandLimits.size())),
      defaultBaseHeight_(defaultBaseHeight),
      node_(nodeHandle) {
  assert(targetCommandLimits_.size() == 4);
  // Target publisher
  commandPublisherPtr_ =
      node_->create_publisher<humanoid_mpc_msgs::msg::WalkingVelocityCommand>(topicPrefix + "/walking_velocity_command", 1);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void VelocityCommandKeyboardPublisher::publishKeyboardCommand(const std::string& commadMsg) {
  while (rclcpp::ok()) {
    // get command line
    std::cout << commadMsg << ": ";
    const vector4_t commandLineInput = getCommandLine().cwiseMin(targetCommandLimits_).cwiseMax(-targetCommandLimits_);

    // display
    std::cout << "The following command is published: [" << toDelimitedString(commandLineInput) << "]\n\n";

    humanoid_mpc_msgs::msg::WalkingVelocityCommand msg;
    msg.linear_velocity_x = commandLineInput[0] / targetCommandLimits_[0];
    msg.linear_velocity_y = commandLineInput[1] / targetCommandLimits_[1];
    msg.desired_pelvis_height = defaultBaseHeight_ + commandLineInput[2];
    msg.angular_velocity_z = commandLineInput[3] / targetCommandLimits_[3];

    // publish TargetTrajectories
    commandPublisherPtr_->publish(msg);
  }  // end of while loop
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector4_t VelocityCommandKeyboardPublisher::getCommandLine() {
  // get command line as one long string
  auto shouldTerminate = []() { return !rclcpp::ok(); };
  const std::string line = getCommandLineString(shouldTerminate);

  // a line to words
  const std::vector<std::string> words = stringToWords(line);

  const size_t targetCommandSize = targetCommandLimits_.size();
  vector4_t targetCommand = vector_t::Zero(targetCommandSize);
  for (size_t i = 0; i < std::min(words.size(), targetCommandSize); i++) {
    targetCommand(i) = static_cast<scalar_t>(stof(words[i]));
  }

  return targetCommand;
}

}  // namespace ocs2::humanoid
