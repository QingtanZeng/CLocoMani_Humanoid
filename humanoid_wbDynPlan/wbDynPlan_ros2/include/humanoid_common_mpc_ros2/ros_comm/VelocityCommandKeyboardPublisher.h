/* Computational Legged Robots: Planning and Control  */

#pragma once

#include <humanoid_common_mpc/common/Types.h>
#include <rclcpp/rclcpp.hpp>
#include "humanoid_mpc_msgs/msg/walking_velocity_command.hpp"

namespace ocs2::humanoid {

/**
 * This class lets the user to publish a velocity command form command line.
 */
class VelocityCommandKeyboardPublisher final {
 public:
  /**
   * Constructor
   *
   * @param [in] nodeHandle: ROS node handle.
   * @param [in] topicPrefix: The TargetTrajectories will be published on "topicPrefix_mpc_target" topic. Moreover, the latest
   * observation is be expected on "topicPrefix_mpc_observation" topic.
   * @param [in] targetCommandLimits: The limits of the loaded command from command-line (for safety purposes).
   */
  VelocityCommandKeyboardPublisher(rclcpp::Node::SharedPtr nodeHandle,
                                   const std::string& topicPrefix,
                                   const scalar_array_t& targetCommandLimits,
                                   scalar_t defaultBaseHeight);

  // VelocityCommandKeyboardPublisher(const VelocityCommandKeyboardPublisher&) = delete;

  /**
   * Publishes command line input. If the input command is shorter than the expected command
   * size (targetCommandSize), the method will set the rest of the command to zero.
   *
   * @param [in] commadMsg: Message to be displayed on screen.
   */
  void publishKeyboardCommand(const std::string& commadMsg = "Enter command, separated by space");

 private:
  /** Gets the target from command line. */
  vector4_t getCommandLine();

  const vector4_t targetCommandLimits_;
  const scalar_t defaultBaseHeight_;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<humanoid_mpc_msgs::msg::WalkingVelocityCommand>::SharedPtr commandPublisherPtr_{};
};

}  // namespace ocs2::humanoid
