/* Computational Legged Robots: Planning and Control  */

#pragma once

#include <string>
#include <vector>

#include <ocs2_ros2_msgs/msg/mode_schedule.hpp>
#include <rclcpp/rclcpp.hpp>

#include <wbDynPlan/gait/ModeSequenceTemplate.h>

namespace ocs2::humanoid {

/** This class implements ModeSequence communication using ROS. */
class GaitKeyboardPublisher {
 public:
  GaitKeyboardPublisher(rclcpp::Node::SharedPtr& nodeHandle,
                        const std::string& gaitFile,
                        const std::string& robotName,
                        bool verbose = false);

  /** Prints the command line interface and responds to user input. Function returns after one user input. */
  void getKeyboardCommand();

 private:
  /** Prints the list of available gaits. */
  void printGaitList(const std::vector<std::string>& gaitList) const;

  std::vector<std::string> gaitList_;
  std::map<std::string, ModeSequenceTemplate> gaitMap_;

  rclcpp::Publisher<ocs2_ros2_msgs::msg::ModeSchedule>::SharedPtr modeSequenceTemplatePublisher_;
};

}  // namespace ocs2::humanoid
