/* Computational Legged Robots: Planning and Control  */

#include "humanoid_common_mpc_ros2/gait/GaitKeyboardPublisher.h"

#include <algorithm>
#include <iostream>

#include <ocs2_core/misc/CommandLine.h>
#include <ocs2_core/misc/LoadData.h>

#include "humanoid_common_mpc_ros2/gait/ModeSequenceTemplateRos.h"

namespace ocs2::humanoid {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
GaitKeyboardPublisher::GaitKeyboardPublisher(rclcpp::Node::SharedPtr& nodeHandle,
                                             const std::string& gaitFile,
                                             const std::string& robotName,
                                             bool verbose) {
  std::cout << (robotName + "_mpc_mode_schedule node is setting up ...") << std::endl;
  loadData::loadStdVector(gaitFile, "list", gaitList_, verbose);

  modeSequenceTemplatePublisher_ = nodeHandle->create_publisher<ocs2_ros2_msgs::msg::ModeSchedule>(robotName + "_mpc_mode_schedule", 1);

  gaitMap_ = getGaitMap(gaitFile);
  std::cout << (robotName + "_mpc_mode_schedule command node is ready.") << std::endl;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void GaitKeyboardPublisher::getKeyboardCommand() {
  const std::string commadMsg = "Enter the desired gait, for the list of available gait enter \"list\"";
  std::cout << commadMsg << ": ";

  auto shouldTerminate = []() { return !rclcpp::ok(); };
  const auto commandLine = stringToWords(getCommandLineString(shouldTerminate));

  if (commandLine.empty()) {
    return;
  }

  if (commandLine.size() > 1) {
    std::cout << "WARNING: The command should be a single word." << std::endl;
    return;
  }

  // lower case transform
  auto gaitCommand = commandLine.front();
  std::transform(gaitCommand.begin(), gaitCommand.end(), gaitCommand.begin(), ::tolower);

  if (gaitCommand == "list") {
    printGaitList(gaitList_);
    return;
  }

  try {
    ModeSequenceTemplate modeSequenceTemplate = gaitMap_.at(gaitCommand);
    modeSequenceTemplatePublisher_->publish(createModeSequenceTemplateMsg(modeSequenceTemplate));
  } catch (const std::out_of_range& e) {
    std::cout << "Gait \"" << gaitCommand << "\" not found.\n";
    printGaitList(gaitList_);
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void GaitKeyboardPublisher::printGaitList(const std::vector<std::string>& gaitList) const {
  std::cout << "List of available gaits:\n";
  size_t itr = 0;
  for (const auto& s : gaitList) {
    std::cout << "[" << itr++ << "]: " << s << "\n";
  }
  std::cout << std::endl;
}

}  // namespace ocs2::humanoid
