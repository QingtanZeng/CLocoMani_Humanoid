/* Computational Legged Robots: Planning and Control  */

#pragma once

#include <vector>

#include <ocs2_ros2_msgs/msg/mode_schedule.hpp>

#include <humanoid_common_mpc/gait/ModeSequenceTemplate.h>

namespace ocs2::humanoid {

/** Convert mode sequence template to ROS message */
inline ocs2_ros2_msgs::msg::ModeSchedule createModeSequenceTemplateMsg(const ModeSequenceTemplate& modeSequenceTemplate) {
  ocs2_ros2_msgs::msg::ModeSchedule modeScheduleMsg;
  modeScheduleMsg.event_times.assign(modeSequenceTemplate.switchingTimes.begin(), modeSequenceTemplate.switchingTimes.end());
  modeScheduleMsg.mode_sequence.assign(modeSequenceTemplate.modeSequence.begin(), modeSequenceTemplate.modeSequence.end());
  return modeScheduleMsg;
}

/** Convert ROS message to mode sequence template */
inline ModeSequenceTemplate readModeSequenceTemplateMsg(const ocs2_ros2_msgs::msg::ModeSchedule& modeScheduleMsg) {
  std::vector<scalar_t> switchingTimes(modeScheduleMsg.event_times.begin(), modeScheduleMsg.event_times.end());
  std::vector<size_t> modeSequence(modeScheduleMsg.mode_sequence.begin(), modeScheduleMsg.mode_sequence.end());
  return {switchingTimes, modeSequence};
}

}  // namespace ocs2::humanoid
