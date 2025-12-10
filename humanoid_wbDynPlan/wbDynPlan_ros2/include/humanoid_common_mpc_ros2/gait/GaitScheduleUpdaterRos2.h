/* Computational Legged Robots: Planning and Control  */

#pragma once

#include <mutex>

#include <rclcpp/rclcpp.hpp>

#include <ocs2_ros2_msgs/msg/mode_schedule.hpp>

#include "humanoid_common_mpc/gait/GaitScheduleUpdater.h"

namespace ocs2::humanoid {

class GaitScheduleUpdaterRos2 : public GaitScheduleUpdater {
 public:
  GaitScheduleUpdaterRos2(rclcpp::Node::SharedPtr& nodeHandle, std::shared_ptr<GaitSchedule> gaitSchedulePtr, const std::string& robotName);

  void preSolverRun(scalar_t initTime,
                    scalar_t finalTime,
                    const vector_t& currentState,
                    const ReferenceManagerInterface& referenceManager) override;

  virtual ModeSequenceTemplate getReceivedGait() override;

 private:
  void mpcModeSequenceCallback(const ocs2_ros2_msgs::msg::ModeSchedule::SharedPtr msg);

  rclcpp::Subscription<ocs2_ros2_msgs::msg::ModeSchedule>::SharedPtr mpcModeSequenceSubscriber_;

  std::mutex receivedGaitMutex_;
  std::atomic_bool gaitUpdatedAtomic_;
};

}  // namespace ocs2::humanoid
