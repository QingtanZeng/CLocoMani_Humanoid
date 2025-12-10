/* Computational Legged Robots: Planning and Control  */

#include "humanoid_common_mpc_ros2/gait/GaitScheduleUpdaterRos2.h"

#include "humanoid_common_mpc_ros2/gait/ModeSequenceTemplateRos.h"

namespace ocs2::humanoid {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
GaitScheduleUpdaterRos2::GaitScheduleUpdaterRos2(rclcpp::Node::SharedPtr& nodeHandle,
                                                 std::shared_ptr<GaitSchedule> gaitSchedulePtr,
                                                 const std::string& robotName)
    : GaitScheduleUpdater(gaitSchedulePtr), gaitUpdatedAtomic_(false) {
  auto qos = rclcpp::QoS(1);
  qos.best_effort();
  mpcModeSequenceSubscriber_ = nodeHandle->create_subscription<ocs2_ros2_msgs::msg::ModeSchedule>(
      robotName + "_mpc_mode_schedule", qos, std::bind(&GaitScheduleUpdaterRos2::mpcModeSequenceCallback, this, std::placeholders::_1));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

void GaitScheduleUpdaterRos2::preSolverRun(scalar_t initTime,
                                           scalar_t finalTime,
                                           const vector_t& currentState,
                                           const ReferenceManagerInterface& referenceManager) {
  if (gaitUpdatedAtomic_.load()) {
    updateGaitSchedule(gaitSchedulePtr_, getReceivedGait(), initTime, finalTime);
    gaitUpdatedAtomic_.store(false);
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ModeSequenceTemplate GaitScheduleUpdaterRos2::getReceivedGait() {
  std::lock_guard<std::mutex> lock(receivedGaitMutex_);
  return receivedGait_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void GaitScheduleUpdaterRos2::mpcModeSequenceCallback(const ocs2_ros2_msgs::msg::ModeSchedule::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(receivedGaitMutex_);
  updateModeSequence(readModeSequenceTemplateMsg(*msg));
  gaitUpdatedAtomic_.store(true);
}

}  // namespace ocs2::humanoid
