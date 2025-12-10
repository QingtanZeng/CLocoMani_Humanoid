/* Computational Legged Robots: Planning and Control  */

// Pinocchio forward declarations must be included first
#include <pinocchio/fwd.hpp>

#include "humanoid_common_mpc_ros2/visualization/HumanoidVisualizerRos2Interface.h"

namespace ocs2::humanoid {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
HumanoidVisualizerRos2Interface::HumanoidVisualizerRos2Interface(const std::string& taskFile,
                                                                 PinocchioInterface pinocchioInterface,
                                                                 const MpcRobotModelBase<scalar_t>& mpcRobotModel,
                                                                 rclcpp::Node::SharedPtr nodeHandle,
                                                                 scalar_t maxUpdateFrequency)
    : HumanoidVisualizer(taskFile, pinocchioInterface, mpcRobotModel, nodeHandle, maxUpdateFrequency), mRTPolicySubscriper_("humanoid") {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

void HumanoidVisualizerRos2Interface::launchSubscribers() {
  auto qos = rclcpp::QoS(10);
  qos.best_effort();
  mRTPolicySubscriper_.launchNodes(node_handle_, qos);

  rclcpp::spin_some(node_handle_);

  observationSubscriberPtr_ = node_handle_->create_subscription<ocs2_ros2_msgs::msg::MpcObservation>(
      "/humanoid/mpc_observation", qos, std::bind(&HumanoidVisualizerRos2Interface::mpcObservationCallback, this, std::placeholders::_1));

  std::cout << "observationSubscriberPtr_ initialized" << std::endl;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

void HumanoidVisualizerRos2Interface::mpcObservationCallback(const ocs2_ros2_msgs::msg::MpcObservation::SharedPtr msg) {
  auto currentObservation = ros_msg_conversions::readObservationMsg(*msg);

  if (mRTPolicySubscriper_.initialPolicyReceived()) {
    mRTPolicySubscriper_.updatePolicy();

    update(currentObservation, mRTPolicySubscriper_.getPolicy(), mRTPolicySubscriper_.getCommand());

  } else {
    update(currentObservation, PrimalSolution(), CommandData());
  }
}

}  // namespace ocs2::humanoid
