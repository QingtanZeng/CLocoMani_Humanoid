/* Computational Legged Robots: Planning and Control  */

#pragma once

#include <ocs2_ros2_interfaces/mrt/MRT_ROS_Interface.h>
#include <ocs2_ros2_msgs/msg/mpc_flattened_controller.hpp>

#include "humanoid_common_mpc_ros2/ros_comm/MRTPolicySubscriber.h"
#include "humanoid_common_mpc_ros2/visualization/HumanoidVisualizer.h"

namespace ocs2::humanoid {

class HumanoidVisualizerRos2Interface : public HumanoidVisualizer {
 public:
  HumanoidVisualizerRos2Interface(const std::string& taskFile,
                                  PinocchioInterface pinocchioInterface,
                                  const MpcRobotModelBase<scalar_t>& mpcRobotModel,
                                  rclcpp::Node::SharedPtr nodeHandle,
                                  scalar_t maxUpdateFrequency = 100.0);

  HumanoidVisualizerRos2Interface(const HumanoidVisualizerRos2Interface&) = delete;

  ~HumanoidVisualizerRos2Interface() override = default;

  void launchSubscribers() override;

 private:
  void mpcObservationCallback(const ocs2_ros2_msgs::msg::MpcObservation::SharedPtr msg) override;

  MRTPolicySubscriber mRTPolicySubscriper_;
};

}  // namespace ocs2::humanoid
