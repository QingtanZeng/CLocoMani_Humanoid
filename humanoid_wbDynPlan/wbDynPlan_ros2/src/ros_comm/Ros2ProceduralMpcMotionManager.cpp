/* Computational Legged Robots: Planning and Control  */

#include <rclcpp/rclcpp.hpp>

#include <mutex>

#include "humanoid_common_mpc_ros2/ros_comm/Ros2ProceduralMpcMotionManager.h"

namespace ocs2::humanoid {

Ros2ProceduralMpcMotionManager::Ros2ProceduralMpcMotionManager(
    const std::string& gaitFile,
    const std::string& referenceFile,
    std::shared_ptr<SwitchedModelReferenceManager> switchedModelReferenceManagerPtr,
    const MpcRobotModelBase<scalar_t>& mpcRobotModel,
    VelocityTargetToTargetTrajectories velocityTargetToTargetTrajectories)
    : ProceduralMpcMotionManager(
          gaitFile, referenceFile, switchedModelReferenceManagerPtr, mpcRobotModel, velocityTargetToTargetTrajectories) {}

void Ros2ProceduralMpcMotionManager::setAndScaleVelocityCommand(const WalkingVelocityCommand& rawVelocityCommand) {
  std::lock_guard<std::mutex> lock(walkingVelCommandMutex_);
  velocityCommand_ = scaleWalkingVelocityCommand(rawVelocityCommand);
}

void Ros2ProceduralMpcMotionManager::subscribe(rclcpp::Node::SharedPtr nodeHandle, const rclcpp::QoS& qos) {
  // ModeSchedule

  // TargetTrajectories
  auto walkingVelocityCallback = [this](const humanoid_mpc_msgs::msg::WalkingVelocityCommand::SharedPtr msg) {
    this->setAndScaleVelocityCommand(getWalkingVelocityCommandFromMsg(*msg));
  };
  velCommandSubscriber_ = nodeHandle->create_subscription<humanoid_mpc_msgs::msg::WalkingVelocityCommand>(
      "humanoid/walking_velocity_command", qos, walkingVelocityCallback);
}

WalkingVelocityCommand Ros2ProceduralMpcMotionManager::getScaledWalkingVelocityCommand() {
  std::lock_guard<std::mutex> lock(walkingVelCommandMutex_);
  return velocityCommand_;
}

}  // namespace ocs2::humanoid
