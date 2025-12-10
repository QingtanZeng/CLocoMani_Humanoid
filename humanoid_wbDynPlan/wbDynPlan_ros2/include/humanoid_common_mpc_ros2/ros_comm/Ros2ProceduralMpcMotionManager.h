/* Computational Legged Robots: Planning and Control  */

#pragma once

#include <rclcpp/rclcpp.hpp>

#include <mutex>

#include "humanoid_common_mpc/reference_manager/ProceduralMpcMotionManager.h"
#include "humanoid_mpc_msgs/msg/walking_velocity_command.hpp"

namespace ocs2::humanoid {

/**
 * Manages the ModeSchedule and the TargetTrajectories for switched model.
 */
class Ros2ProceduralMpcMotionManager : public ProceduralMpcMotionManager {
 public:
  Ros2ProceduralMpcMotionManager(const std::string& gaitFile,
                                 const std::string& referenceFile,
                                 std::shared_ptr<SwitchedModelReferenceManager> switchedModelReferenceManagerPtr,
                                 const MpcRobotModelBase<scalar_t>& mpcRobotModel,
                                 VelocityTargetToTargetTrajectories velocityTargetToTargetTrajectories);

  ~Ros2ProceduralMpcMotionManager() override = default;

  /** Disable copy / move */
  Ros2ProceduralMpcMotionManager& operator=(const Ros2ProceduralMpcMotionManager&) = delete;
  Ros2ProceduralMpcMotionManager(const Ros2ProceduralMpcMotionManager&) = delete;
  Ros2ProceduralMpcMotionManager& operator=(Ros2ProceduralMpcMotionManager&&) = delete;
  Ros2ProceduralMpcMotionManager(Ros2ProceduralMpcMotionManager&&) = delete;

  void setAndScaleVelocityCommand(const WalkingVelocityCommand& rawVelocityCommand) override;

  void subscribe(rclcpp::Node::SharedPtr nodeHandle, const rclcpp::QoS& qos);

 private:
  WalkingVelocityCommand getScaledWalkingVelocityCommand() override;

  rclcpp::Subscription<humanoid_mpc_msgs::msg::WalkingVelocityCommand>::SharedPtr velCommandSubscriber_;
  std::mutex walkingVelCommandMutex_;
};

}  // namespace ocs2::humanoid
