/* Computational Legged Robots: Planning and Control  */

#pragma once

#include <ocs2_mpc/MRT_BASE.h>
#include <ocs2_ros2_msgs/msg/mpc_flattened_controller.hpp>
#include <rclcpp/rclcpp.hpp>

namespace ocs2::humanoid {

// This class implements a Ros subscriber to the policy using the MRT base interface .

class MRTPolicySubscriber : public MRT_BASE {
 public:
  explicit MRTPolicySubscriber(std::string topicPrefix = "anonymousRobot");

  /**
   * Destructor
   */
  ~MRTPolicySubscriber() override;

  void resetMpcNode(const TargetTrajectories& initTargetTrajectories) override {};

  void setCurrentObservation(const SystemObservation& currentObservation) override {};

  /**
   * Shut down the ROS nodes.
   */
  void shutdownNodes();

  /**
   * Launches the ROS publishers and subscribers to communicate with the MPC node.
   * @param [in] nodeHandle
   * @param [in] qos quality of service setting for the ROS publishers and subscribers.
   */
  void launchNodes(rclcpp::Node::SharedPtr node, const rclcpp::QoS& qos);

 private:
  /**
   * Callback method to receive the MPC policy as well as the mode sequence.
   * It only updates the policy variables with suffix (*Buffer_) variables.
   *
   * @param [in] msg: A constant pointer to the message
   */
  void mpcPolicyCallback(const ocs2_ros2_msgs::msg::MpcFlattenedController::SharedPtr msg);

 public:
  /**
   * Helper function to read a MPC policy message.
   *
   * @param [in] msg: A constant pointer to the message
   * @param [out] commandData: The MPC command data
   * @param [out] primalSolution: The MPC policy data
   * @param [out] performanceIndices: The MPC performance indices data
   */
  static void readPolicyMsg(const ocs2_ros2_msgs::msg::MpcFlattenedController& msg,
                            CommandData& commandData,
                            PrimalSolution& primalSolution,
                            PerformanceIndex& performanceIndices);

 private:
  rclcpp::Subscription<ocs2_ros2_msgs::msg::MpcFlattenedController>::SharedPtr mpcPolicySubscriber_;
  rclcpp::Node::SharedPtr node_;
  std::string topicPrefix_;
};

}  // namespace ocs2::humanoid
