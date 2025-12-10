/* Computational Legged Robots: Planning and Control  */

#pragma once

#include <Eigen/Dense>
#include <memory>
#include <string>
#include <unordered_map>

#include "pinocchio/multibody/data.hpp"
#include "pinocchio/multibody/model.hpp"
#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/joint_state.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace ocs2::humanoid {

/**
 * @class COMVisualizer
 * @brief A ROS2 node for visualizing the Center of Mass (COM) of a robot
 *
 * This class loads a URDF model, subscribes to joint states, calculates the
 * robot's COM, and publishes visualization markers for RViz.
 */
class COMVisualizer : public rclcpp::Node {
 public:
  /**
   * @brief Constructor for the COMVisualizer
   *
   * Initializes the node, loads the URDF, sets up subscribers and publishers,
   * and starts the COM calculation and visualization timer.
   */
  COMVisualizer();

 private:
  /**
   * @brief Loads the robot model from a URDF file
   * @param urdf_file Path to the URDF file
   * @return true if the model was loaded successfully, false otherwise
   */
  bool loadRobotModel(const std::string& urdf_file);

  /**
   * @brief Prints information about the joints in the loaded model
   */
  void printJointInfo();

  /**
   * @brief Initializes the joint name to index map
   */
  void initializeJointMap();

  /**
   * @brief Publishes the COM and related visualization markers
   */
  void publishCOM();

  /**
   * @brief Calculates the Center of Mass of the robot
   * @return Eigen::Vector3d representing the COM position
   */
  Eigen::Vector3d calculateCOM();

  /**
   * @brief Callback function for joint state messages
   * @param msg Shared pointer to the received JointState message
   */
  void updateJointPositions(const sensor_msgs::msg::JointState::SharedPtr msg);

  // ROS publishers and subscribers
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr com_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;

  // Pinocchio model and data
  pinocchio::Model pinocchio_model_;
  pinocchio::Data pinocchio_data_;

  // Joint positions and mapping
  Eigen::VectorXd joint_positions_;
  std::unordered_map<std::string, int> joint_name_to_index_;
};
}  // namespace ocs2::humanoid