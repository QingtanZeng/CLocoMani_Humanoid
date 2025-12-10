/* Computational Legged Robots: Planning and Control  */

#include "humanoid_common_mpc_ros2/ros_comm/MRTPolicySubscriber.h"

#include <ocs2_core/control/FeedforwardController.h>
#include <ocs2_core/control/LinearController.h>
#include "ocs2_ros2_interfaces/common/RosMsgConversions.h"

namespace ocs2::humanoid {

MRTPolicySubscriber::MRTPolicySubscriber(std::string topicPrefix) : topicPrefix_(std::move(topicPrefix)) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
MRTPolicySubscriber::~MRTPolicySubscriber() {
  shutdownNodes();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MRTPolicySubscriber::readPolicyMsg(const ocs2_ros2_msgs::msg::MpcFlattenedController& msg,
                                        CommandData& commandData,
                                        PrimalSolution& primalSolution,
                                        PerformanceIndex& performanceIndices) {
  commandData.mpcInitObservation_ = ros_msg_conversions::readObservationMsg(msg.init_observation);
  commandData.mpcTargetTrajectories_ = ros_msg_conversions::readTargetTrajectoriesMsg(msg.plan_target_trajectories);
  performanceIndices = ros_msg_conversions::readPerformanceIndicesMsg(msg.performance_indices);

  const size_t N = msg.time_trajectory.size();
  if (N == 0) {
    throw std::runtime_error("[MRTPolicySubscriber::readPolicyMsg] controller message is empty!");
  }
  if (msg.state_trajectory.size() != N && msg.input_trajectory.size() != N) {
    throw std::runtime_error("[MRTPolicySubscriber::readPolicyMsg] state and input trajectories must have same length!");
  }
  // if (msg.data.size() != N) {
  //   throw std::runtime_error("[MRTPolicySubscriber::readPolicyMsg] Data has the wrong length!");
  // }

  primalSolution.clear();

  primalSolution.modeSchedule_ = ros_msg_conversions::readModeScheduleMsg(msg.mode_schedule);

  size_array_t stateDim(N);
  size_array_t inputDim(N);
  primalSolution.timeTrajectory_.reserve(N);
  primalSolution.stateTrajectory_.reserve(N);
  primalSolution.inputTrajectory_.reserve(N);
  for (size_t i = 0; i < N; i++) {
    stateDim[i] = msg.state_trajectory[i].value.size();
    inputDim[i] = msg.input_trajectory[i].value.size();
    primalSolution.timeTrajectory_.emplace_back(msg.time_trajectory[i]);
    primalSolution.stateTrajectory_.emplace_back(Eigen::Map<const Eigen::VectorXd>(msg.state_trajectory[i].value.data(), stateDim[i]));
    primalSolution.inputTrajectory_.emplace_back(Eigen::Map<const Eigen::VectorXd>(msg.input_trajectory[i].value.data(), inputDim[i]));
  }

  primalSolution.postEventIndices_.reserve(msg.post_event_indices.size());
  for (auto ind : msg.post_event_indices) {
    primalSolution.postEventIndices_.emplace_back(static_cast<size_t>(ind));
  }

  // std::vector<std::vector<double> const*> controllerDataPtrArray(N, nullptr);
  // for (int i = 0; i < N; i++) {
  //   controllerDataPtrArray[i] = &(msg.data[i].data);
  // }

  // // instantiate the correct controller
  // switch (msg.controller_type) {
  //   case ocs2_ros2_msgs::msg::MpcFlattenedController::CONTROLLER_FEEDFORWARD: {
  //     auto controller = FeedforwardController::unFlatten(primalSolution.timeTrajectory_, controllerDataPtrArray);
  //     primalSolution.controllerPtr_.reset(new FeedforwardController(std::move(controller)));
  //     break;
  //   }
  //   case ocs2_ros2_msgs::msg::MpcFlattenedController::CONTROLLER_LINEAR: {
  //     auto controller = LinearController::unFlatten(stateDim, inputDim, primalSolution.timeTrajectory_, controllerDataPtrArray);
  //     primalSolution.controllerPtr_.reset(new LinearController(std::move(controller)));
  //     break;
  //   }
  //   default:
  //     throw std::runtime_error("[MRTPolicySubscriber::readPolicyMsg] Unknown controllerType!");
  // }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MRTPolicySubscriber::mpcPolicyCallback(const ocs2_ros2_msgs::msg::MpcFlattenedController::SharedPtr msg) {
  // read new policy and command from msg
  std::unique_ptr<CommandData> commandPtr(new CommandData);
  std::unique_ptr<PrimalSolution> primalSolutionPtr(new PrimalSolution);
  std::unique_ptr<PerformanceIndex> performanceIndicesPtr(new PerformanceIndex);
  readPolicyMsg(*msg, *commandPtr, *primalSolutionPtr, *performanceIndicesPtr);

  this->moveToBuffer(std::move(commandPtr), std::move(primalSolutionPtr), std::move(performanceIndicesPtr));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MRTPolicySubscriber::shutdownNodes() {
  // clean up callback queue
  mpcPolicySubscriber_.reset();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MRTPolicySubscriber::launchNodes(rclcpp::Node::SharedPtr node, const rclcpp::QoS& qos) {
  this->reset();

  node_ = node;

  // display
  RCLCPP_INFO_STREAM(node->get_logger(), "MRT Policy Subscriber is setting up ...");

  // policy subscriber
  mpcPolicySubscriber_ = node->create_subscription<ocs2_ros2_msgs::msg::MpcFlattenedController>(
      topicPrefix_ + "/mpc_policy", qos, std::bind(&MRTPolicySubscriber::mpcPolicyCallback, this, std::placeholders::_1));
}

}  // namespace ocs2::humanoid
