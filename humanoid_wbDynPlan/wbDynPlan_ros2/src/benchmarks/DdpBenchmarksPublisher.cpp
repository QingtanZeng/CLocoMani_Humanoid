/* Computational Legged Robots: Planning and Control  */

#include <humanoid_common_mpc_ros2/benchmarks/DdpBenchmarksPublisher.h>

namespace ocs2::humanoid {

DdpBenchmarksPublisher::DdpBenchmarksPublisher(rclcpp::Node::SharedPtr node, const ocs2::GaussNewtonDDP* ddpSolver)
    : ddpSolver_(ddpSolver) {
  rclcpp::QoS qos(1);
  qos.best_effort();
  benchmarksPublisher_ = node->create_publisher<ocs2_ros2_msgs::msg::Benchmarks>("/humanoid/mpc_benchmarks", qos);
}

void DdpBenchmarksPublisher::postSolverRun(const ocs2::PrimalSolution& primalSolution) {
  if (primalSolution.timeTrajectory_.size() == 0) return;

  const GaussNewtonDDP::Benchmarks benchmarks = ddpSolver_->getBenchmarks();

  ocs2_ros2_msgs::msg::Benchmarks bmMsg;
  bmMsg.time = primalSolution.timeTrajectory_.front();
  auto addBenchmark = [&](const std::string& description, const double& value) {
    ocs2_ros2_msgs::msg::IndividualBenchmarks ibmMsg;
    ibmMsg.description = description;
    ibmMsg.values.emplace_back(value);
    bmMsg.benchmarks.emplace_back(ibmMsg);
  };
  addBenchmark("initializationTime", benchmarks.initializationTime);
  addBenchmark("linearQuadraticApproximationTime", benchmarks.linearQuadraticApproximationTime);
  addBenchmark("backwardPassTime", benchmarks.backwardPassTime);
  addBenchmark("computeControllerTime", benchmarks.computeControllerTime);
  addBenchmark("searchStrategyTime", benchmarks.searchStrategyTime);
  addBenchmark("totalDualSolutionTime", benchmarks.totalDualSolutionTime);

  benchmarksPublisher_->publish(bmMsg);
}

}  // namespace ocs2::humanoid