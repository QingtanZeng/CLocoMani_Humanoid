/* Computational Legged Robots: Planning and Control  */

#include <humanoid_common_mpc_ros2/benchmarks/SqpBenchmarksPublisher.h>

namespace ocs2::humanoid {

SqpBenchmarksPublisher::SqpBenchmarksPublisher(rclcpp::Node::SharedPtr node, const ocs2::SqpSolver* sqpSolver) : sqpSolver_(sqpSolver) {
  rclcpp::QoS qos(1);
  qos.best_effort();
  benchmarksPublisher_ = node->create_publisher<ocs2_ros2_msgs::msg::Benchmarks>("/humanoid/mpc_benchmarks", qos);
}

void SqpBenchmarksPublisher::postSolverRun(const ocs2::PrimalSolution& primalSolution) {
  if (primalSolution.timeTrajectory_.size() == 0) return;

  const SqpSolver::Benchmarks benchmarks = sqpSolver_->getBenchmarks();

  ocs2_ros2_msgs::msg::Benchmarks bmMsg;
  bmMsg.time = primalSolution.timeTrajectory_.front();
  auto addBenchmark = [&](const std::string& description, const double& value) {
    ocs2_ros2_msgs::msg::IndividualBenchmarks ibmMsg;
    ibmMsg.description = description;
    ibmMsg.values.emplace_back(value);
    bmMsg.benchmarks.emplace_back(ibmMsg);
  };
  addBenchmark("linearQuadraticApproximationTime", benchmarks.linearQuadraticApproximationTime);
  addBenchmark("solveQpTime", benchmarks.solveQpTime);
  addBenchmark("linesearchTime", benchmarks.linesearchTime);
  addBenchmark("computeControllerTime", benchmarks.computeControllerTime);

  benchmarksPublisher_->publish(bmMsg);
}

}  // namespace ocs2::humanoid