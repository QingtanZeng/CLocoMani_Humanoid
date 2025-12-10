/* Computational Legged Robots: Planning and Control  */

#pragma once

#include <ocs2_oc/synchronized_module/SolverSynchronizedModule.h>
#include <ocs2_sqp/SqpSolver.h>
#include <ocs2_ros2_msgs/msg/benchmarks.hpp>
#include <rclcpp/rclcpp.hpp>

namespace ocs2::humanoid {

class SqpBenchmarksPublisher : public SolverSynchronizedModule {
 public:
  SqpBenchmarksPublisher(rclcpp::Node::SharedPtr node, const SqpSolver* sqpSolver);
  ~SqpBenchmarksPublisher() = default;

  void preSolverRun(scalar_t initTime,
                    scalar_t finalTime,
                    const vector_t& currentState,
                    const ReferenceManagerInterface& referenceManager) override {}

  void postSolverRun(const PrimalSolution& primalSolution) override;

 private:
  rclcpp::Publisher<ocs2_ros2_msgs::msg::Benchmarks>::SharedPtr benchmarksPublisher_;
  const SqpSolver* sqpSolver_;
};

}  // namespace ocs2::humanoid