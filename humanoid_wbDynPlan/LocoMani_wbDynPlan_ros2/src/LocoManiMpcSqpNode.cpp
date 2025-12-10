/*
    Node: loco-manipulation of humanoid robot
*/


// ROS2
#include <rclcpp/rclcpp.hpp>
// OCS2 library
#include <ocs2_ros2_interfaces/mpc/MPC_ROS_Interface.h>
#include <ocs2_ros2_interfaces/synchronized_module/RosReferenceManager.h>
#include <ocs2_sqp/SqpMpc.h>
// LocoMani_wbDynPlan
#include <LocoMani_wbDynPlan/LocoManiMpcInterface.h>
#include <LocoMani_wbDynPlan/command/LocoManiMpcTargetTrajectoriesCalculator.h>
// Common MPC ROS2
#include "wbDynPlan_ros2/ros_comm/Ros2ProceduralMpcMotionManager.h"

using namespace ocs2;
using namespace ocs2::humanoid;

int main(int argc, char **argv){
  std::vector<std::string> programArgs;
  programArgs = rclcpp::remove_ros_arguments(argc, argv);
  if (programArgs.size() < 5) {
    throw std::runtime_error("No robot name, config folder, target command file, or description name specified. Aborting.");
  }

  const std::string robotName(argv[1]);
  const std::string taskFile(argv[2]);
  const std::string referenceFile(argv[3]);
  const std::string urdfFile(argv[4]);
  const std::string gaitFile(argv[5]);

  rclcpp::init(argc, argv);

  // 打印提示
    printf("====== DEBUG MODE: Waiting 20 seconds for Debugger Attach ======\n");
    printf("====== PID: %d ======\n", getpid()); // 打印 PID 方便确认
    
    // 睡眠 20 秒，给您充足的时间去 VS Code 点击 "Attach"
    std::this_thread::sleep_for(std::chrono::seconds(30));
    
    printf("====== Resuming Execution ======\n");

//Robot interface
LocoManiMpcInterface interface(taskFile, urdfFile, referenceFile, true)

//SQP MPC Solver
SqpMpc mpc(interface.mpcSettings(), interface.sqpSettings, interface.getOptimalControlProblem(),
                interface.getInitialization())



}

