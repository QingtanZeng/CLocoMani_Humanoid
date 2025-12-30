/******************************************************************************
Copyright (c) 2025, Manuel Yves Galliker. All rights reserved.
Copyright (c) 2024, 1X Technologies. All rights reserved.
Copyright (c) 2025, Qingtan Zeng.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/
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

