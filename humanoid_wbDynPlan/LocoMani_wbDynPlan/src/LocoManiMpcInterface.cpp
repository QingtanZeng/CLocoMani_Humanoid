#include <iostream>
#include <string>

// Pinocchio forward declarations must be included first
#include <pinocchio/fwd.hpp>  // forward declarations must be included first.

#include "LocoMani_wbDynPlan/LocoManiMpcInterface.h"

#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_centroidal_model/FactoryFunctions.h>
#include <ocs2_centroidal_model/ModelHelperFunctions.h>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_core/misc/Numerics.h>
#include <ocs2_core/soft_constraint/StateInputSoftConstraint.h>
#include <ocs2_oc/synchronized_module/SolverSynchronizedModule.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematicsCppAd.h>

#include <wbDynPlan/HumanoidCostConstraintFactory.h>
#include <wbDynPlan/HumanoidPreComputation.h>
#include <wbDynPlan/constraint/EndEffectorKinematicsTwistConstraint.h>
#include <wbDynPlan/cost/EndEffectorKinematicsQuadraticCost.h>
#include <wbDynPlan/pinocchio_model/createPinocchioModel.h>

#include "LocoMani_wbDynPlan/constraint/JointMimicKinematicConstraint.h"
#include "LocoMani_wbDynPlan/constraint/NormalVelocityConstraintCppAd.h"
#include "LocoMani_wbDynPlan/constraint/ZeroVelocityConstraintCppAd.h"
#include "LocoMani_wbDynPlan/cost/CentroidalMpcEndEffectorFootCost.h"
#include "LocoMani_wbDynPlan/cost/ICPCost.h"
#include "LocoMani_wbDynPlan/dynamics/CentroidalDynamicsAD.h"

// Boost
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>

namespace ocs2::humanoid {


void LocoManiMpcInterface::LocoManiMpcInterface(const std::string& taskFile, 
                                                const std::string& urdfFile, 
                                                const std::string& referenceFile,
                                                bool setupOCP)
    : taskFile_(taskFile),
      urdfFile_(urdfFile),
      referenceFile_(referenceFile),
      modelSettings_(taskFile, urdfFile, "LocoManiMpc_", "true") {

  // check that task file exists
  boost::filesystem::path taskFilePath(taskFile);
  if (boost::filesystem::exists(taskFilePath)) {
    std::cerr << "[LocoManiMpcInterface] Loading task file: " << taskFilePath << std::endl;
  } else {
    throw std::invalid_argument("[LocoManiMpcInterface] Task file not found: " + taskFilePath.string());
  }
  // check that urdf file exists
  boost::filesystem::path urdfFilePath(urdfFile);
  if (boost::filesystem::exists(urdfFilePath)) {
    std::cerr << "[LocoManiMpcInterface] Loading Pinocchio model from: " << urdfFilePath << std::endl;
  } else {
    throw std::invalid_argument("[LocoManiMpcInterface] URDF file not found: " + urdfFilePath.string());
  }
  // check that targetCommand file exists
  boost::filesystem::path referenceFilePath(referenceFile);
  if (boost::filesystem::exists(referenceFilePath)) {
    std::cerr << "[LocoManiMpcInterface] Loading target command settings from: " << referenceFilePath << std::endl;
  } else {
    throw std::invalid_argument("[LocoManiMpcInterface] targetCommand file not found: " + referenceFilePath.string());
  }

  loadData::loadCppDataType(taskFile, "interface.verbose", verbose_);

  // load setting from loading file
  ddpSettings_ = ddp::loadSettings(taskFile, "ddp", verbose_);
  mpcSettings_ = mpc::loadSettings(taskFile, "mpc", verbose_);
  rolloutSettings_ = rollout::loadSettings(taskFile, "rollout", verbose_);
  sqpSettings_ = sqp::loadSettings(taskFile, "sqp", verbose_);

  //create Model and info
  pinocchioInterfacePtr_.reset(new PinocchioInterface(createPinocchioModel(
                                    taskFile, urdfFile, modelSettings_, false)));
  centroidalModelInfo_ = centroidal_model::createCentroidalModelInfo(
    *pinocchioInterfacePtr_, centroidal_model::loadCentroidalType(taskFile), 
    centroidal_model::loadCentroidalModelState(pinocchioInterfacePtr_.getModel().nq-6, referenceFile),
    modelSettings_.contactNames3Dof,
    modelSettings_.contactNames6Dof);

  std::cout << "centroidalModelInfo_.numSixDofContacts: " << centroidalModelInfo_.numSixDofContacts << std::endl;
  for (int i = 0; i < centroidalModelInfo_.numSixDofContacts; i++) {
    std::cout << "frameIndices: " << centroidalModelInfo_.endEffectorFrameIndices[i] << std::endl;
  }


  // initial state
  initialState_.setZero(centroidalModelInfo_.stateDim);
  loadData::loadEigenMatrix(taskFile, "initialState", initialState_);

  if(setupOCP){
    setupOptimalControlProblem();
  }
}

void LocoManiMpcInterface::setupOptimalControlProblem() {
  // read OCP Problem from task.info
  HumanoidCostConstraintFactory factory = 
    HumanoidCostConstraintFactory(taskFile_, referenceFile_, *referenceManagerPtr_, *pinocchioInterfacePtr_, 
                                  *mpcRobotModelPtr_, *mpcRobotModelADPtr_, modelSettings_, verbose_);

// reset Optimal Control Problem
    problemPtr_.reset(new OptimalControlProblem);

// 1. dynamics
std::unique_ptr<SystemDynamicsBase> dynamicsPtr;
const std::string modelname = "locomani";
dynamicsPtr.reset(new LocoManiDynamicsAD(*pinocchioInterfacePtr_, centroidalModelInfo_, modelname, modelSettings_));

problemPtr_->dynamicsPtr = std::move(dynamicsPtr);

// 2. Cost terms
// 2.1 add Q ,R 
problemPtr_->costPtr->add("stateInputdQuadraticCost", factory.getStateInputQuadraticCost());
// 2.2 add terminal state cost
problemPtr_->finalCostPtr->add("terminalCost", factory.getTerminalCost());

// 2.3 Foot End-effctor Kinematics Tracking Cost


// 2.4 Hand End-effctor Kinematics Tracking Cost


// 3. Constraint terms
// Joint Limits: angle and velocity
problemPtr_->


}









}   // namespace ocs2::humanoid