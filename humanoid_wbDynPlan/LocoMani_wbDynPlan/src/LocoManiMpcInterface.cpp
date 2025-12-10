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
    const ocs2::vector_t &nominalJointAngles, 
    const std::vector<...> &threeDofContactNames, 
    const std::vector<...> &sixDofContactNames)





}

void LocoManiMpcInterface::setupOptimalControlProblem() {


// reset Optimal Control Problem
    problemPtr_.reset(new )






}









}   // namespace ocs2::humanoid