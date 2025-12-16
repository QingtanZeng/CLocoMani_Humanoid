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
  pinocchioInterfacePtr_.reset(new PinocchioInterface(createCustomPinocchioInterface(
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

  // Setup Centroidal State Input Mapping
  ocpRobotDynSysPtr_.reset(new LocoManiMpcRbotModel<scalar_t>(modelSettings_, *pinocchioInterfacePtr_, centroidalModelInfo_));
  ocpRobotDynSysADPtr_.reset(
      new LocoManiMpcRbotModel<ad_scalar_t>(modelSettings_, (*pinocchioInterfacePtr_).toCppAd(), centroidalModelInfo_.toCppAd()));

  // Swing trajectory planner
  std::unique_ptr<SwingTrajectoryPlanner> swingTrajectoryPlanner(
      new SwingTrajectoryPlanner(loadSwingTrajectorySettings(taskFile, "swing_trajectory_config", verbose_), N_CONTACTS));

  referenceManagerPtr_ =
      std::make_shared<SwitchedModelReferenceManager>(GaitSchedule::loadGaitSchedule(referenceFile, modelSettings_, verbose_),
                                                      std::move(swingTrajectoryPlanner), *pinocchioInterfacePtr_, *ocpRobotDynSysPtr_);
  referenceManagerPtr_->setArmSwingReferenceActive(true);


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
      HumanoidCostConstraintFactory(taskFile_, referenceFile_, *referenceManagerPtr_, *pinocchioInterfacePtr_, *ocpRobotDynSysPtr_,
                                    *ocpRobotDynSysADPtr_, modelSettings_, verbose_);

  // reset Optimal Control Problem
  problemPtr_.reset(new OptimalControlProblem);

  // 1. Dynamics, Kinematics, Task Dynamics, and their AutoDiff
  // autodiff
  const auto infoCppAd = centroidalModelInfo_.toCppAd();
  const CentroidalModelPinocchioMappingCppAd pinocchioMappingCppAd(infoCppAd);

  auto velocityUpdateCallback = [&infoCppAd](const ad_vector_t& state, PinocchioInterfaceCppAd& pinocchioInterfaceAd) {
    const ad_vector_t q = centroidal_model::getGeneralizedCoordinates(state, infoCppAd);
    updateCentroidalDynamics(pinocchioInterfaceAd, infoCppAd, q);
  };

  // 1.1 dynamics
  std::unique_ptr<SystemDynamicsBase> dynamicsPtr;
  const std::string modelname = "locomani";
  dynamicsPtr.reset(new LocoManiDynamicsAD(*pinocchioInterfacePtr_, centroidalModelInfo_, modelname, modelSettings_));

  problemPtr_->dynamicsPtr = std::move(dynamicsPtr);

  // 1.2 End-effector kinematics used in costs and constraints
  std::unique_ptr<EndEffectorKinematics<scalar_t>> eeKinematicsPtr;  // ALL Contacts Kinematics with Pinocchio CppAD

  // 1.3 Task Dynamics (Lift the box with both hands)

  // 2. Cost terms
  // 2.1 add Q ,R for hcom, floating base, default joint angles, and control cost
  problemPtr_->costPtr->add("stateInputdQuadraticCost", factory.getStateInputQuadraticCost());
  // 2.2 add terminal state cost
  problemPtr_->finalCostPtr->add("terminalCost", factory.getTerminalCost());

  // 2.3 Foot End-effctor Kinematics Tracking Cost
  EndEffectorKinematicsWeights footTrackingCostWeights =
      EndEffectorKinematicsWeights::getWeights(taskFile_, "task_space_foot_cost_weights.", verbose_);
  for (size_t i = 0; i < N_FOOTCONTACTS; ++i) {
    const std::string& footName = modelSettings_.contactNames[i];

    std::string footTrackingCostName = footName + "_FootTrjKinematicsCost";

    problemPtr_->costPtr->add(footTrackingCostName, std::unique_ptr<StateInputCost>(new CentroidalMpcEndEffectorFootCost(
                                                        *referenceManagerPtr_, footTrackingCostWeights, *pinocchioInterfacePtr_,
                                                        *ocpRobotDynSysADPtr_, i, footTrackingCostName, modelSettings_)));
    problemPtr_->costPtr->add(footName + "_ExternalTorqueQuadraticCost", factory.getExternalTorqueQuadraticCost(i));
  }

  // 2.4 Hand End-effctor Kinematics Tracking Cost (Lift the box with both hands)
  // including Holding, lifting, gaiting, lay down
  addTaskSpaceKinematicsCosts(pinocchioMappingCppAd, velocityUpdateCallback);  // Hand task tracking

  // 2.5 ICP Balance Cost
  const vector2_t icpWeights = ICPCost::getWeights(taskFile_, "icp_cost_weights.", verbose_);
  problemPtr_->costPtr->add(
      "icp_Cost", std::unique_ptr<StateInputCost>(new ICPCost(*referenceManagerPtr_, std::move(icpWeights), *pinocchioInterfacePtr_,
                                                              *ocpRobotDynSysADPtr_, "icp_Cost", modelSettings_)));
  // 3. Constraint terms
  // Joint Limits: x_angle, u_velocity, torque(u)
  problemPtr_->stateInequalityConstraintPtr->add("jointLimits", factory.getJointLimitsConstraint());

  // 3.1 Foot Constacts:
  // 1) Stance Foot Constraint: a) Zero Velocity b) Friction Cone c) Contact Moment XY(???) d) Normal Velocity
  // 2) Swing Foot Constraint:  a) Zero Wrench
  // 3) Foot Collision (as soft cost in sqp)
  problemPtr_->stateSoftConstraintPtr->add("FootCollisionSoftConstraint", factory.getFootCollisionConstraint());

  for (size_t i = 0; i < N_FOOTCONTACTS; ++i) {
    const std::string& footName = modelSettings_.contactNames[i];

    // end-effector kinematics for foot
    eeKinematicsPtr.reset(new PinocchioEndEffectorKinematicsCppAd(*pinocchioInterfacePtr_, pinocchioMappingCppAd, {footName},
                                                                  centroidalModelInfo_.stateDim, centroidalModelInfo_.inputDim,
                                                                  velocityUpdateCallback, footName, modelSettings_.modelFolderCppAd,
                                                                  modelSettings_.recompileLibrariesCppAd, modelSettings_.verboseCppAd));
    // Friction Cone
    problemPtr_->inequalityConstraintPtr->add(footName + "_frictionCone", factory.getFrictionForceConeConstraint(i));
    // Constact Moment XY Constraint (as soft cost in sqp)
    problemPtr_->softConstraintPtr->add(footName + "_contactMomentXY",
                                        factory.getContactMomentXYConstraint(i, footName + "_contact_moment_XY_constraint"));
    // Zero Velocity
    problemPtr_->equalityConstraintPtr->add(footName + "_zeroVelocity", getStanceFootConstraint(*eeKinematicsPtr, i));
    // Normal Velocity
    problemPtr_->equalityConstraintPtr->add(footName + "_normalVelocity", getNormalVelocityConstraint(*eeKinematicsPtr, i));

    // Swing foot: Zero Wrench
    problemPtr_->equalityConstraintPtr->add(footName + "_zeroWrench", factory.getZeroWrenchConstraint(i));
  }

  // 3.2 Hand Constact(Lift the box with both hands):
  // 1) Hand End-effctor Kinematics Limits:
  // 2）Manipulation Friction fixed connection:
  // 3) Manipulation Collision (as soft cost in sqp)
  //problemPtr_->stateSoftConstraintPtr->add("ManiCollisionSoftConstraint", factory.getManiCollisionConstraint());

  for (size_t i = N_FOOTCONTACTS; i < (N_HANDCONTACTS + N_FOOTCONTACTS); ++i) {
    const std::string& handName = modelSettings_.contactNames[i];

    // end-effector kinematics for hand
    eeKinematicsPtr.reset(new PinocchioEndEffectorKinematicsCppAd(*pinocchioInterfacePtr_, pinocchioMappingCppAd, {footName},
                                                                  centroidalModelInfo_.stateDim, centroidalModelInfo_.inputDim,
                                                                  velocityUpdateCallback, footName, modelSettings_.modelFolderCppAd,
                                                                  modelSettings_.recompileLibrariesCppAd, modelSettings_.verboseCppAd));
    // Manipulation Friction fixed connection (! 修改.get()函数适合手部任务)
    // problemPtr_->inequalityConstraintPtr->add(handName + "_frictionCone", factory.getFrictionForceConeConstraint(i));

    // 1) Hand End-effctor Kinematics Limits
    // relative location to floating base: [x, y, z]
    // relative speed to CoM
  }

  // 3.3 self-collision: feet, knee, hands, elbows

  // Pre-computation (???)

  // Rollout
  rolloutPtr_.reset(new TimeTriggeredRollout(*problemPtr_->dynamicsPtr, rolloutSettings_));

  // Initialization
  constexpr bool extendNormalizedMomentum = true;
  initializerPtr_.reset(
      new CentroidalWeightCompInitializer(centroidalModelInfo_, *referenceManagerPtr_, *ocpRobotDynSysPtr_, extendNormalizedMomentum));
}

void LocoManiMpcInterface::addTaskSpaceKinematicsCosts(
    const CentroidalModelPinocchioMappingCppAd& pinocchioMappingCppAd,
    const PinocchioEndEffectorKinematicsCppAd::update_pinocchio_interface_callback& velocityUpdateCallback) {
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(taskFile_, pt);

  boost::property_tree::ptree task_space_costs_pt = pt.get_child("task_space_costs");

  for (auto& task_space_cost : task_space_costs_pt) {
    std::string costName = task_space_cost.first;
    std::string linkName;

    loadData::loadPtreeValue(task_space_costs_pt, linkName, costName + ".link_name", verbose_);

    std::unique_ptr<EndEffectorKinematics<scalar_t>> eeKinematicsPtr;

    eeKinematicsPtr.reset(new PinocchioEndEffectorKinematicsCppAd(*pinocchioInterfacePtr_, pinocchioMappingCppAd, {linkName},
                                                                  centroidalModelInfo_.stateDim, centroidalModelInfo_.inputDim,
                                                                  velocityUpdateCallback, linkName, modelSettings_.modelFolderCppAd,
                                                                  modelSettings_.recompileLibrariesCppAd, modelSettings_.verboseCppAd));

    EndEffectorKinematicsWeights weights =
        EndEffectorKinematicsWeights::getWeights(taskFile_, "task_space_costs." + costName + ".weights.", verbose_);

    std::unique_ptr<StateInputCost> cost = std::make_unique<EndEffectorKinematicsQuadraticCost>(
        weights, *pinocchioInterfacePtr_, *eeKinematicsPtr, *ocpRobotDynSysADPtr_, linkName, modelSettings_);

    problemPtr_->costPtr->add(costName + "_TaskSpaceKinematicsCost", std::move(cost));

    std::cout << "Initialized Task Space Kinematics Cost for link: " << linkName << std::endl;
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

std::unique_ptr<StateInputConstraint> CentroidalMpcInterface::getStanceFootConstraint(const EndEffectorKinematics<scalar_t>& eeKinematics,
                                                                                      size_t contactPointIndex) {
  auto eeZeroVelConConfig = [](scalar_t positionErrorGain, scalar_t orientationErrorGain) {
    EndEffectorKinematicsTwistConstraint::Config config;
    config.b.setZero(6);
    config.Ax.setZero(6, 6);
    config.Av.setIdentity(6, 6);
    if (!numerics::almost_eq(positionErrorGain, 0.0)) {
      config.Ax(2, 2) = positionErrorGain;
    }
    if (!numerics::almost_eq(orientationErrorGain, 0.0)) {
      config.Ax.block(3, 3, 3, 3) = Eigen::MatrixXd::Identity(3, 3) * orientationErrorGain;
    }

    return config;
  };

  return std::unique_ptr<StateInputConstraint>(
      new ZeroVelocityConstraintCppAd(*referenceManagerPtr_, eeKinematics, contactPointIndex,
                                      eeZeroVelConConfig(modelSettings_.footConstraintConfig.positionErrorGain_z,
                                                         modelSettings_.footConstraintConfig.orientationErrorGain)));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::unique_ptr<StateInputConstraint> CentroidalMpcInterface::getNormalVelocityConstraint(
    const EndEffectorKinematics<scalar_t>& eeKinematics, size_t contactPointIndex) {
  return std::unique_ptr<StateInputConstraint>(new NormalVelocityConstraintCppAd(*referenceManagerPtr_, eeKinematics, contactPointIndex));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::unique_ptr<StateInputConstraint> CentroidalMpcInterface::getJointMimicConstraint(size_t mimicIndex) {
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(taskFile_, pt);
  std::string prefix;
  if (mimicIndex == 0) {
    prefix = "mimicJoints.left_knee.";
  } else if (mimicIndex == 1) {
    prefix = "mimicJoints.right_knee.";
  } else {
    throw std::runtime_error("No mimic joint for index: " + std::to_string(mimicIndex));
  }

  std::string parentJointName;
  std::string childJointName;
  scalar_t multiplier;  // q_child = multiplier* q_parent
  scalar_t positionGain;

  if (verbose_) {
    std::cerr << "\n #### Joint Mimic Kinematic Constraint Config: ";
    std::cerr << "\n #### "
                 "============================================================="
                 "================\n";
  }
  loadData::loadPtreeValue(pt, parentJointName, prefix + "parentJointName", verbose_);
  loadData::loadPtreeValue(pt, childJointName, prefix + "childJointName", verbose_);
  loadData::loadPtreeValue(pt, multiplier, prefix + "multiplier", verbose_);
  loadData::loadPtreeValue(pt, positionGain, prefix + "positionGain", verbose_);
  if (verbose_) {
    std::cerr << " #### "
                 "============================================================="
                 "================\n";
  }

  JointMimicKinematicConstraint::Config config(*mpcRobotModelPtr_, parentJointName, childJointName, multiplier, positionGain);

  return std::unique_ptr<StateInputConstraint>(new JointMimicKinematicConstraint(*mpcRobotModelPtr_, config));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

std::vector<std::string> CentroidalMpcInterface::getCostNames() const {
  std::vector<std::string> costNames;
  for (const auto& [costName, index] : problemPtr_->costPtr->getTermNameMap()) {
    costNames.emplace_back(costName);
  }
  return costNames;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

std::vector<std::string> CentroidalMpcInterface::getTerminalCostNames() const {
  std::vector<std::string> terminalCostNames;
  for (const auto& [costName, index] : problemPtr_->finalCostPtr->getTermNameMap()) {
    terminalCostNames.emplace_back(costName);
  }
  return terminalCostNames;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

std::vector<std::string> CentroidalMpcInterface::getStateSoftConstraintNames() const {
  std::vector<std::string> costNames;
  for (const auto& [costName, index] : problemPtr_->stateSoftConstraintPtr->getTermNameMap()) {
    costNames.emplace_back(costName);
  }
  return costNames;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

std::vector<std::string> CentroidalMpcInterface::getSoftConstraintNames() const {
  std::vector<std::string> costNames;
  for (const auto& [costName, index] : problemPtr_->softConstraintPtr->getTermNameMap()) {
    costNames.emplace_back(costName);
  }
  return costNames;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

std::vector<std::string> CentroidalMpcInterface::getEqualityConstraintNames() const {
  std::vector<std::string> costNames;
  for (const auto& [costName, index] : problemPtr_->equalityConstraintPtr->getTermNameMap()) {
    costNames.emplace_back(costName);
  }
  return costNames;
}

}  // namespace ocs2::humanoid