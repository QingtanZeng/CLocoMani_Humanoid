#pragma once

#include <ocs2_core/Types.h>
#include <ocs2_core/penalties/Penalties.h>
#include <ocs2_ddp/DDP_Settings.h>
#include <ocs2_mpc/MPC_Settings.h>
#include <ocs2_oc/rollout/TimeTriggeredRollout.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematicsCppAd.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include <ocs2_robotic_tools/common/RobotInterface.h>
#include <ocs2_robotic_tools/end_effector/EndEffectorKinematics.h>
#include <ocs2_sqp/SqpSettings.h>

#include "LocoMani_wbDynPlan/common/CentroidalMpcRobotModel.h"
#include "LocoMani_wbDynPlan/initialization/CentroidalWeightCompInitializer.h"
#include "wbDynPlan/common/ModelSettings.h"
#include "wbDynPlan/reference_manager/ProceduralMpcMotionManager.h"
#include "wbDynPlan/reference_manager/SwitchedModelReferenceManager.h"

namespace ocs2::humanoid {

class LocoManiMpcInterface final : public RobotInterface{

 public:
   /**
   * Constructor
   *
   * @throw Invalid argument error if input task file or urdf file does not exist.
   *
   * @param [in] taskFile: The absolute path to the configuration file for the MPC.
   * @param [in] urdfFile: The absolute path to the URDF file for the robot.
   * @param [in] referenceFile: The absolute path to the reference configuration file.
   */
  LocoManiMpcInterface(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile,
                        bool setupOCP = true);
  /** Destructor */
    ~LocoManiMpcInterface() override = default;

  /**
   * @brief Get the optimal control problem definition
   * @return reference to the problem object
   */
  const OptimalControlProblem& getOptimalControlProblem() const override { return *problemPtr_; }
  // CAREFUL: This function is not const, so it can easily be abused. It is currently only for gui purposes. Use with care!
  OptimalControlProblem& getOptimalControlProblemRef() const { return *problemPtr_; }

  const ModelSettings& modelSettings() const { return modelSettings_; }
  const ddp::Settings& ddpSettings() const { return ddpSettings_; }
  const mpc::Settings& mpcSettings() const { return mpcSettings_; }
  const rollout::Settings& rolloutSettings() const { return rolloutSettings_; }
  const sqp::Settings& sqpSettings() const { return sqpSettings_; }

  const vector_t& getInitialState() const { return initialState_; }
  const RolloutBase& getRollout() const { return *rolloutPtr_; }
  PinocchioInterface& getPinocchioInterface() { return *pinocchioInterfacePtr_; }
  const CentroidalModelInfo& getCentroidalModelInfo() const { return centroidalModelInfo_; }
  std::shared_ptr<SwitchedModelReferenceManager> getSwitchedModelReferenceManagerPtr() const { return referenceManagerPtr_; }

  const CentroidalWeightCompInitializer& getInitializer() const override { return *initializerPtr_; }
  std::shared_ptr<ReferenceManagerInterface> getReferenceManagerPtr() const override { return referenceManagerPtr_; }

  const CentroidalMpcRobotModel<scalar_t>& getMpcRobotModel() const { return *ocpRobotDynSysPtr_; }
  const CentroidalMpcRobotModel<ad_scalar_t>& getMpcRobotModelAD() const { return *mpcRobotModelADPtr_; }

  std::vector<std::string> getCostNames() const;
  std::vector<std::string> getTerminalCostNames() const;
  std::vector<std::string> getStateSoftConstraintNames() const;
  std::vector<std::string> getSoftConstraintNames() const;
  std::vector<std::string> getEqualityConstraintNames() const;
  
  private:
  void setupOptimalControlProblem();

  // Loco-Manipulation's Contact Constraints
  std::unique_ptr<StateInputConstraint> getStanceFootConstraint(const EndEffectorKinematics<scalar_t>& eeKinematics,
                                                                 size_t contactPointIndex);
  std::unique_ptr<StateInputConstraint> getNormalVelocityConstraint(const EndEffectorKinematics<scalar_t>& eeKinematics,
                                                                    size_t contactPointIndex);
  std::unique_ptr<StateInputConstraint> getJointMimicConstraint(size_t mimicIndex);

  // Loco-Manipulation's Contact Tracking Cost
  void addTaskSpaceKinematicsCosts(const CentroidalModelPinocchioMappingCppAd& pinMappingCppAd,
                                   const PinocchioEndEffectorKinematicsCppAd::update_pinocchio_interface_callback& velocityUpdateCallback);

  // Model and Solver settings
  ModelSettings modelSettings_;
  ddp::Settings ddpSettings_;
  mpc::Settings mpcSettings_;
  sqp::Settings sqpSettings_;

  // Model
  std::unique_ptr<PinocchioInterface> pinocchioInterfacePtr_;
  CentroidalModelInfo centroidalModelInfo_;

  // OCP Problem and Switched Manager
  std::unique_ptr<OptimalControlProblem> problemPtr_;
  std::shared_ptr<SwitchedModelReferenceManager> referenceManagerPtr_;

  // OCP Dynamic System
  std::unique_ptr<LocoManiMpcRobotModel<scalar_t>> ocpRobotDynSysPtr_;
  std::unique_ptr<LocoManiMpcRobotModel<ad_scalar_t>> ocpRobotDynSysADPtr_;

  // MPC Arthitecture
  rollout::Settings rolloutSettings_;
  std::unique_ptr<RolloutBase> rolloutPtr_;
  std::unique_ptr<CentroidalWeightCompInitializer> initializerPtr_; // MPC Initilization

  vector_t initialState_;

  const std::string taskFile_;
  const std::string urdfFile_;
  const std::string referenceFile_;
  bool verbose_;
};

}   // namespace ocs2::humanoid