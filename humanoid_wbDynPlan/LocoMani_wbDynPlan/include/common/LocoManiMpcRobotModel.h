

#pragma once

#include <array>
#include <cstddef>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "wbDynPlan/common/ModelSettings.h"
#include "wbDynPlan/common/MpcRobotModelBase.h"
#include "wbDynPlan/common/Types.h"

#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_centroidal_model/ModelHelperFunctions.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include <ocs2_pinocchio_interface/PinocchioStateInputMapping.h>

namespace ocs2::humanoid{
/******************************************************************************************************/
/* State Vector definition

  Define the state vector x = [h, q_b, q_j]^T

    Centroidal momentum h = [vcom_x, vcom_y, vcom_z, L_x / mass, L_y / mass, L_z / mass]^T
    Base pose q_b = [p_base_x, p_base_y, p_base_z, theta_base_z, theta_base_y, theta_base_x]
    Joint angles q_j

*/
/******************************************************************************************************/

/******************************************************************************************************/
/* Input Vector definition

  Define the input vector u = [W_l, W_r, q_dot_j]^T

    Left contact Wrench W_l in inertial frame
    Right contact Wrench W_r in inertial frame
    Joint velocities q_dot_j

*/
/******************************************************************************************************/

template <typename SCALAR_T>
class LocoManiMpcRbotModel : public MpcRobotModelBase<SCALAR_T> {
    LocoManiMpcRbotModel(const ModelSettings& modelSettings,
                          const PinocchioInterfaceTpl<SCALAR_T>& pinocchioInterface,
                          const CentroidalModelInfoTpl<SCALAR_T>& centroidalModelInfo)
      : MpcRobotModelBase<SCALAR_T>(modelSettings, 12 + modelSettings.mpc_joint_dim, 6 * N_CONTACTS + modelSettings.mpc_joint_dim),
        pinocchioInterface_(pinocchioInterface),
        centroidalModelInfo_(centroidalModelInfo),
        pinocchioMappingPtr_(new CentroidalModelPinocchioMappingTpl<SCALAR_T>(centroidalModelInfo)) {
    pinocchioMappingPtr_->setPinocchioInterface(pinocchioInterface_);
  };

  ~LocoManiMpcRbotModel() override = default;
  LocoManiMpcRbotModel* clone() const override { return new LocoManiMpcRbotModel(*this); }
  /******************************************************************************************************/
  /*                                          Start indices                                             */
  /******************************************************************************************************/

  size_t getBaseStartindex() const override { return 6; };
  size_t getJointStartindex() const override { return 12; };
  size_t getJointVelocitiesStartindex() const override { return 6 * N_CONTACTS; };

  // Assumes contact wrench [f_x, f_y, f_z, M_x, M_y, M_z]^T
  size_t getContactWrenchStartIndices(size_t contactIndex) const override { return 6 * contactIndex; };
  size_t getContactForceStartIndices(size_t contactIndex) const override { return getContactWrenchStartIndices(contactIndex); };
  size_t getContactMomentStartIndices(size_t contactIndex) const override { return getContactWrenchStartIndices(contactIndex) + 3; };

  /******************************************************************************************************/
  /*                                     Generalized coordinates                                        */
  /******************************************************************************************************/

  VECTOR_T<SCALAR_T> getGeneralizedCoordinates(const VECTOR_T<SCALAR_T>& state) const override {
    assert(state.size() == this->state_dim);
    return state.tail(6 + this->modelSettings.mpc_joint_dim);
  };

  VECTOR6_T<SCALAR_T> getBasePose(const VECTOR_T<SCALAR_T>& state) const override {
    assert(state.size() == this->state_dim);
    return state.segment(6, 6);
  };

  VECTOR3_T<SCALAR_T> getBasePosition(const VECTOR_T<SCALAR_T>& state) const override {
    assert(state.size() == this->state_dim);
    return state.segment(6, 3);
  }

  VECTOR3_T<SCALAR_T> getBaseOrientationEulerZYX(const VECTOR_T<SCALAR_T>& state) const override {
    assert(state.size() == this->state_dim);
    return state.segment(6 + 3, 3);
  }

  // Return Com linear velocity
  VECTOR3_T<SCALAR_T> getBaseComLinearVelocity(const VECTOR_T<SCALAR_T>& state) const override {
    assert(state.size() == this->state_dim);
    return state.head(3);
  };

  VECTOR6_T<SCALAR_T> getBaseComVelocity(const VECTOR_T<SCALAR_T>& state) const override {
    assert(state.size() == this->state_dim);
    return state.head(6);
  };

  VECTOR_T<SCALAR_T> getJointAngles(const VECTOR_T<SCALAR_T>& state) const override {
    assert(state.size() == this->state_dim);
    return state.tail(this->modelSettings.mpc_joint_dim);
  };

  VECTOR_T<SCALAR_T> getJointVelocities(const VECTOR_T<SCALAR_T>& state, const VECTOR_T<SCALAR_T>& input) const override {
    assert(input.size() == this->input_dim);
    return input.tail(this->modelSettings.mpc_joint_dim);
  };

  VECTOR_T<SCALAR_T> getGeneralizedVelocities(const VECTOR_T<SCALAR_T>& state, const VECTOR_T<SCALAR_T>& input) override {
    assert(state.size() == this->state_dim);
    assert(input.size() == this->input_dim);
    updateCentroidalDynamics<SCALAR_T>(pinocchioInterface_, centroidalModelInfo_, pinocchioMappingPtr_->getPinocchioJointPosition(state));
    return pinocchioMappingPtr_->getPinocchioJointVelocity(state, input);
  };

  void setGeneralizedCoordinates(VECTOR_T<SCALAR_T>& state, const VECTOR_T<SCALAR_T>& generalizedCorrdinates) const override {
    assert(state.size() == this->state_dim);
    assert(generalizedCorrdinates.size() == 6 + this->modelSettings.mpc_joint_dim);
    state.tail(6 + this->modelSettings.mpc_joint_dim) = generalizedCorrdinates;
  }

  void setBasePose(VECTOR_T<SCALAR_T>& state, const VECTOR6_T<SCALAR_T>& basePose) const override {
    assert(state.size() == this->state_dim);
    state.segment(6, 6) = basePose;
  }

  void setBasePosition(VECTOR_T<SCALAR_T>& state, const VECTOR3_T<SCALAR_T>& position) const override {
    assert(state.size() == this->state_dim);
    state.segment(6, 3) = position;
  }

  void setBaseOrientationEulerZYX(VECTOR_T<SCALAR_T>& state, const VECTOR3_T<SCALAR_T>& eulerAnglesZYX) const override {
    assert(state.size() == this->state_dim);
    state.segment(6 + 3, 3) = eulerAnglesZYX;
  }

  void setJointAngles(VECTOR_T<SCALAR_T>& state, const VECTOR_T<SCALAR_T>& jointAngles) const override {
    assert(state.size() == this->state_dim);
    state.tail(this->modelSettings.mpc_joint_dim) = jointAngles;
  }

  void setJointVelocities(VECTOR_T<SCALAR_T>& state, VECTOR_T<SCALAR_T>& input, const VECTOR_T<SCALAR_T>& jointVelocities) const override {
    assert(state.size() == this->state_dim);
    assert(input.size() == this->input_dim);
    input.tail(this->modelSettings.mpc_joint_dim) = jointVelocities;
  }

  void adaptBasePoseHeight(VECTOR_T<SCALAR_T>& state, scalar_t heightChange) const {
    assert(state.size() == this->state_dim);
    state[6 + 2] += heightChange;
  }

  /******************************************************************************************************/
  /*                                          Contacts                                                  */
  /******************************************************************************************************/

  VECTOR6_T<SCALAR_T> getContactWrench(const VECTOR_T<SCALAR_T>& input, size_t contactIndex) const override {
    assert(input.size() == this->input_dim);
    return input.segment(getContactWrenchStartIndices(contactIndex), CONTACT_WRENCH_DIM);
  };

  VECTOR3_T<SCALAR_T> getContactForce(const VECTOR_T<SCALAR_T>& input, size_t contactIndex) const override {
    assert(input.size() == this->input_dim);
    return input.segment(getContactWrenchStartIndices(contactIndex), 3);
  };

  VECTOR3_T<SCALAR_T> getContactMoment(const VECTOR_T<SCALAR_T>& input, size_t contactIndex) const override {
    assert(input.size() == this->input_dim);
    return input.segment((getContactWrenchStartIndices(contactIndex) + 3), 3);
  };

  void setContactWrench(VECTOR_T<SCALAR_T>& input, const VECTOR6_T<SCALAR_T>& wrench, size_t contactIndex) const override {
    assert(input.size() == this->input_dim);
    input.segment(getContactWrenchStartIndices(contactIndex), 6) = wrench;
  };

  void setContactForce(VECTOR_T<SCALAR_T>& input, const VECTOR3_T<SCALAR_T>& force, size_t contactIndex) const override {
    assert(input.size() == this->input_dim);
    input.segment(getContactWrenchStartIndices(contactIndex), 3) = force;
  };

  void setContactMoment(VECTOR_T<SCALAR_T>& input, const VECTOR3_T<SCALAR_T>& moment, size_t contactIndex) const override {
    assert(input.size() == this->input_dim);
    input.segment(getContactWrenchStartIndices(contactIndex) + 3, 3) = moment;
  };

  /******************************************************************************************************/
  /*                                    Custom Centroidal Methods                                       */
  /******************************************************************************************************/

  VECTOR_T<SCALAR_T> getCentroidalMomentum(const VECTOR_T<SCALAR_T>& state) const {
    assert(state.size() == this->state_dim);
    return state.head(6);
  };

  const CentroidalModelInfoTpl<SCALAR_T>& getCentroidalModelInfo() const { return centroidalModelInfo_; }

private:
  LocoManiMpcRbotModel(const LocoManiMpcRbotModel& rhs)
      : MpcRobotModelBase<SCALAR_T>(rhs),
        pinocchioInterface_(rhs.pinocchioInterface_),
        centroidalModelInfo_(rhs.centroidalModelInfo_),
        pinocchioMappingPtr_(rhs.pinocchioMappingPtr_->clone()){
    pinocchioMappingPtr_->setPinocchioInterface(pinocchioInterface_);
  };

  PinocchioInterfaceTpl<SCALAR_T> pinocchioInterface_;
  const CentroidalModelInfoTpl<SCALAR_T> centroidalModelInfo_;
  const std::unique_ptr<CentroidalModelPinocchioMappingTpl<SCALAR_T>> pinocchioMappingPtr_;
};


}   //namespace ocs2::humanoid