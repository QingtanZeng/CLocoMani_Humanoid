/******************************************************************************
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


#include <pinocchio/fwd.hpp>  // forward declarations must be included first.

#include "ocs2_centroidal_model/CentroidalModelRbdConversions.h"

#include <pinocchio/algorithm/centroidal.hpp>
#include <pinocchio/algorithm/rnea.hpp>

#include <ocs2_core/misc/LoadData.h>
#include <ocs2_robotic_tools/common/RotationDerivativesTransforms.h>

#include "ocs2_centroidal_model/AccessHelperFunctions.h"
#include "ocs2_centroidal_model/ModelHelperFunctions.h"


void computeBaseKinematicsFromCentroidalModel(){
  // get common classes
  const auto& model = pinocchioInterface_.getModel();
  auto& data = pinocchioInterface_getData();
  const auto& info = mapping_.getCentroidalModelInfo();
  const auto& qPinocchio = mapping_.getPinocchioJointPosition(state);

  // update forward kinematics
  updateFullCentroidalDynamics(pinocchioInterface_, info, qPinocchio);

  // Extract base kinematics
  Vector6 basePose = qPinocchio.head<6>();    //get q_base
  const auto basePosition = basePose.head<3>();
  const auto baseOrientation = basePose.tail<3>();

  // Base Velocity in LWA frame
}

/**
 */
template <typename SCALAR>    // used in AutoDiff
auto baseVelocityFromFullCentroidalKinematics(const vector_t& state, const vector_t& input) const
    -> vector_t {
// get common classes
const auto& model = pinocchioInterface_.getModel();
auto& data = pinocchioInterface_getData();
const auto& info = mapping_.getCentroidalModelInfo();

// check dimensions
assert(info.stateDim == state.rows());
assert(info.inputDim == input.rows());

// Extract joint positions from state

// Extract joint velocities from input
const auto jointVelocity = centroidal_model::getJointVelocities(input, info).head(info.actuatedDofNum);
// Extract Centroidal Momentum from state
Eigen::Matrix<SCALAR, 6, 1> centrdlMmtm = info.robotMass *
                                          centroidal_model::getNormalizedMomentum(state, info);

// Compute CMM and its inverse
const auto& CMM = getCentroidalMomentumMatrix(*pinocchioInterface_);    // get Ag


// Base Velocity in LWA frame
vector_t baseVelocity(6);
baseVelocity = Ab_inv * (centrdlMmtm - Aj*) 



}

template <typename SCALAR>
auto jacobianFullCentroidalKinematics(const vector_t& state, 
                            const matrix_t& Jq, const matrix_t& Jv) const















template <typename SCALAR_T>
void updateCentroidalDynamics(inocchioInterfaceTpl<SCALAR_T>& interface, const CentroidalModelInfoTpl<SCALAR_T>& info,
                                const Eigen::Matrix<SCALAR_T, Eigen::Dynamic, 1>& q) {

const auto& model = interface.getModel();
auto& data = interface.getData();

// update the CMM Matrix
pinocchio::computeCentroidalMap(model, data, q);
// update frame placements
pinocchio::updateFramePlacements(model, data);
}