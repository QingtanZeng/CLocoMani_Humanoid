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

#include "LocoMani_wbDynPlan/dynamics/HandTaskDynamics.h"


namespace ocs2::humanoid {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
HandTaskDynamics::LocoManiDynamicsAD(const PinocchioInterface& pinocchioInterface,
                                           const CentroidalModelInfo& info,
                                           const std::string& modelName,
                                           const ModelSettings& modelSettings)
                                           {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t LocoManiDynamicsAD::computeFlowMap(scalar_t time, const vector_t& state, const vector_t& input, const PreComputation& preComp) {
  
    vector3_t ez(0,0,1); 
    vector3_t funcValueL = (1/massbox)*(-input.head(3) - input.tail(3)) + 9.81*ez;       // R^(3x1) two hands  resultant force
    return funcValueL;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

VectorFunctionLinearApproximation LocoManiDynamicsAD::linearApproximation(scalar_t tieme,
                                                                          const vector_t& state,
                                                                          const vector_t& input,
                                                                          const PreComputation& preComp) {
  vector3_t ez(0,0,1);  
  VectorFunctionLinearApproximation approx;
  approx.f = computeFlowMap(time, state, input, preComp);
  approx.dfdx = matrix3_t::Zero(3, state.size()); // R^(3 x 3)

  matrix3_t::Identity(3,3) I;                                                                          
  MATRIX_T::Zero(3,6) B;
  B << I, I; // R^(3 x 6)
  approx.dfdu = -(1/massbox)*B; // R^(3 x 6)

  return approx;
}   
}  // namespace ocs2::humanoid