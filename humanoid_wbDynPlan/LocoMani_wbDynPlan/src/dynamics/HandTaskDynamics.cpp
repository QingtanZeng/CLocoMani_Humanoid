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