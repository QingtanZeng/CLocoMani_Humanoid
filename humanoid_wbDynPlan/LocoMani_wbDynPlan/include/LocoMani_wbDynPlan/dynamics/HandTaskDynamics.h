#pragma once

#include <ocs2_centroidal_model/PinocchioCentroidalDynamicsAD.h>
#include <ocs2_core/dynamics/SystemDynamicsBase.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>

#include "LocoMani_wbDynPlan/common/ModelSettings.h"

namespace ocs2::humanoid {

class HandTaskDynamics : public SystemDynamicsBase {
    public:
    HandTaskDynamics(const PinocchioInterface& pinocchioInterface,
                       const CentroidalModelInfo& info,
                       const std::string& modelName,
                       const ModelSettings& modelSettings);


    ~HandTaskDynamics() override = default;
    HandTaskDynamics* clone() const override { return new HandTaskDynamics(*this); }

  vector_t computeFlowMap(scalar_t time, const vector_t& state, const vector_t& input, const PreComputation& preComp) override;
  VectorFunctionLinearApproximation linearApproximation(scalar_t time,
                                                        const vector_t& state,
                                                        const vector_t& input,
                                                        const PreComputation& preComp) override;

 private:
  HandTaskDynamics(const CentroidalDynamicsAD& rhs) = default;

  double massbox;
  double mu; // Coefficient of static friction
  double[3] dimbox; // Dimensions of the box (x,y,z), [13, 23, 16]

};

}   // namespace ocs2::humanoid