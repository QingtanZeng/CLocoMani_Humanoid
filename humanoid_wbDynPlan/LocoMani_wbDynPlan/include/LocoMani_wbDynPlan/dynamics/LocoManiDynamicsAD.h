#pragma once

#include <ocs2_centroidal_model/PinocchioCentroidalDynamicsAD.h>
#include <ocs2_core/dynamics/SystemDynamicsBase.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>

#include "wbDynPlan/common/ModelSettings.h"

namespace ocs2::humanoid {

class LocoManiDynamicsAD : public SystemDynamicsBase {
     public:
    LocoManiDynamicsAD(const PinocchioInterface& pinocchioInterface,
                       const CentroidalModelInfo& info,
                       const std::string& modelName,
                       const ModelSettings& modelSettings);


    ~LocoManiDynamicsAD() override = default;
    LocoManiDynamicsAD* clone() const override { return new LocoManiDynamicsAD(*this); }

  vector_t computeFlowMap(scalar_t time, const vector_t& state, const vector_t& input, const PreComputation& preComp) override;
  VectorFunctionLinearApproximation linearApproximation(scalar_t time,
                                                        const vector_t& state,
                                                        const vector_t& input,
                                                        const PreComputation& preComp) override;

 private:
  LocoManiDynamicsAD(const CentroidalDynamicsAD& rhs) = default;

  PinocchioCentroidalDynamicsAD pinocchioCentroidalDynamicsAD_;
  
};
}   // namespace ocs2::humanoid