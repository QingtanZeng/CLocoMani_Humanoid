
/* Computational Legged Robots: Planning and Control  */

#include "wbDynPlan_ros2/visualization/EquivalentContactCornerForcesVisualizer.h"

#include <ocs2_ros2_interfaces/visualization/VisualizationColors.h>
#include <ocs2_ros2_interfaces/visualization/VisualizationHelpers.h>

#include <wbDynPlan/common/ModelSettings.h>
#include <wbDynPlan/contact/ContactRectangle.h>
#include <wbDynPlan/pinocchio_model/DynamicsHelperFunctions.h>
#include <wbDynPlan/pinocchio_model/PinocchioFrameConversions.h>

namespace ocs2::humanoid {

EquivalentContactCornerForcesVisualizer::EquivalentContactCornerForcesVisualizer(const std::string& taskFile,
                                                                                 const PinocchioInterface& pinocchioInterface,
                                                                                 const MpcRobotModelBase<scalar_t>& mpcRobotModel)
    : pinocchioInterfacePtr_(&pinocchioInterface),
      contactFrameIndizes_(getContactFrameIndices(pinocchioInterface, mpcRobotModel)),
      mpcRobotModelPtr_(&mpcRobotModel) {
  contactMappers.reserve(N_CONTACTS);
  polygonPointFrameIndizes_.reserve(N_CONTACTS);
  for (int i = 0; i < N_CONTACTS; i++) {
    ContactWrenchMapper<N_CONTACT_POLYGON_POINTS> contactWrenchMapper(
        ContactRectangle::loadContactRectangle(taskFile, mpcRobotModel.modelSettings, i));
    contactMappers.emplace_back(contactWrenchMapper);
    std::array<pinocchio::FrameIndex, N_CONTACT_POLYGON_POINTS> polygonFootFrameIndizes;
    for (int j = 0; j < N_CONTACT_POLYGON_POINTS; j++) {
      polygonFootFrameIndizes[j] =
          pinocchioInterface.getModel().getFrameId(contactWrenchMapper.contactPolygon_.getPolygonPointFrameName(j));
    }
    polygonPointFrameIndizes_.emplace_back(polygonFootFrameIndizes);
  }
}

visualization_msgs::msg::MarkerArray EquivalentContactCornerForcesVisualizer::generateContactVisualizationForceMarkers(
    const vector_t& input, const contact_flag_t& contactFlags, const scalar_t& forceScale) const {
  visualization_msgs::msg::MarkerArray markerArray;
  markerArray.markers.reserve(N_CONTACTS * N_CONTACT_POLYGON_POINTS);
  const pinocchio::Model& model = pinocchioInterfacePtr_->getModel();
  const pinocchio::Data& data = pinocchioInterfacePtr_->getData();
  for (int i = 0; i < N_CONTACTS; i++) {
    if (contactFlags[i]) {
      // Convert wrench into set of equivalent visualization forces in each corner
      const ContactWrenchMapper<N_CONTACT_POLYGON_POINTS>& currContactWrenchMapper = contactMappers[i];
      vector6_t globalWrench = mpcRobotModelPtr_->getContactWrench(input, i);
      vector6_t localContactWrench = rotateVectorWorldToLocal(globalWrench, data, contactFrameIndizes_[i]);
      auto visualizationForces = currContactWrenchMapper.computeVisualizationForceArray(localContactWrench);
      // Fill marker array with visualization forces
      for (int j = 0; j < N_CONTACT_POLYGON_POINTS; j++) {
        vector3_t footPosition = data.oMf[polygonPointFrameIndizes_[i][j]].translation();
        markerArray.markers.emplace_back(getForceMarker(rotateVectorLocalToWorld(visualizationForces[j], data, contactFrameIndizes_[i]),
                                                        footPosition, contactFlags[i], Color::blue, forceScale,
                                                        "equivalentContactCornerForces"));
      }
    } else {
      for (int j = 0; j < N_CONTACT_POLYGON_POINTS; j++) {
        vector3_t footPosition = data.oMf[polygonPointFrameIndizes_[i][j]].translation();
        markerArray.markers.emplace_back(getForceMarker(vector3_t::Zero(), vector3_t::Zero(), contactFlags[i], Color::blue, forceScale,
                                                        "equivalentContactCornerForces"));
      }
    }
  }
  return markerArray;
}

}  // namespace ocs2::humanoid