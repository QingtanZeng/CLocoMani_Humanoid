/* Computational Legged Robots: Planning and Control  */

#pragma once

#include <pinocchio/multibody/fwd.hpp>

#include <visualization_msgs/msg/marker_array.hpp>

#include <ocs2_pinocchio_interface/PinocchioInterface.h>

#include <humanoid_common_mpc/common/ModelSettings.h>
#include <humanoid_common_mpc/common/MpcRobotModelBase.h>
#include <humanoid_common_mpc/contact/ContactWrenchMapper.h>

namespace ocs2::humanoid {

#define N_CONTACT_POLYGON_POINTS 4

class EquivalentContactCornerForcesVisualizer {
 public:
  EquivalentContactCornerForcesVisualizer(const std::string& taskFile,
                                          const PinocchioInterface& pinocchioInterface,
                                          const MpcRobotModelBase<scalar_t>& mpcRobotModel);

  visualization_msgs::msg::MarkerArray generateContactVisualizationForceMarkers(const vector_t& input,
                                                                                const contact_flag_t& contactFlags,
                                                                                const scalar_t& forceScale) const;

 private:
  /// \brief Rotates a 6D contact wrench vector from world into contact frame
  ///
  /// \param[in] contactWrench contact wrench [forces, torques] expressed in world frame
  /// \param[in] R_ContactToWorld Rotation Matrix from contact to world frame
  ///
  vector6_t rotateWrenchToContactFrame(const vector6_t& contactWrench, const matrix3_t R_ContactToWorld) const {
    matrix3_t R_WorldToContact = R_ContactToWorld.inverse();
    matrix6_t R_mat = matrix6_t::Zero();
    R_mat.block(0, 0, 3, 3) = R_WorldToContact;
    R_mat.block(3, 3, 3, 3) = R_WorldToContact;
    return (R_mat * contactWrench);
  };

  /// \brief Rotates a 3D force vector from local contact frame to world frame
  ///
  /// \param[in] contactForce contact force expressed in local contact frame
  /// \param[in] R_ContactToWorld Rotation Matrix from contact to world frame
  ///
  vector3_t rotateForceToWorldFrame(const vector3_t& contactForce, const matrix3_t R_ContactToWorld) const {
    return (R_ContactToWorld * contactForce);
  };
  const std::vector<pinocchio::FrameIndex> contactFrameIndizes_;
  const MpcRobotModelBase<scalar_t>* mpcRobotModelPtr_;
  const PinocchioInterface* pinocchioInterfacePtr_;
  std::vector<ContactWrenchMapper<N_CONTACT_POLYGON_POINTS>> contactMappers;
  std::vector<std::array<pinocchio::FrameIndex, N_CONTACT_POLYGON_POINTS>> polygonPointFrameIndizes_;
};

}  // namespace ocs2::humanoid
