/* Computational Legged Robots: Planning and Control  */

#include "wbDynPlan/common/ModelSettings.h"

#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include <stdexcept>

#include <ocs2_core/misc/LoadData.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>

#include "wbDynPlan/pinocchio_model/createPinocchioModel.h"

namespace ocs2::humanoid {

/******************************************************************************************************/
/// Helper functions contained in a local anonymous namespace
/******************************************************************************************************/
namespace {

/**
 * @brief Creates a joint Index map from a list of joint names.
 */

static std::unordered_map<std::string, size_t> createJointIndexMap(const std::vector<std::string>& jointNames, size_t offset = 0) {
  std::unordered_map<std::string, size_t> jointIndexMap;
  for (size_t i = 0; i < jointNames.size(); ++i) {
    jointIndexMap[jointNames[i]] = i + offset;
  }
  return jointIndexMap;
}

static std::vector<std::string> initializeJointNames(const std::vector<std::string>& fullJointNames,
                                                     const std::vector<std::string>& fixedJointNames,
                                                     bool verbose) {
  if (verbose) std::cout << "Initialize the following active MPC joints: " << std::endl;
  size_t n_joints = fullJointNames.size() - fixedJointNames.size();
  if (verbose) std::cout << "Num active joints: " << n_joints << std::endl;
  std::vector<std::string> mpcModelJointNames;
  if (n_joints > 0) {
    mpcModelJointNames.reserve(n_joints);
  } else {
    throw std::invalid_argument("Number of joints must be greater than zero");
  }
  for (const auto& joint : fullJointNames) {
    if (std::find(fixedJointNames.begin(), fixedJointNames.end(), joint) == fixedJointNames.end()) {
      // If the joint is not found in fixedJointNames, add it to mpcModelJointNames
      if (verbose) std::cout << joint << std::endl;
      mpcModelJointNames.emplace_back(joint);
    }
  }
  return mpcModelJointNames;
}

std::vector<size_t> initializeMpcToFullJointIndices(const std::vector<std::string>& fullJointNames,
                                                    const std::vector<std::string>& mpcModelJointNames) {
  std::unordered_map<std::string, size_t> fullJointIndexMap = createJointIndexMap(fullJointNames);
  std::vector<size_t> mpcModelJointIndices;
  mpcModelJointIndices.reserve(mpcModelJointNames.size());
  for (size_t i = 0; i < mpcModelJointNames.size(); ++i) {
    mpcModelJointIndices[i] = fullJointIndexMap[mpcModelJointNames[i]];
  }
  return mpcModelJointIndices;
}

std::vector<std::string> concatenateStringVectors(const std::vector<std::string>& a, const std::vector<std::string>& b) {
  std::vector<std::string> temp_vec(a);
  temp_vec.insert(temp_vec.begin(), b.begin(), b.end());
  return temp_vec;
}

}  // namespace

ModelSettings::ModelSettings(const std::string& configFile, const std::string& urdfFile, const std::string& mpcName, bool verbose) {
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(configFile, pt);

  std::string prefix{"model_settings."};

  if (verbose) {
    std::cerr << "\n #### Robot Model Settings:";
    std::cerr << "\n #### =============================================================================\n";
  }

  loadData::loadPtreeValue(pt, this->robotName, prefix + "robotName", verbose);
  loadData::loadPtreeValue(pt, this->verboseCppAd, prefix + "verboseCppAd", verbose);
  loadData::loadPtreeValue(pt, this->recompileLibrariesCppAd, prefix + "recompileLibrariesCppAd", verbose);
  loadData::loadPtreeValue(pt, this->phaseTransitionStanceTime, prefix + "phaseTransitionStanceTime", verbose);

  loadData::loadPtreeValue(pt, this->j_l_shoulder_y_name, prefix + "armJointNames.left_shoulder_y", verbose);
  loadData::loadPtreeValue(pt, this->j_r_shoulder_y_name, prefix + "armJointNames.right_shoulder_y", verbose);
  loadData::loadPtreeValue(pt, this->j_l_elbow_y_name, prefix + "armJointNames.left_elbow_y", verbose);
  loadData::loadPtreeValue(pt, this->j_r_elbow_y_name, prefix + "armJointNames.right_elbow_y", verbose);
  modelFolderCppAd = "cppad_code_gen/cppad_" + mpcName + robotName;

  loadData::loadStdVector(configFile, prefix + "fixedJointNames", fixedJointNames, verbose);
  loadData::loadStdVector(configFile, prefix + "contactNames6DoF", contactNames6DoF, verbose);
  loadData::loadStdVector(configFile, prefix + "contactParentJointNames", contactParentJointNames, verbose);

  if (verbose) {
    std::cout << "Initializing MPC by fixing joints: " << std::endl;
    for (std::string fixedJoint : fixedJointNames) std::cout << fixedJoint << std::endl;
  }

  // Get full joint order from a full pinocchio interface, this removes any joints marked as fix in the urdf.
  PinocchioInterface fullPinocchioInterface = createDefaultPinocchioInterface(urdfFile);
  const pinocchio::Model& model = fullPinocchioInterface.getModel();
  if (verbose) std::cout << "Full URDF joints: " << std::endl;
  fullJointNames.reserve(model.njoints - 2);  // Substract universe and root joint
  for (pinocchio::JointIndex joint_id = 2; joint_id < (pinocchio::JointIndex)model.njoints; ++joint_id) {
    if (verbose) std::cout << model.names[joint_id] << std::endl;
    fullJointNames.emplace_back(model.names[joint_id]);
  }

  this->mpcModelJointNames = initializeJointNames(this->fullJointNames, this->fixedJointNames, verbose);
  this->mpcModelToFullJointsIndices = initializeMpcToFullJointIndices(this->fullJointNames, this->mpcModelJointNames);
  this->jointIndexMap = createJointIndexMap(this->mpcModelJointNames);
  this->contactNames = concatenateStringVectors(this->contactNames3DoF, this->contactNames6DoF);

  this->mpc_joint_dim = this->mpcModelJointNames.size();
  this->full_joint_dim = this->fullJointNames.size();

  j_l_shoulder_y_index = this->jointIndexMap.at(j_l_shoulder_y_name);
  j_r_shoulder_y_index = this->jointIndexMap.at(j_r_shoulder_y_name);
  j_l_elbow_y_index = this->jointIndexMap.at(j_l_elbow_y_name);
  j_r_elbow_y_index = this->jointIndexMap.at(j_r_elbow_y_name);

  const std::string footConstraintPrefix = prefix + "foot_constraint.";

  if (verbose) {
    std::cerr << "\n #### Robot Model Foot Constraint Config:";
    std::cerr << "\n #### =============================================================================\n";
  }

  loadData::loadPtreeValue(pt, this->footConstraintConfig.positionErrorGain_z, footConstraintPrefix + "positionErrorGain_z", verbose);
  loadData::loadPtreeValue(pt, this->footConstraintConfig.orientationErrorGain, footConstraintPrefix + "orientationErrorGain", verbose);
  loadData::loadPtreeValue(pt, this->footConstraintConfig.linearVelocityErrorGain_z, footConstraintPrefix + "linearVelocityErrorGain_z",
                           verbose);
  loadData::loadPtreeValue(pt, this->footConstraintConfig.linearVelocityErrorGain_xy, footConstraintPrefix + "linearVelocityErrorGain_xy",
                           verbose);
  loadData::loadPtreeValue(pt, this->footConstraintConfig.angularVelocityErrorGain, footConstraintPrefix + "angularVelocityErrorGain",
                           verbose);
  loadData::loadPtreeValue(pt, this->footConstraintConfig.linearAccelerationErrorGain_z,
                           footConstraintPrefix + "linearAccelerationErrorGain_z", verbose);
  loadData::loadPtreeValue(pt, this->footConstraintConfig.linearAccelerationErrorGain_xy,
                           footConstraintPrefix + "linearAccelerationErrorGain_xy", verbose);
  loadData::loadPtreeValue(pt, this->footConstraintConfig.angularAccelerationErrorGain,
                           footConstraintPrefix + "angularAccelerationErrorGain", verbose);

  if (verbose) {
    std::cerr << " #### =============================================================================" << std::endl;
    std::cerr << " #### =============================================================================" << std::endl;
  }
}

}  // namespace ocs2::humanoid
