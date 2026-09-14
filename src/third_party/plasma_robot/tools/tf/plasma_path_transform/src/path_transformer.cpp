#include "plasma_path_transform/path_transformer.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <filesystem>
#include <stdexcept>

namespace plasma_path_transform
{
namespace
{

bool statusMeansValidated(const YAML::Node & root)
{
  if (root["validated"] && root["validated"].IsScalar()) {
    try {
      return root["validated"].as<bool>();
    } catch (const YAML::Exception &) {
      // Fall through to the textual status fields.
    }
  }

  const auto text_means_validated = [](const YAML::Node & status) {
      if (!status || !status.IsScalar()) {
        return false;
      }
      std::string value = status.as<std::string>();
      std::transform(value.begin(), value.end(), value.begin(),
        [](unsigned char c) {return static_cast<char>(std::tolower(c));});
      return value == "validated" || value == "approved" ||
             value == "production" || value == "production_ready";
    };

  if (text_means_validated(root["status"])) {
    return true;
  }

  const YAML::Node quality = root["quality"];
  if (quality && text_means_validated(quality["status"])) {
    return true;
  }

  const YAML::Node deployment = root["deployment"];
  if (!deployment) {
    return false;
  }
  const YAML::Node quality_accepted = deployment["quality_accepted"];
  if (quality_accepted && quality_accepted.IsScalar()) {
    try {
      if (quality_accepted.as<bool>()) {
        return true;
      }
    } catch (const YAML::Exception &) {
      // Fall through to textual deployment status.
    }
  }
  return text_means_validated(deployment["status"]);
}

void validateFrameConvention(const YAML::Node & root, const std::string & matrix_key)
{
  const YAML::Node convention = root["frame_convention"];
  if (!convention) {
    return;
  }
  if (!convention.IsMap()) {
    throw std::runtime_error("frame_convention must be a YAML map");
  }

  const YAML::Node declared_matrix = convention["matrix"];
  if (declared_matrix && declared_matrix.as<std::string>() != matrix_key) {
    throw std::runtime_error("frame_convention.matrix does not match " + matrix_key);
  }

  if (matrix_key == "T_camera_to_gripper") {
    const YAML::Node meaning = convention["meaning"];
    if (meaning && meaning.as<std::string>() != "gripper_T_camera") {
      throw std::runtime_error(
              "T_camera_to_gripper must mean gripper_T_camera (^gripper T_camera)");
    }
  }

  const YAML::Node units = convention["units"];
  if (units) {
    std::string value = units.as<std::string>();
    std::transform(value.begin(), value.end(), value.begin(),
      [](unsigned char c) {return static_cast<char>(std::tolower(c));});
    if (value != "m" && value != "meter" && value != "meters") {
      throw std::runtime_error("transform translation units must be meters");
    }
  }
}

Eigen::Matrix4d readMatrix(const YAML::Node & node, const std::string & key)
{
  if (!node || !node.IsSequence() || node.size() != 4) {
    throw std::runtime_error(key + " must be a 4x4 YAML sequence");
  }

  Eigen::Matrix4d matrix;
  for (std::size_t row = 0; row < 4; ++row) {
    if (!node[row].IsSequence() || node[row].size() != 4) {
      throw std::runtime_error(key + " must be a 4x4 YAML sequence");
    }
    for (std::size_t column = 0; column < 4; ++column) {
      matrix(static_cast<Eigen::Index>(row), static_cast<Eigen::Index>(column)) =
        node[row][column].as<double>();
    }
  }
  return matrix;
}

}  // namespace

TransformCalibration PathTransformer::loadTransformYaml(
  const std::string & path, const std::string & matrix_key)
{
  if (path.empty()) {
    throw std::runtime_error("calibration path is empty");
  }

  const YAML::Node root = YAML::LoadFile(path);
  if (!root[matrix_key]) {
    throw std::runtime_error("missing YAML key: " + matrix_key);
  }

  validateFrameConvention(root, matrix_key);

  const Eigen::Matrix4d matrix = readMatrix(root[matrix_key], matrix_key);
  std::string reason;
  if (!validateRigidTransform(matrix, &reason)) {
    throw std::runtime_error(matrix_key + " is invalid: " + reason);
  }

  TransformCalibration calibration;
  calibration.transform.matrix() = matrix;
  calibration.loaded = true;
  calibration.validated = statusMeansValidated(root);
  calibration.source = path;
  if (root["calibration_id"] && root["calibration_id"].IsScalar()) {
    calibration.id = root["calibration_id"].as<std::string>();
  } else if (root["name"] && root["name"].IsScalar()) {
    calibration.id = root["name"].as<std::string>();
  } else {
    calibration.id = std::filesystem::path(path).stem().string();
  }
  if (root["source_frame"] && root["source_frame"].IsScalar()) {
    calibration.frame_id = root["source_frame"].as<std::string>();
  }
  return calibration;
}

Eigen::Isometry3d PathTransformer::loadRigidTransformYaml(
  const std::string & path, const std::string & matrix_key)
{
  if (path.empty()) {
    throw std::runtime_error("transform path is empty");
  }
  const YAML::Node root = YAML::LoadFile(path);
  if (!root[matrix_key]) {
    throw std::runtime_error("missing YAML key: " + matrix_key);
  }
  const Eigen::Matrix4d matrix = readMatrix(root[matrix_key], matrix_key);
  std::string reason;
  if (!validateRigidTransform(matrix, &reason)) {
    throw std::runtime_error(matrix_key + " is invalid: " + reason);
  }
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.matrix() = matrix;
  return transform;
}

bool PathTransformer::validateRigidTransform(
  const Eigen::Matrix4d & matrix, std::string * reason)
{
  auto reject = [reason](const std::string & message) {
      if (reason) {
        *reason = message;
      }
      return false;
    };

  if (!matrix.allFinite()) {
    return reject("matrix contains NaN or infinity");
  }

  const Eigen::RowVector4d expected_last_row(0.0, 0.0, 0.0, 1.0);
  if ((matrix.row(3) - expected_last_row).cwiseAbs().maxCoeff() > 1e-8) {
    return reject("last row is not [0, 0, 0, 1]");
  }

  const Eigen::Matrix3d rotation = matrix.block<3, 3>(0, 0);
  const double orthonormal_error =
    (rotation.transpose() * rotation - Eigen::Matrix3d::Identity()).norm();
  if (orthonormal_error > 1e-5) {
    return reject("rotation is not orthonormal");
  }

  const double determinant = rotation.determinant();
  if (std::abs(determinant - 1.0) > 1e-5) {
    return reject("rotation determinant is not +1");
  }

  if (matrix.block<3, 1>(0, 3).norm() > 5.0) {
    return reject("translation exceeds the 5 m sanity limit");
  }

  if (reason) {
    reason->clear();
  }
  return true;
}

Eigen::Isometry3d PathTransformer::poseToIsometry(
  const geometry_msgs::msg::Pose & pose)
{
  const auto & q = pose.orientation;
  Eigen::Quaterniond quaternion(q.w, q.x, q.y, q.z);
  if (!quaternion.coeffs().allFinite() || quaternion.norm() < 1e-9) {
    throw std::runtime_error("pose quaternion is invalid");
  }
  quaternion.normalize();

  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.linear() = quaternion.toRotationMatrix();
  transform.translation() = Eigen::Vector3d(
    pose.position.x, pose.position.y, pose.position.z);
  if (!transform.matrix().allFinite()) {
    throw std::runtime_error("pose contains NaN or infinity");
  }
  return transform;
}

geometry_msgs::msg::Pose PathTransformer::isometryToPose(
  const Eigen::Isometry3d & transform)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = transform.translation().x();
  pose.position.y = transform.translation().y();
  pose.position.z = transform.translation().z();

  Eigen::Quaterniond quaternion(transform.rotation());
  quaternion.normalize();
  pose.orientation.x = quaternion.x();
  pose.orientation.y = quaternion.y();
  pose.orientation.z = quaternion.z();
  pose.orientation.w = quaternion.w();
  return pose;
}

geometry_msgs::msg::Point PathTransformer::transformPoint(
  const Eigen::Isometry3d & transform,
  const geometry_msgs::msg::Point & point)
{
  const Eigen::Vector3d input(point.x, point.y, point.z);
  if (!input.allFinite()) {
    throw std::runtime_error("surface point contains NaN or infinity");
  }
  const Eigen::Vector3d output = transform * input;
  geometry_msgs::msg::Point result;
  result.x = output.x();
  result.y = output.y();
  result.z = output.z();
  return result;
}

Eigen::Isometry3d PathTransformer::translatedAlongAxis(
  const Eigen::Isometry3d & transform,
  const Eigen::Vector3d & axis,
  double distance_m)
{
  if (!transform.matrix().allFinite() || !axis.allFinite() ||
    !std::isfinite(distance_m) || axis.norm() < 1e-12)
  {
    throw std::runtime_error("cannot translate a transform along an invalid axis");
  }

  Eigen::Isometry3d result = transform;
  result.translation() += distance_m * axis.normalized();
  return result;
}

Eigen::Isometry3d PathTransformer::tcpPoseForNozzleTipPose(
  const Eigen::Isometry3d & nozzle_tip_pose,
  const Eigen::Isometry3d & tcp_to_nozzle_tip)
{
  if (!nozzle_tip_pose.matrix().allFinite() ||
    !tcp_to_nozzle_tip.matrix().allFinite())
  {
    throw std::runtime_error("nozzle-tip entry geometry contains NaN or infinity");
  }
  return nozzle_tip_pose * tcp_to_nozzle_tip.inverse();
}

}  // namespace plasma_path_transform
