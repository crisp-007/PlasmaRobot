#include "plasma_eye_hand/reach_validation.hpp"

#include <algorithm>
#include <cmath>
#include <string>
#include <stdexcept>

namespace plasma_eye_hand
{
namespace
{

void validateTransform(const Eigen::Isometry3d & transform, const char * name)
{
  if (!transform.matrix().allFinite()) {
    throw std::runtime_error(std::string(name) + " transform contains a non-finite value");
  }
  const Eigen::Matrix3d rotation = transform.linear();
  if (!rotation.isUnitary(1e-6) || std::abs(rotation.determinant() - 1.0) > 1e-6) {
    throw std::runtime_error(std::string(name) + " transform has an invalid rotation");
  }
}

}  // namespace

ReachError calculateReachError(
  const Eigen::Isometry3d & target,
  const Eigen::Isometry3d & actual,
  const Eigen::Vector3d & outward_axis)
{
  validateTransform(target, "target");
  validateTransform(actual, "actual");
  if (!outward_axis.allFinite() || outward_axis.norm() < 1e-9) {
    throw std::runtime_error("outward axis is invalid");
  }

  const Eigen::Vector3d axis = outward_axis.normalized();
  ReachError error;
  error.translation_m = actual.translation() - target.translation();
  error.norm_m = error.translation_m.norm();
  error.signed_axial_m = error.translation_m.dot(axis);
  error.lateral_m =
    (error.translation_m - error.signed_axial_m * axis).norm();
  const Eigen::Vector3d target_rod_axis = target.linear().col(0).normalized();
  const Eigen::Vector3d actual_rod_axis = actual.linear().col(0).normalized();
  const double rod_axis_dot = std::clamp(
    target_rod_axis.dot(actual_rod_axis), -1.0, 1.0);
  error.orientation_rad = std::acos(rod_axis_dot);
  error.full_orientation_rad = std::abs(Eigen::AngleAxisd(
      target.linear().transpose() * actual.linear()).angle());
  return error;
}

ReachSummary summarizeReachErrors(
  const std::vector<ReachError> & errors,
  const ReachLimits & limits)
{
  if (limits.min_records == 0 || limits.max_norm_m <= 0.0 ||
    limits.max_abs_axial_m <= 0.0 || limits.max_lateral_m <= 0.0 ||
    limits.max_orientation_rad <= 0.0)
  {
    throw std::runtime_error("reach validation limits must be positive");
  }

  ReachSummary summary;
  summary.record_count = errors.size();
  if (errors.empty()) {
    return summary;
  }

  for (const auto & error : errors) {
    if (!error.translation_m.allFinite() || !std::isfinite(error.norm_m) ||
      !std::isfinite(error.signed_axial_m) || !std::isfinite(error.lateral_m) ||
      !std::isfinite(error.orientation_rad))
    {
      throw std::runtime_error("reach validation error contains a non-finite value");
    }
    summary.mean_translation_m += error.translation_m;
    summary.mean_norm_m += error.norm_m;
    summary.max_norm_m = std::max(summary.max_norm_m, error.norm_m);
    summary.max_abs_axial_m = std::max(
      summary.max_abs_axial_m, std::abs(error.signed_axial_m));
    summary.max_lateral_m = std::max(summary.max_lateral_m, error.lateral_m);
    summary.max_orientation_rad = std::max(
      summary.max_orientation_rad, error.orientation_rad);
  }
  const double count = static_cast<double>(errors.size());
  summary.mean_translation_m /= count;
  summary.mean_norm_m /= count;
  for (const auto & error : errors) {
    const Eigen::Vector3d centered = error.translation_m - summary.mean_translation_m;
    summary.translation_stddev_m += centered.cwiseProduct(centered);
  }
  summary.translation_stddev_m = (summary.translation_stddev_m / count).cwiseSqrt();
  summary.accepted = errors.size() >= limits.min_records &&
    summary.max_norm_m <= limits.max_norm_m &&
    summary.max_abs_axial_m <= limits.max_abs_axial_m &&
    summary.max_lateral_m <= limits.max_lateral_m &&
    summary.max_orientation_rad <= limits.max_orientation_rad;
  return summary;
}

}  // namespace plasma_eye_hand
