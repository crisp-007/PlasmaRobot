#pragma once

#include <Eigen/Geometry>

#include <cstddef>
#include <vector>

namespace plasma_eye_hand
{

struct ReachError
{
  Eigen::Vector3d translation_m = Eigen::Vector3d::Zero();
  double norm_m = 0.0;
  double signed_axial_m = 0.0;
  double lateral_m = 0.0;
  // Entry qualification compares rod axes; roll about the rod is diagnostic.
  double orientation_rad = 0.0;
  double full_orientation_rad = 0.0;
};

struct ReachLimits
{
  std::size_t min_records = 3;
  double max_norm_m = 0.005;
  double max_abs_axial_m = 0.005;
  double max_lateral_m = 0.003;
  double max_orientation_rad = 0.03490658503988659;
};

struct ReachSummary
{
  std::size_t record_count = 0;
  Eigen::Vector3d mean_translation_m = Eigen::Vector3d::Zero();
  Eigen::Vector3d translation_stddev_m = Eigen::Vector3d::Zero();
  double mean_norm_m = 0.0;
  double max_norm_m = 0.0;
  double max_abs_axial_m = 0.0;
  double max_lateral_m = 0.0;
  double max_orientation_rad = 0.0;
  bool accepted = false;
};

ReachError calculateReachError(
  const Eigen::Isometry3d & target,
  const Eigen::Isometry3d & actual,
  const Eigen::Vector3d & outward_axis);

ReachSummary summarizeReachErrors(
  const std::vector<ReachError> & errors,
  const ReachLimits & limits);

}  // namespace plasma_eye_hand
