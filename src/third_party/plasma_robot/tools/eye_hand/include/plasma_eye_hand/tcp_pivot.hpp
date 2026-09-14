#pragma once

#include <Eigen/Geometry>

#include <cstddef>
#include <vector>

namespace plasma_eye_hand
{

struct PivotSample
{
  Eigen::Isometry3d base_to_flange = Eigen::Isometry3d::Identity();
};

struct PivotResult
{
  Eigen::Vector3d flange_to_tcp = Eigen::Vector3d::Zero();
  Eigen::Vector3d fixed_point_in_base = Eigen::Vector3d::Zero();
  std::vector<double> residual_norms_m;
  double mean_residual_m = 0.0;
  double rms_residual_m = 0.0;
  double max_residual_m = 0.0;
  double condition_number = 0.0;
  std::size_t rank = 0;
};

PivotResult solveTcpPivot(const std::vector<PivotSample> & samples);

PivotResult evaluateTcpOffset(
  const std::vector<PivotSample> & samples,
  const Eigen::Vector3d & flange_to_tcp);

}  // namespace plasma_eye_hand
