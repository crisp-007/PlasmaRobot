#pragma once

#include <Eigen/Geometry>

#include <vector>

namespace plasma_eye_hand
{

struct ReachRefinementSample
{
  Eigen::Isometry3d base_to_target = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d source_to_target = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d base_to_observed = Eigen::Isometry3d::Identity();
};

struct ReachRefinementResult
{
  // Applied as: T_base_source_corrected = T_base_source * source_correction.
  Eigen::Isometry3d source_correction = Eigen::Isometry3d::Identity();
  Eigen::Vector3d singular_values = Eigen::Vector3d::Zero();
  std::vector<double> residual_m;
  double rms_residual_m = 0.0;
  double max_residual_m = 0.0;
};

ReachRefinementResult fitSourceCorrection(
  const std::vector<ReachRefinementSample> & samples);

ReachRefinementResult fitSourceTranslationCorrection(
  const std::vector<ReachRefinementSample> & samples);

}  // namespace plasma_eye_hand
