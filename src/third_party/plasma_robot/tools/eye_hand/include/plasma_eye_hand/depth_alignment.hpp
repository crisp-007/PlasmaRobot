#pragma once

#include <Eigen/Geometry>

#include <cstddef>
#include <string>
#include <vector>

namespace plasma_eye_hand
{

struct DepthRay
{
  double x = 0.0;
  double y = 0.0;
  double raw_depth = 0.0;
};

struct DepthPlaneSample
{
  std::string name;
  // OpenCV solvePnP result: color camera <- ChArUco board.
  Eigen::Isometry3d color_from_board = Eigen::Isometry3d::Identity();
  std::vector<DepthRay> rays;
};

struct DepthAlignmentParameters
{
  double depth_scale_factor = 1.0;
  double ray_bias_m = 0.0;
  // Applied as: corrected_color_from_depth = correction * factory_color_from_depth.
  Eigen::Isometry3d correction = Eigen::Isometry3d::Identity();
};

struct DepthResidualStats
{
  std::size_t count = 0;
  double mean_signed_m = 0.0;
  double median_signed_m = 0.0;
  double mean_abs_m = 0.0;
  double median_abs_m = 0.0;
  double rms_m = 0.0;
  double max_abs_m = 0.0;
};

struct DepthAlignmentFitOptions
{
  int max_iterations = 50;
  double huber_delta_m = 0.002;
  double maximum_scale_deviation = 0.05;
  double maximum_bias_m = 0.030;
  double maximum_correction_translation_m = 0.030;
  double maximum_correction_rotation_rad = 3.0 * 3.14159265358979323846 / 180.0;
};

struct DepthAlignmentFitResult
{
  DepthAlignmentParameters parameters;
  DepthResidualStats residuals;
  int iterations = 0;
  bool converged = false;
  double normal_condition_number = 0.0;
};

Eigen::Vector3d correctedDepthPoint(
  const DepthRay & ray,
  const Eigen::Isometry3d & factory_color_from_depth,
  double raw_depth_unit_m,
  const DepthAlignmentParameters & parameters);

double signedPlaneResidual(
  const DepthPlaneSample & sample,
  const DepthRay & ray,
  const Eigen::Isometry3d & factory_color_from_depth,
  double raw_depth_unit_m,
  const DepthAlignmentParameters & parameters);

DepthResidualStats evaluateDepthAlignment(
  const std::vector<DepthPlaneSample> & samples,
  const Eigen::Isometry3d & factory_color_from_depth,
  double raw_depth_unit_m,
  const DepthAlignmentParameters & parameters);

DepthAlignmentFitResult fitDepthAlignment(
  const std::vector<DepthPlaneSample> & samples,
  const Eigen::Isometry3d & factory_color_from_depth,
  double raw_depth_unit_m,
  const DepthAlignmentFitOptions & options = DepthAlignmentFitOptions());

}  // namespace plasma_eye_hand
