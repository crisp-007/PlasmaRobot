#include "plasma_eye_hand/depth_alignment.hpp"

#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <string>

namespace plasma_eye_hand
{
namespace
{

using Vector8d = Eigen::Matrix<double, 8, 1>;
using Matrix8d = Eigen::Matrix<double, 8, 8>;

constexpr double kPi = 3.14159265358979323846;
constexpr double kDegreesToRadians = kPi / 180.0;

Eigen::Matrix3d expRotation(const Eigen::Vector3d & omega)
{
  const double angle = omega.norm();
  if (angle < 1e-14) {
    return Eigen::Matrix3d::Identity();
  }
  return Eigen::AngleAxisd(angle, omega / angle).toRotationMatrix();
}

double rotationAngle(const Eigen::Matrix3d & rotation)
{
  return Eigen::AngleAxisd(rotation).angle();
}

void validateTransform(const Eigen::Isometry3d & transform, const char * name)
{
  if (!transform.matrix().allFinite()) {
    throw std::runtime_error(std::string(name) + " contains a non-finite value");
  }
  if (!transform.linear().isUnitary(1e-6) ||
    std::abs(transform.linear().determinant() - 1.0) > 1e-6)
  {
    throw std::runtime_error(std::string(name) + " has an invalid rotation");
  }
}

void validateInputs(
  const std::vector<DepthPlaneSample> & samples,
  const Eigen::Isometry3d & factory_color_from_depth,
  double raw_depth_unit_m)
{
  if (samples.size() < 6) {
    throw std::runtime_error("depth alignment requires at least six plane groups");
  }
  if (!std::isfinite(raw_depth_unit_m) || raw_depth_unit_m <= 0.0) {
    throw std::runtime_error("raw depth unit must be positive and finite");
  }
  validateTransform(factory_color_from_depth, "factory color-from-depth transform");
  std::size_t ray_count = 0;
  for (const auto & sample : samples) {
    validateTransform(sample.color_from_board, "color-from-board transform");
    if (sample.rays.size() < 100) {
      throw std::runtime_error("depth plane sample has fewer than 100 rays: " + sample.name);
    }
    ray_count += sample.rays.size();
    for (const auto & ray : sample.rays) {
      if (!std::isfinite(ray.x) || !std::isfinite(ray.y) ||
        !std::isfinite(ray.raw_depth) || ray.raw_depth <= 0.0)
      {
        throw std::runtime_error("depth plane sample contains an invalid ray: " + sample.name);
      }
    }
  }
  if (ray_count < 1000) {
    throw std::runtime_error("depth alignment requires at least 1000 rays");
  }
}

DepthAlignmentParameters applyIncrement(
  const DepthAlignmentParameters & parameters, const Vector8d & increment)
{
  DepthAlignmentParameters result = parameters;
  result.depth_scale_factor += increment[0];
  result.ray_bias_m += increment[1] * 0.001;
  Eigen::Isometry3d correction_increment = Eigen::Isometry3d::Identity();
  correction_increment.translation() = increment.segment<3>(2) * 0.001;
  correction_increment.linear() = expRotation(increment.segment<3>(5) * kDegreesToRadians);
  result.correction = correction_increment * parameters.correction;
  return result;
}

bool withinBounds(
  const DepthAlignmentParameters & parameters,
  const DepthAlignmentFitOptions & options)
{
  return std::isfinite(parameters.depth_scale_factor) &&
         std::abs(parameters.depth_scale_factor - 1.0) <= options.maximum_scale_deviation &&
         std::isfinite(parameters.ray_bias_m) &&
         std::abs(parameters.ray_bias_m) <= options.maximum_bias_m &&
         parameters.correction.matrix().allFinite() &&
         parameters.correction.translation().norm() <= options.maximum_correction_translation_m &&
         rotationAngle(parameters.correction.linear()) <=
         options.maximum_correction_rotation_rad;
}

double huberLoss(double residual_m, double delta_m)
{
  const double absolute = std::abs(residual_m);
  if (absolute <= delta_m) {
    return 0.5 * residual_m * residual_m;
  }
  return delta_m * (absolute - 0.5 * delta_m);
}

double huberWeight(double residual_m, double delta_m)
{
  const double absolute = std::abs(residual_m);
  return absolute <= delta_m ? 1.0 : delta_m / absolute;
}

double objective(
  const std::vector<DepthPlaneSample> & samples,
  const Eigen::Isometry3d & factory_color_from_depth,
  double raw_depth_unit_m,
  const DepthAlignmentParameters & parameters,
  double huber_delta_m)
{
  double value = 0.0;
  for (const auto & sample : samples) {
    for (const auto & ray : sample.rays) {
      value += huberLoss(signedPlaneResidual(
          sample, ray, factory_color_from_depth, raw_depth_unit_m, parameters),
        huber_delta_m);
    }
  }
  return value;
}

std::vector<double> collectResiduals(
  const std::vector<DepthPlaneSample> & samples,
  const Eigen::Isometry3d & factory_color_from_depth,
  double raw_depth_unit_m,
  const DepthAlignmentParameters & parameters)
{
  std::size_t count = 0;
  for (const auto & sample : samples) {
    count += sample.rays.size();
  }
  std::vector<double> residuals;
  residuals.reserve(count);
  for (const auto & sample : samples) {
    for (const auto & ray : sample.rays) {
      residuals.push_back(signedPlaneResidual(
          sample, ray, factory_color_from_depth, raw_depth_unit_m, parameters));
    }
  }
  return residuals;
}

double median(std::vector<double> values)
{
  if (values.empty()) {
    return 0.0;
  }
  const std::size_t middle = values.size() / 2;
  std::nth_element(values.begin(), values.begin() + middle, values.end());
  const double upper = values[middle];
  if (values.size() % 2 != 0) {
    return upper;
  }
  std::nth_element(values.begin(), values.begin() + middle - 1, values.begin() + middle);
  return 0.5 * (values[middle - 1] + upper);
}

DepthResidualStats calculateStats(const std::vector<double> & residuals)
{
  DepthResidualStats stats;
  stats.count = residuals.size();
  if (residuals.empty()) {
    return stats;
  }
  std::vector<double> absolute;
  absolute.reserve(residuals.size());
  double signed_sum = 0.0;
  double absolute_sum = 0.0;
  double squared_sum = 0.0;
  for (const double residual : residuals) {
    const double value = std::abs(residual);
    signed_sum += residual;
    absolute_sum += value;
    squared_sum += residual * residual;
    absolute.push_back(value);
    stats.max_abs_m = std::max(stats.max_abs_m, value);
  }
  const double count = static_cast<double>(residuals.size());
  stats.mean_signed_m = signed_sum / count;
  stats.median_signed_m = median(residuals);
  stats.mean_abs_m = absolute_sum / count;
  stats.median_abs_m = median(absolute);
  stats.rms_m = std::sqrt(squared_sum / count);
  return stats;
}

Matrix8d calculateNormal(
  const std::vector<DepthPlaneSample> & samples,
  const Eigen::Isometry3d & factory_color_from_depth,
  double raw_depth_unit_m,
  const DepthAlignmentParameters & parameters,
  double huber_delta_m,
  Vector8d * gradient)
{
  Matrix8d normal = Matrix8d::Zero();
  gradient->setZero();
  const Eigen::Matrix3d combined_rotation =
    parameters.correction.linear() * factory_color_from_depth.linear();
  for (const auto & sample : samples) {
    const Eigen::Vector3d plane_normal = sample.color_from_board.linear().col(2);
    for (const auto & ray : sample.rays) {
      const Eigen::Vector3d direction(ray.x, ray.y, 1.0);
      const Eigen::Vector3d unit_direction = direction.normalized();
      const Eigen::Vector3d point = correctedDepthPoint(
        ray, factory_color_from_depth, raw_depth_unit_m, parameters);
      const double residual_m =
        plane_normal.dot(point - sample.color_from_board.translation());
      Vector8d jacobian;
      // Parameter units are scale factor, millimetres, millimetres and degrees.
      jacobian[0] = 1000.0 * plane_normal.dot(
        combined_rotation * direction * ray.raw_depth * raw_depth_unit_m);
      jacobian[1] = plane_normal.dot(combined_rotation * unit_direction);
      jacobian.segment<3>(2) = plane_normal;
      jacobian.segment<3>(5) =
        point.cross(plane_normal) * 1000.0 * kDegreesToRadians;
      const double residual_mm = residual_m * 1000.0;
      const double weight = huberWeight(residual_m, huber_delta_m);
      normal.noalias() += weight * jacobian * jacobian.transpose();
      gradient->noalias() += weight * jacobian * residual_mm;
    }
  }
  return normal;
}

double conditionNumber(const Matrix8d & normal)
{
  const Eigen::SelfAdjointEigenSolver<Matrix8d> solver(normal);
  if (solver.info() != Eigen::Success) {
    return std::numeric_limits<double>::infinity();
  }
  const double minimum = solver.eigenvalues().minCoeff();
  const double maximum = solver.eigenvalues().maxCoeff();
  if (minimum <= std::numeric_limits<double>::epsilon() * maximum) {
    return std::numeric_limits<double>::infinity();
  }
  return maximum / minimum;
}

}  // namespace

Eigen::Vector3d correctedDepthPoint(
  const DepthRay & ray,
  const Eigen::Isometry3d & factory_color_from_depth,
  double raw_depth_unit_m,
  const DepthAlignmentParameters & parameters)
{
  const Eigen::Vector3d direction(ray.x, ray.y, 1.0);
  const double nominal_depth =
    ray.raw_depth * raw_depth_unit_m * parameters.depth_scale_factor;
  const Eigen::Vector3d depth_point =
    direction * nominal_depth + direction.normalized() * parameters.ray_bias_m;
  return parameters.correction * (factory_color_from_depth * depth_point);
}

double signedPlaneResidual(
  const DepthPlaneSample & sample,
  const DepthRay & ray,
  const Eigen::Isometry3d & factory_color_from_depth,
  double raw_depth_unit_m,
  const DepthAlignmentParameters & parameters)
{
  const Eigen::Vector3d normal = sample.color_from_board.linear().col(2);
  const Eigen::Vector3d point = correctedDepthPoint(
    ray, factory_color_from_depth, raw_depth_unit_m, parameters);
  return normal.dot(point - sample.color_from_board.translation());
}

DepthResidualStats evaluateDepthAlignment(
  const std::vector<DepthPlaneSample> & samples,
  const Eigen::Isometry3d & factory_color_from_depth,
  double raw_depth_unit_m,
  const DepthAlignmentParameters & parameters)
{
  return calculateStats(collectResiduals(
      samples, factory_color_from_depth, raw_depth_unit_m, parameters));
}

DepthAlignmentFitResult fitDepthAlignment(
  const std::vector<DepthPlaneSample> & samples,
  const Eigen::Isometry3d & factory_color_from_depth,
  double raw_depth_unit_m,
  const DepthAlignmentFitOptions & options)
{
  validateInputs(samples, factory_color_from_depth, raw_depth_unit_m);
  if (options.max_iterations <= 0 || options.huber_delta_m <= 0.0) {
    throw std::runtime_error("depth alignment fit options are invalid");
  }

  DepthAlignmentFitResult result;
  double lambda = 1e-3;
  double current_objective = objective(
    samples, factory_color_from_depth, raw_depth_unit_m,
    result.parameters, options.huber_delta_m);

  for (int iteration = 0; iteration < options.max_iterations; ++iteration) {
    Vector8d gradient;
    const Matrix8d normal = calculateNormal(
      samples, factory_color_from_depth, raw_depth_unit_m,
      result.parameters, options.huber_delta_m, &gradient);
    bool accepted = false;
    Vector8d accepted_increment = Vector8d::Zero();
    for (int attempt = 0; attempt < 12; ++attempt) {
      Matrix8d damped = normal;
      damped.diagonal().array() += lambda * normal.diagonal().array().max(1e-9);
      const Vector8d increment = -damped.ldlt().solve(gradient);
      if (!increment.allFinite()) {
        lambda *= 10.0;
        continue;
      }
      const DepthAlignmentParameters candidate = applyIncrement(result.parameters, increment);
      if (!withinBounds(candidate, options)) {
        lambda *= 10.0;
        continue;
      }
      const double candidate_objective = objective(
        samples, factory_color_from_depth, raw_depth_unit_m,
        candidate, options.huber_delta_m);
      if (candidate_objective < current_objective) {
        result.parameters = candidate;
        current_objective = candidate_objective;
        accepted_increment = increment;
        lambda = std::max(1e-12, lambda * 0.3);
        accepted = true;
        break;
      }
      lambda *= 10.0;
    }
    result.iterations = iteration + 1;
    if (!accepted) {
      break;
    }
    if (accepted_increment.norm() < 1e-7) {
      result.converged = true;
      break;
    }
  }

  Vector8d final_gradient;
  const Matrix8d final_normal = calculateNormal(
    samples, factory_color_from_depth, raw_depth_unit_m,
    result.parameters, options.huber_delta_m, &final_gradient);
  result.normal_condition_number = conditionNumber(final_normal);
  result.residuals = evaluateDepthAlignment(
    samples, factory_color_from_depth, raw_depth_unit_m, result.parameters);
  return result;
}

}  // namespace plasma_eye_hand
