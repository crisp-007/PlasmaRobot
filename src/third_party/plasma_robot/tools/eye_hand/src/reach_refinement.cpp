#include "plasma_eye_hand/reach_refinement.hpp"

#include <Eigen/SVD>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

namespace plasma_eye_hand
{
namespace
{

void validateTransform(const Eigen::Isometry3d & transform, const char * name)
{
  if (!transform.matrix().allFinite()) {
    throw std::runtime_error(std::string(name) + " contains a non-finite value");
  }
  const Eigen::Matrix3d rotation = transform.linear();
  if (!rotation.isUnitary(1e-6) || std::abs(rotation.determinant() - 1.0) > 1e-6) {
    throw std::runtime_error(std::string(name) + " has an invalid rotation");
  }
}

void collectSourcePointPairs(
  const std::vector<ReachRefinementSample> & samples,
  std::vector<Eigen::Vector3d> * extracted_points,
  std::vector<Eigen::Vector3d> * required_points)
{
  if (samples.size() < 3) {
    throw std::runtime_error("source correction requires at least three samples");
  }
  extracted_points->clear();
  required_points->clear();
  extracted_points->reserve(samples.size());
  required_points->reserve(samples.size());
  for (const auto & sample : samples) {
    validateTransform(sample.base_to_target, "base-to-target transform");
    validateTransform(sample.source_to_target, "source-to-target transform");
    validateTransform(sample.base_to_observed, "base-to-observed transform");
    const Eigen::Isometry3d base_to_source =
      sample.base_to_target * sample.source_to_target.inverse();
    extracted_points->push_back(sample.source_to_target.translation());
    required_points->push_back(
      base_to_source.inverse() * sample.base_to_observed.translation());
  }
}

void calculateResiduals(
  const std::vector<Eigen::Vector3d> & extracted_points,
  const std::vector<Eigen::Vector3d> & required_points,
  ReachRefinementResult * result)
{
  double squared_residual_sum = 0.0;
  for (std::size_t i = 0; i < extracted_points.size(); ++i) {
    const double residual = (
      result->source_correction * extracted_points[i] - required_points[i]).norm();
    result->residual_m.push_back(residual);
    squared_residual_sum += residual * residual;
    result->max_residual_m = std::max(result->max_residual_m, residual);
  }
  result->rms_residual_m = std::sqrt(
    squared_residual_sum / static_cast<double>(extracted_points.size()));
}

}  // namespace

ReachRefinementResult fitSourceCorrection(
  const std::vector<ReachRefinementSample> & samples)
{
  std::vector<Eigen::Vector3d> extracted_points;
  std::vector<Eigen::Vector3d> required_points;
  collectSourcePointPairs(samples, &extracted_points, &required_points);
  Eigen::Vector3d extracted_centroid = Eigen::Vector3d::Zero();
  Eigen::Vector3d required_centroid = Eigen::Vector3d::Zero();

  for (std::size_t i = 0; i < samples.size(); ++i) {
    extracted_centroid += extracted_points[i];
    required_centroid += required_points[i];
  }

  const double count = static_cast<double>(samples.size());
  extracted_centroid /= count;
  required_centroid /= count;
  Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
  for (std::size_t i = 0; i < samples.size(); ++i) {
    covariance += (extracted_points[i] - extracted_centroid) *
      (required_points[i] - required_centroid).transpose();
  }

  Eigen::JacobiSVD<Eigen::Matrix3d> svd(
    covariance, Eigen::ComputeFullU | Eigen::ComputeFullV);
  if (svd.singularValues()(1) < 1e-10) {
    throw std::runtime_error(
            "source correction samples are collinear or have insufficient pose diversity");
  }
  Eigen::Matrix3d v = svd.matrixV();
  Eigen::Matrix3d rotation = v * svd.matrixU().transpose();
  if (rotation.determinant() < 0.0) {
    v.col(2) *= -1.0;
    rotation = v * svd.matrixU().transpose();
  }

  ReachRefinementResult result;
  result.source_correction.linear() = rotation;
  result.source_correction.translation() =
    required_centroid - rotation * extracted_centroid;
  result.singular_values = svd.singularValues();
  calculateResiduals(extracted_points, required_points, &result);
  return result;
}

ReachRefinementResult fitSourceTranslationCorrection(
  const std::vector<ReachRefinementSample> & samples)
{
  std::vector<Eigen::Vector3d> extracted_points;
  std::vector<Eigen::Vector3d> required_points;
  collectSourcePointPairs(samples, &extracted_points, &required_points);

  ReachRefinementResult result;
  for (std::size_t i = 0; i < samples.size(); ++i) {
    result.source_correction.translation() +=
      required_points[i] - extracted_points[i];
  }
  result.source_correction.translation() /= static_cast<double>(samples.size());
  calculateResiduals(extracted_points, required_points, &result);
  return result;
}

}  // namespace plasma_eye_hand
