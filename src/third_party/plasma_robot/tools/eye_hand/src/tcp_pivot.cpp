#include "plasma_eye_hand/tcp_pivot.hpp"

#include <Eigen/SVD>

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <stdexcept>

namespace plasma_eye_hand
{
namespace
{

void finishResidualStatistics(PivotResult * result)
{
  if (result->residual_norms_m.empty()) {
    throw std::runtime_error("pivot result has no residuals");
  }

  const double sum = std::accumulate(
    result->residual_norms_m.begin(), result->residual_norms_m.end(), 0.0);
  const double sum_squares = std::inner_product(
    result->residual_norms_m.begin(), result->residual_norms_m.end(),
    result->residual_norms_m.begin(), 0.0);
  result->mean_residual_m = sum / static_cast<double>(result->residual_norms_m.size());
  result->rms_residual_m = std::sqrt(
    sum_squares / static_cast<double>(result->residual_norms_m.size()));
  result->max_residual_m = *std::max_element(
    result->residual_norms_m.begin(), result->residual_norms_m.end());
}

void validateSamples(const std::vector<PivotSample> & samples)
{
  if (samples.size() < 3) {
    throw std::runtime_error("TCP pivot requires at least 3 samples; use 6-10 varied poses");
  }
  for (const auto & sample : samples) {
    if (!sample.base_to_flange.matrix().allFinite()) {
      throw std::runtime_error("TCP pivot sample contains a non-finite transform");
    }
    const Eigen::Matrix3d rotation = sample.base_to_flange.linear();
    if (!rotation.isUnitary(1e-6) || rotation.determinant() < 0.999999) {
      throw std::runtime_error("TCP pivot sample rotation is not a proper rotation matrix");
    }
  }
}

}  // namespace

PivotResult solveTcpPivot(const std::vector<PivotSample> & samples)
{
  validateSamples(samples);

  Eigen::MatrixXd system(3 * samples.size(), 6);
  Eigen::VectorXd right_hand_side(3 * samples.size());
  for (std::size_t index = 0; index < samples.size(); ++index) {
    const auto row = static_cast<Eigen::Index>(3 * index);
    system.block<3, 3>(row, 0) = samples[index].base_to_flange.linear();
    system.block<3, 3>(row, 3) = -Eigen::Matrix3d::Identity();
    right_hand_side.segment<3>(row) = -samples[index].base_to_flange.translation();
  }

  Eigen::JacobiSVD<Eigen::MatrixXd> svd(
    system, Eigen::ComputeThinU | Eigen::ComputeThinV);
  const double tolerance = std::numeric_limits<double>::epsilon() *
    static_cast<double>(std::max(system.rows(), system.cols())) *
    svd.singularValues()(0);
  const std::size_t rank = static_cast<std::size_t>(
    (svd.singularValues().array() > tolerance).count());
  if (rank < 6) {
    throw std::runtime_error(
      "TCP pivot poses are rank deficient; change wrist orientation around multiple axes");
  }

  const Eigen::VectorXd solution = svd.solve(right_hand_side);
  PivotResult result;
  result.flange_to_tcp = solution.head<3>();
  result.fixed_point_in_base = solution.tail<3>();
  result.rank = rank;
  result.condition_number = svd.singularValues()(0) /
    svd.singularValues()(svd.singularValues().size() - 1);
  if (!std::isfinite(result.condition_number) || result.condition_number > 1e6) {
    throw std::runtime_error(
      "TCP pivot poses are numerically ill-conditioned; use larger rotations around multiple axes");
  }
  result.residual_norms_m.reserve(samples.size());
  for (const auto & sample : samples) {
    const Eigen::Vector3d tcp = sample.base_to_flange * result.flange_to_tcp;
    result.residual_norms_m.push_back((tcp - result.fixed_point_in_base).norm());
  }
  finishResidualStatistics(&result);
  return result;
}

PivotResult evaluateTcpOffset(
  const std::vector<PivotSample> & samples,
  const Eigen::Vector3d & flange_to_tcp)
{
  validateSamples(samples);
  if (!flange_to_tcp.allFinite()) {
    throw std::runtime_error("TCP offset contains a non-finite value");
  }

  PivotResult result;
  result.flange_to_tcp = flange_to_tcp;
  for (const auto & sample : samples) {
    result.fixed_point_in_base += sample.base_to_flange * flange_to_tcp;
  }
  result.fixed_point_in_base /= static_cast<double>(samples.size());
  result.residual_norms_m.reserve(samples.size());
  for (const auto & sample : samples) {
    const Eigen::Vector3d tcp = sample.base_to_flange * flange_to_tcp;
    result.residual_norms_m.push_back((tcp - result.fixed_point_in_base).norm());
  }
  finishResidualStatistics(&result);
  return result;
}

}  // namespace plasma_eye_hand
