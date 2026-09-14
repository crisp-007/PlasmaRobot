#include "plasma_eye_hand/depth_alignment.hpp"

#include <gtest/gtest.h>

#include <Eigen/Geometry>

#include <cmath>
#include <vector>

namespace
{

constexpr double kPi = 3.14159265358979323846;

Eigen::Matrix3d expRotation(const Eigen::Vector3d & omega)
{
  const double angle = omega.norm();
  return angle < 1e-14 ? Eigen::Matrix3d::Identity() :
         Eigen::AngleAxisd(angle, omega / angle).toRotationMatrix();
}

std::vector<plasma_eye_hand::DepthPlaneSample> makeSyntheticSamples(
  const Eigen::Isometry3d & factory,
  const plasma_eye_hand::DepthAlignmentParameters & truth,
  double raw_unit)
{
  std::vector<plasma_eye_hand::DepthPlaneSample> samples;
  const std::vector<Eigen::Vector3d> normals = {
    {0.00, 0.00, -1.00}, {0.15, 0.02, -1.00}, {-0.14, 0.03, -1.00},
    {0.03, 0.16, -1.00}, {-0.02, -0.15, -1.00}, {0.10, -0.12, -1.00},
    {-0.12, -0.10, -1.00}, {0.17, 0.09, -1.00}, {-0.16, 0.12, -1.00}};
  const std::vector<Eigen::Vector3d> origins = {
    {0.00, 0.00, 0.35}, {-0.08, 0.04, 0.38}, {0.09, -0.03, 0.41},
    {0.01, 0.07, 0.49}, {-0.07, -0.04, 0.52}, {0.08, 0.03, 0.55},
    {0.00, -0.06, 0.63}, {-0.09, 0.02, 0.67}, {0.09, -0.02, 0.70}};
  const Eigen::Matrix3d combined_rotation = truth.correction.linear() * factory.linear();
  const Eigen::Vector3d combined_translation =
    truth.correction.linear() * factory.translation() + truth.correction.translation();

  for (std::size_t index = 0; index < normals.size(); ++index) {
    plasma_eye_hand::DepthPlaneSample sample;
    sample.name = "sample_" + std::to_string(index + 1);
    const Eigen::Vector3d normal = normals[index].normalized();
    sample.color_from_board.linear() =
      Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitZ(), normal).toRotationMatrix();
    sample.color_from_board.translation() = origins[index];
    for (int row = -5; row <= 5; ++row) {
      for (int column = -5; column <= 5; ++column) {
        const Eigen::Vector3d direction(column * 0.025, row * 0.025, 1.0);
        const Eigen::Vector3d unit_direction = direction.normalized();
        const double numerator = normal.dot(
          origins[index] - combined_translation -
          combined_rotation * unit_direction * truth.ray_bias_m);
        const double denominator = normal.dot(combined_rotation * direction);
        const double nominal_depth = numerator / denominator;
        plasma_eye_hand::DepthRay ray;
        ray.x = direction.x();
        ray.y = direction.y();
        ray.raw_depth = nominal_depth / (raw_unit * truth.depth_scale_factor);
        sample.rays.push_back(ray);
      }
    }
    samples.push_back(std::move(sample));
  }
  return samples;
}

TEST(DepthAlignment, RecoversSyntheticCombinedGeometry)
{
  const double raw_unit = 0.001;
  Eigen::Isometry3d factory = Eigen::Isometry3d::Identity();
  factory.linear() = expRotation(Eigen::Vector3d(0.01, -0.006, 0.004));
  factory.translation() = Eigen::Vector3d(0.001, -0.012, 0.004);
  plasma_eye_hand::DepthAlignmentParameters truth;
  truth.depth_scale_factor = 1.004;
  truth.ray_bias_m = -0.006;
  truth.correction.linear() = expRotation(Eigen::Vector3d(0.002, -0.003, 0.001));
  truth.correction.translation() = Eigen::Vector3d(0.002, -0.001, 0.003);
  const auto samples = makeSyntheticSamples(factory, truth, raw_unit);

  const auto baseline = plasma_eye_hand::evaluateDepthAlignment(
    samples, factory, raw_unit, plasma_eye_hand::DepthAlignmentParameters());
  const auto fit = plasma_eye_hand::fitDepthAlignment(samples, factory, raw_unit);

  EXPECT_GT(baseline.median_abs_m, 0.0005);
  EXPECT_LT(fit.residuals.rms_m, 1e-7);
  EXPECT_LT(fit.residuals.median_abs_m, baseline.median_abs_m * 0.01);
  EXPECT_NEAR(fit.parameters.depth_scale_factor, truth.depth_scale_factor, 2e-5);
  EXPECT_NEAR(fit.parameters.ray_bias_m, truth.ray_bias_m, 2e-5);
  const Eigen::Isometry3d expected = truth.correction * factory;
  const Eigen::Isometry3d actual = fit.parameters.correction * factory;
  EXPECT_LT((expected.inverse() * actual).translation().norm(), 2e-5);
  EXPECT_LT(Eigen::AngleAxisd((expected.inverse() * actual).linear()).angle(), 2e-5);
}

TEST(DepthAlignment, RejectsInsufficientGroups)
{
  std::vector<plasma_eye_hand::DepthPlaneSample> samples(5);
  EXPECT_THROW(
    plasma_eye_hand::fitDepthAlignment(
      samples, Eigen::Isometry3d::Identity(), 0.001),
    std::runtime_error);
}

}  // namespace
