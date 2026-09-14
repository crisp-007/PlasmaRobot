#include "plasma_eye_hand/tcp_pivot.hpp"

#include <gtest/gtest.h>

#include <Eigen/Geometry>

#include <vector>

namespace
{

plasma_eye_hand::PivotSample sampleFor(
  const Eigen::Vector3d & tcp,
  const Eigen::Vector3d & fixed_point,
  const Eigen::Matrix3d & rotation)
{
  plasma_eye_hand::PivotSample sample;
  sample.base_to_flange.linear() = rotation;
  sample.base_to_flange.translation() = fixed_point - rotation * tcp;
  return sample;
}

TEST(TcpPivot, RecoversSyntheticOffset)
{
  const Eigen::Vector3d expected_tcp(0.002, 0.0, 0.302);
  const Eigen::Vector3d fixed_point(0.42, -0.08, 0.31);
  std::vector<plasma_eye_hand::PivotSample> samples;
  samples.push_back(sampleFor(expected_tcp, fixed_point, Eigen::Matrix3d::Identity()));
  samples.push_back(sampleFor(expected_tcp, fixed_point,
    Eigen::AngleAxisd(0.35, Eigen::Vector3d::UnitX()).toRotationMatrix()));
  samples.push_back(sampleFor(expected_tcp, fixed_point,
    Eigen::AngleAxisd(-0.42, Eigen::Vector3d::UnitY()).toRotationMatrix()));
  samples.push_back(sampleFor(expected_tcp, fixed_point,
    Eigen::AngleAxisd(0.50, Eigen::Vector3d(1.0, 1.0, 0.2).normalized()).toRotationMatrix()));
  samples.push_back(sampleFor(expected_tcp, fixed_point,
    Eigen::AngleAxisd(-0.30, Eigen::Vector3d(0.1, 1.0, 1.0).normalized()).toRotationMatrix()));
  samples.push_back(sampleFor(expected_tcp, fixed_point,
    Eigen::AngleAxisd(0.28, Eigen::Vector3d(1.0, 0.2, 1.0).normalized()).toRotationMatrix()));

  const auto result = plasma_eye_hand::solveTcpPivot(samples);
  EXPECT_EQ(result.rank, 6U);
  EXPECT_TRUE(result.flange_to_tcp.isApprox(expected_tcp, 1e-10));
  EXPECT_TRUE(result.fixed_point_in_base.isApprox(fixed_point, 1e-10));
  EXPECT_LT(result.max_residual_m, 1e-10);
}

TEST(TcpPivot, RejectsUnchangedOrientation)
{
  const Eigen::Vector3d tcp(0.002, 0.0, 0.302);
  const Eigen::Vector3d fixed_point(0.4, 0.0, 0.3);
  std::vector<plasma_eye_hand::PivotSample> samples(4,
    sampleFor(tcp, fixed_point, Eigen::Matrix3d::Identity()));
  EXPECT_THROW(plasma_eye_hand::solveTcpPivot(samples), std::runtime_error);
}

}  // namespace
