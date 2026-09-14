#include "plasma_eye_hand/reach_validation.hpp"

#include <gtest/gtest.h>

#include <Eigen/Geometry>

#include <vector>

namespace
{

TEST(ReachValidation, SeparatesAxialAndLateralError)
{
  const Eigen::Isometry3d target = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d actual = Eigen::Isometry3d::Identity();
  actual.translation() = Eigen::Vector3d(0.003, 0.004, -0.002);
  actual.linear() = Eigen::AngleAxisd(
    0.01, Eigen::Vector3d::UnitY()).toRotationMatrix();

  const auto error = plasma_eye_hand::calculateReachError(
    target, actual, Eigen::Vector3d::UnitX());
  EXPECT_NEAR(error.signed_axial_m, 0.003, 1e-12);
  EXPECT_NEAR(error.lateral_m, std::sqrt(0.000020), 1e-12);
  EXPECT_NEAR(error.norm_m, std::sqrt(0.000029), 1e-12);
  EXPECT_NEAR(error.orientation_rad, 0.01, 1e-12);
  EXPECT_NEAR(error.full_orientation_rad, 0.01, 1e-12);
}

TEST(ReachValidation, IgnoresRollAboutRodForEntryAlignment)
{
  const Eigen::Isometry3d target = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d actual = Eigen::Isometry3d::Identity();
  actual.linear() = Eigen::AngleAxisd(
    0.5, Eigen::Vector3d::UnitX()).toRotationMatrix();

  const auto error = plasma_eye_hand::calculateReachError(
    target, actual, Eigen::Vector3d::UnitX());
  EXPECT_NEAR(error.orientation_rad, 0.0, 1e-12);
  EXPECT_NEAR(error.full_orientation_rad, 0.5, 1e-12);
}

TEST(ReachValidation, RequiresEnoughRecordsAndAllLimits)
{
  plasma_eye_hand::ReachLimits limits;
  limits.min_records = 3;
  limits.max_norm_m = 0.005;
  limits.max_abs_axial_m = 0.004;
  limits.max_lateral_m = 0.003;
  limits.max_orientation_rad = 0.02;

  plasma_eye_hand::ReachError good;
  good.translation_m = Eigen::Vector3d(0.001, -0.001, 0.0);
  good.norm_m = good.translation_m.norm();
  good.signed_axial_m = 0.001;
  good.lateral_m = 0.001;
  good.orientation_rad = 0.01;

  EXPECT_FALSE(plasma_eye_hand::summarizeReachErrors({good, good}, limits).accepted);
  EXPECT_TRUE(plasma_eye_hand::summarizeReachErrors({good, good, good}, limits).accepted);

  auto bad = good;
  bad.lateral_m = 0.004;
  EXPECT_FALSE(plasma_eye_hand::summarizeReachErrors({good, good, bad}, limits).accepted);
}

}  // namespace
