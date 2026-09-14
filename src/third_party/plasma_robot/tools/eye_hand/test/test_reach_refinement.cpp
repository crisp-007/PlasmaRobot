#include "plasma_eye_hand/reach_refinement.hpp"

#include <gtest/gtest.h>

#include <Eigen/Geometry>

#include <vector>

namespace
{

plasma_eye_hand::ReachRefinementSample makeSample(
  const Eigen::Isometry3d & base_to_source,
  const Eigen::Vector3d & extracted,
  const Eigen::Isometry3d & correction)
{
  plasma_eye_hand::ReachRefinementSample sample;
  sample.source_to_target.translation() = extracted;
  sample.base_to_target = base_to_source * sample.source_to_target;
  sample.base_to_observed.translation() = base_to_source * (correction * extracted);
  return sample;
}

TEST(ReachRefinement, RecoversRigidSourceCorrection)
{
  Eigen::Isometry3d correction = Eigen::Isometry3d::Identity();
  correction.linear() = Eigen::AngleAxisd(
    0.08, Eigen::Vector3d(1.0, 2.0, -1.0).normalized()).toRotationMatrix();
  correction.translation() = Eigen::Vector3d(-0.018, -0.051, 0.004);

  std::vector<plasma_eye_hand::ReachRefinementSample> samples;
  samples.push_back(makeSample(Eigen::Isometry3d::Identity(), {0.02, 0.01, 0.42}, correction));
  Eigen::Isometry3d base_to_source = Eigen::Isometry3d::Identity();
  base_to_source.linear() = Eigen::AngleAxisd(0.4, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  base_to_source.translation() = Eigen::Vector3d(0.2, -0.1, 0.3);
  samples.push_back(makeSample(base_to_source, {-0.04, 0.09, 0.46}, correction));
  base_to_source.linear() = Eigen::AngleAxisd(-0.3, Eigen::Vector3d::UnitY()).toRotationMatrix();
  samples.push_back(makeSample(base_to_source, {0.08, -0.03, 0.51}, correction));
  base_to_source.linear() = Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitX()).toRotationMatrix();
  samples.push_back(makeSample(base_to_source, {-0.07, -0.05, 0.39}, correction));

  const auto result = plasma_eye_hand::fitSourceCorrection(samples);
  EXPECT_TRUE(result.source_correction.matrix().isApprox(correction.matrix(), 1e-10));
  EXPECT_NEAR(result.rms_residual_m, 0.0, 1e-10);
  EXPECT_NEAR(result.max_residual_m, 0.0, 1e-10);
}

TEST(ReachRefinement, RejectsCollinearSamples)
{
  std::vector<plasma_eye_hand::ReachRefinementSample> samples;
  for (double x : {0.0, 0.1, 0.2}) {
    samples.push_back(makeSample(
        Eigen::Isometry3d::Identity(), {x, 0.0, 0.4}, Eigen::Isometry3d::Identity()));
  }
  EXPECT_THROW(plasma_eye_hand::fitSourceCorrection(samples), std::runtime_error);
}

TEST(ReachRefinement, RecoversTranslationOnlySourceCorrection)
{
  Eigen::Isometry3d correction = Eigen::Isometry3d::Identity();
  correction.translation() = Eigen::Vector3d(-0.018, -0.051, 0.004);

  std::vector<plasma_eye_hand::ReachRefinementSample> samples;
  samples.push_back(makeSample(Eigen::Isometry3d::Identity(), {0.02, 0.01, 0.42}, correction));
  Eigen::Isometry3d base_to_source = Eigen::Isometry3d::Identity();
  base_to_source.linear() = Eigen::AngleAxisd(0.4, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  base_to_source.translation() = Eigen::Vector3d(0.2, -0.1, 0.3);
  samples.push_back(makeSample(base_to_source, {-0.04, 0.09, 0.46}, correction));
  base_to_source.linear() = Eigen::AngleAxisd(-0.3, Eigen::Vector3d::UnitY()).toRotationMatrix();
  samples.push_back(makeSample(base_to_source, {0.08, -0.03, 0.51}, correction));

  const auto result = plasma_eye_hand::fitSourceTranslationCorrection(samples);
  EXPECT_TRUE(result.source_correction.matrix().isApprox(correction.matrix(), 1e-10));
  EXPECT_NEAR(result.rms_residual_m, 0.0, 1e-10);
  EXPECT_NEAR(result.max_residual_m, 0.0, 1e-10);
}

}  // namespace
