#include "plasma_path_transform/path_transformer.hpp"

#include <gtest/gtest.h>

#include <Eigen/Geometry>
#include <filesystem>
#include <fstream>
#include <string>

namespace plasma_path_transform
{

TEST(PathTransformer, RejectsNonRigidMatrix)
{
  Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity();
  matrix(0, 0) = 2.0;
  std::string reason;
  EXPECT_FALSE(PathTransformer::validateRigidTransform(matrix, &reason));
  EXPECT_FALSE(reason.empty());
}

TEST(PathTransformer, PoseRoundTripPreservesTransform)
{
  Eigen::Isometry3d expected = Eigen::Isometry3d::Identity();
  expected.translation() = Eigen::Vector3d(0.2, -0.1, 0.4);
  expected.linear() = Eigen::AngleAxisd(0.7, Eigen::Vector3d::UnitY()).toRotationMatrix();

  const auto pose = PathTransformer::isometryToPose(expected);
  const auto actual = PathTransformer::poseToIsometry(pose);
  EXPECT_NEAR((expected.matrix() - actual.matrix()).norm(), 0.0, 1e-12);
}

TEST(PathTransformer, LoadsValidatedYamlWithExpectedConvention)
{
  const auto path = std::filesystem::temp_directory_path() /
    "plasma_path_transform_test_handeye.yaml";
  {
    std::ofstream output(path);
    output << "calibration_id: unit_test\n"
              "validated: true\n"
              "T_camera_to_gripper:\n"
              "  - [1, 0, 0, 0.1]\n"
              "  - [0, 1, 0, 0.2]\n"
              "  - [0, 0, 1, 0.3]\n"
              "  - [0, 0, 0, 1]\n";
  }

  const auto calibration = PathTransformer::loadTransformYaml(
    path.string(), "T_camera_to_gripper");
  EXPECT_TRUE(calibration.loaded);
  EXPECT_TRUE(calibration.validated);
  EXPECT_EQ(calibration.id, "unit_test");
  EXPECT_NEAR(calibration.transform.translation().x(), 0.1, 1e-12);
  EXPECT_NEAR(calibration.transform.translation().y(), 0.2, 1e-12);
  EXPECT_NEAR(calibration.transform.translation().z(), 0.3, 1e-12);

  std::filesystem::remove(path);
}

TEST(PathTransformer, AcceptsDeploymentQualityFlag)
{
  const auto path = std::filesystem::temp_directory_path() /
    "plasma_path_transform_test_deployment_quality.yaml";
  {
    std::ofstream output(path);
    output << "deployment:\n"
              "  quality_accepted: true\n"
              "T_camera_to_gripper:\n"
              "  - [1, 0, 0, 0]\n"
              "  - [0, 1, 0, 0]\n"
              "  - [0, 0, 1, 0]\n"
              "  - [0, 0, 0, 1]\n";
  }

  const auto calibration = PathTransformer::loadTransformYaml(
    path.string(), "T_camera_to_gripper");
  EXPECT_TRUE(calibration.loaded);
  EXPECT_TRUE(calibration.validated);

  std::filesystem::remove(path);
}

TEST(PathTransformer, AcceptsDeploymentProductionStatus)
{
  const auto path = std::filesystem::temp_directory_path() /
    "plasma_path_transform_test_deployment_status.yaml";
  {
    std::ofstream output(path);
    output << "deployment:\n"
              "  status: production_ready\n"
              "T_gripper_to_tcp:\n"
              "  - [1, 0, 0, 0]\n"
              "  - [0, 1, 0, 0]\n"
              "  - [0, 0, 1, 0]\n"
              "  - [0, 0, 0, 1]\n";
  }

  const auto calibration = PathTransformer::loadTransformYaml(
    path.string(), "T_gripper_to_tcp");
  EXPECT_TRUE(calibration.loaded);
  EXPECT_TRUE(calibration.validated);

  std::filesystem::remove(path);
}

TEST(PathTransformer, LoadsUserApprovedProvisionalMatrixWithoutExecutionApproval)
{
  const auto path = std::filesystem::temp_directory_path() /
    "plasma_path_transform_test_provisional_handeye.yaml";
  {
    std::ofstream output(path);
    output << "name: l515_provisional_test\n"
              "frame_convention:\n"
              "  matrix: T_camera_to_gripper\n"
              "  meaning: gripper_T_camera\n"
              "  units: meters\n"
              "deployment:\n"
              "  status: user_approved_provisional_use\n"
              "T_camera_to_gripper:\n"
              "  - [1, 0, 0, -0.04005]\n"
              "  - [0, 1, 0, -0.06191]\n"
              "  - [0, 0, 1, 0.02424]\n"
              "  - [0, 0, 0, 1]\n";
  }

  const auto calibration = PathTransformer::loadTransformYaml(
    path.string(), "T_camera_to_gripper");
  EXPECT_TRUE(calibration.loaded);
  EXPECT_FALSE(calibration.validated);
  EXPECT_EQ(calibration.id, "l515_provisional_test");

  std::filesystem::remove(path);
}

TEST(PathTransformer, LoadsTrainingSourceCorrectionWithFrameAndWithoutApproval)
{
  const auto path = std::filesystem::temp_directory_path() /
    "plasma_path_transform_test_source_refinement.yaml";
  {
    std::ofstream output(path);
    output << "name: reach_candidate\n"
              "status: training_candidate_only\n"
              "validated: false\n"
              "source_frame: camera_depth_optical_frame\n"
              "T_source_correction:\n"
              "  - [1, 0, 0, -0.05]\n"
              "  - [0, 1, 0, 0.02]\n"
              "  - [0, 0, 1, -0.01]\n"
              "  - [0, 0, 0, 1]\n";
  }

  const auto calibration = PathTransformer::loadTransformYaml(
    path.string(), "T_source_correction");
  EXPECT_TRUE(calibration.loaded);
  EXPECT_FALSE(calibration.validated);
  EXPECT_EQ(calibration.id, "reach_candidate");
  EXPECT_EQ(calibration.frame_id, "camera_depth_optical_frame");

  std::filesystem::remove(path);
}

TEST(PathTransformer, RejectsWrongHandEyeConvention)
{
  const auto path = std::filesystem::temp_directory_path() /
    "plasma_path_transform_test_wrong_convention.yaml";
  {
    std::ofstream output(path);
    output << "frame_convention:\n"
              "  matrix: T_camera_to_gripper\n"
              "  meaning: camera_T_gripper\n"
              "  units: meters\n"
              "T_camera_to_gripper:\n"
              "  - [1, 0, 0, 0]\n"
              "  - [0, 1, 0, 0]\n"
              "  - [0, 0, 1, 0]\n"
              "  - [0, 0, 0, 1]\n";
  }

  EXPECT_THROW(
    PathTransformer::loadTransformYaml(path.string(), "T_camera_to_gripper"),
    std::runtime_error);

  std::filesystem::remove(path);
}

TEST(PathTransformer, LoadsToolProcessGeometryFromSharedYaml)
{
  const auto path = std::filesystem::temp_directory_path() /
    "plasma_path_transform_test_process_geometry.yaml";
  {
    std::ofstream output(path);
    output << "frame_convention:\n"
              "  matrix: T_gripper_to_tcp\n"
              "  meaning: gripper_T_tcp\n"
              "  units: meters\n"
              "T_tcp_to_nozzle_tip:\n"
              "  - [1, 0, 0, 0.003]\n"
              "  - [0, 1, 0, 0]\n"
              "  - [0, 0, 1, -0.002]\n"
              "  - [0, 0, 0, 1]\n"
              "T_tcp_to_spray_outlet:\n"
              "  - [1, 0, 0, 0]\n"
              "  - [0, 1, 0, 0]\n"
              "  - [0, 0, 1, 0]\n"
              "  - [0, 0, 0, 1]\n";
  }

  const auto tip = PathTransformer::loadRigidTransformYaml(
    path.string(), "T_tcp_to_nozzle_tip");
  const auto outlet = PathTransformer::loadRigidTransformYaml(
    path.string(), "T_tcp_to_spray_outlet");
  EXPECT_NEAR(tip.translation().x(), 0.003, 1e-12);
  EXPECT_NEAR(tip.translation().z(), -0.002, 1e-12);
  EXPECT_NEAR(outlet.translation().norm(), 0.0, 1e-12);

  std::filesystem::remove(path);
}

TEST(PathTransformer, DerivesMotionTcpThatPlacesRoundedTipAtEntry)
{
  Eigen::Isometry3d desired_tip = Eigen::Isometry3d::Identity();
  desired_tip.translation() = Eigen::Vector3d(0.4, -0.2, 0.7);
  Eigen::Isometry3d tcp_to_tip = Eigen::Isometry3d::Identity();
  tcp_to_tip.translation() = Eigen::Vector3d(0.008, 0.0, -0.002);

  const Eigen::Isometry3d tcp = PathTransformer::tcpPoseForNozzleTipPose(
    desired_tip, tcp_to_tip);

  EXPECT_TRUE((tcp * tcp_to_tip).matrix().isApprox(desired_tip.matrix(), 1e-12));
  EXPECT_NEAR(tcp.translation().x(), 0.392, 1e-12);
  EXPECT_NEAR(tcp.translation().y(), -0.2, 1e-12);
  EXPECT_NEAR(tcp.translation().z(), 0.702, 1e-12);
}

TEST(PathTransformer, DerivesMotionTcpForRoundedTipOutsideMouth)
{
  Eigen::Isometry3d mouth = Eigen::Isometry3d::Identity();
  mouth.translation() = Eigen::Vector3d(0.4, -0.2, 0.7);
  const Eigen::Vector3d outward(-1.0, 0.0, 0.0);
  Eigen::Isometry3d desired_tip = mouth;
  desired_tip.translation() += 0.030 * outward;
  Eigen::Isometry3d tcp_to_tip = Eigen::Isometry3d::Identity();
  tcp_to_tip.translation() = Eigen::Vector3d(0.008, 0.0, -0.002);

  const Eigen::Isometry3d tcp = PathTransformer::tcpPoseForNozzleTipPose(
    desired_tip, tcp_to_tip);

  EXPECT_TRUE((tcp * tcp_to_tip).matrix().isApprox(desired_tip.matrix(), 1e-12));
  EXPECT_NEAR((tcp * tcp_to_tip).translation().x(), 0.370, 1e-12);
}

TEST(PathTransformer, TranslatesCompleteFrameAlongNormalizedCavityAxis)
{
  Eigen::Isometry3d input = Eigen::Isometry3d::Identity();
  input.translation() = Eigen::Vector3d(0.4, -0.2, 0.7);
  input.linear() = Eigen::AngleAxisd(0.6, Eigen::Vector3d::UnitY()).toRotationMatrix();

  const Eigen::Isometry3d output = PathTransformer::translatedAlongAxis(
    input, Eigen::Vector3d(3.0, 0.0, 4.0), 0.030);

  EXPECT_TRUE(output.linear().isApprox(input.linear(), 1e-12));
  EXPECT_TRUE(output.translation().isApprox(
    Eigen::Vector3d(0.418, -0.2, 0.724), 1e-12));
}

TEST(PathTransformer, RejectsInvalidCompensationAxis)
{
  EXPECT_THROW(
    PathTransformer::translatedAlongAxis(
      Eigen::Isometry3d::Identity(), Eigen::Vector3d::Zero(), 0.030),
    std::runtime_error);
}

}  // namespace plasma_path_transform
