#include "plasma_path_executor/path_validation.hpp"

#include <gtest/gtest.h>

#include <plasma_robot_interfaces/msg/spray_path_point.hpp>

#include <cstdint>
#include <string>

namespace
{

using Path = plasma_robot_interfaces::msg::SprayPath;
using Point = plasma_robot_interfaces::msg::SprayPathPoint;

geometry_msgs::msg::Pose identityPose(double x = 0.0)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.orientation.w = 1.0;
  return pose;
}

void addPoint(Path * path, int sequence, int phase, double angle, bool plasma)
{
  Point point;
  point.tcp_pose = identityPose();
  point.safety_point.x = 0.003;
  point.safety_point.z = -0.002;
  point.safety_point_valid = true;
  point.spray_outlet_pose = identityPose();
  point.spray_outlet_pose_valid = true;
  point.flange_pose = identityPose();
  point.flange_pose_valid = true;
  point.sequence_index = sequence;
  point.layer_index = 0;
  point.motion_phase = static_cast<std::uint8_t>(phase);
  point.plasma_enabled = plasma;
  point.joint6_geometric_deg = angle;
  point.surface_target.z = 0.05;
  path->points.push_back(point);
}

Path validPath()
{
  Path path;
  path.header.frame_id = "baselink";
  path.path_id = "test_path";
  path.tool_frame = "plasma_motion_tcp";
  path.base_frame = "baselink";
  path.gripper_frame = "Link6";
  path.transform_valid = true;
  path.execution_permitted = false;
  addPoint(&path, 0, Point::PHASE_PROCESS_POSITIVE, 0.0, true);
  addPoint(&path, 1, Point::PHASE_PROCESS_POSITIVE, 180.0, true);
  addPoint(&path, 2, Point::PHASE_RETURN_FROM_POSITIVE, 180.0, false);
  addPoint(&path, 3, Point::PHASE_RETURN_FROM_POSITIVE, 0.0, false);
  addPoint(&path, 4, Point::PHASE_PROCESS_NEGATIVE, 0.0, true);
  addPoint(&path, 5, Point::PHASE_PROCESS_NEGATIVE, -180.0, true);
  addPoint(&path, 6, Point::PHASE_RETURN_FROM_NEGATIVE, -180.0, false);
  addPoint(&path, 7, Point::PHASE_RETURN_FROM_NEGATIVE, 0.0, false);
  return path;
}

void addValidEntryGeometry(Path * path)
{
  path->cavity_mouth_tip_pose = identityPose(0.050);
  path->cavity_mouth_tip_pose_valid = true;
  path->cavity_entry_tcp_pose = identityPose(0.020);
  path->cavity_entry_tcp_pose_valid = true;
  path->pre_entry_tcp_pose = identityPose(0.080);
  path->pre_entry_tcp_pose_valid = true;
  path->cavity_axis.x = 1.0;
  path->cavity_axis_valid = true;
  path->cavity_entry_slice_index = 3;
  path->pre_entry_distance_m = 0.060;
  path->entry_tip_standoff_m = 0.030;
  path->cavity_entry_flange_pose = path->cavity_entry_tcp_pose;
  path->cavity_entry_flange_pose_valid = true;
  path->pre_entry_flange_pose = path->pre_entry_tcp_pose;
  path->pre_entry_flange_pose_valid = true;
}

TEST(PathValidation, AcceptsFourStagePlusMinus180Sequence)
{
  std::string error;
  EXPECT_TRUE(plasma_path_executor::validatePathStructure(
    validPath(), plasma_path_executor::ValidationLimits{}, &error)) << error;
}

TEST(PathValidation, RejectsPlasmaDuringReturn)
{
  Path path = validPath();
  path.points[2].plasma_enabled = true;
  std::string error;
  EXPECT_FALSE(plasma_path_executor::validatePathStructure(
    path, plasma_path_executor::ValidationLimits{}, &error));
  EXPECT_NE(error.find("plasma state"), std::string::npos);
}

TEST(PathValidation, RejectsIncompleteNegativeSweep)
{
  Path path = validPath();
  path.points[5].joint6_geometric_deg = -90.0;
  std::string error;
  EXPECT_FALSE(plasma_path_executor::validatePathStructure(
    path, plasma_path_executor::ValidationLimits{}, &error));
  EXPECT_NE(error.find("-180"), std::string::npos);
}

TEST(PathValidation, RejectsTcpMotionWithinOneRotationLayer)
{
  Path path = validPath();
  path.points[4].tcp_pose.position.x = 0.001;
  std::string error;
  EXPECT_FALSE(plasma_path_executor::validatePathStructure(
    path, plasma_path_executor::ValidationLimits{}, &error));
  EXPECT_NE(error.find("TCP position changes"), std::string::npos);
}

TEST(PathValidation, RejectsMissingProcessGeometry)
{
  Path path = validPath();
  path.points[0].safety_point_valid = false;
  std::string error;
  EXPECT_FALSE(plasma_path_executor::validatePathStructure(
    path, plasma_path_executor::ValidationLimits{}, &error));
  EXPECT_NE(error.find("nozzle-tip"), std::string::npos);
}

TEST(PathValidation, RejectsContactOrZeroSprayDistance)
{
  Path path = validPath();
  path.points[0].surface_target.z = 0.001;
  std::string error;
  EXPECT_FALSE(plasma_path_executor::validatePathStructure(
    path, plasma_path_executor::ValidationLimits{}, &error));
  EXPECT_NE(error.find("non-contact range"), std::string::npos);
}

TEST(PathValidation, RejectsTargetOutsideSprayRay)
{
  Path path = validPath();
  path.points[0].surface_target.x = 0.05;
  path.points[0].surface_target.z = 0.0;
  std::string error;
  EXPECT_FALSE(plasma_path_executor::validatePathStructure(
    path, plasma_path_executor::ValidationLimits{}, &error));
  EXPECT_NE(error.find("TCP +Z spray ray"), std::string::npos);
}

TEST(PathValidation, AcceptsExplicitCoaxialEntryGeometry)
{
  Path path = validPath();
  addValidEntryGeometry(&path);
  std::string error;
  EXPECT_TRUE(plasma_path_executor::validatePathStructure(
    path, plasma_path_executor::ValidationLimits{}, &error)) << error;
}

TEST(PathValidation, RejectsIncompleteEntryGeometry)
{
  Path path = validPath();
  addValidEntryGeometry(&path);
  path.pre_entry_flange_pose_valid = false;
  std::string error;
  EXPECT_FALSE(plasma_path_executor::validatePathStructure(
    path, plasma_path_executor::ValidationLimits{}, &error));
  EXPECT_NE(error.find("incomplete"), std::string::npos);
}

TEST(PathValidation, RejectsUnsafeEntryTipStandoff)
{
  Path path = validPath();
  addValidEntryGeometry(&path);
  path.entry_tip_standoff_m = 0.151;
  std::string error;
  EXPECT_FALSE(plasma_path_executor::validatePathStructure(
    path, plasma_path_executor::ValidationLimits{}, &error));
  EXPECT_NE(error.find("standoff"), std::string::npos);
}

TEST(PathValidation, RejectsEntryOnInsideOfFirstLayer)
{
  Path path = validPath();
  addValidEntryGeometry(&path);
  path.cavity_entry_tcp_pose.position.x = -0.020;
  path.cavity_entry_flange_pose.position.x = -0.020;
  path.pre_entry_tcp_pose.position.x = 0.040;
  path.pre_entry_flange_pose.position.x = 0.040;
  std::string error;
  EXPECT_FALSE(plasma_path_executor::validatePathStructure(
    path, plasma_path_executor::ValidationLimits{}, &error));
  EXPECT_NE(error.find("outward"), std::string::npos);
}

TEST(PathValidation, RejectsPreEntryOutsideCavityAxis)
{
  Path path = validPath();
  addValidEntryGeometry(&path);
  path.pre_entry_tcp_pose.position.y = 0.003;
  path.pre_entry_flange_pose.position.y = 0.003;
  std::string error;
  EXPECT_FALSE(plasma_path_executor::validatePathStructure(
    path, plasma_path_executor::ValidationLimits{}, &error));
  EXPECT_NE(error.find("outward point"), std::string::npos);
}

TEST(PathValidation, RejectsInvalidEntryOrientation)
{
  Path path = validPath();
  addValidEntryGeometry(&path);
  path.cavity_entry_tcp_pose.orientation.w = 0.0;
  std::string error;
  EXPECT_FALSE(plasma_path_executor::validatePathStructure(
    path, plasma_path_executor::ValidationLimits{}, &error));
  EXPECT_NE(error.find("invalid pose"), std::string::npos);
}

TEST(PathValidation, QuaternionSignDoesNotChangePoseError)
{
  auto first = identityPose();
  auto second = identityPose();
  second.orientation.w = -1.0;
  const auto error = plasma_path_executor::poseError(first, second);
  EXPECT_DOUBLE_EQ(error.translation_m, 0.0);
  EXPECT_DOUBLE_EQ(error.rotation_rad, 0.0);
}

TEST(PathValidation, KeepsRevoluteTrajectoryContinuousAcrossSignedPi)
{
  constexpr double kDegreesToRadians = 3.14159265358979323846 / 180.0;
  double adjusted = 0.0;
  EXPECT_TRUE(plasma_path_executor::nearestEquivalentRevolutePosition(
    -179.5 * kDegreesToRadians, 179.5 * kDegreesToRadians,
    -360.0 * kDegreesToRadians, 360.0 * kDegreesToRadians, &adjusted));
  EXPECT_NEAR(adjusted / kDegreesToRadians, 180.5, 1e-9);
}

TEST(PathValidation, DoesNotCrossARevoluteJointLimitToRemoveWrap)
{
  constexpr double kDegreesToRadians = 3.14159265358979323846 / 180.0;
  double adjusted = 0.0;
  EXPECT_TRUE(plasma_path_executor::nearestEquivalentRevolutePosition(
    -179.5 * kDegreesToRadians, 179.5 * kDegreesToRadians,
    -180.0 * kDegreesToRadians, 180.0 * kDegreesToRadians, &adjusted));
  EXPECT_NEAR(adjusted / kDegreesToRadians, -179.5, 1e-9);
}

TEST(PathValidation, RejectsRevolutePositionWithNoEquivalentInsideLimits)
{
  double adjusted = 0.0;
  EXPECT_FALSE(plasma_path_executor::nearestEquivalentRevolutePosition(
    7.0, 0.0, -0.5, 0.5, &adjusted));
}

TEST(PathValidation, IkSeedScheduleTriesTheOppositeWindingBeforeGridSeeds)
{
  constexpr double kDegreesToRadians = 3.14159265358979323846 / 180.0;
  const auto seeds = plasma_path_executor::revoluteIkSeedSchedule(
    90.0 * kDegreesToRadians,
    -360.0 * kDegreesToRadians,
    360.0 * kDegreesToRadians, 8);

  ASSERT_EQ(seeds.size(), 8U);
  EXPECT_NEAR(seeds[0] / kDegreesToRadians, 90.0, 1e-9);
  EXPECT_NEAR(seeds[1] / kDegreesToRadians, -270.0, 1e-9);
  for (double seed : seeds) {
    EXPECT_GE(seed / kDegreesToRadians, -360.0);
    EXPECT_LE(seed / kDegreesToRadians, 360.0);
  }
}

TEST(PathValidation, IkSeedScheduleRejectsInvalidBounds)
{
  EXPECT_TRUE(plasma_path_executor::revoluteIkSeedSchedule(
      0.0, 1.0, -1.0, 8).empty());
  EXPECT_TRUE(plasma_path_executor::revoluteIkSeedSchedule(
      2.0, -1.0, 1.0, 8).empty());
}

TEST(PathValidation, FindsContiguousFirstLayerBoundary)
{
  Path path = validPath();
  for (auto & point : path.points) {
    point.layer_index = 3;
  }
  Point transition = path.points.front();
  transition.sequence_index = static_cast<std::int32_t>(path.points.size());
  transition.layer_index = 4;
  transition.motion_phase = Point::PHASE_LAYER_TRANSITION;
  path.points.push_back(transition);
  Point next = transition;
  next.sequence_index += 1;
  next.motion_phase = Point::PHASE_PROCESS_POSITIVE;
  path.points.push_back(next);

  EXPECT_EQ(plasma_path_executor::firstLayerPointCount(path), 8U);
  path.points.clear();
  EXPECT_EQ(plasma_path_executor::firstLayerPointCount(path), 0U);
}

TEST(PathValidation, AcceptsConfirmedEntryAlongToolAxis)
{
  Path path;
  Point first;
  first.flange_pose = identityPose();
  first.flange_pose_valid = true;
  first.tcp_pose = identityPose();
  first.tcp_pose.position.z = 0.310;
  path.points.push_back(first);
  Point second = first;
  second.tcp_pose.position.z = 0.292;
  path.points.push_back(second);
  auto current = identityPose();
  current.position.z = -0.1;
  plasma_path_executor::EntryApproachResult result;
  std::string error;
  EXPECT_TRUE(plasma_path_executor::validateEntryApproach(
    current, path, plasma_path_executor::EntryApproachLimits{}, &result, &error)) << error;
  EXPECT_NEAR(result.translation_m, 0.1, 1e-9);
  EXPECT_NEAR(result.axis_error_rad, 0.0, 1e-9);
}

TEST(PathValidation, RejectsEntryApproachOutsideAxisCone)
{
  Path path;
  Point first;
  first.flange_pose = identityPose();
  first.flange_pose_valid = true;
  first.tcp_pose = identityPose();
  first.tcp_pose.position.z = 0.310;
  path.points.push_back(first);
  Point second = first;
  second.tcp_pose.position.z = 0.292;
  path.points.push_back(second);
  auto current = identityPose();
  current.position.x = 0.1;
  current.position.z = -0.1;
  plasma_path_executor::EntryApproachResult result;
  std::string error;
  EXPECT_FALSE(plasma_path_executor::validateEntryApproach(
    current, path, plasma_path_executor::EntryApproachLimits{}, &result, &error));
  EXPECT_TRUE(error.find("tool axis") != std::string::npos ||
              error.find("cavity axis") != std::string::npos);
}

TEST(PathValidation, RejectsEntryApproachThatIsTooLong)
{
  Path path;
  Point first;
  first.flange_pose = identityPose();
  first.flange_pose_valid = true;
  first.tcp_pose = identityPose();
  first.tcp_pose.position.z = 0.310;
  path.points.push_back(first);
  Point second = first;
  second.tcp_pose.position.z = 0.292;
  path.points.push_back(second);
  auto current = identityPose();
  current.position.z = -0.3;
  plasma_path_executor::EntryApproachResult result;
  std::string error;
  EXPECT_FALSE(plasma_path_executor::validateEntryApproach(
    current, path, plasma_path_executor::EntryApproachLimits{}, &result, &error));
  EXPECT_NE(error.find("distance"), std::string::npos);
}

}  // namespace
