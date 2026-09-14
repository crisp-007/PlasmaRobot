#include "plasma_path_executor/retreat_path.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

namespace
{

using Path = plasma_robot_interfaces::msg::SprayPath;
using Point = plasma_robot_interfaces::msg::SprayPathPoint;

geometry_msgs::msg::Pose pose(double x, double qz = 0.0)
{
  geometry_msgs::msg::Pose value;
  value.position.x = x;
  value.orientation.z = qz;
  value.orientation.w = std::sqrt(1.0 - qz * qz);
  return value;
}

void addPoint(
  Path * path, std::int32_t layer, std::uint8_t phase, double angle,
  double x, double qz = 0.0)
{
  Point point;
  point.layer_index = layer;
  point.motion_phase = phase;
  point.joint6_geometric_deg = angle;
  point.flange_pose = pose(x, qz);
  point.flange_pose_valid = true;
  path->points.push_back(std::move(point));
}

Path threeLayerPath()
{
  Path path;
  path.cavity_entry_flange_pose = pose(0.40, 0.3);
  path.cavity_entry_flange_pose_valid = true;
  path.pre_entry_flange_pose = pose(0.50, 0.4);
  path.pre_entry_flange_pose_valid = true;
  addPoint(&path, 0, Point::PHASE_PROCESS_POSITIVE, 180.0, 9.0, 0.7);
  addPoint(&path, 0, Point::PHASE_RETURN_FROM_NEGATIVE, 0.0, 0.30, 0.1);
  addPoint(&path, 1, Point::PHASE_PROCESS_NEGATIVE, -180.0, 8.0, 0.7);
  addPoint(&path, 1, Point::PHASE_RETURN_FROM_NEGATIVE, 0.0, 0.20, 0.2);
  addPoint(&path, 2, Point::PHASE_PROCESS_POSITIVE, 180.0, 7.0, 0.7);
  addPoint(&path, 2, Point::PHASE_RETURN_FROM_NEGATIVE, 0.0, 0.10, 0.25);
  return path;
}

TEST(RetreatPath, KeepsOneZeroPosePerPriorLayerAndAppendsBothEntryPoses)
{
  const Path path = threeLayerPath();
  std::vector<geometry_msgs::msg::Pose> waypoints;
  plasma_path_executor::RetreatWaypointResult result;
  std::string error;

  ASSERT_TRUE(plasma_path_executor::buildZeroRotationRetreatWaypoints(
      path, &waypoints, &result, &error)) << error;
  EXPECT_EQ(result.layer_count, 3U);
  EXPECT_EQ(result.traversed_layer_indices, (std::vector<std::int32_t>{1, 0}));
  ASSERT_EQ(waypoints.size(), 4U);
  EXPECT_DOUBLE_EQ(waypoints[0].position.x, 0.20);
  EXPECT_DOUBLE_EQ(waypoints[1].position.x, 0.30);
  EXPECT_DOUBLE_EQ(waypoints[2].position.x, 0.40);
  EXPECT_DOUBLE_EQ(waypoints[3].position.x, 0.50);
}

TEST(RetreatPath, UsesTheFinalZeroOrientationForEveryWaypoint)
{
  const Path path = threeLayerPath();
  std::vector<geometry_msgs::msg::Pose> waypoints;
  std::string error;

  ASSERT_TRUE(plasma_path_executor::buildZeroRotationRetreatWaypoints(
      path, &waypoints, nullptr, &error)) << error;
  for (const auto & waypoint : waypoints) {
    EXPECT_DOUBLE_EQ(waypoint.orientation.x, path.points.back().flange_pose.orientation.x);
    EXPECT_DOUBLE_EQ(waypoint.orientation.y, path.points.back().flange_pose.orientation.y);
    EXPECT_DOUBLE_EQ(waypoint.orientation.z, path.points.back().flange_pose.orientation.z);
    EXPECT_DOUBLE_EQ(waypoint.orientation.w, path.points.back().flange_pose.orientation.w);
  }
}

TEST(RetreatPath, NeverIncludesPositiveOrNegativeSweepPoses)
{
  const Path path = threeLayerPath();
  std::vector<geometry_msgs::msg::Pose> waypoints;
  std::string error;

  ASSERT_TRUE(plasma_path_executor::buildZeroRotationRetreatWaypoints(
      path, &waypoints, nullptr, &error)) << error;
  for (const auto & waypoint : waypoints) {
    EXPECT_LT(waypoint.position.x, 1.0);
  }
}

TEST(RetreatPath, RejectsAPathThatDoesNotFinishAtZeroDegrees)
{
  Path path = threeLayerPath();
  addPoint(&path, 2, Point::PHASE_PROCESS_POSITIVE, 5.0, 0.10);
  std::vector<geometry_msgs::msg::Pose> waypoints;
  std::string error;

  EXPECT_FALSE(plasma_path_executor::buildZeroRotationRetreatWaypoints(
      path, &waypoints, nullptr, &error));
  EXPECT_NE(error.find("does not finish"), std::string::npos);
}

TEST(RetreatPath, RejectsAnyLayerWithoutAFinalZeroReturn)
{
  Path path = threeLayerPath();
  path.points[3].joint6_geometric_deg = -1.0;
  std::vector<geometry_msgs::msg::Pose> waypoints;
  std::string error;

  EXPECT_FALSE(plasma_path_executor::buildZeroRotationRetreatWaypoints(
      path, &waypoints, nullptr, &error));
  EXPECT_NE(error.find("every spray layer"), std::string::npos);
}

Path stoppedRetreatPath()
{
  Path path;
  path.cavity_axis.x = 1.0;
  path.cavity_axis_valid = true;
  path.pre_entry_tcp_pose = pose(0.50);
  path.pre_entry_tcp_pose_valid = true;
  Point point;
  point.flange_pose = pose(0.0);
  point.flange_pose_valid = true;
  point.tcp_pose = pose(0.31);
  path.points.push_back(point);
  return path;
}

TEST(StoppedRetreatPath, PreservesStoppedOrientationAndMovesOnlyOutward)
{
  const Path path = stoppedRetreatPath();
  geometry_msgs::msg::Pose stopped = pose(-0.20);
  stopped.orientation.x = 0.5;
  stopped.orientation.w = std::sqrt(0.75);
  std::vector<geometry_msgs::msg::Pose> waypoints;
  plasma_path_executor::StoppedRetreatResult result;
  std::string error;

  ASSERT_TRUE(plasma_path_executor::buildStoppedRetreatWaypoints(
      path, stopped, {}, &waypoints, &result, &error)) << error;
  ASSERT_EQ(waypoints.size(), 1U);
  EXPECT_NEAR(result.outward_distance_m, 0.39, 1e-9);
  EXPECT_NEAR(result.axis_offset_m, 0.0, 1e-9);
  EXPECT_NEAR(waypoints.front().position.x, 0.19, 1e-9);
  EXPECT_DOUBLE_EQ(waypoints.front().position.y, stopped.position.y);
  EXPECT_DOUBLE_EQ(waypoints.front().position.z, stopped.position.z);
  EXPECT_DOUBLE_EQ(waypoints.front().orientation.z, stopped.orientation.z);
  EXPECT_DOUBLE_EQ(waypoints.front().orientation.w, stopped.orientation.w);
}

TEST(StoppedRetreatPath, KeepsCurrentLateralOffsetWithoutSidewaysCorrection)
{
  Path path = stoppedRetreatPath();
  geometry_msgs::msg::Pose stopped = pose(-0.10);
  stopped.position.y = 0.01;
  std::vector<geometry_msgs::msg::Pose> waypoints;
  plasma_path_executor::StoppedRetreatResult result;
  std::string error;

  ASSERT_TRUE(plasma_path_executor::buildStoppedRetreatWaypoints(
      path, stopped, {}, &waypoints, &result, &error)) << error;
  EXPECT_NEAR(result.axis_offset_m, 0.01, 1e-9);
  EXPECT_NEAR(waypoints.front().position.y, 0.01, 1e-9);
}

TEST(StoppedRetreatPath, RejectsExcessiveLateralOffset)
{
  const Path path = stoppedRetreatPath();
  geometry_msgs::msg::Pose stopped = pose(-0.10);
  stopped.position.y = 0.03;
  std::vector<geometry_msgs::msg::Pose> waypoints;
  std::string error;

  EXPECT_FALSE(plasma_path_executor::buildStoppedRetreatWaypoints(
      path, stopped, {}, &waypoints, nullptr, &error));
  EXPECT_NE(error.find("too far"), std::string::npos);
}

TEST(StoppedRetreatPath, RejectsPoseAlreadyOutsidePreEntryPlane)
{
  const Path path = stoppedRetreatPath();
  const geometry_msgs::msg::Pose stopped = pose(0.25);
  std::vector<geometry_msgs::msg::Pose> waypoints;
  std::string error;

  EXPECT_FALSE(plasma_path_executor::buildStoppedRetreatWaypoints(
      path, stopped, {}, &waypoints, nullptr, &error));
  EXPECT_NE(error.find("already at or outside"), std::string::npos);
}

TEST(StoppedRetreatStability, MatchesJointsByNameInsteadOfArrayOrder)
{
  const std::vector<std::string> reference_names{"joint1", "joint2", "joint6"};
  const std::vector<double> reference_positions{0.10, -0.20, 1.00};
  const std::vector<std::string> current_names{"joint6", "joint1", "joint2"};
  const std::vector<double> current_positions{1.0002, 0.1001, -0.2001};
  double max_delta = 0.0;

  EXPECT_TRUE(plasma_path_executor::namedJointPositionsWithinTolerance(
      reference_names, reference_positions, current_names, current_positions,
      0.001, &max_delta));
  EXPECT_NEAR(max_delta, 0.0002, 1e-12);
}

TEST(StoppedRetreatStability, RejectsMotionBeyondTolerance)
{
  const std::vector<std::string> names{"joint1", "joint2"};
  double max_delta = 0.0;

  EXPECT_FALSE(plasma_path_executor::namedJointPositionsWithinTolerance(
      names, {0.0, 0.0}, names, {0.0, 0.002}, 0.001, &max_delta));
  EXPECT_NEAR(max_delta, 0.002, 1e-12);
}

TEST(StoppedRetreatStability, RejectsMissingOrInvalidJointSamples)
{
  EXPECT_FALSE(plasma_path_executor::namedJointPositionsWithinTolerance(
      {"joint1", "joint2"}, {0.0, 0.0}, {"joint1"}, {0.0}, 0.001));
  EXPECT_FALSE(plasma_path_executor::namedJointPositionsWithinTolerance(
      {"joint1"}, {0.0}, {"joint1"}, {0.0}, 0.0));
}

}  // namespace
