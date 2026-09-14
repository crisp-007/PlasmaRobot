#include "plasma_path_executor/reviewed_trajectory.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <string>
#include <vector>

namespace
{

using Trajectory = moveit_msgs::msg::RobotTrajectory;

trajectory_msgs::msg::JointTrajectoryPoint point(
  std::int64_t nanoseconds, std::vector<double> positions)
{
  trajectory_msgs::msg::JointTrajectoryPoint value;
  value.positions = std::move(positions);
  value.time_from_start.sec = static_cast<std::int32_t>(nanoseconds / 1000000000LL);
  value.time_from_start.nanosec = static_cast<std::uint32_t>(nanoseconds % 1000000000LL);
  return value;
}

Trajectory trajectory(
  std::initializer_list<trajectory_msgs::msg::JointTrajectoryPoint> points)
{
  Trajectory value;
  value.joint_trajectory.joint_names = {"joint1", "joint2"};
  value.joint_trajectory.points = points;
  return value;
}

std::int64_t timeNanoseconds(const trajectory_msgs::msg::JointTrajectoryPoint & point)
{
  return static_cast<std::int64_t>(point.time_from_start.sec) * 1000000000LL +
         static_cast<std::int64_t>(point.time_from_start.nanosec);
}

TEST(ReviewedTrajectory, RemovesAdjacentDuplicateSprayPointsWithEqualTimestamps)
{
  const Trajectory entry = trajectory({
      point(0, {0.0, 0.0}),
      point(1000000000LL, {0.5, 0.5})});
  const Trajectory spray = trajectory({
      point(0, {0.5, 0.5}),
      point(100000000LL, {0.6, 0.5}),
      point(100000000LL, {0.6, 0.5}),
      point(200000000LL, {0.7, 0.5})});
  Trajectory combined;
  plasma_path_executor::TrajectoryCombineResult result;
  std::string error;

  ASSERT_TRUE(plasma_path_executor::combineReviewedTrajectories(
      entry, spray, &combined, &result, &error)) << error;
  EXPECT_EQ(result.removed_duplicate_points, 1U);
  ASSERT_EQ(combined.joint_trajectory.points.size(), 4U);
  EXPECT_EQ(timeNanoseconds(combined.joint_trajectory.points[2]), 1100000000LL);
  EXPECT_EQ(timeNanoseconds(combined.joint_trajectory.points[3]), 1200000000LL);
}

TEST(ReviewedTrajectory, RemovesDuplicateAtEntrySprayBoundary)
{
  const Trajectory entry = trajectory({
      point(0, {0.0, 0.0}),
      point(1000000000LL, {0.5, 0.5})});
  const Trajectory spray = trajectory({
      point(0, {0.5, 0.5}),
      point(0, {0.5, 0.5}),
      point(200000000LL, {0.7, 0.5})});
  Trajectory combined;
  plasma_path_executor::TrajectoryCombineResult result;
  std::string error;

  ASSERT_TRUE(plasma_path_executor::combineReviewedTrajectories(
      entry, spray, &combined, &result, &error)) << error;
  EXPECT_EQ(result.removed_duplicate_points, 1U);
  ASSERT_EQ(combined.joint_trajectory.points.size(), 3U);
  EXPECT_EQ(combined.joint_trajectory.points.back().positions[0], 0.7);
  EXPECT_EQ(timeNanoseconds(combined.joint_trajectory.points.back()), 1200000000LL);
}

TEST(ReviewedTrajectory, RejectsDifferentPositionsWithEqualTimestamps)
{
  const Trajectory entry = trajectory({
      point(0, {0.0, 0.0}),
      point(1000000000LL, {0.5, 0.5})});
  const Trajectory spray = trajectory({
      point(0, {0.5, 0.5}),
      point(100000000LL, {0.6, 0.5}),
      point(100000000LL, {0.7, 0.5})});
  Trajectory combined;
  std::string error;

  EXPECT_FALSE(plasma_path_executor::combineReviewedTrajectories(
      entry, spray, &combined, nullptr, &error));
  EXPECT_NE(error.find("point 2"), std::string::npos);
  EXPECT_NE(error.find("previous="), std::string::npos);
}

TEST(ReviewedTrajectory, RejectsDiscontinuousBoundary)
{
  const Trajectory entry = trajectory({
      point(0, {0.0, 0.0}),
      point(1000000000LL, {0.5, 0.5})});
  const Trajectory spray = trajectory({
      point(0, {0.6, 0.5}),
      point(100000000LL, {0.7, 0.5})});
  Trajectory combined;
  std::string error;

  EXPECT_FALSE(plasma_path_executor::combineReviewedTrajectories(
      entry, spray, &combined, nullptr, &error));
  EXPECT_NE(error.find("not joint-continuous"), std::string::npos);
}

TEST(ReviewedTrajectory, CombinedTimestampsAreStrictlyIncreasing)
{
  const Trajectory entry = trajectory({
      point(0, {0.0, 0.0}),
      point(400000000LL, {0.2, 0.2}),
      point(900000000LL, {0.5, 0.5})});
  const Trajectory spray = trajectory({
      point(300000000LL, {0.5, 0.5}),
      point(500000000LL, {0.6, 0.5}),
      point(800000000LL, {0.7, 0.5})});
  Trajectory combined;
  std::string error;

  ASSERT_TRUE(plasma_path_executor::combineReviewedTrajectories(
      entry, spray, &combined, nullptr, &error)) << error;
  for (std::size_t index = 1; index < combined.joint_trajectory.points.size(); ++index) {
    EXPECT_GT(
      timeNanoseconds(combined.joint_trajectory.points[index]),
      timeNanoseconds(combined.joint_trajectory.points[index - 1]));
  }
}

}  // namespace
