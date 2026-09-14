#pragma once

#include <moveit_msgs/msg/robot_trajectory.hpp>

#include <cstddef>
#include <string>

namespace plasma_path_executor
{

struct TrajectoryCombineResult
{
  std::size_t removed_duplicate_points = 0;
};

bool combineReviewedTrajectories(
  const moveit_msgs::msg::RobotTrajectory & entry,
  const moveit_msgs::msg::RobotTrajectory & spray,
  moveit_msgs::msg::RobotTrajectory * combined,
  TrajectoryCombineResult * result,
  std::string * error);

}  // namespace plasma_path_executor
