#include "plasma_path_executor/reviewed_trajectory.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>

namespace plasma_path_executor
{
namespace
{

constexpr double kBoundaryJointToleranceRad = 1e-6;
constexpr double kSameJointPositionToleranceRad = 1e-9;

std::int64_t durationNanoseconds(const builtin_interfaces::msg::Duration & duration)
{
  return static_cast<std::int64_t>(duration.sec) * 1000000000LL +
         static_cast<std::int64_t>(duration.nanosec);
}

void setDurationNanoseconds(
  builtin_interfaces::msg::Duration * duration, std::int64_t nanoseconds)
{
  duration->sec = static_cast<std::int32_t>(nanoseconds / 1000000000LL);
  duration->nanosec = static_cast<std::uint32_t>(nanoseconds % 1000000000LL);
}

bool sameJointPosition(
  const trajectory_msgs::msg::JointTrajectoryPoint & left,
  const trajectory_msgs::msg::JointTrajectoryPoint & right,
  double tolerance)
{
  if (left.positions.size() != right.positions.size()) {
    return false;
  }
  for (std::size_t index = 0; index < left.positions.size(); ++index) {
    if (std::abs(left.positions[index] - right.positions[index]) > tolerance) {
      return false;
    }
  }
  return true;
}

bool completeFinitePoint(
  const trajectory_msgs::msg::JointTrajectoryPoint & point,
  std::size_t joint_count)
{
  return point.positions.size() == joint_count &&
         std::all_of(point.positions.begin(), point.positions.end(),
    [](double position) {return std::isfinite(position);});
}

bool fail(std::string * error, const std::string & message)
{
  if (error) {
    *error = message;
  }
  return false;
}

}  // namespace

bool combineReviewedTrajectories(
  const moveit_msgs::msg::RobotTrajectory & entry,
  const moveit_msgs::msg::RobotTrajectory & spray,
  moveit_msgs::msg::RobotTrajectory * combined,
  TrajectoryCombineResult * result,
  std::string * error)
{
  if (result) {
    *result = TrajectoryCombineResult{};
  }
  const auto & entry_joints = entry.joint_trajectory;
  const auto & spray_joints = spray.joint_trajectory;
  if (!combined || entry_joints.points.size() < 2 || spray_joints.points.size() < 2 ||
    entry_joints.joint_names.empty() || entry_joints.joint_names != spray_joints.joint_names)
  {
    return fail(
      error, "reviewed entry and spray trajectories are incomplete or use different joints");
  }
  if (!entry.multi_dof_joint_trajectory.points.empty() ||
    !spray.multi_dof_joint_trajectory.points.empty())
  {
    return fail(error, "reviewed multi-DOF trajectory combination is not supported");
  }

  const std::size_t joint_count = entry_joints.joint_names.size();
  for (std::size_t index = 0; index < entry_joints.points.size(); ++index) {
    if (!completeFinitePoint(entry_joints.points[index], joint_count)) {
      return fail(
        error, "reviewed entry trajectory contains an incomplete or non-finite joint point at " +
        std::to_string(index));
    }
    if (index > 0 &&
      durationNanoseconds(entry_joints.points[index].time_from_start) <=
      durationNanoseconds(entry_joints.points[index - 1].time_from_start))
    {
      return fail(
        error, "reviewed entry trajectory timestamps are not strictly increasing at point " +
        std::to_string(index));
    }
  }
  for (std::size_t index = 0; index < spray_joints.points.size(); ++index) {
    if (!completeFinitePoint(spray_joints.points[index], joint_count)) {
      return fail(
        error, "reviewed spray trajectory contains an incomplete or non-finite joint point at " +
        std::to_string(index));
    }
  }

  const auto & entry_end = entry_joints.points.back();
  const auto & spray_start = spray_joints.points.front();
  if (!sameJointPosition(entry_end, spray_start, kBoundaryJointToleranceRad)) {
    return fail(error, "reviewed entry and spray trajectories are not joint-continuous");
  }

  const std::int64_t entry_end_ns = durationNanoseconds(entry_end.time_from_start);
  const std::int64_t spray_start_ns = durationNanoseconds(spray_start.time_from_start);
  if (entry_end_ns <= 0) {
    return fail(error, "reviewed entry trajectory has no positive duration");
  }

  *combined = entry;
  auto & destination = combined->joint_trajectory.points;
  std::size_t duplicate_points = 0;
  for (std::size_t index = 1; index < spray_joints.points.size(); ++index) {
    const auto & source_point = spray_joints.points[index];
    if (sameJointPosition(
        destination.back(), source_point, kSameJointPositionToleranceRad))
    {
      ++duplicate_points;
      continue;
    }

    const std::int64_t source_ns = durationNanoseconds(source_point.time_from_start);
    if ((spray_start_ns < 0 && source_ns > std::numeric_limits<std::int64_t>::max() + spray_start_ns) ||
      (spray_start_ns > 0 && source_ns < std::numeric_limits<std::int64_t>::min() + spray_start_ns))
    {
      return fail(error, "reviewed spray trajectory duration overflows at point " + std::to_string(index));
    }
    const std::int64_t relative_ns = source_ns - spray_start_ns;
    if (relative_ns > std::numeric_limits<std::int64_t>::max() - entry_end_ns) {
      return fail(error, "reviewed spray trajectory duration overflows at point " + std::to_string(index));
    }
    const std::int64_t combined_ns = entry_end_ns + relative_ns;
    const std::int64_t previous_ns = durationNanoseconds(destination.back().time_from_start);
    if (combined_ns <= previous_ns) {
      return fail(
        error, "different joint positions in reviewed spray trajectory do not have strictly "
        "increasing timestamps at point " + std::to_string(index) +
        " (previous=" + std::to_string(previous_ns) + " ns, current=" +
        std::to_string(combined_ns) + " ns)");
    }
    destination.push_back(source_point);
    setDurationNanoseconds(&destination.back().time_from_start, combined_ns);
  }
  if (destination.size() < 3) {
    return fail(error, "combined reviewed trajectory has fewer than three distinct points");
  }
  if (result) {
    result->removed_duplicate_points = duplicate_points;
  }
  return true;
}

}  // namespace plasma_path_executor
