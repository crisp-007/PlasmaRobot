#pragma once

#include <geometry_msgs/msg/pose.hpp>
#include <plasma_robot_interfaces/msg/spray_path.hpp>

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace plasma_path_executor
{

struct RetreatWaypointResult
{
  std::size_t layer_count = 0;
  std::vector<std::int32_t> traversed_layer_indices;
};

struct StoppedRetreatLimits
{
  double max_axis_offset_m = 0.02;
  double min_outward_distance_m = 0.002;
  double max_outward_distance_m = 0.50;
};

struct StoppedRetreatResult
{
  double outward_distance_m = 0.0;
  double axis_offset_m = 0.0;
  geometry_msgs::msg::Pose target_flange_pose;
};

// Builds a direct cavity retreat after the last spray layer has returned to 0 deg.
// The current deepest-layer pose is omitted because it is the Cartesian start state.
bool buildZeroRotationRetreatWaypoints(
  const plasma_robot_interfaces::msg::SprayPath & path,
  std::vector<geometry_msgs::msg::Pose> * waypoints,
  RetreatWaypointResult * result,
  std::string * error);

// Builds a straight outward retreat from an arbitrary stopped cavity pose. The
// stopped tool orientation is preserved, so no recovery rotation occurs inside
// the cavity. The current TCP is projected onto the pre-entry plane along the
// configured outward cavity axis.
bool buildStoppedRetreatWaypoints(
  const plasma_robot_interfaces::msg::SprayPath & path,
  const geometry_msgs::msg::Pose & stopped_flange_pose,
  const StoppedRetreatLimits & limits,
  std::vector<geometry_msgs::msg::Pose> * waypoints,
  StoppedRetreatResult * result,
  std::string * error);

// Compares named joint samples without relying on message ordering. This is
// used as a stop-verification fallback when the controller's arm-status field
// is configured but is not refreshed after a stop command.
bool namedJointPositionsWithinTolerance(
  const std::vector<std::string> & reference_names,
  const std::vector<double> & reference_positions,
  const std::vector<std::string> & current_names,
  const std::vector<double> & current_positions,
  double tolerance_rad,
  double * observed_max_delta_rad = nullptr);

}  // namespace plasma_path_executor
