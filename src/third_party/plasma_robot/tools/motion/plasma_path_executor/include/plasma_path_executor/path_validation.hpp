#pragma once

#include <geometry_msgs/msg/pose.hpp>
#include <plasma_robot_interfaces/msg/spray_path.hpp>

#include <cstddef>
#include <string>
#include <vector>

namespace plasma_path_executor
{

struct ValidationLimits
{
  std::size_t max_points = 20000;
  double max_segment_translation_m = 0.05;
  double max_segment_rotation_rad = 0.35;
  double min_spray_distance_m = 0.005;
  double max_spray_distance_m = 1.0;
  double max_spray_direction_error_rad = 0.034906585;
};

struct PoseError
{
  double translation_m = 0.0;
  double rotation_rad = 0.0;
};

struct EntryApproachLimits
{
  double max_translation_m = 0.2;
  double max_rotation_rad = 3.14159265358979323846;
  double max_axis_error_rad = 0.3490658503988659;
  double max_tool_to_path_axis_error_rad = 0.08726646259971647;
  double max_path_axis_offset_m = 0.02;
};

struct EntryApproachResult
{
  double translation_m = 0.0;
  double rotation_rad = 0.0;
  double axis_error_rad = 0.0;
  double tool_to_path_axis_error_rad = 0.0;
  double path_axis_offset_m = 0.0;
};

bool validatePathStructure(
  const plasma_robot_interfaces::msg::SprayPath & path,
  const ValidationLimits & limits,
  std::string * error);

PoseError poseError(
  const geometry_msgs::msg::Pose & current,
  const geometry_msgs::msg::Pose & target);

bool poseIsFiniteAndNormalized(const geometry_msgs::msg::Pose & pose);

bool phaseAllowsPlasma(std::uint8_t motion_phase);

bool nearestEquivalentRevolutePosition(
  double position,
  double reference,
  double lower_limit,
  double upper_limit,
  double * adjusted_position);

std::vector<double> revoluteIkSeedSchedule(
  double current_position,
  double lower_limit,
  double upper_limit,
  std::size_t requested_count);

std::size_t firstLayerPointCount(
  const plasma_robot_interfaces::msg::SprayPath & path);

bool validateEntryApproach(
  const geometry_msgs::msg::Pose & current_flange,
  const plasma_robot_interfaces::msg::SprayPath & path,
  const EntryApproachLimits & limits,
  EntryApproachResult * result,
  std::string * error);

}  // namespace plasma_path_executor
