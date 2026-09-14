#include "plasma_path_executor/retreat_path.hpp"

#include "plasma_path_executor/path_validation.hpp"

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

namespace plasma_path_executor
{
namespace
{

using Point = plasma_robot_interfaces::msg::SprayPathPoint;

constexpr double kZeroAngleToleranceDeg = 1e-6;

struct LayerZeroPose
{
  std::int32_t layer_index = 0;
  geometry_msgs::msg::Pose pose;
  std::size_t source_index = 0;
};

bool fail(std::string * error, const std::string & message)
{
  if (error) {
    *error = message;
  }
  return false;
}

bool isFinalZeroPoint(const Point & point)
{
  return point.motion_phase == Point::PHASE_RETURN_FROM_NEGATIVE &&
         std::isfinite(point.joint6_geometric_deg) &&
         std::abs(point.joint6_geometric_deg) <= kZeroAngleToleranceDeg;
}

Eigen::Quaterniond quaternion(const geometry_msgs::msg::Pose & pose)
{
  Eigen::Quaterniond value(
    pose.orientation.w,
    pose.orientation.x,
    pose.orientation.y,
    pose.orientation.z);
  value.normalize();
  return value;
}

}  // namespace

bool buildZeroRotationRetreatWaypoints(
  const plasma_robot_interfaces::msg::SprayPath & path,
  std::vector<geometry_msgs::msg::Pose> * waypoints,
  RetreatWaypointResult * result,
  std::string * error)
{
  if (waypoints) {
    waypoints->clear();
  }
  if (result) {
    *result = RetreatWaypointResult{};
  }
  if (!waypoints || path.points.empty()) {
    return fail(error, "spray path is empty or retreat waypoint output is null");
  }
  if (!path.cavity_entry_flange_pose_valid || !path.pre_entry_flange_pose_valid ||
    !poseIsFiniteAndNormalized(path.cavity_entry_flange_pose) ||
    !poseIsFiniteAndNormalized(path.pre_entry_flange_pose))
  {
    return fail(error, "valid safe-entry and pre-entry flange poses are required");
  }

  std::vector<std::int32_t> layer_order;
  std::vector<LayerZeroPose> layer_zero_poses;
  for (std::size_t index = 0; index < path.points.size(); ++index) {
    const auto & point = path.points[index];
    if (layer_order.empty() || layer_order.back() != point.layer_index) {
      if (std::find(layer_order.begin(), layer_order.end(), point.layer_index) !=
        layer_order.end())
      {
        return fail(error, "a completed spray layer appears again later in the path");
      }
      layer_order.push_back(point.layer_index);
    }
    if (!isFinalZeroPoint(point)) {
      continue;
    }
    if (!point.flange_pose_valid || !poseIsFiniteAndNormalized(point.flange_pose)) {
      return fail(
        error, "layer " + std::to_string(point.layer_index) +
        " has an invalid final zero-degree flange pose");
    }
    const auto duplicate = std::find_if(
      layer_zero_poses.begin(), layer_zero_poses.end(),
      [&point](const LayerZeroPose & value) {
        return value.layer_index == point.layer_index;
      });
    if (duplicate != layer_zero_poses.end()) {
      return fail(
        error, "layer " + std::to_string(point.layer_index) +
        " has more than one final zero-degree return point");
    }
    layer_zero_poses.push_back({point.layer_index, point.flange_pose, index});
  }

  if (layer_zero_poses.size() != layer_order.size()) {
    return fail(error, "every spray layer must end at joint6 geometric 0 degrees before retreat");
  }
  for (std::size_t index = 0; index < layer_order.size(); ++index) {
    if (layer_zero_poses[index].layer_index != layer_order[index]) {
      return fail(error, "spray layer zero-degree return points do not follow layer order");
    }
  }
  if (layer_zero_poses.back().source_index != path.points.size() - 1U) {
    return fail(error, "spray path does not finish at the last layer zero-degree return pose");
  }

  const auto fixed_orientation = layer_zero_poses.back().pose.orientation;
  waypoints->reserve(layer_zero_poses.size() + 1U);
  if (result) {
    result->layer_count = layer_zero_poses.size();
    result->traversed_layer_indices.reserve(layer_zero_poses.size() - 1U);
  }

  // The robot already occupies the last element. Traverse prior layer centers in reverse.
  for (std::size_t index = layer_zero_poses.size() - 1U; index > 0U; --index) {
    geometry_msgs::msg::Pose waypoint = layer_zero_poses[index - 1U].pose;
    waypoint.orientation = fixed_orientation;
    waypoints->push_back(std::move(waypoint));
    if (result) {
      result->traversed_layer_indices.push_back(layer_zero_poses[index - 1U].layer_index);
    }
  }

  geometry_msgs::msg::Pose safe_entry = path.cavity_entry_flange_pose;
  safe_entry.orientation = fixed_orientation;
  waypoints->push_back(std::move(safe_entry));
  geometry_msgs::msg::Pose pre_entry = path.pre_entry_flange_pose;
  pre_entry.orientation = fixed_orientation;
  waypoints->push_back(std::move(pre_entry));
  return true;
}

bool buildStoppedRetreatWaypoints(
  const plasma_robot_interfaces::msg::SprayPath & path,
  const geometry_msgs::msg::Pose & stopped_flange_pose,
  const StoppedRetreatLimits & limits,
  std::vector<geometry_msgs::msg::Pose> * waypoints,
  StoppedRetreatResult * result,
  std::string * error)
{
  if (waypoints) {
    waypoints->clear();
  }
  if (result) {
    const StoppedRetreatResult empty_result;
    *result = empty_result;
  }
  if (!waypoints || path.points.empty()) {
    return fail(error, "spray path is empty or stopped-retreat waypoint output is null");
  }
  if (!std::isfinite(limits.max_axis_offset_m) || limits.max_axis_offset_m <= 0.0 ||
    !std::isfinite(limits.min_outward_distance_m) ||
    limits.min_outward_distance_m <= 0.0 ||
    !std::isfinite(limits.max_outward_distance_m) ||
    limits.max_outward_distance_m <= limits.min_outward_distance_m)
  {
    return fail(error, "stopped-retreat limits are invalid");
  }
  const auto & reference = path.points.front();
  if (!path.cavity_axis_valid || !path.pre_entry_tcp_pose_valid ||
    !poseIsFiniteAndNormalized(path.pre_entry_tcp_pose) ||
    !poseIsFiniteAndNormalized(stopped_flange_pose) ||
    !reference.flange_pose_valid ||
    !poseIsFiniteAndNormalized(reference.flange_pose) ||
    !poseIsFiniteAndNormalized(reference.tcp_pose))
  {
    return fail(error, "stopped retreat requires valid axis, TCP, and flange geometry");
  }

  Eigen::Vector3d outward_axis(
    path.cavity_axis.x, path.cavity_axis.y, path.cavity_axis.z);
  if (!outward_axis.allFinite() || outward_axis.norm() < 1e-6) {
    return fail(error, "stopped retreat has an invalid cavity axis");
  }
  outward_axis.normalize();

  const Eigen::Quaterniond reference_rotation = quaternion(reference.flange_pose);
  const Eigen::Quaterniond stopped_rotation = quaternion(stopped_flange_pose);
  const Eigen::Vector3d reference_flange(
    reference.flange_pose.position.x,
    reference.flange_pose.position.y,
    reference.flange_pose.position.z);
  const Eigen::Vector3d reference_tcp(
    reference.tcp_pose.position.x,
    reference.tcp_pose.position.y,
    reference.tcp_pose.position.z);
  const Eigen::Vector3d local_flange_to_tcp =
    reference_rotation.inverse() * (reference_tcp - reference_flange);
  if (!local_flange_to_tcp.allFinite() || local_flange_to_tcp.norm() < 0.05 ||
    local_flange_to_tcp.norm() > 1.0)
  {
    return fail(error, "stopped retreat cannot recover a valid flange-to-TCP offset");
  }

  const Eigen::Vector3d stopped_flange(
    stopped_flange_pose.position.x,
    stopped_flange_pose.position.y,
    stopped_flange_pose.position.z);
  const Eigen::Vector3d stopped_tcp =
    stopped_flange + stopped_rotation * local_flange_to_tcp;
  const Eigen::Vector3d pre_entry_tcp(
    path.pre_entry_tcp_pose.position.x,
    path.pre_entry_tcp_pose.position.y,
    path.pre_entry_tcp_pose.position.z);
  const Eigen::Vector3d stopped_to_pre_entry = pre_entry_tcp - stopped_tcp;
  const double outward_distance = stopped_to_pre_entry.dot(outward_axis);
  const double axis_offset =
    (stopped_to_pre_entry - outward_axis * outward_distance).norm();
  if (!std::isfinite(outward_distance) || !std::isfinite(axis_offset)) {
    return fail(error, "stopped retreat produced a non-finite axial projection");
  }
  if (axis_offset > limits.max_axis_offset_m) {
    return fail(
      error, "stopped TCP is too far from the reviewed cavity axis; offset_m=" +
      std::to_string(axis_offset));
  }
  if (outward_distance < limits.min_outward_distance_m) {
    return fail(error, "stopped TCP is already at or outside the pre-entry plane");
  }
  if (outward_distance > limits.max_outward_distance_m) {
    return fail(
      error, "stopped retreat distance exceeds the configured limit; distance_m=" +
      std::to_string(outward_distance));
  }

  geometry_msgs::msg::Pose target = stopped_flange_pose;
  const Eigen::Vector3d target_flange = stopped_flange + outward_axis * outward_distance;
  target.position.x = target_flange.x();
  target.position.y = target_flange.y();
  target.position.z = target_flange.z();
  waypoints->push_back(target);
  if (result) {
    result->outward_distance_m = outward_distance;
    result->axis_offset_m = axis_offset;
    result->target_flange_pose = target;
  }
  return true;
}

bool namedJointPositionsWithinTolerance(
  const std::vector<std::string> & reference_names,
  const std::vector<double> & reference_positions,
  const std::vector<std::string> & current_names,
  const std::vector<double> & current_positions,
  double tolerance_rad,
  double * observed_max_delta_rad)
{
  if (observed_max_delta_rad) {
    *observed_max_delta_rad = 0.0;
  }
  if (reference_names.empty() || reference_positions.size() < reference_names.size() ||
    current_positions.size() < current_names.size() ||
    !std::isfinite(tolerance_rad) || tolerance_rad <= 0.0)
  {
    return false;
  }

  double max_delta = 0.0;
  for (std::size_t reference_index = 0;
    reference_index < reference_names.size(); ++reference_index)
  {
    const auto current = std::find(
      current_names.begin(), current_names.end(), reference_names[reference_index]);
    if (current == current_names.end()) {
      return false;
    }
    const auto current_index = static_cast<std::size_t>(
      std::distance(current_names.begin(), current));
    const double reference_position = reference_positions[reference_index];
    const double current_position = current_positions[current_index];
    if (!std::isfinite(reference_position) || !std::isfinite(current_position)) {
      return false;
    }
    max_delta = std::max(max_delta, std::abs(current_position - reference_position));
  }
  if (observed_max_delta_rad) {
    *observed_max_delta_rad = max_delta;
  }
  return max_delta <= tolerance_rad;
}

}  // namespace plasma_path_executor
