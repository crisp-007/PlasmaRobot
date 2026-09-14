#include "plasma_path_executor/path_validation.hpp"

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <set>

namespace plasma_path_executor
{
namespace
{

constexpr double kTwoPi = 6.28318530717958647692;
constexpr double kLimitTolerance = 1e-9;

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

bool pointIsFinite(const geometry_msgs::msg::Point & point)
{
  return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

}  // namespace

bool poseIsFiniteAndNormalized(const geometry_msgs::msg::Pose & pose)
{
  if (!pointIsFinite(pose.position)) {
    return false;
  }
  const double norm_squared =
    pose.orientation.x * pose.orientation.x +
    pose.orientation.y * pose.orientation.y +
    pose.orientation.z * pose.orientation.z +
    pose.orientation.w * pose.orientation.w;
  return std::isfinite(norm_squared) && norm_squared >= 0.99 * 0.99 &&
         norm_squared <= 1.01 * 1.01;
}

PoseError poseError(
  const geometry_msgs::msg::Pose & current,
  const geometry_msgs::msg::Pose & target)
{
  PoseError error;
  const double dx = current.position.x - target.position.x;
  const double dy = current.position.y - target.position.y;
  const double dz = current.position.z - target.position.z;
  error.translation_m = std::sqrt(dx * dx + dy * dy + dz * dz);

  const Eigen::Quaterniond current_q = quaternion(current);
  const Eigen::Quaterniond target_q = quaternion(target);
  const double dot = std::clamp(std::abs(current_q.dot(target_q)), 0.0, 1.0);
  error.rotation_rad = 2.0 * std::acos(dot);
  return error;
}

bool phaseAllowsPlasma(std::uint8_t motion_phase)
{
  using Point = plasma_robot_interfaces::msg::SprayPathPoint;
  return motion_phase == Point::PHASE_PROCESS_POSITIVE ||
         motion_phase == Point::PHASE_PROCESS_NEGATIVE;
}

bool nearestEquivalentRevolutePosition(
  double position,
  double reference,
  double lower_limit,
  double upper_limit,
  double * adjusted_position)
{
  if (!adjusted_position || !std::isfinite(position) || !std::isfinite(reference) ||
    !std::isfinite(lower_limit) || !std::isfinite(upper_limit) ||
    lower_limit > upper_limit) {
    return false;
  }

  const double minimum_turns = std::ceil(
    (lower_limit - position - kLimitTolerance) / kTwoPi);
  const double maximum_turns = std::floor(
    (upper_limit - position + kLimitTolerance) / kTwoPi);
  if (minimum_turns > maximum_turns) {
    return false;
  }

  const double nearest_turns = std::round((reference - position) / kTwoPi);
  const double selected_turns = std::clamp(
    nearest_turns, minimum_turns, maximum_turns);
  *adjusted_position = position + selected_turns * kTwoPi;
  return *adjusted_position >= lower_limit - kLimitTolerance &&
         *adjusted_position <= upper_limit + kLimitTolerance;
}

std::vector<double> revoluteIkSeedSchedule(
  double current_position,
  double lower_limit,
  double upper_limit,
  std::size_t requested_count)
{
  std::vector<double> seeds;
  if (requested_count == 0 || !std::isfinite(current_position) ||
    !std::isfinite(lower_limit) || !std::isfinite(upper_limit) ||
    lower_limit >= upper_limit || current_position < lower_limit - kLimitTolerance ||
    current_position > upper_limit + kLimitTolerance)
  {
    return seeds;
  }

  const auto append = [&](double candidate) {
      if (seeds.size() >= requested_count || !std::isfinite(candidate) ||
        candidate < lower_limit - kLimitTolerance ||
        candidate > upper_limit + kLimitTolerance)
      {
        return;
      }
      const bool duplicate = std::any_of(
        seeds.begin(), seeds.end(), [candidate](double seed) {
          return std::abs(seed - candidate) <= 1e-9;
        });
      if (!duplicate) {
        seeds.push_back(std::clamp(candidate, lower_limit, upper_limit));
      }
    };

  append(current_position);
  // Try another physical winding first. This is the important alternative when a
  // +/-180 degree process sweep would otherwise cross the bounded +/-2pi limit.
  append(current_position - kTwoPi);
  append(current_position + kTwoPi);

  const double midpoint = 0.5 * (lower_limit + upper_limit);
  append(midpoint);
  constexpr double kFractions[] = {
    0.25, 0.75, 0.125, 0.375, 0.625, 0.875, 0.0625, 0.9375};
  for (double fraction : kFractions) {
    append(lower_limit + fraction * (upper_limit - lower_limit));
  }
  return seeds;
}

bool validateEntryApproach(
  const geometry_msgs::msg::Pose & current_flange,
  const plasma_robot_interfaces::msg::SprayPath & path,
  const EntryApproachLimits & limits,
  EntryApproachResult * result,
  std::string * error)
{
  auto reject = [error](const std::string & message) {
      if (error) {
        *error = message;
      }
      return false;
    };

  if (path.points.empty()) {
    return reject("entry approach path has no points");
  }
  const auto & first_point = path.points.front();
  if (!poseIsFiniteAndNormalized(current_flange) ||
      !poseIsFiniteAndNormalized(first_point.tcp_pose) ||
      !poseIsFiniteAndNormalized(first_point.flange_pose) ||
      !first_point.flange_pose_valid) {
    return reject("entry approach contains an invalid flange or TCP pose");
  }
  if (!std::isfinite(limits.max_translation_m) || limits.max_translation_m <= 0.0 ||
      !std::isfinite(limits.max_rotation_rad) || limits.max_rotation_rad <= 0.0 ||
      !std::isfinite(limits.max_axis_error_rad) || limits.max_axis_error_rad <= 0.0 ||
      limits.max_axis_error_rad >= 1.570796327 ||
      !std::isfinite(limits.max_tool_to_path_axis_error_rad) ||
      limits.max_tool_to_path_axis_error_rad <= 0.0 ||
      limits.max_tool_to_path_axis_error_rad >= 1.570796327 ||
      !std::isfinite(limits.max_path_axis_offset_m) ||
      limits.max_path_axis_offset_m <= 0.0) {
    return reject("entry approach limits are invalid");
  }

  const Eigen::Quaterniond target_rotation = quaternion(first_point.flange_pose);
  const Eigen::Quaterniond current_rotation = quaternion(current_flange);
  const Eigen::Vector3d target_flange(
    first_point.flange_pose.position.x,
    first_point.flange_pose.position.y,
    first_point.flange_pose.position.z);
  const Eigen::Vector3d target_tcp(
    first_point.tcp_pose.position.x,
    first_point.tcp_pose.position.y,
    first_point.tcp_pose.position.z);
  const Eigen::Vector3d local_flange_to_tcp =
    target_rotation.inverse() * (target_tcp - target_flange);
  if (!local_flange_to_tcp.allFinite() || local_flange_to_tcp.norm() < 0.05) {
    return reject("entry approach cannot recover a valid flange-to-TCP tool axis");
  }

  const Eigen::Vector3d current_flange_position(
    current_flange.position.x, current_flange.position.y, current_flange.position.z);
  const Eigen::Vector3d current_axis =
    (current_rotation * local_flange_to_tcp).normalized();
  const Eigen::Vector3d current_tcp =
    current_flange_position + current_rotation * local_flange_to_tcp;
  const Eigen::Vector3d approach = target_tcp - current_tcp;

  Eigen::Vector3d path_axis;
  bool have_path_axis = false;
  for (std::size_t index = 1; index < path.points.size(); ++index) {
    const auto & candidate = path.points[index].tcp_pose.position;
    const Eigen::Vector3d delta(
      candidate.x - target_tcp.x(),
      candidate.y - target_tcp.y(),
      candidate.z - target_tcp.z());
    if (delta.norm() > 0.001) {
      path_axis = delta.normalized();
      have_path_axis = true;
      break;
    }
  }
  if (!have_path_axis) {
    return reject("entry approach requires at least two distinct layer TCP positions");
  }
  if (path_axis.dot(current_axis) < 0.0) {
    path_axis = -path_axis;
  }

  EntryApproachResult value;
  value.translation_m = approach.norm();
  value.rotation_rad = poseError(current_flange, first_point.flange_pose).rotation_rad;
  if (value.translation_m > 1e-6) {
    const double axis_dot = std::clamp(
      current_axis.dot(approach / value.translation_m), -1.0, 1.0);
    value.axis_error_rad = std::acos(axis_dot);
  }
  const double path_axis_dot = std::clamp(current_axis.dot(path_axis), -1.0, 1.0);
  value.tool_to_path_axis_error_rad = std::acos(path_axis_dot);
  const Eigen::Vector3d first_to_entry = current_tcp - target_tcp;
  value.path_axis_offset_m =
    (first_to_entry - path_axis * first_to_entry.dot(path_axis)).norm();
  if (result) {
    *result = value;
  }

  if (value.translation_m > limits.max_translation_m) {
    return reject("entry-to-first-point distance exceeds the configured limit");
  }
  if (value.rotation_rad > limits.max_rotation_rad) {
    return reject("entry-to-first-point rotation exceeds the configured limit");
  }
  if (value.translation_m > 1e-6 && value.axis_error_rad > limits.max_axis_error_rad) {
    return reject("entry-to-first-point direction is not aligned with the current tool axis");
  }
  if (value.tool_to_path_axis_error_rad > limits.max_tool_to_path_axis_error_rad) {
    return reject("current tool axis is not aligned with the planned cavity axis");
  }
  if (value.path_axis_offset_m > limits.max_path_axis_offset_m) {
    return reject("confirmed entry TCP is too far from the planned cavity axis");
  }
  return true;
}

bool validatePathStructure(
  const plasma_robot_interfaces::msg::SprayPath & path,
  const ValidationLimits & limits,
  std::string * error)
{
  auto reject = [error](const std::string & message) {
      if (error) {
        *error = message;
      }
      return false;
    };

  if (path.path_id.empty()) {
    return reject("path_id is empty");
  }
  if (path.header.frame_id != "baselink" || path.base_frame != "baselink") {
    return reject("path must be expressed in baselink");
  }
  if (path.gripper_frame != "Link6") {
    return reject("path gripper_frame must be Link6");
  }
  if (path.tool_frame != "plasma_motion_tcp") {
    return reject("path tool_frame must be plasma_motion_tcp");
  }
  if (!path.transform_valid) {
    return reject("path transform_valid is false");
  }
  if (path.points.empty()) {
    return reject("path has no points");
  }
  if (path.points.size() > limits.max_points) {
    return reject("path exceeds max_points");
  }
  const bool any_entry_geometry =
    path.cavity_mouth_tip_pose_valid || path.cavity_entry_tcp_pose_valid ||
    path.pre_entry_tcp_pose_valid ||
    path.cavity_axis_valid || path.cavity_entry_flange_pose_valid ||
    path.pre_entry_flange_pose_valid;
  if (any_entry_geometry) {
    if (!path.cavity_mouth_tip_pose_valid || !path.cavity_entry_tcp_pose_valid ||
        !path.pre_entry_tcp_pose_valid ||
        !path.cavity_axis_valid || !path.cavity_entry_flange_pose_valid ||
        !path.pre_entry_flange_pose_valid) {
      return reject("cavity approach geometry is incomplete");
    }
    if (!poseIsFiniteAndNormalized(path.cavity_mouth_tip_pose) ||
        !poseIsFiniteAndNormalized(path.cavity_entry_tcp_pose) ||
        !poseIsFiniteAndNormalized(path.pre_entry_tcp_pose) ||
        !poseIsFiniteAndNormalized(path.cavity_entry_flange_pose) ||
        !poseIsFiniteAndNormalized(path.pre_entry_flange_pose)) {
      return reject("cavity approach contains an invalid pose");
    }
    Eigen::Vector3d axis(
      path.cavity_axis.x, path.cavity_axis.y, path.cavity_axis.z);
    if (!axis.allFinite() || axis.norm() < 0.99 || axis.norm() > 1.01) {
      return reject("cavity axis is not a finite unit vector");
    }
    axis.normalize();
    if (!std::isfinite(path.pre_entry_distance_m) ||
        path.pre_entry_distance_m < 0.01 || path.pre_entry_distance_m > 0.25) {
      return reject("pre-entry distance is outside [0.01, 0.25] m");
    }
    if (!std::isfinite(path.entry_tip_standoff_m) ||
        path.entry_tip_standoff_m < 0.0 || path.entry_tip_standoff_m > 0.15) {
      return reject("entry tip standoff is outside [0, 0.15] m");
    }
    const Eigen::Vector3d entry(
      path.cavity_entry_tcp_pose.position.x,
      path.cavity_entry_tcp_pose.position.y,
      path.cavity_entry_tcp_pose.position.z);
    const Eigen::Vector3d pre_entry(
      path.pre_entry_tcp_pose.position.x,
      path.pre_entry_tcp_pose.position.y,
      path.pre_entry_tcp_pose.position.z);
    const Eigen::Vector3d first(
      path.points.front().tcp_pose.position.x,
      path.points.front().tcp_pose.position.y,
      path.points.front().tcp_pose.position.z);
    const Eigen::Vector3d entry_to_pre = pre_entry - entry;
    const double outward = entry_to_pre.dot(axis);
    const double pre_offset = (entry_to_pre - outward * axis).norm();
    const Eigen::Vector3d entry_to_first = first - entry;
    const double inward = -entry_to_first.dot(axis);
    const double first_offset = (entry_to_first + inward * axis).norm();
    if (outward <= 0.0 || std::abs(outward - path.pre_entry_distance_m) > 0.005 ||
        pre_offset > 0.002) {
      return reject("pre-entry pose is not the configured outward point on the cavity axis");
    }
    if (inward <= 0.0005 || first_offset > 0.020) {
      return reject("entry pose is not outward and coaxial with the first spray layer");
    }
  }
  if (!std::isfinite(limits.max_segment_translation_m) ||
      limits.max_segment_translation_m <= 0.0 ||
      !std::isfinite(limits.max_segment_rotation_rad) ||
      limits.max_segment_rotation_rad <= 0.0 ||
      !std::isfinite(limits.min_spray_distance_m) ||
      !std::isfinite(limits.max_spray_distance_m) ||
      limits.min_spray_distance_m <= 0.0 ||
      limits.max_spray_distance_m <= limits.min_spray_distance_m ||
      !std::isfinite(limits.max_spray_direction_error_rad) ||
      limits.max_spray_direction_error_rad <= 0.0 ||
      limits.max_spray_direction_error_rad >= 1.570796327) {
    return reject("validation limits are invalid");
  }

  std::int32_t previous_sequence = std::numeric_limits<std::int32_t>::min();
  std::set<std::int32_t> finished_layers;
  std::int32_t current_layer = path.points.front().layer_index;
  int expected_phase = 0;
  double previous_angle = 0.0;
  double phase_start_angle = 0.0;
  double phase_extreme_angle = 0.0;
  geometry_msgs::msg::Point layer_tcp = path.points.front().tcp_pose.position;
  bool layer_has_process_point = false;
  bool first_point_in_layer = true;
  constexpr double angle_tolerance_deg = 1.0;
  constexpr double fixed_tcp_tolerance_m = 0.0002;

  auto finish_phase = [&](int phase, double final_angle) {
      if (phase == 0 && std::abs(phase_extreme_angle - 180.0) > angle_tolerance_deg) {
        return reject("positive spray sweep does not reach +180 degrees");
      }
      if (phase == 1 && std::abs(final_angle) > angle_tolerance_deg) {
        return reject("positive return sweep does not return to 0 degrees");
      }
      if (phase == 2 && std::abs(phase_extreme_angle + 180.0) > angle_tolerance_deg) {
        return reject("negative spray sweep does not reach -180 degrees");
      }
      if (phase == 3 && std::abs(final_angle) > angle_tolerance_deg) {
        return reject("negative return sweep does not return to 0 degrees");
      }
      return true;
    };

  for (std::size_t index = 0; index < path.points.size(); ++index) {
    const auto & point = path.points[index];
    if (!poseIsFiniteAndNormalized(point.tcp_pose) ||
        !poseIsFiniteAndNormalized(point.spray_outlet_pose) ||
        !poseIsFiniteAndNormalized(point.flange_pose) ||
        !pointIsFinite(point.safety_point) ||
        !pointIsFinite(point.surface_target)) {
      return reject("path point " + std::to_string(index) + " contains an invalid pose or point");
    }
    if (!point.safety_point_valid || !point.spray_outlet_pose_valid) {
      return reject("path point " + std::to_string(index) +
             " has no valid nozzle-tip clearance point or spray outlet pose");
    }
    if (!point.flange_pose_valid) {
      return reject("path point " + std::to_string(index) + " has no valid flange pose");
    }

    const Eigen::Vector3d tcp_position(
      point.tcp_pose.position.x, point.tcp_pose.position.y, point.tcp_pose.position.z);
    const Eigen::Vector3d surface_target(
      point.surface_target.x, point.surface_target.y, point.surface_target.z);
    const Eigen::Vector3d tcp_to_target = surface_target - tcp_position;
    const double spray_distance = tcp_to_target.norm();
    if (spray_distance < limits.min_spray_distance_m ||
        spray_distance > limits.max_spray_distance_m) {
      return reject("path point " + std::to_string(index) +
             " spray distance is outside the configured non-contact range");
    }
    const Eigen::Vector3d spray_axis =
      quaternion(point.tcp_pose) * Eigen::Vector3d::UnitZ();
    const double direction_dot = std::clamp(
      spray_axis.dot(tcp_to_target / spray_distance), -1.0, 1.0);
    const double direction_error = std::acos(direction_dot);
    if (direction_error > limits.max_spray_direction_error_rad) {
      return reject("path point " + std::to_string(index) +
             " surface target is not on the TCP +Z spray ray");
    }
    if (point.sequence_index <= previous_sequence) {
      return reject("sequence_index must be strictly increasing");
    }
    previous_sequence = point.sequence_index;
    if (point.motion_phase >
        plasma_robot_interfaces::msg::SprayPathPoint::PHASE_LAYER_TRANSITION) {
      return reject("path point " + std::to_string(index) + " has an unknown motion phase");
    }
    if (point.plasma_enabled != phaseAllowsPlasma(point.motion_phase)) {
      return reject("plasma state does not match the four-stage spray sequence");
    }

    if (point.layer_index != current_layer) {
      if (expected_phase != 3 || !finish_phase(3, previous_angle)) {
        return false;
      }
      if (!layer_has_process_point) {
        return reject("layer contains no spray sweep");
      }
      finished_layers.insert(current_layer);
      if (finished_layers.count(point.layer_index) > 0) {
        return reject("a completed layer appears again later in the path");
      }
      current_layer = point.layer_index;
      expected_phase = 0;
      previous_angle = 0.0;
      phase_start_angle = 0.0;
      phase_extreme_angle = 0.0;
      layer_tcp = point.tcp_pose.position;
      layer_has_process_point = false;
      first_point_in_layer = true;
    }

    const double tcp_dx = point.tcp_pose.position.x - layer_tcp.x;
    const double tcp_dy = point.tcp_pose.position.y - layer_tcp.y;
    const double tcp_dz = point.tcp_pose.position.z - layer_tcp.z;
    if (std::sqrt(tcp_dx * tcp_dx + tcp_dy * tcp_dy + tcp_dz * tcp_dz) >
        fixed_tcp_tolerance_m) {
      return reject("TCP position changes during a layer rotation");
    }

    if (point.motion_phase ==
        plasma_robot_interfaces::msg::SprayPathPoint::PHASE_LAYER_TRANSITION) {
      if (!first_point_in_layer || index == 0 || std::abs(point.joint6_geometric_deg) >
          angle_tolerance_deg) {
        return reject("layer transition must be the first zero-degree point of a new layer");
      }
      first_point_in_layer = false;
    } else {
      const int phase = static_cast<int>(point.motion_phase);
      if (phase < expected_phase || phase > expected_phase + 1 || phase > 3) {
        return reject("spray phases are not ordered as +180/return/-180/return");
      }
      if (phase == expected_phase + 1) {
        if (!finish_phase(expected_phase, previous_angle)) {
          return false;
        }
        expected_phase = phase;
        phase_start_angle = point.joint6_geometric_deg;
        phase_extreme_angle = point.joint6_geometric_deg;
      }

      const double angle = point.joint6_geometric_deg;
      if (!std::isfinite(angle)) {
        return reject("spray point contains a non-finite rotation angle");
      }
      if (expected_phase == 0) {
        if (!layer_has_process_point && std::abs(angle) > angle_tolerance_deg) {
          return reject("positive spray sweep must start at 0 degrees");
        }
        if (angle + angle_tolerance_deg < previous_angle || angle < -angle_tolerance_deg ||
            angle > 180.0 + angle_tolerance_deg) {
          return reject("positive spray sweep angle is not monotonic from 0 to +180 degrees");
        }
        phase_extreme_angle = std::max(phase_extreme_angle, angle);
        layer_has_process_point = true;
      } else if (expected_phase == 1) {
        if (angle > previous_angle + angle_tolerance_deg || angle < -angle_tolerance_deg ||
            angle > 180.0 + angle_tolerance_deg) {
          return reject("positive return angle is not monotonic toward 0 degrees");
        }
      } else if (expected_phase == 2) {
        if (std::abs(phase_start_angle) > angle_tolerance_deg ||
            angle > previous_angle + angle_tolerance_deg || angle < -180.0 - angle_tolerance_deg ||
            angle > angle_tolerance_deg) {
          return reject("negative spray sweep angle is not monotonic from 0 to -180 degrees");
        }
        phase_extreme_angle = std::min(phase_extreme_angle, angle);
      } else if (expected_phase == 3) {
        if (angle + angle_tolerance_deg < previous_angle || angle < -180.0 - angle_tolerance_deg ||
            angle > angle_tolerance_deg) {
          return reject("negative return angle is not monotonic toward 0 degrees");
        }
      }
      previous_angle = angle;
      first_point_in_layer = false;
    }
    if (index > 0) {
      const PoseError segment = poseError(path.points[index - 1].flange_pose, point.flange_pose);
      if (segment.translation_m > limits.max_segment_translation_m) {
        return reject("path contains an excessive translation between adjacent points");
      }
      if (segment.rotation_rad > limits.max_segment_rotation_rad) {
        return reject("path contains an excessive rotation between adjacent points");
      }
    }
  }
  if (expected_phase != 3 || !finish_phase(3, previous_angle)) {
    return false;
  }
  if (!layer_has_process_point) {
    return reject("last layer contains no spray sweep");
  }
  return true;
}

std::size_t firstLayerPointCount(
  const plasma_robot_interfaces::msg::SprayPath & path)
{
  if (path.points.empty()) {
    return 0;
  }
  const std::int32_t first_layer = path.points.front().layer_index;
  const auto next_layer = std::find_if(
    path.points.begin(), path.points.end(),
    [first_layer](const auto & point) {
      return point.layer_index != first_layer;
    });
  return static_cast<std::size_t>(std::distance(path.points.begin(), next_layer));
}

}  // namespace plasma_path_executor
