#include "plasma_path_executor/single_layer_path.hpp"

#include <geometry_msgs/msg/pose.hpp>
#include <plasma_robot_interfaces/msg/spray_path_point.hpp>

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <stdexcept>

namespace plasma_path_executor
{
namespace
{

constexpr double kDegToRad = 3.14159265358979323846 / 180.0;

geometry_msgs::msg::Pose isometryToPose(const Eigen::Isometry3d & transform)
{
  const Eigen::Quaterniond quaternion(transform.linear());
  geometry_msgs::msg::Pose pose;
  pose.position.x = transform.translation().x();
  pose.position.y = transform.translation().y();
  pose.position.z = transform.translation().z();
  pose.orientation.x = quaternion.x();
  pose.orientation.y = quaternion.y();
  pose.orientation.z = quaternion.z();
  pose.orientation.w = quaternion.w();
  return pose;
}

void validateTransform(const Eigen::Isometry3d & transform, const char * name)
{
  if (!transform.matrix().allFinite() || !transform.linear().isUnitary(1e-6) ||
    transform.linear().determinant() < 0.999999)
  {
    throw std::runtime_error(std::string(name) + " is not a finite rigid transform");
  }
}

}  // namespace

plasma_robot_interfaces::msg::SprayPath buildSingleLayerCommissioningPath(
  const Eigen::Isometry3d & base_to_flange,
  const Eigen::Isometry3d & flange_to_tcp,
  const Eigen::Isometry3d & tcp_to_nozzle_tip,
  const SingleLayerPathConfig & config)
{
  validateTransform(base_to_flange, "base_to_flange");
  validateTransform(flange_to_tcp, "flange_to_tcp");
  validateTransform(tcp_to_nozzle_tip, "tcp_to_nozzle_tip");
  if (config.path_id.empty()) {
    throw std::runtime_error("single-layer path_id is empty");
  }
  if (!std::isfinite(config.angular_step_deg) || config.angular_step_deg <= 0.0 ||
    config.angular_step_deg > 20.0)
  {
    throw std::runtime_error("angular_step_deg must be in (0, 20]");
  }
  if (!std::isfinite(config.virtual_spray_distance_m) ||
    config.virtual_spray_distance_m < 0.005 || config.virtual_spray_distance_m > 1.0)
  {
    throw std::runtime_error("virtual_spray_distance_m must be in [0.005, 1.0]");
  }
  if (config.layer_count < 1 || config.layer_count > 20) {
    throw std::runtime_error("layer_count must be in [1, 20]");
  }
  if (!std::isfinite(config.layer_step_m) || config.layer_step_m <= 0.0 ||
    config.layer_step_m > 0.05)
  {
    throw std::runtime_error("layer_step_m must be in (0, 0.05]");
  }

  using Point = plasma_robot_interfaces::msg::SprayPathPoint;
  plasma_robot_interfaces::msg::SprayPath path;
  path.header.frame_id = "baselink";
  path.path_id = config.path_id;
  path.tool_frame = "plasma_motion_tcp";
  path.base_frame = "baselink";
  path.gripper_frame = "Link6";
  path.capture_gripper_pose = isometryToPose(base_to_flange);
  path.capture_gripper_pose_valid = true;
  path.transform_valid = true;
  path.execution_permitted = true;
  path.calibration_id = "explicit_layer_commissioning_with_nominal_tcp";
  path.status_message =
    "operator-requested current-pose single-layer path; plasma output remains disabled";

  const Eigen::Isometry3d base_to_tcp_first_layer = base_to_flange * flange_to_tcp;
  std::int32_t sequence_index = 0;

  auto append_point = [&](const Eigen::Isometry3d & base_to_tcp_zero, int layer_index,
      double angle_deg, std::uint8_t phase, bool plasma_enabled) {
      const double angle_rad = angle_deg * kDegToRad;
      Eigen::Isometry3d base_to_tcp = base_to_tcp_zero;
      base_to_tcp.linear() = base_to_tcp_zero.linear() *
        Eigen::AngleAxisd(angle_rad, Eigen::Vector3d::UnitX()).toRotationMatrix();
      const Eigen::Isometry3d base_to_target_flange = base_to_tcp * flange_to_tcp.inverse();
      const Eigen::Isometry3d base_to_nozzle_tip = base_to_tcp * tcp_to_nozzle_tip;

      Point point;
      point.tcp_pose = isometryToPose(base_to_tcp);
      point.spray_outlet_pose = point.tcp_pose;
      point.spray_outlet_pose_valid = true;
      point.flange_pose = isometryToPose(base_to_target_flange);
      point.flange_pose_valid = true;
      point.safety_point.x = base_to_nozzle_tip.translation().x();
      point.safety_point.y = base_to_nozzle_tip.translation().y();
      point.safety_point.z = base_to_nozzle_tip.translation().z();
      point.safety_point_valid = true;
      const Eigen::Vector3d surface_target = base_to_tcp.translation() +
        config.virtual_spray_distance_m * base_to_tcp.linear() * Eigen::Vector3d::UnitZ();
      point.surface_target.x = surface_target.x();
      point.surface_target.y = surface_target.y();
      point.surface_target.z = surface_target.z();
      point.sequence_index = sequence_index++;
      point.layer_index = layer_index;
      point.motion_phase = phase;
      point.plasma_enabled = plasma_enabled;
      point.joint6_geometric_deg = angle_deg;
      path.points.push_back(std::move(point));
    };

  auto append_sweep = [&](const Eigen::Isometry3d & base_to_tcp_zero, int layer_index,
      double start_deg, double end_deg, std::uint8_t phase, bool plasma_enabled) {
      const int steps = std::max(1, static_cast<int>(
        std::ceil(std::abs(end_deg - start_deg) / config.angular_step_deg)));
      for (int step = 0; step <= steps; ++step) {
        const double ratio = static_cast<double>(step) / static_cast<double>(steps);
        append_point(base_to_tcp_zero, layer_index,
          start_deg + ratio * (end_deg - start_deg), phase, plasma_enabled);
      }
    };

  for (int layer_index = 0; layer_index < config.layer_count; ++layer_index) {
    Eigen::Isometry3d base_to_tcp_layer = base_to_tcp_first_layer;
    base_to_tcp_layer.translation() += static_cast<double>(layer_index) *
      config.layer_step_m * base_to_tcp_first_layer.linear() * Eigen::Vector3d::UnitX();
    if (layer_index > 0) {
      append_point(base_to_tcp_layer, layer_index, 0.0, Point::PHASE_LAYER_TRANSITION, false);
    }
    append_sweep(base_to_tcp_layer, layer_index,
      0.0, 180.0, Point::PHASE_PROCESS_POSITIVE, true);
    append_sweep(base_to_tcp_layer, layer_index,
      180.0, 0.0, Point::PHASE_RETURN_FROM_POSITIVE, false);
    append_sweep(base_to_tcp_layer, layer_index,
      0.0, -180.0, Point::PHASE_PROCESS_NEGATIVE, true);
    append_sweep(base_to_tcp_layer, layer_index,
      -180.0, 0.0, Point::PHASE_RETURN_FROM_NEGATIVE, false);
  }
  return path;
}

}  // namespace plasma_path_executor
