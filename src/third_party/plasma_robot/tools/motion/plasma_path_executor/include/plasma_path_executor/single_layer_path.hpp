#pragma once

#include <Eigen/Geometry>
#include <plasma_robot_interfaces/msg/spray_path.hpp>

#include <string>

namespace plasma_path_executor
{

struct SingleLayerPathConfig
{
  std::string path_id = "single_layer_commission";
  double angular_step_deg = 5.0;
  double virtual_spray_distance_m = 0.05;
  int layer_count = 1;
  double layer_step_m = 0.010;
};

plasma_robot_interfaces::msg::SprayPath buildSingleLayerCommissioningPath(
  const Eigen::Isometry3d & base_to_flange,
  const Eigen::Isometry3d & flange_to_tcp,
  const Eigen::Isometry3d & tcp_to_nozzle_tip,
  const SingleLayerPathConfig & config);

}  // namespace plasma_path_executor
