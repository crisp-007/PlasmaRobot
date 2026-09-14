#include "plasma_path_executor/path_validation.hpp"
#include "plasma_path_executor/single_layer_path.hpp"

#include <gtest/gtest.h>

#include <Eigen/Geometry>

#include <cmath>
#include <string>

namespace
{

Eigen::Vector3d position(const geometry_msgs::msg::Pose & pose)
{
  return Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
}

TEST(SingleLayerPath, KeepsPhysicalSprayTcpFixedForFullSequence)
{
  Eigen::Isometry3d base_to_flange = Eigen::Isometry3d::Identity();
  base_to_flange.translation() = Eigen::Vector3d(0.4, -0.1, 0.5);

  Eigen::Isometry3d flange_to_tcp = Eigen::Isometry3d::Identity();
  flange_to_tcp.translation() = Eigen::Vector3d(0.002, 0.0, 0.310);
  Eigen::Isometry3d tcp_to_nozzle_tip = Eigen::Isometry3d::Identity();
  tcp_to_nozzle_tip.translation() = Eigen::Vector3d(0.008, 0.0, -0.002);

  plasma_path_executor::SingleLayerPathConfig config;
  config.path_id = "unit_test_single_layer";
  config.angular_step_deg = 5.0;
  config.virtual_spray_distance_m = 0.05;
  const auto path = plasma_path_executor::buildSingleLayerCommissioningPath(
    base_to_flange, flange_to_tcp, tcp_to_nozzle_tip, config);

  ASSERT_EQ(path.points.size(), 148U);
  EXPECT_TRUE(path.execution_permitted);
  EXPECT_EQ(path.points.front().joint6_geometric_deg, 0.0);
  EXPECT_EQ(path.points[36].joint6_geometric_deg, 180.0);
  EXPECT_EQ(path.points[73].joint6_geometric_deg, 0.0);
  EXPECT_EQ(path.points[110].joint6_geometric_deg, -180.0);
  EXPECT_EQ(path.points.back().joint6_geometric_deg, 0.0);

  const Eigen::Vector3d expected_tcp = position(path.points.front().tcp_pose);
  for (const auto & point : path.points) {
    EXPECT_LT((position(point.tcp_pose) - expected_tcp).norm(), 1e-12);
    EXPECT_LT((position(point.spray_outlet_pose) - expected_tcp).norm(), 1e-12);
    const Eigen::Vector3d target(
      point.surface_target.x, point.surface_target.y, point.surface_target.z);
    EXPECT_NEAR((target - expected_tcp).norm(), 0.05, 1e-12);
  }

  std::string error;
  EXPECT_TRUE(plasma_path_executor::validatePathStructure(
    path, plasma_path_executor::ValidationLimits{}, &error)) << error;
}

TEST(SingleLayerPath, BuildsTwoLayersWithPositiveToolAxisTransition)
{
  const Eigen::Isometry3d base_to_flange = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d flange_to_tcp = Eigen::Isometry3d::Identity();
  flange_to_tcp.translation() = Eigen::Vector3d(0.002, 0.0, 0.310);
  Eigen::Isometry3d tcp_to_nozzle_tip = Eigen::Isometry3d::Identity();
  tcp_to_nozzle_tip.translation() = Eigen::Vector3d(0.008, 0.0, -0.002);

  plasma_path_executor::SingleLayerPathConfig config;
  config.path_id = "unit_test_two_layers";
  config.layer_count = 2;
  config.layer_step_m = 0.010;
  const auto path = plasma_path_executor::buildSingleLayerCommissioningPath(
    base_to_flange, flange_to_tcp, tcp_to_nozzle_tip, config);

  ASSERT_EQ(path.points.size(), 297U);
  EXPECT_EQ(path.points[147].layer_index, 0);
  EXPECT_EQ(path.points[148].layer_index, 1);
  EXPECT_EQ(path.points[148].motion_phase,
    plasma_robot_interfaces::msg::SprayPathPoint::PHASE_LAYER_TRANSITION);
  EXPECT_FALSE(path.points[148].plasma_enabled);
  EXPECT_DOUBLE_EQ(path.points[148].joint6_geometric_deg, 0.0);

  const Eigen::Vector3d first_layer_tcp = position(path.points.front().tcp_pose);
  const Eigen::Vector3d second_layer_tcp = position(path.points[148].tcp_pose);
  EXPECT_TRUE((second_layer_tcp - first_layer_tcp).isApprox(
    Eigen::Vector3d(0.010, 0.0, 0.0), 1e-12));
  for (std::size_t index = 149; index < path.points.size(); ++index) {
    EXPECT_LT((position(path.points[index].tcp_pose) - second_layer_tcp).norm(), 1e-12);
  }

  std::string error;
  EXPECT_TRUE(plasma_path_executor::validatePathStructure(
    path, plasma_path_executor::ValidationLimits{}, &error)) << error;
}

}  // namespace
