#include "plasma_path_executor/single_layer_path.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <plasma_robot_interfaces/msg/spray_path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <yaml-cpp/yaml.h>

#include <Eigen/Geometry>

#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>

namespace plasma_path_executor
{
namespace
{

Eigen::Isometry3d loadRigidTransform(const std::string & path, const std::string & key)
{
  const YAML::Node matrix = YAML::LoadFile(path)[key];
  if (!matrix || !matrix.IsSequence() || matrix.size() != 4) {
    throw std::runtime_error(path + " is missing 4x4 " + key);
  }
  Eigen::Matrix4d value;
  for (std::size_t row = 0; row < 4; ++row) {
    if (!matrix[row].IsSequence() || matrix[row].size() != 4) {
      throw std::runtime_error(key + " is not a 4x4 matrix");
    }
    for (std::size_t column = 0; column < 4; ++column) {
      value(static_cast<Eigen::Index>(row), static_cast<Eigen::Index>(column)) =
        matrix[row][column].as<double>();
    }
  }
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.matrix() = value;
  if (!value.allFinite() ||
    (value.row(3) - Eigen::RowVector4d(0.0, 0.0, 0.0, 1.0)).norm() > 1e-9 ||
    !transform.linear().isUnitary(1e-6) || transform.linear().determinant() < 0.999999)
  {
    throw std::runtime_error(key + " is not a finite rigid transform");
  }
  return transform;
}

Eigen::Isometry3d poseToIsometry(const geometry_msgs::msg::Pose & pose)
{
  Eigen::Quaterniond quaternion(
    pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
  if (!std::isfinite(quaternion.norm()) || quaternion.norm() < 1e-9) {
    throw std::runtime_error("current flange pose has an invalid quaternion");
  }
  quaternion.normalize();
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.linear() = quaternion.toRotationMatrix();
  transform.translation() = Eigen::Vector3d(
    pose.position.x, pose.position.y, pose.position.z);
  if (!transform.matrix().allFinite()) {
    throw std::runtime_error("current flange pose contains a non-finite value");
  }
  return transform;
}

}  // namespace

class SingleLayerCommissionNode : public rclcpp::Node
{
public:
  SingleLayerCommissionNode()
  : Node("plasma_single_layer_commission")
  {
    const std::string default_tcp_yaml =
      ament_index_cpp::get_package_share_directory("plasma_tool_description") +
      "/config/tcp_3_4_200_1x10.yaml";
    pose_topic_ = declare_parameter<std::string>(
      "pose_topic", "/rm_driver/udp_arm_position");
    path_topic_ = declare_parameter<std::string>(
      "path_topic", "/plasma/planned_spray_path/base");
    tcp_yaml_ = declare_parameter<std::string>("tcp_yaml", default_tcp_yaml);
    commissioning_enabled_ = declare_parameter<bool>("commissioning_enabled", false);
    max_pose_age_ms_ = declare_parameter<int>("max_pose_age_ms", 500);
    config_.angular_step_deg = declare_parameter<double>("angular_step_deg", 5.0);
    config_.virtual_spray_distance_m = declare_parameter<double>(
      "virtual_spray_distance_m", 0.05);
    config_.layer_count = declare_parameter<int>("layer_count", 1);
    config_.layer_step_m = declare_parameter<double>("layer_step_m", 0.010);

    if (max_pose_age_ms_ <= 0) {
      throw std::runtime_error("max_pose_age_ms must be positive");
    }
    flange_to_tcp_ = loadRigidTransform(tcp_yaml_, "T_gripper_to_tcp");
    tcp_to_nozzle_tip_ = loadRigidTransform(tcp_yaml_, "T_tcp_to_nozzle_tip");

    const auto path_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    path_publisher_ = create_publisher<plasma_robot_interfaces::msg::SprayPath>(
      path_topic_, path_qos);
    pose_subscription_ = create_subscription<geometry_msgs::msg::Pose>(
      pose_topic_, rclcpp::QoS(10),
      [this](geometry_msgs::msg::Pose::SharedPtr pose) {
        std::scoped_lock lock(mutex_);
        latest_pose_ = *pose;
        latest_pose_time_ = std::chrono::steady_clock::now();
        have_pose_ = true;
      });
    prepare_service_ = create_service<std_srvs::srv::Trigger>(
      "~/prepare",
      std::bind(&SingleLayerCommissionNode::onPrepare, this,
        std::placeholders::_1, std::placeholders::_2));

    RCLCPP_WARN(get_logger(),
      "Single-layer commissioning loaded: enabled=%s, no motion is sent by this node",
      commissioning_enabled_ ? "true" : "false");
  }

private:
  void onPrepare(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    std::scoped_lock lock(mutex_);
    if (!commissioning_enabled_) {
      response->success = false;
      response->message = "commissioning_enabled=false; no path published";
      return;
    }
    if (!have_pose_) {
      response->success = false;
      response->message = "no current Link6 pose received from " + pose_topic_;
      return;
    }
    const auto age_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now() - latest_pose_time_).count();
    if (age_ms > max_pose_age_ms_) {
      response->success = false;
      response->message = "current Link6 pose is stale";
      return;
    }

    try {
      const auto stamp = now();
      config_.path_id = "layer_commission_" + std::to_string(config_.layer_count) +
        "x_" + std::to_string(stamp.nanoseconds());
      auto path = buildSingleLayerCommissioningPath(
        poseToIsometry(latest_pose_), flange_to_tcp_, tcp_to_nozzle_tip_, config_);
      path.header.stamp = stamp;
      path_publisher_->publish(path);
      response->success = true;
      response->message = "published " + path.path_id + " with " +
        std::to_string(path.points.size()) +
        " poses; call the executor start service separately";
      RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
    } catch (const std::exception & error) {
      response->success = false;
      response->message = std::string("failed to prepare single-layer path: ") + error.what();
    }
  }

  std::string pose_topic_;
  std::string path_topic_;
  std::string tcp_yaml_;
  bool commissioning_enabled_ = false;
  int max_pose_age_ms_ = 500;
  SingleLayerPathConfig config_;
  Eigen::Isometry3d flange_to_tcp_ = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tcp_to_nozzle_tip_ = Eigen::Isometry3d::Identity();

  std::mutex mutex_;
  geometry_msgs::msg::Pose latest_pose_;
  std::chrono::steady_clock::time_point latest_pose_time_;
  bool have_pose_ = false;

  rclcpp::Publisher<plasma_robot_interfaces::msg::SprayPath>::SharedPtr path_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pose_subscription_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr prepare_service_;
};

}  // namespace plasma_path_executor

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<plasma_path_executor::SingleLayerCommissionNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("plasma_single_layer_commission"), "%s", error.what());
  }
  rclcpp::shutdown();
  return 0;
}
