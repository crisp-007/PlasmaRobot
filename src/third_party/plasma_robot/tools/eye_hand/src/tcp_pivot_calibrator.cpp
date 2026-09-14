#include "plasma_eye_hand/tcp_pivot.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>

#include <Eigen/Geometry>

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace plasma_eye_hand
{
namespace
{

using namespace std::chrono_literals;

Eigen::Isometry3d poseToIsometry(const geometry_msgs::msg::Pose & pose)
{
  Eigen::Quaterniond quaternion(
    pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
  if (!std::isfinite(quaternion.norm()) || quaternion.norm() < 1e-9) {
    throw std::runtime_error("received flange pose has an invalid quaternion");
  }
  quaternion.normalize();

  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.linear() = quaternion.toRotationMatrix();
  transform.translation() = Eigen::Vector3d(
    pose.position.x, pose.position.y, pose.position.z);
  if (!transform.matrix().allFinite()) {
    throw std::runtime_error("received flange pose contains a non-finite value");
  }
  return transform;
}

Eigen::Vector3d loadNominalTcp(const std::string & path)
{
  const YAML::Node matrix = YAML::LoadFile(path)["T_gripper_to_tcp"];
  if (!matrix || !matrix.IsSequence() || matrix.size() != 4) {
    throw std::runtime_error("TCP YAML is missing a 4x4 T_gripper_to_tcp matrix");
  }
  for (std::size_t row = 0; row < 4; ++row) {
    if (!matrix[row].IsSequence() || matrix[row].size() != 4) {
      throw std::runtime_error("T_gripper_to_tcp is not a 4x4 matrix");
    }
  }
  return Eigen::Vector3d(
    matrix[0][3].as<double>(), matrix[1][3].as<double>(), matrix[2][3].as<double>());
}

std::string defaultOutputPath()
{
  const char * home = std::getenv("HOME");
  const auto now = std::chrono::system_clock::now();
  const std::time_t now_time = std::chrono::system_clock::to_time_t(now);
  std::tm local_time{};
  localtime_r(&now_time, &local_time);
  std::ostringstream stamp;
  stamp << std::put_time(&local_time, "%Y%m%d_%H%M%S");
  const std::filesystem::path root = home ? home : "/tmp";
  return (root / ".ros" / "plasma_eye_hand" /
    ("tcp_pivot_" + stamp.str() + ".yaml")).string();
}

YAML::Node vectorNode(const Eigen::Vector3d & value)
{
  YAML::Node node;
  node.push_back(value.x());
  node.push_back(value.y());
  node.push_back(value.z());
  return node;
}

YAML::Node matrixNode(const Eigen::Isometry3d & transform)
{
  YAML::Node matrix;
  for (Eigen::Index row = 0; row < 4; ++row) {
    YAML::Node values;
    for (Eigen::Index column = 0; column < 4; ++column) {
      values.push_back(transform.matrix()(row, column));
    }
    matrix.push_back(values);
  }
  return matrix;
}

void writeResult(
  const std::string & output_path,
  const std::string & pose_topic,
  const std::string & nominal_tcp_yaml,
  const std::vector<PivotSample> & samples,
  const PivotResult & solved,
  const PivotResult & nominal)
{
  YAML::Node root;
  root["schema_version"] = 1;
  root["status"] = "measurement_requires_review";
  root["validated"] = false;
  root["execution_allowed"] = false;
  root["frames"]["base"] = "baselink";
  root["frames"]["flange"] = "Link6";
  root["frames"]["tcp"] = "plasma_motion_tcp";
  root["source"]["pose_topic"] = pose_topic;
  root["source"]["nominal_tcp_yaml"] = nominal_tcp_yaml;
  root["source"]["motion_commands_published"] = false;
  root["sample_count"] = samples.size();

  for (const auto & sample : samples) {
    YAML::Node sample_node;
    sample_node["T_base_to_flange"] = matrixNode(sample.base_to_flange);
    root["samples"].push_back(sample_node);
  }

  root["result"]["flange_to_tcp_m"] = vectorNode(solved.flange_to_tcp);
  root["result"]["fixed_point_in_base_m"] = vectorNode(solved.fixed_point_in_base);
  root["result"]["rank"] = solved.rank;
  root["result"]["condition_number"] = solved.condition_number;
  root["result"]["mean_residual_mm"] = solved.mean_residual_m * 1000.0;
  root["result"]["rms_residual_mm"] = solved.rms_residual_m * 1000.0;
  root["result"]["max_residual_mm"] = solved.max_residual_m * 1000.0;
  for (const double residual : solved.residual_norms_m) {
    root["result"]["residual_norms_mm"].push_back(residual * 1000.0);
  }

  root["nominal_check"]["flange_to_tcp_m"] = vectorNode(nominal.flange_to_tcp);
  root["nominal_check"]["mean_residual_mm"] = nominal.mean_residual_m * 1000.0;
  root["nominal_check"]["rms_residual_mm"] = nominal.rms_residual_m * 1000.0;
  root["nominal_check"]["max_residual_mm"] = nominal.max_residual_m * 1000.0;

  const std::filesystem::path path(output_path);
  if (path.has_parent_path()) {
    std::filesystem::create_directories(path.parent_path());
  }
  std::ofstream output(path);
  if (!output) {
    throw std::runtime_error("cannot open output YAML: " + output_path);
  }
  output << root;
}

void printResult(
  const PivotResult & solved,
  const PivotResult & nominal,
  const std::string & output_path)
{
  const auto print_vector_mm = [](const std::string & label, const Eigen::Vector3d & value) {
      std::cout << label << " [mm]: [" << std::fixed << std::setprecision(3)
                << value.x() * 1000.0 << ", " << value.y() * 1000.0 << ", "
                << value.z() * 1000.0 << "]\n";
    };
  std::cout << "\nTCP pivot result (review required)\n";
  print_vector_mm("solved Link6 -> spray TCP", solved.flange_to_tcp);
  print_vector_mm("fixed point in base", solved.fixed_point_in_base);
  std::cout << "residual solved [mm]: mean=" << solved.mean_residual_m * 1000.0
            << ", rms=" << solved.rms_residual_m * 1000.0
            << ", max=" << solved.max_residual_m * 1000.0 << '\n';
  std::cout << "system: rank=" << solved.rank
            << ", condition=" << solved.condition_number << '\n';
  print_vector_mm("nominal Link6 -> spray TCP", nominal.flange_to_tcp);
  std::cout << "residual nominal [mm]: mean=" << nominal.mean_residual_m * 1000.0
            << ", rms=" << nominal.rms_residual_m * 1000.0
            << ", max=" << nominal.max_residual_m * 1000.0 << '\n';
  std::cout << "saved: " << output_path << '\n';
  std::cout << "Current TCP YAML was not modified; automatic execution remains disabled.\n";
}

}  // namespace

class TcpPivotNode : public rclcpp::Node
{
public:
  TcpPivotNode()
  : Node("tcp_pivot_calibrator")
  {
    pose_topic_ = declare_parameter<std::string>(
      "pose_topic", "/rm_driver/udp_arm_position");
    max_pose_age_ms_ = declare_parameter<int>("max_pose_age_ms", 500);
    subscription_ = create_subscription<geometry_msgs::msg::Pose>(
      pose_topic_, rclcpp::QoS(10),
      [this](geometry_msgs::msg::Pose::SharedPtr pose) {
        std::scoped_lock lock(mutex_);
        latest_pose_ = *pose;
        latest_pose_time_ = std::chrono::steady_clock::now();
        have_pose_ = true;
      });
  }

  std::string poseTopic() const
  {
    return pose_topic_;
  }

  bool capture(PivotSample * sample, std::string * error) const
  {
    std::scoped_lock lock(mutex_);
    if (!have_pose_) {
      *error = "no flange pose received from " + pose_topic_;
      return false;
    }
    const auto age = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now() - latest_pose_time_).count();
    if (age > max_pose_age_ms_) {
      *error = "latest flange pose is stale (" + std::to_string(age) + " ms)";
      return false;
    }
    try {
      sample->base_to_flange = poseToIsometry(latest_pose_);
    } catch (const std::exception & exception) {
      *error = exception.what();
      return false;
    }
    return true;
  }

private:
  std::string pose_topic_;
  int max_pose_age_ms_ = 500;
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr subscription_;
  mutable std::mutex mutex_;
  geometry_msgs::msg::Pose latest_pose_;
  std::chrono::steady_clock::time_point latest_pose_time_;
  bool have_pose_ = false;
};

}  // namespace plasma_eye_hand

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<plasma_eye_hand::TcpPivotNode>();
    const std::string default_tcp_yaml =
      ament_index_cpp::get_package_share_directory("plasma_tool_description") +
      "/config/tcp_3_4_200_1x10.yaml";
    const std::string tcp_yaml = node->declare_parameter<std::string>(
      "tcp_yaml", default_tcp_yaml);
    const std::string output_path = node->declare_parameter<std::string>(
      "output_yaml", plasma_eye_hand::defaultOutputPath());
    const int min_samples = node->declare_parameter<int>("min_samples", 6);
    if (min_samples < 3) {
      throw std::runtime_error("min_samples must be at least 3");
    }
    const Eigen::Vector3d nominal_tcp = plasma_eye_hand::loadNominalTcp(tcp_yaml);

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread spin_thread([&executor]() {executor.spin();});

    std::vector<plasma_eye_hand::PivotSample> samples;
    std::cout << "TCP pivot recorder is read-only and publishes no motion commands.\n"
              << "This is optional metrology, not a spraying operation. Continue only with a\n"
              << "purpose-built locator that can hold the outlet center without binding or damage.\n"
              << "The outlet and workpiece never contact during spraying. Use the teach\n"
              << "pendant at low speed to vary wrist orientation around multiple axes, then press\n"
              << "Enter to capture.\n"
              << "Pose source: " << node->poseTopic() << " (must represent baselink -> Link6).\n"
              << "Commands: Enter=capture, s=solve, q=quit. Recommended: 6-10 poses.\n";

    bool solve = false;
    std::string command;
    while (rclcpp::ok() && std::getline(std::cin, command)) {
      if (command == "q" || command == "Q") {
        break;
      }
      if (command == "s" || command == "S") {
        if (samples.size() < static_cast<std::size_t>(min_samples)) {
          std::cout << "Need at least " << min_samples << " samples before solving.\n";
          continue;
        }
        solve = true;
        break;
      }
      if (!command.empty()) {
        std::cout << "Unknown command. Enter=capture, s=solve, q=quit.\n";
        continue;
      }

      plasma_eye_hand::PivotSample sample;
      std::string error;
      if (!node->capture(&sample, &error)) {
        std::cout << "Capture rejected: " << error << '\n';
        continue;
      }
      samples.push_back(sample);
      const Eigen::Vector3d position = sample.base_to_flange.translation();
      std::cout << "Captured #" << samples.size() << " flange [m]: ["
                << position.x() << ", " << position.y() << ", " << position.z() << "]\n";
    }

    executor.cancel();
    spin_thread.join();
    if (solve) {
      const auto result = plasma_eye_hand::solveTcpPivot(samples);
      const auto nominal = plasma_eye_hand::evaluateTcpOffset(samples, nominal_tcp);
      plasma_eye_hand::writeResult(
        output_path, node->poseTopic(), tcp_yaml, samples, result, nominal);
      plasma_eye_hand::printResult(result, nominal, output_path);
    } else {
      std::cout << "Exited without solving or changing calibration.\n";
    }
    rclcpp::shutdown();
    return 0;
  } catch (const std::exception & exception) {
    std::cerr << "TCP pivot failed: " << exception.what() << '\n';
    rclcpp::shutdown();
    return 1;
  }
}
