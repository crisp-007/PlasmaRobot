#include "plasma_eye_hand/reach_validation.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <plasma_robot_interfaces/msg/spray_path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>

#include <Eigen/Geometry>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <ctime>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace plasma_eye_hand
{
namespace
{

Eigen::Isometry3d poseToIsometry(const geometry_msgs::msg::Pose & pose)
{
  Eigen::Quaterniond q(
    pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
  if (!q.coeffs().allFinite() || q.norm() < 1e-9) {
    throw std::runtime_error("pose quaternion is invalid");
  }
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.linear() = q.normalized().toRotationMatrix();
  result.translation() = Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
  if (!result.matrix().allFinite()) {
    throw std::runtime_error("pose contains a non-finite value");
  }
  return result;
}

Eigen::Isometry3d matrixFromYaml(const YAML::Node & node, const std::string & key)
{
  if (!node || !node.IsSequence() || node.size() != 4) {
    throw std::runtime_error(key + " must be a 4x4 matrix");
  }
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  for (std::size_t row = 0; row < 4; ++row) {
    if (!node[row].IsSequence() || node[row].size() != 4) {
      throw std::runtime_error(key + " must be a 4x4 matrix");
    }
    for (std::size_t column = 0; column < 4; ++column) {
      result.matrix()(row, column) = node[row][column].as<double>();
    }
  }
  return result;
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

YAML::Node vectorNode(const Eigen::Vector3d & value)
{
  YAML::Node node;
  node.push_back(value.x());
  node.push_back(value.y());
  node.push_back(value.z());
  return node;
}

Eigen::Vector3d vectorFromYaml(const YAML::Node & node, const std::string & key)
{
  if (!node || !node.IsSequence() || node.size() != 3) {
    throw std::runtime_error(key + " must contain three values");
  }
  return {node[0].as<double>(), node[1].as<double>(), node[2].as<double>()};
}

std::string timestamp(const char * format)
{
  const std::time_t now = std::time(nullptr);
  std::tm local{};
  localtime_r(&now, &local);
  std::ostringstream stream;
  stream << std::put_time(&local, format);
  return stream.str();
}

std::string acquisitionId(const plasma_robot_interfaces::msg::SprayPath & path)
{
  std::ostringstream stream;
  stream << path.path_id << '@' << path.header.stamp.sec << '.'
         << std::setw(9) << std::setfill('0') << path.header.stamp.nanosec;
  return stream.str();
}

std::string defaultOutputPath()
{
  const char * home = std::getenv("HOME");
  const std::filesystem::path root = home ? home : "/tmp";
  return (root / ".ros" / "plasma_eye_hand" /
    ("entry_reach_" + timestamp("%Y%m%d_%H%M%S") + ".yaml")).string();
}

void writeYaml(const std::string & path, const YAML::Node & root)
{
  const std::filesystem::path output(path);
  if (output.has_parent_path()) {
    std::filesystem::create_directories(output.parent_path());
  }
  const std::filesystem::path temporary = output.string() + ".tmp";
  {
    std::ofstream stream(temporary);
    if (!stream) {
      throw std::runtime_error("cannot open output YAML: " + temporary.string());
    }
    stream << root;
  }
  std::filesystem::rename(temporary, output);
}

void putError(YAML::Node node, const ReachError & error)
{
  node["translation_m"] = vectorNode(error.translation_m);
  node["norm_mm"] = error.norm_m * 1000.0;
  node["signed_axial_mm"] = error.signed_axial_m * 1000.0;
  node["lateral_mm"] = error.lateral_m * 1000.0;
  node.remove("orientation_deg");
  node["axis_alignment_deg"] = error.orientation_rad * 180.0 / M_PI;
  node["full_orientation_deg"] = error.full_orientation_rad * 180.0 / M_PI;
}

ReachError errorFromNode(const YAML::Node & node)
{
  ReachError error;
  error.translation_m = vectorFromYaml(node["translation_m"], "translation_m");
  error.norm_m = node["norm_mm"].as<double>() / 1000.0;
  error.signed_axial_m = node["signed_axial_mm"].as<double>() / 1000.0;
  error.lateral_m = node["lateral_mm"].as<double>() / 1000.0;
  const YAML::Node axis_alignment = node["axis_alignment_deg"];
  const YAML::Node legacy_orientation = node["orientation_deg"];
  error.orientation_rad = (axis_alignment ? axis_alignment : legacy_orientation)
    .as<double>() * M_PI / 180.0;
  if (node["full_orientation_deg"]) {
    error.full_orientation_rad =
      node["full_orientation_deg"].as<double>() * M_PI / 180.0;
  } else {
    error.full_orientation_rad = error.orientation_rad;
  }
  return error;
}

}  // namespace

class ReachValidationRecorder : public rclcpp::Node
{
public:
  ReachValidationRecorder()
  : Node("reach_validation_recorder")
  {
    pose_topic_ = declare_parameter<std::string>(
      "pose_topic", "/rm_driver/udp_arm_position");
    path_topic_ = declare_parameter<std::string>(
      "path_topic", "/plasma/planned_spray_path/base");
    camera_path_topic_ = declare_parameter<std::string>(
      "camera_path_topic", "/plasma/planned_spray_path/camera");
    output_yaml_ = declare_parameter<std::string>("output_yaml", defaultOutputPath());
    replace_existing_acquisition_ =
      declare_parameter<bool>("replace_existing_acquisition", false);
    allow_repeated_acquisition_ =
      declare_parameter<bool>("allow_repeated_acquisition", false);
    max_pose_age_ms_ = declare_parameter<int>("max_pose_age_ms", 500);
    limits_.min_records = static_cast<std::size_t>(std::max<int64_t>(1,
      declare_parameter<int64_t>("min_records", 3)));
    limits_.max_norm_m = declare_parameter<double>("max_norm_mm", 5.0) / 1000.0;
    limits_.max_abs_axial_m =
      declare_parameter<double>("max_abs_axial_mm", 5.0) / 1000.0;
    limits_.max_lateral_m = declare_parameter<double>("max_lateral_mm", 3.0) / 1000.0;
    limits_.max_orientation_rad =
      declare_parameter<double>("max_orientation_deg", 2.0) * M_PI / 180.0;

    const std::string default_tcp =
      ament_index_cpp::get_package_share_directory("plasma_tool_description") +
      "/config/tcp_3_4_200_1x10.yaml";
    tcp_yaml_ = declare_parameter<std::string>("tcp_yaml", default_tcp);
    const YAML::Node tcp_config = YAML::LoadFile(tcp_yaml_);
    flange_to_tcp_ = matrixFromYaml(
      tcp_config["T_gripper_to_tcp"], "T_gripper_to_tcp");
    tcp_to_nozzle_tip_ = matrixFromYaml(
      tcp_config["T_tcp_to_nozzle_tip"], "T_tcp_to_nozzle_tip");

    pose_subscription_ = create_subscription<geometry_msgs::msg::Pose>(
      pose_topic_, rclcpp::QoS(10),
      [this](geometry_msgs::msg::Pose::SharedPtr message) {
        std::scoped_lock lock(mutex_);
        latest_flange_pose_ = *message;
        latest_pose_time_ = std::chrono::steady_clock::now();
        have_pose_ = true;
      });
    path_subscription_ = create_subscription<plasma_robot_interfaces::msg::SprayPath>(
      path_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
      [this](plasma_robot_interfaces::msg::SprayPath::SharedPtr message) {
        std::scoped_lock lock(mutex_);
        latest_path_ = *message;
        have_path_ = true;
      });
    camera_path_subscription_ = create_subscription<plasma_robot_interfaces::msg::SprayPath>(
      camera_path_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
      [this](plasma_robot_interfaces::msg::SprayPath::SharedPtr message) {
        std::scoped_lock lock(mutex_);
        latest_camera_path_ = *message;
        have_camera_path_ = true;
      });

    loadOrInitialize();
  }

  const std::string & outputPath() const {return output_yaml_;}

  bool captureObserved(std::string * result)
  {
    Eigen::Isometry3d actual_tip;
    plasma_robot_interfaces::msg::SprayPath path;
    plasma_robot_interfaces::msg::SprayPath camera_path;
    if (!snapshot(&actual_tip, &path, &camera_path, result)) {
      return false;
    }

    // Insertion validation uses the explicit physical mouth target for the
    // visible rounded tip. The side-outlet entry TCP remains a motion target.
    const Eigen::Isometry3d target_tip = poseToIsometry(path.cavity_mouth_tip_pose);
    Eigen::Vector3d axis(path.cavity_axis.x, path.cavity_axis.y, path.cavity_axis.z);
    const ReachError error = calculateReachError(target_tip, actual_tip, axis);

    YAML::Node record;
    record["record_id"] = timestamp("%Y%m%d_%H%M%S");
    record["captured_at"] = timestamp("%Y-%m-%dT%H:%M:%S%z");
    record["path_id"] = path.path_id;
    record["acquisition_id"] = acquisitionId(path);
    record["calibration_id"] = path.calibration_id;
    record["path_execution_permitted"] = path.execution_permitted;
    record["target_kind"] = "geometric_cavity_mouth_for_rounded_nozzle_tip";
    record["T_base_to_target_nozzle_tip"] = matrixNode(target_tip);
    record["cavity_outward_axis_base"] = vectorNode(axis.normalized());
    record["T_base_to_observed_nozzle_tip"] = matrixNode(actual_tip);
    record["source_geometry"]["frame_id"] = camera_path.header.frame_id;
    record["source_geometry"]["T_source_to_target_nozzle_tip"] = matrixNode(
      poseToIsometry(camera_path.cavity_mouth_tip_pose));
    record["source_geometry"]["cavity_outward_axis"] = vectorNode(Eigen::Vector3d(
      camera_path.cavity_axis.x, camera_path.cavity_axis.y, camera_path.cavity_axis.z));
    YAML::Node joint_names;
    for (const auto & name : camera_path.capture_joint_state.name) {
      joint_names.push_back(name);
    }
    record["capture_joint_state"]["name"] = joint_names;
    YAML::Node joint_positions;
    for (const double position : camera_path.capture_joint_state.position) {
      joint_positions.push_back(position);
    }
    record["capture_joint_state"]["position_rad"] = joint_positions;
    putError(record["observed_minus_target"], error);
    bool enriched_existing = false;
    bool replaced_existing = false;
    std::size_t matching_acquisitions = 0;
    for (std::size_t index = 0; index < database_["records"].size(); ++index) {
      YAML::Node existing = database_["records"][index];
      if (!existing["acquisition_id"] ||
        existing["acquisition_id"].as<std::string>() != record["acquisition_id"].as<std::string>())
      {
        continue;
      }
      ++matching_acquisitions;
      if (allow_repeated_acquisition_) {
        continue;
      }
      if (replace_existing_acquisition_) {
        YAML::Node superseded = YAML::Clone(existing);
        superseded["superseded_at"] = timestamp("%Y-%m-%dT%H:%M:%S%z");
        superseded["superseded_reason"] =
          "operator recaptured after current geometry was republished";
        database_["superseded_records"].push_back(superseded);
        database_["records"][index] = record;
        replaced_existing = true;
        break;
      }
      // Preserve the original physical observation; only add source data that
      // was unavailable in records produced by the earlier recorder version.
      existing["source_geometry"] = record["source_geometry"];
      existing["capture_joint_state"] = record["capture_joint_state"];
      existing["source_geometry_enriched_at"] = timestamp("%Y-%m-%dT%H:%M:%S%z");
      enriched_existing = true;
      break;
    }
    if (allow_repeated_acquisition_) {
      record["repeatability_only"] = true;
      record["repeat_index"] = matching_acquisitions + 1U;
    }
    if (!enriched_existing && !replaced_existing) {
      database_["records"].push_back(record);
    }
    updateQualification();
    writeYaml(output_yaml_, database_);

    std::ostringstream stream;
    if (replaced_existing) {
      stream << "replaced existing " << path.path_id
             << " after preserving the old record in superseded_records";
      *result = stream.str();
      return true;
    }
    if (allow_repeated_acquisition_) {
      stream << std::fixed << std::setprecision(2)
             << "recorded repeat " << (matching_acquisitions + 1U) << " for " << path.path_id
             << ": norm=" << error.norm_m * 1000.0
             << " mm, axial=" << error.signed_axial_m * 1000.0
             << " mm, lateral=" << error.lateral_m * 1000.0
             << " mm, axis alignment=" << error.orientation_rad * 180.0 / M_PI
             << " deg";
      *result = stream.str();
      return true;
    }
    if (enriched_existing) {
      stream << "enriched existing " << path.path_id
             << " with source geometry; original observation preserved";
      *result = stream.str();
      return true;
    }
    stream << std::fixed << std::setprecision(2)
           << "recorded " << path.path_id << ": norm=" << error.norm_m * 1000.0
           << " mm, axial=" << error.signed_axial_m * 1000.0
           << " mm, lateral=" << error.lateral_m * 1000.0
           << " mm, axis alignment=" << error.orientation_rad * 180.0 / M_PI
           << " deg, full orientation=" << error.full_orientation_rad * 180.0 / M_PI
           << " deg";
    *result = stream.str();
    return true;
  }

  std::string report()
  {
    updateQualification();
    writeYaml(output_yaml_, database_);
    const YAML::Node q = database_["qualification"];
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(2)
           << "complete records=" << q["complete_records"].as<std::size_t>()
           << ", distinct acquisitions=" <<
      q["distinct_acquisition_ids"].as<std::size_t>()
           << ", max norm=" << q["max_norm_mm"].as<double>() << " mm"
           << ", max |axial|=" << q["max_abs_axial_mm"].as<double>() << " mm"
           << ", max lateral=" << q["max_lateral_mm"].as<double>() << " mm"
           << ", max axis alignment=" <<
      q["max_axis_alignment_deg"].as<double>() << " deg"
           << ", accepted=" << (q["accepted"].as<bool>() ? "true" : "false");
    return stream.str();
  }

private:
  bool snapshot(
    Eigen::Isometry3d * actual_tcp,
    plasma_robot_interfaces::msg::SprayPath * path,
    plasma_robot_interfaces::msg::SprayPath * camera_path,
    std::string * error) const
  {
    std::scoped_lock lock(mutex_);
    if (!have_path_) {
      *error = "no transformed path received from " + path_topic_;
      return false;
    }
    if (!latest_path_.transform_valid || !latest_path_.cavity_mouth_tip_pose_valid ||
      !latest_path_.cavity_axis_valid)
    {
      *error = "latest path has no valid transformed physical cavity mouth";
      return false;
    }
    if (!have_camera_path_) {
      *error = "no camera-frame path received from " + camera_path_topic_;
      return false;
    }
    if (!latest_camera_path_.cavity_mouth_tip_pose_valid ||
      !latest_camera_path_.cavity_axis_valid)
    {
      *error = "latest camera-frame path has no valid physical cavity mouth";
      return false;
    }
    if (latest_camera_path_.path_id != latest_path_.path_id ||
      latest_camera_path_.header.stamp.sec != latest_path_.header.stamp.sec ||
      latest_camera_path_.header.stamp.nanosec != latest_path_.header.stamp.nanosec)
    {
      *error = "camera-frame and transformed paths do not describe the same acquisition";
      return false;
    }
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
      *actual_tcp = poseToIsometry(latest_flange_pose_) *
        flange_to_tcp_ * tcp_to_nozzle_tip_;
    } catch (const std::exception & exception) {
      *error = exception.what();
      return false;
    }
    *path = latest_path_;
    *camera_path = latest_camera_path_;
    return true;
  }

  void loadOrInitialize()
  {
    if (std::filesystem::exists(output_yaml_)) {
      database_ = YAML::LoadFile(output_yaml_);
    }
    if (!database_ || !database_.IsMap()) {
      database_ = YAML::Node(YAML::NodeType::Map);
    }
    database_["schema_version"] = 2;
    database_["status"] = "measurement_requires_review";
    database_["motion_commands_published"] = false;
    database_["sources"]["pose_topic"] = pose_topic_;
    database_["sources"]["path_topic"] = path_topic_;
    database_["sources"]["camera_path_topic"] = camera_path_topic_;
    database_["sources"]["tcp_yaml"] = tcp_yaml_;
    if (!database_["records"]) {
      database_["records"] = YAML::Node(YAML::NodeType::Sequence);
    }
    updateQualification();
    writeYaml(output_yaml_, database_);
  }

  void updateQualification()
  {
    std::vector<ReachError> errors;
    std::set<std::string> acquisitions;
    for (auto record : database_["records"]) {
      if (!record["observed_minus_target"]) {
        continue;
      }
      ReachError error;
      if (record["T_base_to_target_nozzle_tip"] &&
        record["T_base_to_observed_nozzle_tip"] && record["cavity_outward_axis_base"])
      {
        error = calculateReachError(
          matrixFromYaml(
            record["T_base_to_target_nozzle_tip"], "T_base_to_target_nozzle_tip"),
          matrixFromYaml(
            record["T_base_to_observed_nozzle_tip"], "T_base_to_observed_nozzle_tip"),
          vectorFromYaml(record["cavity_outward_axis_base"], "cavity_outward_axis_base"));
        putError(record["observed_minus_target"], error);
      } else {
        error = errorFromNode(record["observed_minus_target"]);
      }
      errors.push_back(error);
      if (record["acquisition_id"]) {
        acquisitions.insert(record["acquisition_id"].as<std::string>());
      } else if (record["path_id"]) {
        acquisitions.insert(record["path_id"].as<std::string>());
      }
    }
    const ReachSummary summary = summarizeReachErrors(errors, limits_);
    const bool enough_distinct_acquisitions = acquisitions.size() >= limits_.min_records;
    YAML::Node q = database_["qualification"];
    q["complete_records"] = summary.record_count;
    q.remove("distinct_path_ids");
    q.remove("required_distinct_paths");
    q["distinct_acquisition_ids"] = acquisitions.size();
    q["required_distinct_acquisitions"] = limits_.min_records;
    q["mean_translation_mm"] = vectorNode(summary.mean_translation_m * 1000.0);
    q["translation_stddev_mm"] = vectorNode(summary.translation_stddev_m * 1000.0);
    q["mean_norm_mm"] = summary.mean_norm_m * 1000.0;
    q["max_norm_mm"] = summary.max_norm_m * 1000.0;
    q["max_abs_axial_mm"] = summary.max_abs_axial_m * 1000.0;
    q["max_lateral_mm"] = summary.max_lateral_m * 1000.0;
    q.remove("max_orientation_deg");
    q["max_axis_alignment_deg"] = summary.max_orientation_rad * 180.0 / M_PI;
    q["limits"]["max_norm_mm"] = limits_.max_norm_m * 1000.0;
    q["limits"]["max_abs_axial_mm"] = limits_.max_abs_axial_m * 1000.0;
    q["limits"]["max_lateral_mm"] = limits_.max_lateral_m * 1000.0;
    q["limits"].remove("max_orientation_deg");
    q["limits"]["max_axis_alignment_deg"] =
      limits_.max_orientation_rad * 180.0 / M_PI;
    q["all_errors_within_limits"] = summary.accepted;
    q["accepted"] = summary.accepted && enough_distinct_acquisitions;
    q["scope"] = "camera-to-base and rounded-nozzle-tip insertion accuracy at cavity mouth";
    q["automatic_yaml_approval"] = false;
  }

  std::string pose_topic_;
  std::string path_topic_;
  std::string camera_path_topic_;
  std::string tcp_yaml_;
  std::string output_yaml_;
  bool replace_existing_acquisition_ = false;
  bool allow_repeated_acquisition_ = false;
  int max_pose_age_ms_ = 500;
  ReachLimits limits_;
  Eigen::Isometry3d flange_to_tcp_ = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tcp_to_nozzle_tip_ = Eigen::Isometry3d::Identity();
  YAML::Node database_;

  mutable std::mutex mutex_;
  geometry_msgs::msg::Pose latest_flange_pose_;
  std::chrono::steady_clock::time_point latest_pose_time_;
  plasma_robot_interfaces::msg::SprayPath latest_path_;
  plasma_robot_interfaces::msg::SprayPath latest_camera_path_;
  bool have_pose_ = false;
  bool have_path_ = false;
  bool have_camera_path_ = false;
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pose_subscription_;
  rclcpp::Subscription<plasma_robot_interfaces::msg::SprayPath>::SharedPtr path_subscription_;
  rclcpp::Subscription<plasma_robot_interfaces::msg::SprayPath>::SharedPtr
    camera_path_subscription_;
};

}  // namespace plasma_eye_hand

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<plasma_eye_hand::ReachValidationRecorder>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread spin_thread([&executor]() {executor.spin();});

    std::cout << "Read-only entry reach recorder. It publishes no robot commands.\n"
              << "Align the rounded nozzle tip and rod axis with the cavity mouth without contact.\n"
              << "Commands: Enter/o=capture observed alignment, r=report, q=quit.\n"
              << "Output: " << node->outputPath() << '\n';
    std::string command;
    while (rclcpp::ok() && std::getline(std::cin, command)) {
      if (command == "q" || command == "Q") {
        break;
      }
      if (command == "r" || command == "R") {
        std::cout << node->report() << '\n';
        continue;
      }
      if (command.empty() || command == "o" || command == "O") {
        std::string result;
        if (node->captureObserved(&result)) {
          std::cout << result << '\n';
        } else {
          std::cerr << "capture rejected: " << result << '\n';
        }
        continue;
      }
      std::cout << "Commands: Enter/o=capture, r=report, q=quit.\n";
    }
    executor.cancel();
    spin_thread.join();
    rclcpp::shutdown();
    return 0;
  } catch (const std::exception & exception) {
    std::cerr << "reach validation recorder failed: " << exception.what() << '\n';
    rclcpp::shutdown();
    return 1;
  }
}
