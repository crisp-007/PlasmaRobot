#include "plasma_path_transform/path_transformer.hpp"

#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <plasma_robot_interfaces/msg/path_transform_status.hpp>
#include <plasma_robot_interfaces/msg/spray_path.hpp>
#include <plasma_robot_interfaces/srv/transform_spray_path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <fstream>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace plasma_path_transform
{
namespace
{

std::string normalizedFrame(std::string frame)
{
  while (!frame.empty() && frame.front() == '/') {
    frame.erase(frame.begin());
  }
  return frame;
}

Eigen::Isometry3d kdlFrameToIsometry(const KDL::Frame & frame)
{
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double w = 1.0;
  frame.M.GetQuaternion(x, y, z, w);

  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.linear() = Eigen::Quaterniond(w, x, y, z).normalized().toRotationMatrix();
  transform.translation() = Eigen::Vector3d(frame.p.x(), frame.p.y(), frame.p.z());
  return transform;
}

Eigen::Isometry3d transformMsgToIsometry(const geometry_msgs::msg::Transform & message)
{
  Eigen::Quaterniond rotation(
    message.rotation.w, message.rotation.x, message.rotation.y, message.rotation.z);
  if (!rotation.coeffs().allFinite() || rotation.norm() < 1e-12) {
    throw std::runtime_error("camera-frame TF contains an invalid rotation");
  }

  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.linear() = rotation.normalized().toRotationMatrix();
  transform.translation() = Eigen::Vector3d(
    message.translation.x, message.translation.y, message.translation.z);
  if (!transform.matrix().allFinite()) {
    throw std::runtime_error("camera-frame TF contains non-finite values");
  }
  return transform;
}

Eigen::Vector3d axisVector(const geometry_msgs::msg::Vector3 & vector)
{
  return {vector.x, vector.y, vector.z};
}

double rotationError(
  const Eigen::Isometry3d & first, const Eigen::Isometry3d & second)
{
  const Eigen::AngleAxisd error(first.linear().transpose() * second.linear());
  return std::abs(error.angle());
}

}  // namespace

class PathTransformNode : public rclcpp::Node
{
public:
  PathTransformNode()
  : Node("plasma_path_transform")
  {
    input_topic_ = declare_parameter<std::string>(
      "input_topic", "/plasma/planned_spray_path/camera");
    output_topic_ = declare_parameter<std::string>(
      "output_topic", "/plasma/planned_spray_path/base");
    status_topic_ = declare_parameter<std::string>(
      "status_topic", "/plasma/path_transform/status");
    base_frame_ = normalizedFrame(declare_parameter<std::string>("base_frame", "baselink"));
    gripper_frame_ = normalizedFrame(
      declare_parameter<std::string>("gripper_frame", "Link6"));
    camera_frame_ = normalizedFrame(declare_parameter<std::string>(
      "camera_frame", "camera_color_optical_frame"));
    tcp_frame_ = normalizedFrame(declare_parameter<std::string>(
      "tcp_frame", "plasma_motion_tcp"));
    handeye_yaml_ = declare_parameter<std::string>("handeye_yaml", "");
    tcp_yaml_ = declare_parameter<std::string>("tcp_yaml", "");
    source_refinement_yaml_ = declare_parameter<std::string>("source_refinement_yaml", "");
    robot_description_file_ = declare_parameter<std::string>("robot_description_file", "");
    robot_description_ = declare_parameter<std::string>("robot_description", "");
    require_validated_handeye_ = declare_parameter<bool>(
      "require_validated_handeye", true);
    require_validated_tcp_ = declare_parameter<bool>("require_validated_tcp", true);
    publish_unvalidated_preview_ = declare_parameter<bool>(
      "publish_unvalidated_preview", true);
    allow_camera_frame_tf_ = declare_parameter<bool>("allow_camera_frame_tf", true);
    camera_frame_tf_timeout_sec_ = std::max(0.0,
      declare_parameter<double>("camera_frame_tf_timeout_sec", 0.5));
    max_path_points_ = static_cast<std::size_t>(std::max<int64_t>(1,
      declare_parameter<int64_t>("max_path_points", 200000)));
    max_entry_axis_offset_m_ = declare_parameter<double>(
      "max_entry_axis_offset_m", 0.002);
    max_first_layer_axis_offset_m_ = declare_parameter<double>(
      "max_first_layer_axis_offset_m", 0.020);
    max_entry_orientation_error_rad_ = declare_parameter<double>(
      "max_entry_orientation_error_rad", 0.08726646259971647);
    cavity_axis_outward_compensation_m_ = declare_parameter<double>(
      "cavity_axis_outward_compensation_m", 0.0);
    spray_axis_outward_compensation_m_ = declare_parameter<double>(
      "spray_axis_outward_compensation_m", 0.0);
    if (max_entry_axis_offset_m_ <= 0.0 || max_first_layer_axis_offset_m_ <= 0.0 ||
      max_entry_orientation_error_rad_ <= 0.0) {
      throw std::runtime_error("cavity-entry validation parameters must be positive");
    }
    if (!std::isfinite(cavity_axis_outward_compensation_m_) ||
      cavity_axis_outward_compensation_m_ < 0.0 ||
      cavity_axis_outward_compensation_m_ > 0.15)
    {
      throw std::runtime_error(
              "cavity_axis_outward_compensation_m must be within [0, 0.15] m");
    }
    if (!std::isfinite(spray_axis_outward_compensation_m_) ||
      spray_axis_outward_compensation_m_ < 0.0 ||
      spray_axis_outward_compensation_m_ > 0.15)
    {
      throw std::runtime_error(
              "spray_axis_outward_compensation_m must be within [0, 0.15] m");
    }

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    const auto output_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    output_publisher_ = create_publisher<plasma_robot_interfaces::msg::SprayPath>(
      output_topic_, output_qos);
    status_publisher_ = create_publisher<plasma_robot_interfaces::msg::PathTransformStatus>(
      status_topic_, output_qos);
    input_subscription_ = create_subscription<plasma_robot_interfaces::msg::SprayPath>(
      input_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
      std::bind(&PathTransformNode::onPath, this, std::placeholders::_1));
    transform_service_ = create_service<plasma_robot_interfaces::srv::TransformSprayPath>(
      "~/transform_path",
      std::bind(&PathTransformNode::onTransformService, this,
        std::placeholders::_1, std::placeholders::_2));
    reload_service_ = create_service<std_srvs::srv::Trigger>(
      "~/reload_calibration",
      std::bind(&PathTransformNode::onReloadCalibration, this,
        std::placeholders::_1, std::placeholders::_2));

    loadRobotModel();
    reloadCalibration();
    publishStatus(statusMessage());

    RCLCPP_INFO(get_logger(),
      "Path transform ready: %s -> %s, input=%s, output=%s",
      camera_frame_.c_str(), base_frame_.c_str(), input_topic_.c_str(), output_topic_.c_str());
  }

private:
  void loadRobotModel()
  {
    std::string description = robot_description_;
    if (description.empty() && !robot_description_file_.empty()) {
      std::ifstream input(robot_description_file_);
      if (!input) {
        RCLCPP_ERROR(get_logger(), "Cannot open robot description: %s",
          robot_description_file_.c_str());
        return;
      }
      std::ostringstream buffer;
      buffer << input.rdbuf();
      description = buffer.str();
    }

    if (description.empty()) {
      RCLCPP_WARN(get_logger(),
        "Robot description is empty; capture_gripper_pose is required for conversion");
      return;
    }

    KDL::Tree tree;
    if (!kdl_parser::treeFromString(description, tree)) {
      RCLCPP_ERROR(get_logger(), "Failed to parse robot_description as URDF");
      return;
    }
    if (!tree.getChain(base_frame_, gripper_frame_, kdl_chain_)) {
      RCLCPP_ERROR(get_logger(), "No KDL chain from %s to %s",
        base_frame_.c_str(), gripper_frame_.c_str());
      return;
    }

    joint_names_.clear();
    for (const auto & segment : kdl_chain_.segments) {
      if (segment.getJoint().getType() != KDL::Joint::None) {
        joint_names_.push_back(segment.getJoint().getName());
      }
    }
    fk_solver_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(kdl_chain_);
    robot_model_loaded_ = !joint_names_.empty() &&
      joint_names_.size() == kdl_chain_.getNrOfJoints();

    if (robot_model_loaded_) {
      RCLCPP_INFO(get_logger(), "Loaded robot chain %s -> %s with %zu joints",
        base_frame_.c_str(), gripper_frame_.c_str(), joint_names_.size());
    } else {
      RCLCPP_ERROR(get_logger(), "Robot chain joint metadata is incomplete");
    }
  }

  void reloadCalibration()
  {
    std::scoped_lock lock(transform_mutex_);
    get_parameter("handeye_yaml", handeye_yaml_);
    get_parameter("tcp_yaml", tcp_yaml_);
    get_parameter("source_refinement_yaml", source_refinement_yaml_);

    handeye_ = TransformCalibration{};
    tcp_ = TransformCalibration{};
    source_refinement_ = TransformCalibration{};
    tcp_to_nozzle_tip_ = Eigen::Isometry3d::Identity();
    tcp_to_spray_outlet_ = Eigen::Isometry3d::Identity();
    process_geometry_loaded_ = false;
    calibration_error_.clear();

    if (!handeye_yaml_.empty()) {
      try {
        handeye_ = PathTransformer::loadTransformYaml(
          handeye_yaml_, "T_camera_to_gripper");
      } catch (const std::exception & error) {
        calibration_error_ = std::string("hand-eye: ") + error.what();
        RCLCPP_ERROR(get_logger(), "%s", calibration_error_.c_str());
      }
    }

    if (!tcp_yaml_.empty()) {
      try {
        tcp_ = PathTransformer::loadTransformYaml(tcp_yaml_, "T_gripper_to_tcp");
        tcp_to_nozzle_tip_ = PathTransformer::loadRigidTransformYaml(
          tcp_yaml_, "T_tcp_to_nozzle_tip");
        tcp_to_spray_outlet_ = PathTransformer::loadRigidTransformYaml(
          tcp_yaml_, "T_tcp_to_spray_outlet");
        process_geometry_loaded_ = true;
      } catch (const std::exception & error) {
        const std::string tcp_error = std::string("TCP: ") + error.what();
        if (!calibration_error_.empty()) {
          calibration_error_ += "; ";
        }
        calibration_error_ += tcp_error;
        RCLCPP_ERROR(get_logger(), "%s", tcp_error.c_str());
      }
    }

    if (!source_refinement_yaml_.empty()) {
      try {
        source_refinement_ = PathTransformer::loadTransformYaml(
          source_refinement_yaml_, "T_source_correction");
        source_refinement_.frame_id = normalizedFrame(source_refinement_.frame_id);
        if (source_refinement_.frame_id.empty()) {
          throw std::runtime_error("source-frame refinement YAML must declare source_frame");
        }
        // A fitted reach correction remains preview-only until blind validation.
        source_refinement_.validated = false;
        RCLCPP_WARN(get_logger(),
          "Training-only source refinement '%s' active for %s; execution is forced off",
          source_refinement_.id.c_str(), source_refinement_.frame_id.c_str());
      } catch (const std::exception & error) {
        source_refinement_ = TransformCalibration{};
        const std::string refinement_error =
          std::string("source refinement: ") + error.what();
        if (!calibration_error_.empty()) {
          calibration_error_ += "; ";
        }
        calibration_error_ += refinement_error;
        RCLCPP_ERROR(get_logger(), "%s", refinement_error.c_str());
      }
    }
  }

  bool capturePoseFromJoints(
    const sensor_msgs::msg::JointState & joint_state,
    Eigen::Isometry3d * result, std::string * reason) const
  {
    if (!robot_model_loaded_ || !fk_solver_) {
      *reason = "robot model is not loaded and no direct capture gripper pose was supplied";
      return false;
    }
    if (joint_state.position.size() < joint_names_.size()) {
      *reason = "capture joint state has fewer positions than the robot chain";
      return false;
    }

    std::unordered_map<std::string, double> named_positions;
    if (!joint_state.name.empty()) {
      const std::size_t count = std::min(joint_state.name.size(), joint_state.position.size());
      for (std::size_t index = 0; index < count; ++index) {
        named_positions[normalizedFrame(joint_state.name[index])] = joint_state.position[index];
      }
    }

    KDL::JntArray joints(kdl_chain_.getNrOfJoints());
    for (std::size_t index = 0; index < joint_names_.size(); ++index) {
      if (!named_positions.empty()) {
        const auto found = named_positions.find(joint_names_[index]);
        if (found == named_positions.end()) {
          *reason = "capture joint state is missing " + joint_names_[index];
          return false;
        }
        joints(static_cast<unsigned int>(index)) = found->second;
      } else {
        joints(static_cast<unsigned int>(index)) = joint_state.position[index];
      }
    }

    KDL::Frame frame;
    if (fk_solver_->JntToCart(joints, frame) < 0) {
      *reason = "KDL forward kinematics failed for capture joint state";
      return false;
    }
    *result = kdlFrameToIsometry(frame);
    return true;
  }

  bool transformPath(
    const plasma_robot_interfaces::msg::SprayPath & input,
    plasma_robot_interfaces::msg::SprayPath * output,
    std::string * message)
  {
    std::scoped_lock lock(transform_mutex_);

    if (input.points.empty()) {
      *message = "input spray path is empty";
      return false;
    }
    if (input.points.size() > max_path_points_) {
      *message = "input spray path exceeds max_path_points";
      return false;
    }

    const std::string source_frame = normalizedFrame(input.header.frame_id);
    if (source_frame.empty()) {
      *message = "input header.frame_id is empty";
      return false;
    }

    Eigen::Isometry3d base_to_source = Eigen::Isometry3d::Identity();
    const bool source_is_base = source_frame == base_frame_;
    if (!source_is_base) {
      if (!handeye_.loaded) {
        *message = "hand-eye matrix is not loaded; refusing to generate fake robot coordinates";
        return false;
      }
      if (!handeye_.validated && !publish_unvalidated_preview_) {
        *message = "hand-eye matrix is not validated and preview output is disabled";
        return false;
      }
      if (source_refinement_.loaded && source_frame != source_refinement_.frame_id) {
        *message = "source refinement expects frame '" + source_refinement_.frame_id +
          "' but input path uses '" + source_frame + "'";
        return false;
      }

      Eigen::Isometry3d base_to_gripper = Eigen::Isometry3d::Identity();
      if (input.capture_gripper_pose_valid) {
        if (!input.base_frame.empty() && normalizedFrame(input.base_frame) != base_frame_) {
          *message = "capture gripper pose base frame does not match configured base frame";
          return false;
        }
        if (!input.gripper_frame.empty() &&
          normalizedFrame(input.gripper_frame) != gripper_frame_)
        {
          *message = "capture gripper pose child frame does not match configured gripper frame";
          return false;
        }
        try {
          base_to_gripper = PathTransformer::poseToIsometry(input.capture_gripper_pose);
        } catch (const std::exception & error) {
          *message = std::string("invalid capture gripper pose: ") + error.what();
          return false;
        }
      } else {
        if (!capturePoseFromJoints(input.capture_joint_state, &base_to_gripper, message)) {
          return false;
        }
      }
      Eigen::Isometry3d calibrated_camera_to_source = Eigen::Isometry3d::Identity();
      if (source_frame != camera_frame_) {
        if (!allow_camera_frame_tf_) {
          *message = "input frame '" + source_frame +
            "' does not match configured camera frame '" + camera_frame_ + "'";
          return false;
        }
        try {
          const auto camera_to_source = tf_buffer_->lookupTransform(
            camera_frame_, source_frame, tf2::TimePointZero,
            tf2::durationFromSec(camera_frame_tf_timeout_sec_));
          calibrated_camera_to_source = transformMsgToIsometry(camera_to_source.transform);
          RCLCPP_INFO(get_logger(),
            "Using camera TF %s <- %s for path '%s'",
            camera_frame_.c_str(), source_frame.c_str(), input.path_id.c_str());
        } catch (const std::exception & error) {
          *message = "cannot transform input frame '" + source_frame +
            "' into calibrated camera frame '" + camera_frame_ + "': " + error.what();
          return false;
        }
      }
      base_to_source = base_to_gripper * handeye_.transform * calibrated_camera_to_source;
      if (source_refinement_.loaded) {
        base_to_source = base_to_source * source_refinement_.transform;
      }
    }

    *output = input;
    output->header.frame_id = base_frame_;
    output->base_frame = base_frame_;
    output->gripper_frame = gripper_frame_;
    output->calibration_id = source_is_base ? "source_already_in_base" : handeye_.id;
    if (!source_is_base && source_refinement_.loaded) {
      output->calibration_id += "+training_refinement:" + source_refinement_.id;
    }
    output->transform_valid = false;
    output->execution_permitted = false;

    const bool any_entry_geometry =
      input.cavity_mouth_tip_pose_valid || input.cavity_entry_tcp_pose_valid ||
      input.pre_entry_tcp_pose_valid ||
      input.cavity_axis_valid;
    if ((cavity_axis_outward_compensation_m_ > 0.0 ||
      spray_axis_outward_compensation_m_ > 0.0) && !any_entry_geometry)
    {
      *message = "cavity-axis compensation requires complete cavity approach geometry";
      return false;
    }
    if (any_entry_geometry &&
      (!input.cavity_mouth_tip_pose_valid || !input.cavity_entry_tcp_pose_valid ||
      !input.pre_entry_tcp_pose_valid ||
      !input.cavity_axis_valid))
    {
      *message = "cavity approach geometry is incomplete";
      return false;
    }

    Eigen::Isometry3d base_to_compensated_source = base_to_source;
    Eigen::Isometry3d base_to_compensated_spray_source = base_to_source;
    try {
      if (any_entry_geometry) {
        const Eigen::Isometry3d source_to_mouth_tip =
          PathTransformer::poseToIsometry(input.cavity_mouth_tip_pose);
        const Eigen::Isometry3d source_to_nominal_entry =
          PathTransformer::poseToIsometry(input.cavity_entry_tcp_pose);
        const Eigen::Isometry3d source_to_nominal_pre_entry =
          PathTransformer::poseToIsometry(input.pre_entry_tcp_pose);
        Eigen::Vector3d source_axis = axisVector(input.cavity_axis);
        if (!source_axis.allFinite() || source_axis.norm() < 0.99 ||
          source_axis.norm() > 1.01) {
          throw std::runtime_error("cavity axis is not a finite unit vector");
        }
        source_axis.normalize();
        const Eigen::Vector3d base_axis = base_to_source.linear() * source_axis;
        base_to_compensated_source = PathTransformer::translatedAlongAxis(
          base_to_source, base_axis, cavity_axis_outward_compensation_m_);
        base_to_compensated_spray_source = PathTransformer::translatedAlongAxis(
          base_to_compensated_source, base_axis, spray_axis_outward_compensation_m_);
        if (cavity_axis_outward_compensation_m_ > 0.0) {
          RCLCPP_WARN(
            get_logger(),
            "Applying measured cavity-axis outward compensation %.3f mm to the complete path",
            cavity_axis_outward_compensation_m_ * 1000.0);
          std::ostringstream compensation_id;
          compensation_id.setf(std::ios::fixed);
          compensation_id.precision(3);
          compensation_id << "+axis_outward_compensation:"
                          << cavity_axis_outward_compensation_m_ * 1000.0 << "mm";
          output->calibration_id += compensation_id.str();
        }
        if (spray_axis_outward_compensation_m_ > 0.0) {
          RCLCPP_WARN(
            get_logger(),
            "Applying measured spray-only outward compensation %.3f mm; entry poses are unchanged",
            spray_axis_outward_compensation_m_ * 1000.0);
          std::ostringstream compensation_id;
          compensation_id.setf(std::ios::fixed);
          compensation_id.precision(3);
          compensation_id << "+spray_axis_outward_compensation:"
                          << spray_axis_outward_compensation_m_ * 1000.0 << "mm";
          output->calibration_id += compensation_id.str();
        }
        if (!std::isfinite(input.pre_entry_distance_m) ||
          input.pre_entry_distance_m < 0.01 || input.pre_entry_distance_m > 0.25) {
          throw std::runtime_error("pre-entry distance is outside [0.01, 0.25] m");
        }
        if (!std::isfinite(input.entry_tip_standoff_m) ||
          input.entry_tip_standoff_m < 0.0 || input.entry_tip_standoff_m > 0.15) {
          throw std::runtime_error("entry tip standoff is outside [0, 0.15] m");
        }

        const Eigen::Vector3d entry_to_pre =
          source_to_nominal_pre_entry.translation() -
          source_to_nominal_entry.translation();
        const double outward_distance = entry_to_pre.dot(source_axis);
        const double entry_axis_offset =
          (entry_to_pre - outward_distance * source_axis).norm();
        if (outward_distance <= 0.0 ||
          std::abs(outward_distance - input.pre_entry_distance_m) > 0.005) {
          throw std::runtime_error("pre-entry pose is not the configured outward distance from entry");
        }
        if (entry_axis_offset > max_entry_axis_offset_m_) {
          throw std::runtime_error("pre-entry pose is not on the cavity axis");
        }

        const Eigen::Vector3d mouth_to_entry =
          source_to_nominal_entry.translation() - source_to_mouth_tip.translation();
        const double mouth_entry_axial = mouth_to_entry.dot(source_axis);
        const double mouth_entry_axis_offset =
          (mouth_to_entry - mouth_entry_axial * source_axis).norm();
        if (mouth_entry_axis_offset > max_entry_axis_offset_m_) {
          throw std::runtime_error("physical cavity mouth is not on the cavity entry axis");
        }

        Eigen::Isometry3d source_to_entry = source_to_nominal_entry;
        if (process_geometry_loaded_) {
          // The reviewed entry stop is constrained by the rounded end, while spray
          // waypoints remain side-outlet TCP targets. Keep the tip outside the detected
          // physical mouth by the explicit motion standoff.
          Eigen::Isometry3d source_to_entry_tip = source_to_mouth_tip;
          source_to_entry_tip.translation() += input.entry_tip_standoff_m * source_axis;
          source_to_entry = PathTransformer::tcpPoseForNozzleTipPose(
            source_to_entry_tip, tcp_to_nozzle_tip_);
          const Eigen::Vector3d correction =
            source_to_entry.translation() - source_to_nominal_entry.translation();
          const double correction_axial = correction.dot(source_axis);
          const double correction_lateral =
            (correction - correction_axial * source_axis).norm();
          if (std::abs(correction_axial) > 0.005 || correction_lateral > 0.005) {
            throw std::runtime_error(
                    "nominal entry does not match the configured rounded-tip standoff");
          }
          RCLCPP_INFO(
            get_logger(),
            "Rounded-tip safe entry: standoff=%.3f mm, correction outward=%.3f mm, lateral=%.3f mm",
            input.entry_tip_standoff_m * 1000.0,
            correction_axial * 1000.0, correction_lateral * 1000.0);
        }
        Eigen::Isometry3d source_to_pre_entry = source_to_entry;
        source_to_pre_entry.translation() +=
          input.pre_entry_distance_m * source_axis;

        const Eigen::Isometry3d source_to_first =
          PathTransformer::poseToIsometry(input.points.front().tcp_pose);
        const Eigen::Vector3d entry_to_first =
          source_to_first.translation() - source_to_entry.translation();
        const double first_inward_distance = -entry_to_first.dot(source_axis);
        const double first_axis_offset =
          (entry_to_first + first_inward_distance * source_axis).norm();
        if (first_inward_distance - spray_axis_outward_compensation_m_ <= 0.0005) {
          throw std::runtime_error("cavity entry is not outward of the first spray layer");
        }
        if (first_axis_offset > max_first_layer_axis_offset_m_) {
          throw std::runtime_error("cavity entry and first spray layer are not coaxial");
        }
        if (rotationError(source_to_entry, source_to_first) >
          max_entry_orientation_error_rad_ ||
          rotationError(source_to_pre_entry, source_to_first) >
          max_entry_orientation_error_rad_ ||
          rotationError(source_to_mouth_tip, source_to_first) >
          max_entry_orientation_error_rad_) {
          throw std::runtime_error(
                  "mouth/entry orientation does not match the first zero-degree spray pose");
        }

        const Eigen::Isometry3d base_to_mouth_tip =
          base_to_compensated_source * source_to_mouth_tip;
        const Eigen::Isometry3d base_to_entry =
          base_to_compensated_source * source_to_entry;
        const Eigen::Isometry3d base_to_pre_entry =
          base_to_compensated_source * source_to_pre_entry;
        output->cavity_mouth_tip_pose = PathTransformer::isometryToPose(base_to_mouth_tip);
        output->cavity_entry_tcp_pose = PathTransformer::isometryToPose(base_to_entry);
        output->pre_entry_tcp_pose = PathTransformer::isometryToPose(base_to_pre_entry);
        output->cavity_axis.x = base_axis.x();
        output->cavity_axis.y = base_axis.y();
        output->cavity_axis.z = base_axis.z();
        output->cavity_mouth_tip_pose_valid = true;
        output->cavity_entry_tcp_pose_valid = true;
        output->pre_entry_tcp_pose_valid = true;
        output->cavity_axis_valid = true;
        if (tcp_.loaded) {
          output->cavity_entry_flange_pose = PathTransformer::isometryToPose(
            base_to_entry * tcp_.transform.inverse());
          output->pre_entry_flange_pose = PathTransformer::isometryToPose(
            base_to_pre_entry * tcp_.transform.inverse());
          output->cavity_entry_flange_pose_valid = true;
          output->pre_entry_flange_pose_valid = true;
        } else {
          output->cavity_entry_flange_pose_valid = false;
          output->pre_entry_flange_pose_valid = false;
        }
      } else {
        output->cavity_mouth_tip_pose_valid = false;
        output->cavity_entry_tcp_pose_valid = false;
        output->pre_entry_tcp_pose_valid = false;
        output->cavity_axis_valid = false;
        output->cavity_entry_flange_pose_valid = false;
        output->pre_entry_flange_pose_valid = false;
      }

      for (std::size_t index = 0; index < output->points.size(); ++index) {
        const Eigen::Isometry3d source_to_tcp =
          PathTransformer::poseToIsometry(input.points[index].tcp_pose);
        const Eigen::Isometry3d base_to_tcp =
          base_to_compensated_spray_source * source_to_tcp;
        output->points[index].tcp_pose = PathTransformer::isometryToPose(base_to_tcp);
        output->points[index].surface_target = PathTransformer::transformPoint(
          base_to_compensated_spray_source, input.points[index].surface_target);

        if (tcp_.loaded) {
          const Eigen::Isometry3d base_to_flange = base_to_tcp * tcp_.transform.inverse();
          output->points[index].flange_pose = PathTransformer::isometryToPose(base_to_flange);
          output->points[index].flange_pose_valid = true;
        } else {
          output->points[index].flange_pose_valid = false;
        }
        if (process_geometry_loaded_) {
          const Eigen::Isometry3d base_to_nozzle_tip =
            base_to_tcp * tcp_to_nozzle_tip_;
          output->points[index].safety_point.x = base_to_nozzle_tip.translation().x();
          output->points[index].safety_point.y = base_to_nozzle_tip.translation().y();
          output->points[index].safety_point.z = base_to_nozzle_tip.translation().z();
          output->points[index].safety_point_valid = true;
          output->points[index].spray_outlet_pose = PathTransformer::isometryToPose(
            base_to_tcp * tcp_to_spray_outlet_);
          output->points[index].spray_outlet_pose_valid = true;
        } else {
          output->points[index].safety_point_valid = false;
          output->points[index].spray_outlet_pose_valid = false;
        }
      }
    } catch (const std::exception & error) {
      *message = std::string("path point conversion failed: ") + error.what();
      return false;
    }

    const bool handeye_execution_ok = source_is_base ||
      !require_validated_handeye_ || handeye_.validated;
    const bool tcp_execution_ok = tcp_.loaded && process_geometry_loaded_ &&
      (!require_validated_tcp_ || tcp_.validated);
    output->transform_valid = true;
    const bool axial_compensation_active = cavity_axis_outward_compensation_m_ > 0.0 ||
      spray_axis_outward_compensation_m_ > 0.0;
    output->execution_permitted = handeye_execution_ok && tcp_execution_ok &&
      !source_refinement_.loaded && !axial_compensation_active;
    output->status_message = output->execution_permitted
      ? "coordinate transform valid and calibration approved for downstream execution"
      : (source_refinement_.loaded
        ? "training-only source refinement active; preview only and execution forced off"
        : (axial_compensation_active
          ? "measured cavity/spray-axis compensation active; commissioning dry-run only"
          : "coordinate transform available for preview only; calibration is incomplete or unvalidated"));
    *message = output->status_message;
    return true;
  }

  void onPath(const plasma_robot_interfaces::msg::SprayPath::SharedPtr input)
  {
    plasma_robot_interfaces::msg::SprayPath output;
    std::string message;
    if (!transformPath(*input, &output, &message)) {
      RCLCPP_ERROR(get_logger(), "Rejected path '%s': %s",
        input->path_id.c_str(), message.c_str());
      publishStatus(message);
      return;
    }

    output_publisher_->publish(output);
    publishStatus(message);
    RCLCPP_INFO(get_logger(), "Transformed path '%s': %zu poses, executable=%s",
      output.path_id.c_str(), output.points.size(),
      output.execution_permitted ? "true" : "false");
  }

  void onTransformService(
    const std::shared_ptr<plasma_robot_interfaces::srv::TransformSprayPath::Request> request,
    std::shared_ptr<plasma_robot_interfaces::srv::TransformSprayPath::Response> response)
  {
    response->success = transformPath(request->input, &response->output, &response->message);
    publishStatus(response->message);
  }

  void onReloadCalibration(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    reloadCalibration();
    response->success = handeye_.loaded;
    response->message = statusMessage();
    publishStatus(response->message);
  }

  std::string statusMessage() const
  {
    if (!calibration_error_.empty()) {
      return calibration_error_;
    }
    if (!handeye_.loaded) {
      return "waiting for hand-eye YAML; no robot-frame path will be published";
    }
    if (!tcp_.loaded) {
      return "hand-eye loaded; waiting for T_gripper_to_tcp YAML (preview only)";
    }
    if (!process_geometry_loaded_) {
      return "tool TCP loaded; waiting for nozzle-tip and spray-outlet transforms (preview only)";
    }
    if (source_refinement_.loaded) {
      return "training-only source refinement active (preview only; execution forced off)";
    }
    if (cavity_axis_outward_compensation_m_ > 0.0 ||
      spray_axis_outward_compensation_m_ > 0.0)
    {
      std::ostringstream message;
      message.setf(std::ios::fixed);
      message.precision(1);
      message << "measured outward compensation active: complete_path="
              << cavity_axis_outward_compensation_m_ * 1000.0
              << " mm, spray_only=" << spray_axis_outward_compensation_m_ * 1000.0
              << " mm (commissioning dry-run only)";
      return message.str();
    }
    if ((require_validated_handeye_ && !handeye_.validated) ||
      (require_validated_tcp_ && !tcp_.validated))
    {
      return "calibration loaded but not approved (preview only)";
    }
    return "coordinate transform calibration ready for downstream execution";
  }

  void publishStatus(const std::string & message)
  {
    plasma_robot_interfaces::msg::PathTransformStatus status;
    status.header.stamp = now();
    status.header.frame_id = base_frame_;
    status.robot_model_loaded = robot_model_loaded_;
    status.handeye_loaded = handeye_.loaded;
    status.handeye_validated = handeye_.validated;
    status.tcp_loaded = tcp_.loaded;
    status.tcp_validated = tcp_.validated;
    status.ready_for_preview = handeye_.loaded &&
      (robot_model_loaded_ || !robot_description_.empty());
    status.ready_for_execution = status.ready_for_preview && tcp_.loaded &&
      process_geometry_loaded_ &&
      !source_refinement_.loaded &&
      cavity_axis_outward_compensation_m_ <= 0.0 &&
      spray_axis_outward_compensation_m_ <= 0.0 &&
      (!require_validated_handeye_ || handeye_.validated) &&
      (!require_validated_tcp_ || tcp_.validated);
    status.calibration_id = handeye_.id;
    if (source_refinement_.loaded) {
      status.calibration_id += "+training_refinement:" + source_refinement_.id;
    }
    if (cavity_axis_outward_compensation_m_ > 0.0) {
      std::ostringstream compensation_id;
      compensation_id.setf(std::ios::fixed);
      compensation_id.precision(3);
      compensation_id << "+axis_outward_compensation:"
                      << cavity_axis_outward_compensation_m_ * 1000.0 << "mm";
      status.calibration_id += compensation_id.str();
    }
    if (spray_axis_outward_compensation_m_ > 0.0) {
      std::ostringstream compensation_id;
      compensation_id.setf(std::ios::fixed);
      compensation_id.precision(3);
      compensation_id << "+spray_axis_outward_compensation:"
                      << spray_axis_outward_compensation_m_ * 1000.0 << "mm";
      status.calibration_id += compensation_id.str();
    }
    status.message = message;
    status_publisher_->publish(status);
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string status_topic_;
  std::string base_frame_;
  std::string gripper_frame_;
  std::string camera_frame_;
  std::string tcp_frame_;
  std::string handeye_yaml_;
  std::string tcp_yaml_;
  std::string source_refinement_yaml_;
  std::string robot_description_file_;
  std::string robot_description_;
  bool require_validated_handeye_ = true;
  bool require_validated_tcp_ = true;
  bool publish_unvalidated_preview_ = true;
  bool allow_camera_frame_tf_ = true;
  double camera_frame_tf_timeout_sec_ = 0.5;
  std::size_t max_path_points_ = 200000;
  double max_entry_axis_offset_m_ = 0.002;
  double max_first_layer_axis_offset_m_ = 0.020;
  double max_entry_orientation_error_rad_ = 0.08726646259971647;
  double cavity_axis_outward_compensation_m_ = 0.0;
  double spray_axis_outward_compensation_m_ = 0.0;

  bool robot_model_loaded_ = false;
  KDL::Chain kdl_chain_;
  std::vector<std::string> joint_names_;
  std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
  TransformCalibration handeye_;
  TransformCalibration tcp_;
  TransformCalibration source_refinement_;
  Eigen::Isometry3d tcp_to_nozzle_tip_ = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tcp_to_spray_outlet_ = Eigen::Isometry3d::Identity();
  bool process_geometry_loaded_ = false;
  std::string calibration_error_;
  mutable std::mutex transform_mutex_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Publisher<plasma_robot_interfaces::msg::SprayPath>::SharedPtr output_publisher_;
  rclcpp::Publisher<plasma_robot_interfaces::msg::PathTransformStatus>::SharedPtr status_publisher_;
  rclcpp::Subscription<plasma_robot_interfaces::msg::SprayPath>::SharedPtr input_subscription_;
  rclcpp::Service<plasma_robot_interfaces::srv::TransformSprayPath>::SharedPtr transform_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reload_service_;
};

}  // namespace plasma_path_transform

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<plasma_path_transform::PathTransformNode>());
  rclcpp::shutdown();
  return 0;
}
