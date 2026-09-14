#include "plasma_path_executor/path_validation.hpp"

#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose.hpp>
#include <plasma_robot_interfaces/msg/path_execution_status.hpp>
#include <plasma_robot_interfaces/msg/spray_path.hpp>
#include <plasma_robot_interfaces/srv/start_spray_path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rm_ros_interfaces/msg/jointcurrent.hpp>
#include <rm_ros_interfaces/msg/movel.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int16.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace plasma_path_executor
{

class PathExecutorNode : public rclcpp::Node
{
public:
  PathExecutorNode()
  : Node("plasma_path_executor")
  {
    path_topic_ = declare_parameter<std::string>(
      "path_topic", "/plasma/planned_spray_path/base");
    flange_pose_topic_ = declare_parameter<std::string>(
      "flange_pose_topic", "/rm_driver/udp_arm_position");
    motion_enabled_ = declare_parameter<bool>("motion_enabled", false);
    speed_percent_ = declare_parameter<int>("speed_percent", 5);
    entry_approach_enabled_ = declare_parameter<bool>("entry_approach_enabled", false);
    collision_clearance_required_ = declare_parameter<bool>(
      "collision_clearance_required", true);
    collision_clearance_max_age_sec_ = declare_parameter<double>(
      "collision_clearance_max_age_sec", 120.0);
    required_collision_stage_ = declare_parameter<int>("required_collision_stage", 8);
    current_monitor_enabled_ = declare_parameter<bool>("current_monitor_enabled", true);
    current_max_age_ms_ = declare_parameter<int>("current_max_age_ms", 500);
    current_delta_threshold_ma_ = declare_parameter<double>(
      "current_delta_threshold_ma", 1500.0);
    max_pose_age_ms_ = declare_parameter<int>("max_pose_age_ms", 500);
    command_timeout_ms_ = declare_parameter<int>("command_timeout_ms", 90000);
    pose_settle_timeout_ms_ = declare_parameter<int>("pose_settle_timeout_ms", 2500);
    stop_timeout_ms_ = declare_parameter<int>("stop_timeout_ms", 3000);
    entry_translation_tolerance_m_ = declare_parameter<double>(
      "entry_translation_tolerance_m", 0.003);
    entry_rotation_tolerance_rad_ = declare_parameter<double>(
      "entry_rotation_tolerance_rad", 0.0872664626);
    goal_translation_tolerance_m_ = declare_parameter<double>(
      "goal_translation_tolerance_m", 0.002);
    goal_rotation_tolerance_rad_ = declare_parameter<double>(
      "goal_rotation_tolerance_rad", 0.0523598776);
    linear_entry_step_m_ = declare_parameter<double>("linear_entry_step_m", 0.005);
    manual_correction_max_translation_m_ = declare_parameter<double>(
      "manual_correction_max_translation_m", 0.030);
    manual_correction_max_axis_offset_m_ = declare_parameter<double>(
      "manual_correction_max_axis_offset_m", 0.010);
    manual_correction_max_rotation_rad_ = declare_parameter<double>(
      "manual_correction_max_rotation_rad", 0.17453292519943295);
    entry_approach_limits_.max_translation_m = declare_parameter<double>(
      "max_entry_approach_translation_m", 0.2);
    entry_approach_limits_.max_rotation_rad = declare_parameter<double>(
      "max_entry_approach_rotation_rad", 3.14159265358979323846);
    entry_approach_limits_.max_axis_error_rad = declare_parameter<double>(
      "max_entry_approach_axis_error_rad", 0.3490658503988659);
    entry_approach_limits_.max_tool_to_path_axis_error_rad = declare_parameter<double>(
      "max_entry_tool_to_path_axis_error_rad", 0.08726646259971647);
    entry_approach_limits_.max_path_axis_offset_m = declare_parameter<double>(
      "max_entry_path_axis_offset_m", 0.02);
    limits_.max_points = static_cast<std::size_t>(std::max<int64_t>(1,
      declare_parameter<int64_t>("max_points", 20000)));
    limits_.max_segment_translation_m = declare_parameter<double>(
      "max_segment_translation_m", 0.05);
    limits_.max_segment_rotation_rad = declare_parameter<double>(
      "max_segment_rotation_rad", 0.35);
    limits_.min_spray_distance_m = declare_parameter<double>(
      "min_spray_distance_m", 0.005);
    limits_.max_spray_distance_m = declare_parameter<double>(
      "max_spray_distance_m", 1.0);
    limits_.max_spray_direction_error_rad = declare_parameter<double>(
      "max_spray_direction_error_rad", 0.034906585);
    validateParameters();

    const auto latched_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    path_subscription_ = create_subscription<plasma_robot_interfaces::msg::SprayPath>(
      path_topic_, latched_qos,
      std::bind(&PathExecutorNode::onPath, this, std::placeholders::_1));
    flange_pose_subscription_ = create_subscription<geometry_msgs::msg::Pose>(
      flange_pose_topic_, rclcpp::QoS(10),
      std::bind(&PathExecutorNode::onFlangePose, this, std::placeholders::_1));
    movel_result_subscription_ = create_subscription<std_msgs::msg::Bool>(
      "/rm_driver/movel_result", rclcpp::ParametersQoS(),
      std::bind(&PathExecutorNode::onMoveResult, this, std::placeholders::_1));
    stop_result_subscription_ = create_subscription<std_msgs::msg::Bool>(
      "/rm_driver/move_stop_result", rclcpp::ParametersQoS(),
      std::bind(&PathExecutorNode::onStopResult, this, std::placeholders::_1));
    clearance_subscription_ = create_subscription<std_msgs::msg::String>(
      "/plasma/entry_motion/collision_checked_path", latched_qos,
      std::bind(&PathExecutorNode::onCollisionClearance, this, std::placeholders::_1));
    collision_stage_subscription_ = create_subscription<std_msgs::msg::UInt16>(
      "/rm_driver/collision_stage", rclcpp::ParametersQoS(),
      std::bind(&PathExecutorNode::onCollisionStage, this, std::placeholders::_1));
    joint_current_subscription_ = create_subscription<rm_ros_interfaces::msg::Jointcurrent>(
      "/rm_driver/udp_joint_current", rclcpp::QoS(10),
      std::bind(&PathExecutorNode::onJointCurrent, this, std::placeholders::_1));

    movel_publisher_ = create_publisher<rm_ros_interfaces::msg::Movel>(
      "/rm_driver/movel_cmd", rclcpp::ParametersQoS());
    stop_publisher_ = create_publisher<std_msgs::msg::Empty>(
      "/rm_driver/move_stop_cmd", rclcpp::ParametersQoS());
    collision_stage_command_publisher_ = create_publisher<std_msgs::msg::UInt16>(
      "/rm_driver/set_collision_stage_cmd", rclcpp::ParametersQoS());
    status_publisher_ = create_publisher<plasma_robot_interfaces::msg::PathExecutionStatus>(
      "/plasma/path_executor/status", latched_qos);

    start_service_ = create_service<plasma_robot_interfaces::srv::StartSprayPath>(
      "~/start",
      std::bind(&PathExecutorNode::onStart, this,
        std::placeholders::_1, std::placeholders::_2));
    stop_service_ = create_service<std_srvs::srv::Trigger>(
      "~/stop",
      std::bind(&PathExecutorNode::onStop, this,
        std::placeholders::_1, std::placeholders::_2));
    watchdog_ = create_wall_timer(
      std::chrono::milliseconds(100), std::bind(&PathExecutorNode::onWatchdog, this));

    std::scoped_lock lock(mutex_);
    message_ = motion_enabled_ ?
      "waiting for a transformed spray path" :
      "motion_enabled=false; path checking only";
    publishStatusLocked();
  }

  ~PathExecutorNode() override
  {
    std::scoped_lock lock(mutex_);
    if (state_ == Status::STATE_EXECUTING || state_ == Status::STATE_STOPPING) {
      stop_publisher_->publish(std_msgs::msg::Empty{});
    }
  }

private:
  using Status = plasma_robot_interfaces::msg::PathExecutionStatus;
  using SprayPath = plasma_robot_interfaces::msg::SprayPath;
  using SprayPoint = plasma_robot_interfaces::msg::SprayPathPoint;
  enum class ApproachPhase {NONE, TO_PRE_ENTRY, TO_ENTRY, TO_FIRST_POINT};

  void validateParameters() const
  {
    if (speed_percent_ < 1 || speed_percent_ > 100) {
      throw std::runtime_error("speed_percent must be in [1, 100]");
    }
    if (collision_clearance_max_age_sec_ <= 0.0 || required_collision_stage_ < 1 ||
      required_collision_stage_ > 8 || current_max_age_ms_ <= 0 ||
      current_delta_threshold_ma_ <= 0.0) {
      throw std::runtime_error("collision safety parameters are invalid");
    }
    if (max_pose_age_ms_ <= 0 || command_timeout_ms_ <= 0 ||
        pose_settle_timeout_ms_ <= 0 || stop_timeout_ms_ <= 0) {
      throw std::runtime_error("executor timeout parameters must be positive");
    }
    if (entry_translation_tolerance_m_ <= 0.0 || entry_rotation_tolerance_rad_ <= 0.0 ||
        goal_translation_tolerance_m_ <= 0.0 || goal_rotation_tolerance_rad_ <= 0.0) {
      throw std::runtime_error("executor pose tolerances must be positive");
    }
    if (linear_entry_step_m_ <= 0.0 || linear_entry_step_m_ > 0.02 ||
        manual_correction_max_translation_m_ <= 0.0 ||
        manual_correction_max_axis_offset_m_ <= 0.0 ||
        manual_correction_max_rotation_rad_ <= 0.0) {
      throw std::runtime_error("linear entry and manual correction parameters are invalid");
    }
    if (entry_approach_limits_.max_translation_m <= 0.0 ||
        entry_approach_limits_.max_rotation_rad <= 0.0 ||
        entry_approach_limits_.max_axis_error_rad <= 0.0 ||
        entry_approach_limits_.max_axis_error_rad >= 1.570796327 ||
        entry_approach_limits_.max_tool_to_path_axis_error_rad <= 0.0 ||
        entry_approach_limits_.max_tool_to_path_axis_error_rad >= 1.570796327 ||
        entry_approach_limits_.max_path_axis_offset_m <= 0.0) {
      throw std::runtime_error("entry approach limits are invalid");
    }
    if (limits_.min_spray_distance_m <= 0.0 ||
        limits_.max_spray_distance_m <= limits_.min_spray_distance_m ||
        limits_.max_spray_direction_error_rad <= 0.0 ||
        limits_.max_spray_direction_error_rad >= 1.570796327) {
      throw std::runtime_error("spray target validation limits are invalid");
    }
  }

  bool isActiveLocked() const
  {
    return state_ == Status::STATE_EXECUTING || state_ == Status::STATE_STOPPING;
  }

  bool poseFreshLocked() const
  {
    if (!have_flange_pose_) {
      return false;
    }
    const auto age = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now() - flange_pose_time_).count();
    return age <= max_pose_age_ms_;
  }

  bool currentFreshLocked() const
  {
    if (!have_joint_current_ || latest_joint_currents_.size() < 6) {
      return false;
    }
    const auto age = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now() - joint_current_time_).count();
    return age <= current_max_age_ms_;
  }

  bool collisionStageReadyLocked() const
  {
    return collision_stage_subscription_->get_publisher_count() > 0 &&
           have_collision_stage_ && collision_stage_ == required_collision_stage_;
  }

  bool collisionClearanceReadyLocked() const
  {
    if (!collision_clearance_required_) {
      return true;
    }
    if (!path_available_ || collision_checked_path_id_ != path_.path_id ||
      collision_checked_path_stamp_sec_ != path_.header.stamp.sec ||
      collision_checked_path_stamp_nanosec_ != path_.header.stamp.nanosec ||
      collision_clearance_stamp_ns_ <= 0) {
      return false;
    }
    const auto age_ns = now().nanoseconds() - collision_clearance_stamp_ns_;
    return age_ns >= 0 && static_cast<double>(age_ns) * 1e-9 <=
           collision_clearance_max_age_sec_;
  }

  void requestCollisionStageLocked()
  {
    if (collision_stage_subscription_->get_publisher_count() == 0) {
      have_collision_stage_ = false;
      return;
    }
    if (have_collision_stage_ && collision_stage_ == required_collision_stage_) {
      return;
    }
    const auto current_time = std::chrono::steady_clock::now();
    if (last_collision_stage_request_.time_since_epoch().count() != 0 &&
      current_time - last_collision_stage_request_ < std::chrono::seconds(5)) {
      return;
    }
    std_msgs::msg::UInt16 command;
    command.data = static_cast<std::uint16_t>(required_collision_stage_);
    collision_stage_command_publisher_->publish(command);
    last_collision_stage_request_ = current_time;
  }

  bool driverReadyLocked() const
  {
    return movel_publisher_->get_subscription_count() > 0 &&
           movel_result_subscription_->get_publisher_count() > 0 &&
           stop_publisher_->get_subscription_count() > 0 &&
           stop_result_subscription_->get_publisher_count() > 0 &&
           collision_stage_command_publisher_->get_subscription_count() > 0 &&
           collision_stage_subscription_->get_publisher_count() > 0 &&
           (!current_monitor_enabled_ || joint_current_subscription_->get_publisher_count() > 0);
  }

  bool entryPoseValidLocked(PoseError * error = nullptr) const
  {
    if (!path_available_ || !poseFreshLocked()) {
      return false;
    }
    const geometry_msgs::msg::Pose & target = path_.cavity_entry_flange_pose_valid ?
      path_.cavity_entry_flange_pose : path_.points.front().flange_pose;
    const PoseError value = poseError(
      latest_flange_pose_, target);
    if (error) {
      *error = value;
    }
    return value.translation_m <= entry_translation_tolerance_m_ &&
           value.rotation_rad <= entry_rotation_tolerance_rad_;
  }

  bool firstPoseValidLocked() const
  {
    if (!path_available_ || !poseFreshLocked()) {
      return false;
    }
    const PoseError value = poseError(latest_flange_pose_, path_.points.front().flange_pose);
    return value.translation_m <= entry_translation_tolerance_m_ &&
           value.rotation_rad <= entry_rotation_tolerance_rad_;
  }

  bool preEntryPoseValidLocked() const
  {
    if (!path_available_ || !poseFreshLocked() ||
      !path_.pre_entry_flange_pose_valid) {
      return false;
    }
    const PoseError value = poseError(latest_flange_pose_, path_.pre_entry_flange_pose);
    return value.translation_m <= entry_translation_tolerance_m_ &&
           value.rotation_rad <= entry_rotation_tolerance_rad_;
  }

  bool manualEntryCorrectionValidLocked(std::string * reason) const
  {
    if (!path_.cavity_entry_flange_pose_valid || !path_.cavity_axis_valid) {
      *reason = "path has no explicit cavity entry geometry";
      return false;
    }
    const PoseError error = poseError(
      latest_flange_pose_, path_.cavity_entry_flange_pose);
    const Eigen::Vector3d axis(
      path_.cavity_axis.x, path_.cavity_axis.y, path_.cavity_axis.z);
    const Eigen::Vector3d delta(
      latest_flange_pose_.position.x - path_.cavity_entry_flange_pose.position.x,
      latest_flange_pose_.position.y - path_.cavity_entry_flange_pose.position.y,
      latest_flange_pose_.position.z - path_.cavity_entry_flange_pose.position.z);
    const double axis_offset = (delta - axis * delta.dot(axis)).norm();
    if (error.translation_m > manual_correction_max_translation_m_) {
      *reason = "manual pose is farther than the correction translation limit";
      return false;
    }
    if (axis_offset > manual_correction_max_axis_offset_m_) {
      *reason = "manual pose is too far from the cavity axis";
      return false;
    }
    if (error.rotation_rad > manual_correction_max_rotation_rad_) {
      *reason = "manual pose orientation error exceeds the correction limit";
      return false;
    }
    return true;
  }

  bool manualPreEntryCorrectionValidLocked(std::string * reason) const
  {
    if (!path_.pre_entry_flange_pose_valid || !path_.cavity_axis_valid) {
      *reason = "path has no explicit pre-entry geometry";
      return false;
    }
    const PoseError error = poseError(latest_flange_pose_, path_.pre_entry_flange_pose);
    const Eigen::Vector3d axis(
      path_.cavity_axis.x, path_.cavity_axis.y, path_.cavity_axis.z);
    const Eigen::Vector3d delta(
      latest_flange_pose_.position.x - path_.pre_entry_flange_pose.position.x,
      latest_flange_pose_.position.y - path_.pre_entry_flange_pose.position.y,
      latest_flange_pose_.position.z - path_.pre_entry_flange_pose.position.z);
    const double axis_offset = (delta - axis * delta.dot(axis)).norm();
    if (error.translation_m > manual_correction_max_translation_m_) {
      *reason = "current pose is farther than the pre-entry correction translation limit";
      return false;
    }
    if (axis_offset > manual_correction_max_axis_offset_m_) {
      *reason = "current pose is too far from the pre-entry axis";
      return false;
    }
    if (error.rotation_rad > manual_correction_max_rotation_rad_) {
      *reason = "current pose orientation error exceeds the pre-entry correction limit";
      return false;
    }
    return true;
  }

  void onPath(const SprayPath::SharedPtr path)
  {
    std::scoped_lock lock(mutex_);
    if (isActiveLocked()) {
      RCLCPP_WARN(get_logger(), "Ignored replacement path '%s' while execution is active",
        path->path_id.c_str());
      return;
    }

    // Every newly received path invalidates the previous MoveIt clearance, even if its name is reused.
    collision_checked_path_id_.clear();
    collision_checked_path_stamp_sec_ = 0;
    collision_checked_path_stamp_nanosec_ = 0;
    collision_clearance_stamp_ns_ = 0;

    std::string error;
    if (!validatePathStructure(*path, limits_, &error)) {
      path_available_ = false;
      path_ = SprayPath{};
      state_ = Status::STATE_ERROR;
      current_index_ = -1;
      message_ = "rejected transformed path: " + error;
      publishStatusLocked();
      return;
    }

    path_ = *path;
    path_available_ = true;
    current_index_ = 0;
    state_ = Status::STATE_PATH_READY;
    dry_run_ = true;
    desired_plasma_enabled_ = false;
    message_ = path_.execution_permitted ?
      "path structurally valid; waiting for explicit start" :
      "path valid for preview but calibration does not permit motion";
    publishStatusLocked();
  }

  void onFlangePose(const geometry_msgs::msg::Pose::SharedPtr pose)
  {
    std::scoped_lock lock(mutex_);
    if (!poseIsFiniteAndNormalized(*pose)) {
      return;
    }
    latest_flange_pose_ = *pose;
    flange_pose_time_ = std::chrono::steady_clock::now();
    have_flange_pose_ = true;
    ++flange_pose_sequence_;
  }

  void onCollisionClearance(const std_msgs::msg::String::SharedPtr clearance)
  {
    std::scoped_lock lock(mutex_);
    collision_checked_path_id_.clear();
    collision_checked_path_stamp_sec_ = 0;
    collision_checked_path_stamp_nanosec_ = 0;
    collision_clearance_stamp_ns_ = 0;
    std::vector<std::string> fields;
    std::size_t begin = 0;
    while (begin <= clearance->data.size()) {
      const auto separator = clearance->data.find('|', begin);
      fields.push_back(clearance->data.substr(
        begin, separator == std::string::npos ? std::string::npos : separator - begin));
      if (separator == std::string::npos) {
        break;
      }
      begin = separator + 1;
    }
    if (fields.size() != 4 || fields.front().empty()) {
      return;
    }
    try {
      collision_checked_path_id_ = fields[0];
      collision_checked_path_stamp_sec_ = std::stoll(fields[1]);
      collision_checked_path_stamp_nanosec_ = static_cast<std::uint32_t>(std::stoul(fields[2]));
      collision_clearance_stamp_ns_ = std::stoll(fields[3]);
    } catch (const std::exception &) {
      collision_checked_path_id_.clear();
      collision_checked_path_stamp_sec_ = 0;
      collision_checked_path_stamp_nanosec_ = 0;
      collision_clearance_stamp_ns_ = 0;
    }
  }

  void onCollisionStage(const std_msgs::msg::UInt16::SharedPtr stage)
  {
    std::scoped_lock lock(mutex_);
    collision_stage_ = static_cast<int>(stage->data);
    collision_stage_time_ = std::chrono::steady_clock::now();
    have_collision_stage_ = true;
  }

  void onJointCurrent(const rm_ros_interfaces::msg::Jointcurrent::SharedPtr current)
  {
    std::scoped_lock lock(mutex_);
    latest_joint_currents_.assign(current->joint_current.begin(), current->joint_current.end());
    joint_current_time_ = std::chrono::steady_clock::now();
    have_joint_current_ = latest_joint_currents_.size() >= 6;
    if (!current_monitor_enabled_ || state_ != Status::STATE_EXECUTING ||
      !command_in_flight_ || command_current_baseline_.size() != latest_joint_currents_.size()) {
      return;
    }
    double max_raise = 0.0;
    std::size_t max_joint = 0;
    for (std::size_t index = 0; index < latest_joint_currents_.size(); ++index) {
      const double raise = std::abs(static_cast<double>(latest_joint_currents_[index])) -
        std::abs(static_cast<double>(command_current_baseline_[index]));
      if (raise > max_raise) {
        max_raise = raise;
        max_joint = index;
      }
    }
    if (max_raise > current_delta_threshold_ma_) {
      beginStopLocked(
        "joint " + std::to_string(max_joint + 1) + " current magnitude rose " +
        std::to_string(max_raise) + " mA; possible collision",
        Status::STATE_ERROR);
    }
  }

  void onStart(
    const std::shared_ptr<plasma_robot_interfaces::srv::StartSprayPath::Request> request,
    std::shared_ptr<plasma_robot_interfaces::srv::StartSprayPath::Response> response)
  {
    std::scoped_lock lock(mutex_);
    auto reject = [&](const std::string & message) {
        response->accepted = false;
        response->message = message;
        message_ = message;
        publishStatusLocked();
      };

    if (motion_enabled_) {
      reject(
        "real Cartesian MoveL execution is disabled: it can select an IK branch different "
        "from the reviewed MoveIt joint trajectory; use the entry planner reviewed dry-run service");
      return;
    }

    if (isActiveLocked()) {
      reject("an execution or stop operation is already active");
      return;
    }
    if (!request->dry_run) {
      reject("live plasma execution is not implemented; start with dry_run=true only");
      return;
    }
    if (!path_available_) {
      reject("no structurally valid transformed path is available");
      return;
    }
    if (!request->path_id.empty() && request->path_id != path_.path_id) {
      reject("requested path_id does not match the buffered path");
      return;
    }
    if (!driverReadyLocked()) {
      reject("RealMan motion, collision-stage, or current-monitor topics are not ready");
      return;
    }
    if (!collisionClearanceReadyLocked()) {
      reject("this path has no fresh full-path MoveIt collision clearance; plan to pre-entry again");
      return;
    }
    if (!collisionStageReadyLocked()) {
      requestCollisionStageLocked();
      reject("RealMan collision detection stage 8 is not freshly verified");
      return;
    }
    if (current_monitor_enabled_ && !currentFreshLocked()) {
      reject("joint-current stream is missing or stale; motion is locked");
      return;
    }
    if (!poseFreshLocked()) {
      reject("current Link6 pose is missing or stale; motion is locked");
      return;
    }
    const bool already_at_first_point = firstPoseValidLocked();
    const bool already_at_entry = entryPoseValidLocked();
    const bool already_at_pre_entry = preEntryPoseValidLocked();
    bool pre_entry_correction_allowed = false;
    std::string pre_entry_correction_reason;
    bool manual_correction_allowed = false;
    std::string manual_correction_reason;
    if (!already_at_first_point) {
      if (!request->approach_from_confirmed_entry) {
        reject("robot is not at the first spray pose and cavity-entry approach was not confirmed");
        return;
      }
      if (!entry_approach_enabled_) {
        reject("entry_approach_enabled=false; automatic cavity-entry approach is locked");
        return;
      }
      if (!path_.cavity_entry_flange_pose_valid ||
        !path_.pre_entry_flange_pose_valid || !path_.cavity_axis_valid) {
        reject("automatic entry requires explicit transformed entry and pre-entry poses");
        return;
      }
      manual_correction_allowed = manualEntryCorrectionValidLocked(
        &manual_correction_reason);
      pre_entry_correction_allowed = manualPreEntryCorrectionValidLocked(
        &pre_entry_correction_reason);
      if (!already_at_entry && !already_at_pre_entry &&
        !pre_entry_correction_allowed && !manual_correction_allowed) {
        reject("robot is neither at pre-entry nor within a bounded correction envelope: " +
          pre_entry_correction_reason + "; " + manual_correction_reason +
          "; use the MoveIt pre-entry planner first");
        return;
      }
    }
    if (!path_.execution_permitted) {
      reject("path execution_permitted=false; hand-eye and TCP acceptance are incomplete");
      return;
    }
    if (!motion_enabled_) {
      reject("motion_enabled=false; executor is locked in path-checking mode");
      return;
    }

    dry_run_ = true;
    current_index_ = already_at_first_point ? 0 : -1;
    command_index_ = -1;
    command_in_flight_ = false;
    waiting_pose_verification_ = false;
    desired_plasma_enabled_ = already_at_first_point && path_.points.front().plasma_enabled;
    state_ = Status::STATE_EXECUTING;
    message_ = already_at_first_point ?
      "first spray pose confirmed; executing spray path in plasma-off dry-run" :
      already_at_pre_entry ?
      "pre-entry confirmed; starting 5 mm segmented linear cavity entry" :
      pre_entry_correction_allowed ?
      "near pre-entry; starting bounded 5 mm correction before cavity entry" :
      already_at_entry ?
      "cavity entry confirmed; moving to first spray layer in 5 mm segments" :
      "manual entry pose accepted; correcting to the planned entry in 5 mm segments";
    response->accepted = true;
    response->message = message_;
    publishStatusLocked();
    if (already_at_first_point) {
      advanceLocked();
    } else if (already_at_entry) {
      startLinearApproachLocked(
        path_.points.front().flange_pose, ApproachPhase::TO_FIRST_POINT,
        "starting segmented move from cavity entry to first spray layer");
    } else if (already_at_pre_entry) {
      startLinearApproachLocked(
        path_.cavity_entry_flange_pose, ApproachPhase::TO_ENTRY,
        "starting segmented move from pre-entry to cavity entry");
    } else if (pre_entry_correction_allowed) {
      startLinearApproachLocked(
        path_.pre_entry_flange_pose, ApproachPhase::TO_PRE_ENTRY,
        "starting bounded correction to the planned pre-entry");
    } else {
      startLinearApproachLocked(
        path_.cavity_entry_flange_pose, ApproachPhase::TO_ENTRY,
        "starting bounded correction from manual pose to cavity entry");
    }
  }

  void onStop(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    std::scoped_lock lock(mutex_);
    if (state_ != Status::STATE_EXECUTING) {
      response->success = false;
      response->message = "no active path execution";
      return;
    }
    response->success = true;
    response->message = "stop command sent";
    beginStopLocked("operator requested stop", Status::STATE_PATH_READY);
  }

  void onMoveResult(const std_msgs::msg::Bool::SharedPtr result)
  {
    std::scoped_lock lock(mutex_);
    if (state_ != Status::STATE_EXECUTING || !command_in_flight_) {
      return;
    }
    if (!result->data) {
      beginStopLocked("RealMan MoveL returned failure", Status::STATE_ERROR);
      return;
    }
    command_in_flight_ = false;
    waiting_pose_verification_ = true;
    pose_verification_started_ = std::chrono::steady_clock::now();
    message_ = "MoveL succeeded; verifying reached flange pose";
    publishStatusLocked();
  }

  void onStopResult(const std_msgs::msg::Bool::SharedPtr result)
  {
    std::scoped_lock lock(mutex_);
    if (state_ != Status::STATE_STOPPING) {
      return;
    }
    command_in_flight_ = false;
    waiting_pose_verification_ = false;
    desired_plasma_enabled_ = false;
    if (result->data) {
      state_ = state_after_stop_;
      message_ = stop_reason_;
    } else {
      state_ = Status::STATE_ERROR;
      message_ = stop_reason_ + "; RealMan stop command failed";
    }
    publishStatusLocked();
  }

  void onWatchdog()
  {
    std::scoped_lock lock(mutex_);
    const auto now = std::chrono::steady_clock::now();
    requestCollisionStageLocked();
    if (state_ == Status::STATE_EXECUTING) {
      if (!poseFreshLocked()) {
        beginStopLocked("flange pose stream became stale during execution", Status::STATE_ERROR);
        return;
      }
      if (!collisionStageReadyLocked()) {
        beginStopLocked("RealMan collision stage 8 verification became stale", Status::STATE_ERROR);
        return;
      }
      if (current_monitor_enabled_ && !currentFreshLocked()) {
        beginStopLocked("joint-current stream became stale during execution", Status::STATE_ERROR);
        return;
      }
      if (command_in_flight_) {
        const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
          now - command_started_).count();
        if (elapsed > command_timeout_ms_) {
          beginStopLocked("MoveL result timeout", Status::STATE_ERROR);
        }
        return;
      }
      if (waiting_pose_verification_) {
        const PoseError error = poseError(latest_flange_pose_, command_target_pose_);
        if (flange_pose_sequence_ > pose_sequence_at_command_ &&
            error.translation_m <= goal_translation_tolerance_m_ &&
            error.rotation_rad <= goal_rotation_tolerance_rad_) {
          waiting_pose_verification_ = false;
          if (approach_phase_ != ApproachPhase::NONE) {
            advanceLinearApproachLocked();
          } else {
            current_index_ = command_index_;
            message_ = "reached and verified path point " + std::to_string(current_index_);
            publishStatusLocked();
            advanceLocked();
          }
          return;
        }
        const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
          now - pose_verification_started_).count();
        if (elapsed > pose_settle_timeout_ms_) {
          beginStopLocked("MoveL result succeeded but reached pose was not verified",
            Status::STATE_ERROR);
        }
      }
    } else if (state_ == Status::STATE_STOPPING) {
      const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
        now - stop_started_).count();
      if (elapsed > stop_timeout_ms_) {
        state_ = Status::STATE_ERROR;
        command_in_flight_ = false;
        waiting_pose_verification_ = false;
        desired_plasma_enabled_ = false;
        message_ = stop_reason_ + "; stop result timeout";
        publishStatusLocked();
      }
    }
  }

  void advanceLocked()
  {
    constexpr double duplicate_translation_m = 1e-6;
    constexpr double duplicate_rotation_rad = 1e-6;
    while (state_ == Status::STATE_EXECUTING) {
      const auto next_index = current_index_ + 1;
      if (next_index >= static_cast<std::int32_t>(path_.points.size())) {
        state_ = Status::STATE_COMPLETED;
        desired_plasma_enabled_ = false;
        message_ = "after-entry spray path dry-run completed";
        publishStatusLocked();
        return;
      }

      const auto & current = path_.points[static_cast<std::size_t>(current_index_)];
      const auto & next = path_.points[static_cast<std::size_t>(next_index)];
      desired_plasma_enabled_ = next.plasma_enabled;
      const PoseError step = poseError(current.flange_pose, next.flange_pose);
      if (step.translation_m <= duplicate_translation_m &&
          step.rotation_rad <= duplicate_rotation_rad) {
        current_index_ = next_index;
        message_ = "applied spray phase transition at stationary path point " +
          std::to_string(current_index_);
        publishStatusLocked();
        continue;
      }

      sendMoveLocked(next_index, "sent MoveL for path point " + std::to_string(next_index));
      return;
    }
  }

  void sendMoveLocked(std::int32_t index, const std::string & message)
  {
    const auto & target = path_.points[static_cast<std::size_t>(index)];
    sendPoseLocked(target.flange_pose, index, message);
  }

  void sendPoseLocked(
    const geometry_msgs::msg::Pose & target, std::int32_t index,
    const std::string & message)
  {
    if (current_monitor_enabled_) {
      if (!currentFreshLocked()) {
        beginStopLocked("cannot send MoveL without fresh joint-current baseline", Status::STATE_ERROR);
        return;
      }
      command_current_baseline_ = latest_joint_currents_;
    }
    rm_ros_interfaces::msg::Movel command;
    command.pose = target;
    command.speed = static_cast<std::uint8_t>(speed_percent_);
    command.trajectory_connect = 0;
    command.block = true;
    command_index_ = index;
    command_target_pose_ = target;
    command_started_ = std::chrono::steady_clock::now();
    pose_sequence_at_command_ = flange_pose_sequence_;
    command_in_flight_ = true;
    waiting_pose_verification_ = false;
    message_ = message;
    publishStatusLocked();
    movel_publisher_->publish(command);
  }

  geometry_msgs::msg::Pose interpolatedApproachPoseLocked(double ratio) const
  {
    geometry_msgs::msg::Pose pose;
    pose.position.x = approach_start_pose_.position.x +
      ratio * (approach_goal_pose_.position.x - approach_start_pose_.position.x);
    pose.position.y = approach_start_pose_.position.y +
      ratio * (approach_goal_pose_.position.y - approach_start_pose_.position.y);
    pose.position.z = approach_start_pose_.position.z +
      ratio * (approach_goal_pose_.position.z - approach_start_pose_.position.z);
    Eigen::Quaterniond start(
      approach_start_pose_.orientation.w, approach_start_pose_.orientation.x,
      approach_start_pose_.orientation.y, approach_start_pose_.orientation.z);
    Eigen::Quaterniond goal(
      approach_goal_pose_.orientation.w, approach_goal_pose_.orientation.x,
      approach_goal_pose_.orientation.y, approach_goal_pose_.orientation.z);
    const Eigen::Quaterniond rotation = start.normalized().slerp(ratio, goal.normalized());
    pose.orientation.x = rotation.x();
    pose.orientation.y = rotation.y();
    pose.orientation.z = rotation.z();
    pose.orientation.w = rotation.w();
    return pose;
  }

  void startLinearApproachLocked(
    const geometry_msgs::msg::Pose & goal, ApproachPhase phase,
    const std::string & message)
  {
    approach_start_pose_ = latest_flange_pose_;
    approach_goal_pose_ = goal;
    approach_phase_ = phase;
    const PoseError error = poseError(approach_start_pose_, approach_goal_pose_);
    const int translation_steps = static_cast<int>(
      std::ceil(error.translation_m / linear_entry_step_m_));
    const int rotation_steps = static_cast<int>(
      std::ceil(error.rotation_rad / limits_.max_segment_rotation_rad));
    approach_total_steps_ = std::max(1, std::max(translation_steps, rotation_steps));
    approach_next_step_ = 1;
    message_ = message + "; steps=" + std::to_string(approach_total_steps_);
    publishStatusLocked();
    const double ratio = static_cast<double>(approach_next_step_) / approach_total_steps_;
    sendPoseLocked(interpolatedApproachPoseLocked(ratio), -1,
      "sent segmented cavity-entry MoveL step " +
      std::to_string(approach_next_step_) + "/" +
      std::to_string(approach_total_steps_));
  }

  void advanceLinearApproachLocked()
  {
    if (approach_next_step_ < approach_total_steps_) {
      ++approach_next_step_;
      const double ratio = static_cast<double>(approach_next_step_) / approach_total_steps_;
      sendPoseLocked(interpolatedApproachPoseLocked(ratio), -1,
        "sent segmented cavity-entry MoveL step " +
        std::to_string(approach_next_step_) + "/" +
        std::to_string(approach_total_steps_));
      return;
    }

    if (approach_phase_ == ApproachPhase::TO_PRE_ENTRY) {
      startLinearApproachLocked(
        path_.cavity_entry_flange_pose, ApproachPhase::TO_ENTRY,
        "pre-entry reached; starting segmented move to cavity entry");
      return;
    }

    if (approach_phase_ == ApproachPhase::TO_ENTRY) {
      startLinearApproachLocked(
        path_.points.front().flange_pose, ApproachPhase::TO_FIRST_POINT,
        "cavity entry reached; starting segmented move to first spray layer");
      return;
    }

    approach_phase_ = ApproachPhase::NONE;
    current_index_ = 0;
    desired_plasma_enabled_ = path_.points.front().plasma_enabled;
    message_ = "first spray layer reached and verified";
    publishStatusLocked();
    advanceLocked();
  }

  void beginStopLocked(const std::string & reason, std::uint8_t state_after_stop)
  {
    desired_plasma_enabled_ = false;
    command_in_flight_ = false;
    waiting_pose_verification_ = false;
    stop_reason_ = reason;
    state_after_stop_ = state_after_stop;
    if (stop_publisher_->get_subscription_count() == 0) {
      state_ = Status::STATE_ERROR;
      message_ = reason + "; stop command topic is unavailable";
      publishStatusLocked();
      return;
    }
    state_ = Status::STATE_STOPPING;
    stop_started_ = std::chrono::steady_clock::now();
    message_ = reason + "; waiting for RealMan stop result";
    publishStatusLocked();
    stop_publisher_->publish(std_msgs::msg::Empty{});
  }

  void publishStatusLocked()
  {
    Status status;
    status.header.stamp = now();
    status.header.frame_id = "baselink";
    status.state = state_;
    status.path_id = path_available_ ? path_.path_id : "";
    status.current_index = current_index_;
    status.total_points = path_available_ ? static_cast<std::int32_t>(path_.points.size()) : 0;
    status.motion_enabled = motion_enabled_;
    status.dry_run = dry_run_;
    status.path_execution_permitted = path_available_ && path_.execution_permitted;
    status.driver_ready = driverReadyLocked();
    status.entry_pose_valid = entryPoseValidLocked();
    status.command_in_flight = command_in_flight_ || waiting_pose_verification_;
    status.desired_plasma_enabled = desired_plasma_enabled_;
    status.plasma_output_enabled = false;
    status.message = message_;
    if (path_available_ && current_index_ >= 0 &&
        current_index_ < static_cast<std::int32_t>(path_.points.size())) {
      const SprayPoint & point = path_.points[static_cast<std::size_t>(current_index_)];
      status.layer_index = point.layer_index;
      status.motion_phase = point.motion_phase;
    } else {
      status.layer_index = -1;
      status.motion_phase = SprayPoint::PHASE_LAYER_TRANSITION;
    }
    status_publisher_->publish(status);
  }

  std::string path_topic_;
  std::string flange_pose_topic_;
  bool motion_enabled_ = false;
  int speed_percent_ = 5;
  bool entry_approach_enabled_ = false;
  bool collision_clearance_required_ = true;
  double collision_clearance_max_age_sec_ = 120.0;
  int required_collision_stage_ = 8;
  bool current_monitor_enabled_ = true;
  int current_max_age_ms_ = 500;
  double current_delta_threshold_ma_ = 1500.0;
  int max_pose_age_ms_ = 500;
  int command_timeout_ms_ = 90000;
  int pose_settle_timeout_ms_ = 2500;
  int stop_timeout_ms_ = 3000;
  double entry_translation_tolerance_m_ = 0.003;
  double entry_rotation_tolerance_rad_ = 0.0872664626;
  double goal_translation_tolerance_m_ = 0.002;
  double goal_rotation_tolerance_rad_ = 0.0523598776;
  double linear_entry_step_m_ = 0.005;
  double manual_correction_max_translation_m_ = 0.030;
  double manual_correction_max_axis_offset_m_ = 0.010;
  double manual_correction_max_rotation_rad_ = 0.17453292519943295;
  EntryApproachLimits entry_approach_limits_;
  ValidationLimits limits_;

  rclcpp::Subscription<SprayPath>::SharedPtr path_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr flange_pose_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr movel_result_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr stop_result_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr clearance_subscription_;
  rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr collision_stage_subscription_;
  rclcpp::Subscription<rm_ros_interfaces::msg::Jointcurrent>::SharedPtr
    joint_current_subscription_;
  rclcpp::Publisher<rm_ros_interfaces::msg::Movel>::SharedPtr movel_publisher_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr stop_publisher_;
  rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr collision_stage_command_publisher_;
  rclcpp::Publisher<Status>::SharedPtr status_publisher_;
  rclcpp::Service<plasma_robot_interfaces::srv::StartSprayPath>::SharedPtr start_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_service_;
  rclcpp::TimerBase::SharedPtr watchdog_;

  mutable std::mutex mutex_;
  SprayPath path_;
  bool path_available_ = false;
  geometry_msgs::msg::Pose latest_flange_pose_;
  std::chrono::steady_clock::time_point flange_pose_time_;
  bool have_flange_pose_ = false;
  std::uint64_t flange_pose_sequence_ = 0;
  std::uint64_t pose_sequence_at_command_ = 0;
  std::vector<float> latest_joint_currents_;
  std::vector<float> command_current_baseline_;
  std::chrono::steady_clock::time_point joint_current_time_;
  bool have_joint_current_ = false;
  int collision_stage_ = -1;
  bool have_collision_stage_ = false;
  std::chrono::steady_clock::time_point collision_stage_time_;
  std::chrono::steady_clock::time_point last_collision_stage_request_;
  std::string collision_checked_path_id_;
  std::int64_t collision_checked_path_stamp_sec_ = 0;
  std::uint32_t collision_checked_path_stamp_nanosec_ = 0;
  std::int64_t collision_clearance_stamp_ns_ = 0;

  std::uint8_t state_ = Status::STATE_IDLE;
  std::uint8_t state_after_stop_ = Status::STATE_ERROR;
  std::int32_t current_index_ = -1;
  std::int32_t command_index_ = -1;
  ApproachPhase approach_phase_ = ApproachPhase::NONE;
  geometry_msgs::msg::Pose approach_start_pose_;
  geometry_msgs::msg::Pose approach_goal_pose_;
  geometry_msgs::msg::Pose command_target_pose_;
  int approach_total_steps_ = 0;
  int approach_next_step_ = 0;
  bool dry_run_ = true;
  bool command_in_flight_ = false;
  bool waiting_pose_verification_ = false;
  bool desired_plasma_enabled_ = false;
  std::chrono::steady_clock::time_point command_started_;
  std::chrono::steady_clock::time_point pose_verification_started_;
  std::chrono::steady_clock::time_point stop_started_;
  std::string stop_reason_;
  std::string message_;
};

}  // namespace plasma_path_executor

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<plasma_path_executor::PathExecutorNode>());
  rclcpp::shutdown();
  return 0;
}
