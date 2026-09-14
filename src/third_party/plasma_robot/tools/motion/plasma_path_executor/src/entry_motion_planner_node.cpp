#include "plasma_path_executor/path_validation.hpp"
#include "plasma_path_executor/retreat_path.hpp"
#include "plasma_path_executor/reviewed_trajectory.hpp"

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/msg/robot_state.hpp>
#include <plasma_robot_interfaces/msg/entry_motion_status.hpp>
#include <plasma_robot_interfaces/msg/spray_path.hpp>
#include <rclcpp/parameter_client.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rm_ros_interfaces/msg/armcurrentstatus.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int16.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <future>
#include <memory>
#include <mutex>
#include <limits>
#include <string>
#include <vector>

namespace plasma_path_executor
{

class EntryMotionPlanner
{
public:
  explicit EntryMotionPlanner(const rclcpp::Node::SharedPtr & node)
  : node_(node)
  {
    path_topic_ = node_->declare_parameter<std::string>(
      "path_topic", "/plasma/planned_spray_path/base");
    flange_pose_topic_ = node_->declare_parameter<std::string>(
      "flange_pose_topic", "/rm_driver/udp_arm_position");
    planning_group_ = node_->declare_parameter<std::string>("planning_group", "rm_group");
    flange_link_ = node_->declare_parameter<std::string>("flange_link", "Link6");
    base_frame_ = node_->declare_parameter<std::string>("base_frame", "baselink");
    controller_node_name_ = node_->declare_parameter<std::string>(
      "controller_node_name", "/rm_control");
    motion_enabled_ = node_->declare_parameter<bool>("motion_enabled", false);
    allow_unvalidated_dry_run_ = node_->declare_parameter<bool>(
      "allow_unvalidated_dry_run", false);
    planning_time_sec_ = node_->declare_parameter<double>("planning_time_sec", 10.0);
    planning_attempts_ = node_->declare_parameter<int>("planning_attempts", 10);
    complete_plan_attempts_ = node_->declare_parameter<int>("complete_plan_attempts", 8);
    velocity_scaling_ = node_->declare_parameter<double>("velocity_scaling", 0.05);
    acceleration_scaling_ = node_->declare_parameter<double>("acceleration_scaling", 0.05);
    approach_velocity_scaling_ = node_->declare_parameter<double>(
      "approach_velocity_scaling", 0.10);
    approach_acceleration_scaling_ = node_->declare_parameter<double>(
      "approach_acceleration_scaling", 0.10);
    max_plan_age_sec_ = node_->declare_parameter<double>("max_plan_age_sec", 120.0);
    max_pose_age_ms_ = node_->declare_parameter<int>("max_pose_age_ms", 500);
    max_arm_status_age_ms_ = node_->declare_parameter<int>("max_arm_status_age_ms", 500);
    max_joint_state_age_ms_ = node_->declare_parameter<int>("max_joint_state_age_ms", 500);
    joint_state_wait_sec_ = node_->declare_parameter<double>("joint_state_wait_sec", 5.0);
    planning_position_tolerance_m_ = node_->declare_parameter<double>(
      "planning_position_tolerance_m", 0.001);
    planning_rotation_tolerance_rad_ = node_->declare_parameter<double>(
      "planning_rotation_tolerance_rad", 0.017453292519943295);
    pose_tolerance_m_ = node_->declare_parameter<double>("pose_tolerance_m", 0.003);
    rotation_tolerance_rad_ = node_->declare_parameter<double>(
      "rotation_tolerance_rad", 0.08726646259971647);
    cartesian_step_m_ = node_->declare_parameter<double>("cartesian_step_m", 0.005);
    max_joint_step_rad_ = node_->declare_parameter<double>(
      "max_joint_step_rad", 0.17453292519943295);
    reviewed_start_joint_tolerance_rad_ = node_->declare_parameter<double>(
      "reviewed_start_joint_tolerance_rad", 0.03490658503988659);
    min_cartesian_fraction_ = node_->declare_parameter<double>(
      "min_cartesian_fraction", 0.99999);
    required_collision_stage_ = node_->declare_parameter<int>("required_collision_stage", 8);
    stop_settle_time_ms_ = node_->declare_parameter<int>("stop_settle_time_ms", 500);
    stop_joint_stability_time_ms_ = node_->declare_parameter<int>(
      "stop_joint_stability_time_ms", 1000);
    stop_joint_stability_tolerance_rad_ = node_->declare_parameter<double>(
      "stop_joint_stability_tolerance_rad", 0.0008726646259971648);
    stopped_retreat_max_axis_offset_m_ = node_->declare_parameter<double>(
      "stopped_retreat_max_axis_offset_m", 0.02);
    stopped_retreat_max_distance_m_ = node_->declare_parameter<double>(
      "stopped_retreat_max_distance_m", 0.50);
    if (planning_time_sec_ <= 0.0 || planning_attempts_ <= 0 ||
      complete_plan_attempts_ <= 0 || complete_plan_attempts_ > 20 ||
      velocity_scaling_ <= 0.0 || velocity_scaling_ > 1.0 ||
      acceleration_scaling_ <= 0.0 || acceleration_scaling_ > 1.0 ||
      approach_velocity_scaling_ <= 0.0 || approach_velocity_scaling_ > 1.0 ||
      approach_acceleration_scaling_ <= 0.0 || approach_acceleration_scaling_ > 1.0 ||
      max_plan_age_sec_ <= 0.0 || max_pose_age_ms_ <= 0 || max_arm_status_age_ms_ <= 0 ||
      max_joint_state_age_ms_ <= 0 || max_joint_state_age_ms_ > 5000 ||
      joint_state_wait_sec_ <= 0.0 || joint_state_wait_sec_ > 30.0 ||
      planning_position_tolerance_m_ <= 0.0 ||
      planning_position_tolerance_m_ > pose_tolerance_m_ ||
      planning_rotation_tolerance_rad_ <= 0.0 ||
      planning_rotation_tolerance_rad_ > rotation_tolerance_rad_ ||
      pose_tolerance_m_ <= 0.0 ||
      rotation_tolerance_rad_ <= 0.0 || cartesian_step_m_ <= 0.0 ||
      cartesian_step_m_ > 0.02 || max_joint_step_rad_ <= 0.0 ||
      max_joint_step_rad_ > 0.5235987755982988 ||
      reviewed_start_joint_tolerance_rad_ <= 0.0 ||
      reviewed_start_joint_tolerance_rad_ > 0.08726646259971647 ||
      min_cartesian_fraction_ <= 0.0 || min_cartesian_fraction_ > 1.0 ||
      required_collision_stage_ < 1 || required_collision_stage_ > 8 ||
      stop_settle_time_ms_ < 100 || stop_settle_time_ms_ > 5000 ||
      stop_joint_stability_time_ms_ < 500 || stop_joint_stability_time_ms_ > 5000 ||
      stop_joint_stability_tolerance_rad_ <= 0.0 ||
      stop_joint_stability_tolerance_rad_ > 0.008726646259971648 ||
      stopped_retreat_max_axis_offset_m_ <= 0.0 ||
      stopped_retreat_max_axis_offset_m_ > 0.10 ||
      stopped_retreat_max_distance_m_ <= 0.05 ||
      stopped_retreat_max_distance_m_ > 1.0) {
      throw std::runtime_error("entry planner parameters are invalid");
    }
    if (controller_node_name_.empty()) {
      throw std::runtime_error("controller_node_name must not be empty");
    }

    const auto latched_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    operation_callback_group_ = node_->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
    safety_callback_group_ = node_->create_callback_group(
      rclcpp::CallbackGroupType::Reentrant);
    controller_parameters_ = std::make_shared<rclcpp::AsyncParametersClient>(
      node_, controller_node_name_, rmw_qos_profile_parameters, safety_callback_group_);
    status_publisher_ = node_->create_publisher<Status>(
      "/plasma/entry_motion/status", latched_qos);
    display_publisher_ = node_->create_publisher<moveit_msgs::msg::DisplayTrajectory>(
      "/display_planned_path", latched_qos);
    clearance_publisher_ = node_->create_publisher<std_msgs::msg::String>(
      "/plasma/entry_motion/collision_checked_path", latched_qos);
    collision_stage_command_publisher_ = node_->create_publisher<std_msgs::msg::UInt16>(
      "/rm_driver/set_collision_stage_cmd", rclcpp::ParametersQoS());
    rclcpp::SubscriptionOptions safety_subscription_options;
    safety_subscription_options.callback_group = safety_callback_group_;
    collision_stage_subscription_ = node_->create_subscription<std_msgs::msg::UInt16>(
      "/rm_driver/collision_stage", rclcpp::ParametersQoS(),
      std::bind(&EntryMotionPlanner::onCollisionStage, this, std::placeholders::_1),
      safety_subscription_options);
    arm_status_subscription_ =
      node_->create_subscription<rm_ros_interfaces::msg::Armcurrentstatus>(
      "/rm_driver/udp_arm_current_status", rclcpp::QoS(10),
      std::bind(&EntryMotionPlanner::onArmStatus, this, std::placeholders::_1),
      safety_subscription_options);
    joint_state_subscription_ = node_->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", rclcpp::SensorDataQoS(),
      std::bind(&EntryMotionPlanner::onJointState, this, std::placeholders::_1),
      safety_subscription_options);
    path_subscription_ = node_->create_subscription<SprayPath>(
      path_topic_, latched_qos,
      std::bind(&EntryMotionPlanner::onPath, this, std::placeholders::_1));
    pose_callback_group_ = node_->create_callback_group(
      rclcpp::CallbackGroupType::Reentrant);
    rclcpp::SubscriptionOptions pose_subscription_options;
    pose_subscription_options.callback_group = pose_callback_group_;
    flange_pose_subscription_ = node_->create_subscription<geometry_msgs::msg::Pose>(
      flange_pose_topic_, rclcpp::QoS(10),
      std::bind(&EntryMotionPlanner::onFlangePose, this, std::placeholders::_1),
      pose_subscription_options);
    plan_service_ = node_->create_service<std_srvs::srv::Trigger>(
      "~/plan_to_pre_entry",
      std::bind(&EntryMotionPlanner::onPlan, this,
        std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, operation_callback_group_);
    execute_service_ = node_->create_service<std_srvs::srv::Trigger>(
      "~/execute_to_pre_entry",
      std::bind(&EntryMotionPlanner::onExecute, this,
        std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, operation_callback_group_);
    reviewed_dry_run_service_ = node_->create_service<std_srvs::srv::Trigger>(
      "~/execute_reviewed_dry_run",
      std::bind(&EntryMotionPlanner::onExecuteReviewedDryRun, this,
        std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, operation_callback_group_);
    reviewed_entry_service_ = node_->create_service<std_srvs::srv::Trigger>(
      "~/execute_reviewed_entry",
      std::bind(&EntryMotionPlanner::onExecuteReviewedEntry, this,
        std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, operation_callback_group_);
    reviewed_first_layer_service_ = node_->create_service<std_srvs::srv::Trigger>(
      "~/execute_reviewed_first_layer",
      std::bind(&EntryMotionPlanner::onExecuteReviewedFirstLayer, this,
        std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, operation_callback_group_);
    retreat_service_ = node_->create_service<std_srvs::srv::Trigger>(
      "~/retreat_to_pre_entry",
      std::bind(&EntryMotionPlanner::onRetreatToPreEntry, this,
        std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, operation_callback_group_);
    stop_service_ = node_->create_service<std_srvs::srv::Trigger>(
      "~/stop",
      std::bind(&EntryMotionPlanner::onStop, this,
        std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, safety_callback_group_);
    status_timer_ = node_->create_wall_timer(
      std::chrono::milliseconds(500), [this]() {
        std::scoped_lock lock(mutex_);
        requestCollisionStageLocked();
        updateStoppedRetreatStateLocked();
        publishStatusLocked();
      }, safety_callback_group_);

    move_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(
      node_, planning_group_, std::shared_ptr<tf2_ros::Buffer>(),
      rclcpp::Duration::from_seconds(10.0));
    move_group_->setPoseReferenceFrame(base_frame_);
    if (!move_group_->setEndEffectorLink(flange_link_)) {
      throw std::runtime_error("MoveIt planning group does not contain Link6");
    }
    move_group_->setPlanningTime(planning_time_sec_);
    move_group_->setNumPlanningAttempts(planning_attempts_);
    move_group_->setMaxVelocityScalingFactor(velocity_scaling_);
    move_group_->setMaxAccelerationScalingFactor(acceleration_scaling_);
    if (allow_unvalidated_dry_run_) {
      RCLCPP_WARN(
        node_->get_logger(),
        "Unvalidated calibration commissioning override is enabled for robot motion; "
        "this node still provides dry-run motion only and never enables plasma output");
    }
    // The planner target must be tighter than the post-motion acceptance window.
    // Otherwise a legal plan at the edge of the goal tolerance can fail verification
    // after normal controller tracking error is added.
    move_group_->setGoalPositionTolerance(planning_position_tolerance_m_);
    move_group_->setGoalOrientationTolerance(planning_rotation_tolerance_rad_);
    message_ = "waiting for transformed path with cavity entry geometry";
    publishStatusLocked();
  }

private:
  using SprayPath = plasma_robot_interfaces::msg::SprayPath;
  using Status = plasma_robot_interfaces::msg::EntryMotionStatus;
  using Plan = moveit::planning_interface::MoveGroupInterface::Plan;

  struct CompletePlanAttempt
  {
    Plan approach;
    moveit_msgs::msg::RobotTrajectory entry;
    moveit_msgs::msg::RobotTrajectory first_layer;
    moveit_msgs::msg::RobotTrajectory remaining_spray;
    moveit_msgs::msg::RobotTrajectory spray;
    moveit_msgs::msg::RobotTrajectory after_entry;
    moveit_msgs::msg::RobotTrajectory first_layer_retreat;
    moveit_msgs::msg::RobotTrajectory retreat;
    std::size_t first_layer_point_count = 0;
    std::size_t first_layer_retreat_waypoint_count = 0;
    std::size_t retreat_waypoint_count = 0;
    double approach_joint_travel = std::numeric_limits<double>::infinity();
    double max_approach_joint_delta = 0.0;
    std::string max_approach_joint_name;
    double max_joint_delta = 0.0;
    std::string max_joint_name;
    double pre_entry_ik_seed_joint6 = 0.0;
    double pre_entry_ik_joint6 = 0.0;
    std::string error;
  };

  bool updateStartStateFromTrajectory(
    const moveit_msgs::msg::RobotTrajectory & trajectory,
    moveit_msgs::msg::RobotState * state, std::string * reason) const
  {
    const auto & joints = trajectory.joint_trajectory;
    if (!state || joints.joint_names.empty() || joints.points.empty() ||
      joints.points.back().positions.size() != joints.joint_names.size())
    {
      *reason = "trajectory has no complete final joint state";
      return false;
    }
    const auto & final_positions = joints.points.back().positions;
    for (std::size_t index = 0; index < joints.joint_names.size(); ++index) {
      const auto found = std::find(
        state->joint_state.name.begin(), state->joint_state.name.end(),
        joints.joint_names[index]);
      if (found == state->joint_state.name.end()) {
        state->joint_state.name.push_back(joints.joint_names[index]);
        state->joint_state.position.push_back(final_positions[index]);
      } else {
        const auto state_index = static_cast<std::size_t>(
          std::distance(state->joint_state.name.begin(), found));
        if (state->joint_state.position.size() <= state_index) {
          state->joint_state.position.resize(state_index + 1);
        }
        state->joint_state.position[state_index] = final_positions[index];
      }
    }
    state->is_diff = false;
    return true;
  }

  void clearClearanceLocked()
  {
    std_msgs::msg::String message;
    clearance_publisher_->publish(message);
  }

  void publishClearanceLocked(const SprayPath & path)
  {
    std_msgs::msg::String message;
    message.data = path.path_id + "|" + std::to_string(path.header.stamp.sec) + "|" +
      std::to_string(path.header.stamp.nanosec) + "|" +
      std::to_string(node_->now().nanoseconds());
    clearance_publisher_->publish(message);
  }

  bool setControllerSpeed(double scaling, std::string * reason)
  {
    const int speed_percent = std::clamp(
      static_cast<int>(std::lround(scaling * 100.0)), 1, 100);
    if (!controller_parameters_->wait_for_service(std::chrono::seconds(2))) {
      *reason = "rm_control parameter service is unavailable";
      return false;
    }
    auto future = controller_parameters_->set_parameters({
      rclcpp::Parameter("final_settle_speed_percent", speed_percent)});
    if (future.wait_for(std::chrono::seconds(2)) != std::future_status::ready) {
      *reason = "timed out while setting rm_control speed";
      return false;
    }
    const auto results = future.get();
    if (results.size() != 1 || !results.front().successful) {
      *reason = results.empty() ? "rm_control returned no parameter result" :
        "rm_control rejected speed: " + results.front().reason;
      return false;
    }
    RCLCPP_INFO(
      node_->get_logger(), "Verified rm_control speed for next trajectory: %d%%",
      speed_percent);
    return true;
  }

  void onCollisionStage(const std_msgs::msg::UInt16::SharedPtr stage)
  {
    std::scoped_lock lock(mutex_);
    collision_stage_ = static_cast<int>(stage->data);
    collision_stage_time_ = std::chrono::steady_clock::now();
    have_collision_stage_ = true;
  }

  void onArmStatus(const rm_ros_interfaces::msg::Armcurrentstatus::SharedPtr status)
  {
    std::scoped_lock lock(mutex_);
    arm_status_ = static_cast<int>(status->arm_current_status);
    arm_status_time_ = std::chrono::steady_clock::now();
    have_arm_status_ = true;
  }

  void onJointState(const sensor_msgs::msg::JointState::SharedPtr state)
  {
    if (!state || state->name.empty() || state->position.size() < state->name.size() ||
      !std::all_of(state->position.begin(), state->position.end(),
        [](double position) {return std::isfinite(position);}))
    {
      return;
    }
    {
      std::scoped_lock lock(mutex_);
      const auto now = std::chrono::steady_clock::now();
      latest_joint_state_ = *state;
      latest_joint_state_time_ = now;
      have_joint_state_ = true;
      if (stop_joint_stability_active_) {
        double observed_max_delta = 0.0;
        const bool remains_stable = have_stop_joint_anchor_ &&
          namedJointPositionsWithinTolerance(
          stop_joint_anchor_.name, stop_joint_anchor_.position,
          state->name, state->position, stop_joint_stability_tolerance_rad_,
          &observed_max_delta);
        if (!remains_stable) {
          stop_joint_anchor_ = *state;
          stop_joint_stable_since_ = now;
          have_stop_joint_anchor_ = true;
        }
      }
    }
    joint_state_condition_.notify_all();
  }

  bool jointStateFreshLocked() const
  {
    if (!have_joint_state_) {
      return false;
    }
    const auto age_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now() - latest_joint_state_time_).count();
    return age_ms <= max_joint_state_age_ms_;
  }

  moveit::core::RobotStatePtr waitForCurrentRobotState(
    double wait_sec, std::string * reason)
  {
    sensor_msgs::msg::JointState joint_state;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      if (!joint_state_condition_.wait_for(
          lock, std::chrono::duration<double>(wait_sec),
          [this]() {return jointStateFreshLocked();}))
      {
        *reason = "fresh /joint_states data did not arrive within " +
          std::to_string(wait_sec) + " seconds; publisher_count=" +
          std::to_string(node_->count_publishers("/joint_states"));
        return {};
      }
      joint_state = latest_joint_state_;
    }

    const auto model = move_group_->getRobotModel();
    const auto * group = model ? model->getJointModelGroup(planning_group_) : nullptr;
    if (!model || !group) {
      *reason = "MoveIt robot model or planning group is unavailable";
      return {};
    }
    auto state = std::make_shared<moveit::core::RobotState>(model);
    state->setToDefaultValues();
    for (const auto & variable : group->getVariableNames()) {
      const auto found = std::find(joint_state.name.begin(), joint_state.name.end(), variable);
      if (found == joint_state.name.end()) {
        *reason = "latest /joint_states is missing planning variable " + variable;
        return {};
      }
      const auto index = static_cast<std::size_t>(
        std::distance(joint_state.name.begin(), found));
      if (index >= joint_state.position.size() ||
        !std::isfinite(joint_state.position[index]))
      {
        *reason = "latest /joint_states contains an invalid value for " + variable;
        return {};
      }
      state->setVariablePosition(variable, joint_state.position[index]);
    }
    state->update();
    if (!state->satisfiesBounds(group)) {
      *reason = "latest /joint_states is outside the configured MoveIt joint bounds";
      return {};
    }
    return state;
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
    const auto now = std::chrono::steady_clock::now();
    if (last_collision_stage_request_.time_since_epoch().count() != 0 &&
      now - last_collision_stage_request_ < std::chrono::seconds(5)) {
      return;
    }
    std_msgs::msg::UInt16 command;
    command.data = static_cast<std::uint16_t>(required_collision_stage_);
    collision_stage_command_publisher_->publish(command);
    last_collision_stage_request_ = now;
  }

  bool collisionStageReadyLocked() const
  {
    return collision_stage_subscription_->get_publisher_count() > 0 &&
           have_collision_stage_ && collision_stage_ == required_collision_stage_;
  }

  bool armIdleLocked() const
  {
    if (arm_status_subscription_->get_publisher_count() == 0 || !have_arm_status_) {
      return false;
    }
    const auto age_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now() - arm_status_time_).count();
    return age_ms <= max_arm_status_age_ms_ && arm_status_ == 0;
  }

  bool armStatusFreshLocked() const
  {
    if (arm_status_subscription_->get_publisher_count() == 0 || !have_arm_status_) {
      return false;
    }
    const auto age_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now() - arm_status_time_).count();
    return age_ms <= max_arm_status_age_ms_;
  }

  void beginStopJointStabilityCheckLocked()
  {
    stop_joint_stability_active_ = true;
    have_stop_joint_anchor_ = have_joint_state_;
    stop_joint_stable_since_ = std::chrono::steady_clock::now();
    if (have_stop_joint_anchor_) {
      stop_joint_anchor_ = latest_joint_state_;
    }
  }

  void clearStopJointStabilityCheckLocked()
  {
    stop_joint_stability_active_ = false;
    have_stop_joint_anchor_ = false;
    stop_joint_anchor_ = sensor_msgs::msg::JointState{};
  }

  bool jointsStableAfterStopLocked() const
  {
    if (!stop_joint_stability_active_ || !have_stop_joint_anchor_ ||
      !jointStateFreshLocked())
    {
      return false;
    }
    const auto stable_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now() - stop_joint_stable_since_).count();
    return stable_ms >= stop_joint_stability_time_ms_;
  }

  bool stoppedRobotIdleLocked() const
  {
    // A fresh controller status is authoritative. Some controller/driver
    // combinations stop refreshing this optional field after move_stop; only
    // then use a continuous named-joint stability window as the fallback.
    return armStatusFreshLocked() ? arm_status_ == 0 : jointsStableAfterStopLocked();
  }

  void updateStoppedRetreatStateLocked()
  {
    if (state_ != Status::STATE_STOPPING_IN_CAVITY ||
      !cavity_retreat_context_active_)
    {
      return;
    }
    const auto settled_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now() - stop_requested_time_).count();
    if (settled_ms < stop_settle_time_ms_ || !stoppedRobotIdleLocked() ||
      !poseFreshLocked() || !jointStateFreshLocked())
    {
      return;
    }
    state_ = Status::STATE_STOPPED_IN_CAVITY;
    message_ = "robot stop verified inside the cavity using fresh controller-idle or "
      "continuous joint stability; a new straight outward retreat can now be "
      "collision-checked from the actual stopped pose";
  }

  bool jointStepsWithinLimit(
    const moveit_msgs::msg::RobotState & start,
    moveit_msgs::msg::RobotTrajectory & trajectory,
    double * max_delta,
    std::string * max_joint,
    std::size_t * max_point,
    double * max_from,
    double * max_to) const
  {
    const auto & names = trajectory.joint_trajectory.joint_names;
    auto & points = trajectory.joint_trajectory.points;
    if (names.empty() || points.empty()) {
      return false;
    }

    std::vector<double> previous(names.size(), 0.0);
    for (std::size_t joint_index = 0; joint_index < names.size(); ++joint_index) {
      const auto start_it = std::find(
        start.joint_state.name.begin(), start.joint_state.name.end(), names[joint_index]);
      if (start_it == start.joint_state.name.end()) {
        return false;
      }
      const auto start_index = static_cast<std::size_t>(
        std::distance(start.joint_state.name.begin(), start_it));
      if (start_index >= start.joint_state.position.size()) {
        return false;
      }
      previous[joint_index] = start.joint_state.position[start_index];
    }

    double largest_delta = 0.0;
    std::string largest_joint;
    std::size_t largest_point = 0;
    double largest_from = 0.0;
    double largest_to = 0.0;
    for (std::size_t point_index = 0; point_index < points.size(); ++point_index) {
      auto & positions = points[point_index].positions;
      if (positions.size() != names.size()) {
        return false;
      }
      for (std::size_t joint_index = 0; joint_index < names.size(); ++joint_index) {
        const auto * joint_model = move_group_->getRobotModel()->getJointOfVariable(
          names[joint_index]);
        if (!joint_model) {
          return false;
        }
        if (joint_model->getType() == moveit::core::JointModel::REVOLUTE) {
          const auto & bounds = move_group_->getRobotModel()->getVariableBounds(
            names[joint_index]);
          if (bounds.position_bounded_) {
            double continuous_position = 0.0;
            if (!nearestEquivalentRevolutePosition(
                positions[joint_index], previous[joint_index],
                bounds.min_position_, bounds.max_position_, &continuous_position)) {
              return false;
            }
            positions[joint_index] = continuous_position;
          }
        }
        const double delta = std::abs(positions[joint_index] - previous[joint_index]);
        if (!std::isfinite(delta)) {
          return false;
        }
        if (delta > largest_delta) {
          largest_delta = delta;
          largest_joint = names[joint_index];
          largest_point = point_index;
          largest_from = previous[joint_index];
          largest_to = positions[joint_index];
        }
        previous[joint_index] = positions[joint_index];
      }
    }

    if (max_delta) {
      *max_delta = largest_delta;
    }
    if (max_joint) {
      *max_joint = largest_joint;
    }
    if (max_point) {
      *max_point = largest_point;
    }
    if (max_from) {
      *max_from = largest_from;
    }
    if (max_to) {
      *max_to = largest_to;
    }
    return largest_delta <= max_joint_step_rad_;
  }

  bool currentJointsMatchReviewedStart(
    const moveit_msgs::msg::RobotTrajectory & trajectory, std::string * reason)
  {
    const auto & joint_trajectory = trajectory.joint_trajectory;
    if (joint_trajectory.joint_names.empty() || joint_trajectory.points.empty() ||
      joint_trajectory.points.front().positions.size() != joint_trajectory.joint_names.size()) {
      *reason = "reviewed after-entry trajectory has no complete joint start state";
      return false;
    }
    const auto current_state = waitForCurrentRobotState(
      std::min(1.0, joint_state_wait_sec_), reason);
    if (!current_state) {
      return false;
    }

    double max_delta = 0.0;
    std::string max_joint;
    for (std::size_t index = 0; index < joint_trajectory.joint_names.size(); ++index) {
      const auto & joint_name = joint_trajectory.joint_names[index];
      const double current = current_state->getVariablePosition(joint_name);
      const double planned = joint_trajectory.points.front().positions[index];
      const double delta = std::abs(current - planned);
      if (!std::isfinite(current) || !std::isfinite(planned) || !std::isfinite(delta)) {
        *reason = "current or reviewed joint start state is not finite";
        return false;
      }
      if (delta > max_delta) {
        max_delta = delta;
        max_joint = joint_name;
      }
    }
    if (max_delta > reviewed_start_joint_tolerance_rad_) {
      *reason = "current joints do not match the reviewed trajectory start; joint=" +
        max_joint + ", delta_deg=" + std::to_string(max_delta * 180.0 / M_PI) +
        ", limit_deg=" +
        std::to_string(reviewed_start_joint_tolerance_rad_ * 180.0 / M_PI) +
        "; replan from the current robot state";
      return false;
    }
    return true;
  }

  double trajectoryJointTravel(
    const moveit_msgs::msg::RobotState & start,
    const moveit_msgs::msg::RobotTrajectory & trajectory) const
  {
    const auto & names = trajectory.joint_trajectory.joint_names;
    const auto & points = trajectory.joint_trajectory.points;
    if (names.empty() || points.empty()) {
      return std::numeric_limits<double>::infinity();
    }

    std::vector<double> previous(names.size(), 0.0);
    for (std::size_t joint_index = 0; joint_index < names.size(); ++joint_index) {
      const auto start_it = std::find(
        start.joint_state.name.begin(), start.joint_state.name.end(), names[joint_index]);
      if (start_it == start.joint_state.name.end()) {
        return std::numeric_limits<double>::infinity();
      }
      const auto start_index = static_cast<std::size_t>(
        std::distance(start.joint_state.name.begin(), start_it));
      if (start_index >= start.joint_state.position.size()) {
        return std::numeric_limits<double>::infinity();
      }
      previous[joint_index] = start.joint_state.position[start_index];
    }

    double travel = 0.0;
    for (const auto & point : points) {
      if (point.positions.size() != names.size()) {
        return std::numeric_limits<double>::infinity();
      }
      for (std::size_t joint_index = 0; joint_index < names.size(); ++joint_index) {
        const double delta = std::abs(point.positions[joint_index] - previous[joint_index]);
        if (!std::isfinite(delta)) {
          return std::numeric_limits<double>::infinity();
        }
        travel += delta;
        previous[joint_index] = point.positions[joint_index];
      }
    }
    return travel;
  }

  void onPath(const SprayPath::SharedPtr path)
  {
    std::scoped_lock lock(mutex_);
    if (executing_ || cavity_retreat_context_active_) {
      RCLCPP_WARN(
        node_->get_logger(),
        "Ignored path replacement while the robot is moving or may still be inside the cavity");
      return;
    }
    plan_available_ = false;
    retreat_available_ = false;
    first_layer_point_count_ = 0;
    first_layer_preview_ = moveit_msgs::msg::RobotTrajectory{};
    remaining_spray_preview_ = moveit_msgs::msg::RobotTrajectory{};
    reviewed_first_layer_retreat_ = moveit_msgs::msg::RobotTrajectory{};
    planned_path_id_.clear();
    clearClearanceLocked();
    if (path->header.frame_id != base_frame_ || path->base_frame != base_frame_ ||
      path->gripper_frame != flange_link_ || !path->transform_valid ||
      !path->cavity_entry_tcp_pose_valid || !path->pre_entry_tcp_pose_valid ||
      !path->cavity_axis_valid || !path->cavity_entry_flange_pose_valid ||
      !path->pre_entry_flange_pose_valid ||
      !poseIsFiniteAndNormalized(path->pre_entry_flange_pose)) {
      path_available_ = false;
      state_ = Status::STATE_ERROR;
      message_ = "rejected path: complete transformed entry/pre-entry geometry is required";
      publishStatusLocked();
      return;
    }
    path_ = *path;
    path_available_ = true;
    cavity_retreat_context_active_ = false;
    clearStopJointStabilityCheckLocked();
    state_ = Status::STATE_READY_TO_PLAN;
    message_ = path_.execution_permitted ?
      "entry target ready; request MoveIt planning" :
      "entry target ready for preview; calibration still locks execution";
    publishStatusLocked();
  }

  void onFlangePose(const geometry_msgs::msg::Pose::SharedPtr pose)
  {
    if (!poseIsFiniteAndNormalized(*pose)) {
      return;
    }
    std::scoped_lock lock(mutex_);
    latest_flange_pose_ = *pose;
    latest_flange_pose_time_ = std::chrono::steady_clock::now();
    have_flange_pose_ = true;
  }

  bool poseFreshLocked() const
  {
    if (!have_flange_pose_) {
      return false;
    }
    const auto age_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now() - latest_flange_pose_time_).count();
    return age_ms <= max_pose_age_ms_;
  }

  bool reviewWindowExpiredLocked(std::string * reason)
  {
    const double elapsed_sec = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - plan_created_).count();
    if (elapsed_sec <= max_plan_age_sec_) {
      return false;
    }
    plan_available_ = false;
    if (reason) {
      *reason = "reviewed trajectory approval window expired; elapsed_sec=" +
        std::to_string(elapsed_sec) + ", limit_sec=" +
        std::to_string(max_plan_age_sec_) + "; plan again from current joints";
    }
    state_ = Status::STATE_ERROR;
    message_ = reason ? *reason : "reviewed trajectory approval window expired";
    publishStatusLocked();
    return true;
  }

  bool timeParameterizeCartesianTrajectory(
    const std::string & phase,
    const moveit_msgs::msg::RobotState & start,
    double velocity_scaling,
    double acceleration_scaling,
    moveit_msgs::msg::RobotTrajectory * trajectory,
    std::string * error) const
  {
    if (!trajectory || trajectory->joint_trajectory.joint_names.empty() ||
      trajectory->joint_trajectory.points.size() < 2)
    {
      *error = phase + " trajectory is incomplete before time parameterization";
      return false;
    }

    auto & points = trajectory->joint_trajectory.points;
    const auto same_position = [](const auto & left, const auto & right) {
        if (left.positions.size() != right.positions.size()) {
          return false;
        }
        constexpr double kDuplicateToleranceRad = 1e-10;
        for (std::size_t index = 0; index < left.positions.size(); ++index) {
          if (!std::isfinite(left.positions[index]) ||
            !std::isfinite(right.positions[index]) ||
            std::abs(left.positions[index] - right.positions[index]) >
            kDuplicateToleranceRad)
          {
            return false;
          }
        }
        return true;
      };
    points.erase(std::unique(points.begin(), points.end(), same_position), points.end());
    if (points.size() < 2) {
      *error = phase + " trajectory has fewer than two distinct joint points";
      return false;
    }

    moveit::core::RobotState reference_state(move_group_->getRobotModel());
    reference_state.setToDefaultValues();
    if (!moveit::core::robotStateMsgToRobotState(start, reference_state, true)) {
      *error = phase + " start state cannot be converted for time parameterization";
      return false;
    }
    robot_trajectory::RobotTrajectory timed_trajectory(
      move_group_->getRobotModel(), planning_group_);
    timed_trajectory.setRobotTrajectoryMsg(reference_state, *trajectory);
    trajectory_processing::IterativeParabolicTimeParameterization time_parameterization;
    if (!time_parameterization.computeTimeStamps(
        timed_trajectory, velocity_scaling, acceleration_scaling))
    {
      *error = phase + " MoveIt time parameterization failed";
      return false;
    }
    timed_trajectory.getRobotTrajectoryMsg(*trajectory);

    std::int64_t previous_ns = -1;
    for (std::size_t index = 0;
      index < trajectory->joint_trajectory.points.size(); ++index)
    {
      const auto & duration = trajectory->joint_trajectory.points[index].time_from_start;
      const std::int64_t current_ns =
        static_cast<std::int64_t>(duration.sec) * 1000000000LL +
        static_cast<std::int64_t>(duration.nanosec);
      if (current_ns <= previous_ns) {
        *error = phase + " time parameterization produced a non-increasing timestamp at point " +
          std::to_string(index);
        return false;
      }
      previous_ns = current_ns;
    }
    if (previous_ns <= 0) {
      *error = phase + " time parameterization produced no positive duration";
      return false;
    }
    return true;
  }

  bool computeCheckedCartesianTrajectory(
    const std::string & phase,
    const moveit_msgs::msg::RobotState & start,
    const std::vector<geometry_msgs::msg::Pose> & waypoints,
    moveit_msgs::msg::RobotTrajectory * trajectory,
    double velocity_scaling,
    double acceleration_scaling,
    double * max_delta,
    std::string * max_joint,
    std::string * error)
  {
    if (!trajectory || !error || waypoints.empty()) {
      if (error) {
        *error = phase + " has no Cartesian waypoint output";
      }
      return false;
    }
    moveit_msgs::msg::MoveItErrorCodes cartesian_error;
    move_group_->setStartState(start);
    const double fraction = move_group_->computeCartesianPath(
      waypoints, cartesian_step_m_, 0.0, *trajectory, true, &cartesian_error);
    move_group_->setStartStateToCurrentState();
    if (cartesian_error.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS ||
      fraction < min_cartesian_fraction_ || trajectory->joint_trajectory.points.empty())
    {
      moveit_msgs::msg::RobotTrajectory no_collision_probe;
      moveit_msgs::msg::MoveItErrorCodes no_collision_error;
      move_group_->setStartState(start);
      const double no_collision_fraction = move_group_->computeCartesianPath(
        waypoints, cartesian_step_m_, 0.0, no_collision_probe,
        false, &no_collision_error);
      move_group_->setStartStateToCurrentState();
      const std::string failure_class = no_collision_fraction > fraction + 1e-6 ?
        "collision" : "IK/reachability";
      *error = phase + " Cartesian path failed (" + failure_class +
        "); checked=" + std::to_string(fraction) + ", no_collision=" +
        std::to_string(no_collision_fraction) + ", waypoints=" +
        std::to_string(waypoints.size()) + ", errors=" +
        std::to_string(cartesian_error.val) + "/" +
        std::to_string(no_collision_error.val);
      return false;
    }

    std::size_t max_point = 0;
    double max_from = 0.0;
    double max_to = 0.0;
    if (!jointStepsWithinLimit(
        start, *trajectory, max_delta, max_joint,
        &max_point, &max_from, &max_to))
    {
      *error = phase +
        " Cartesian path has an unsafe absolute joint step; joint=" + *max_joint +
        ", point=" + std::to_string(max_point) + ", from_deg=" +
        std::to_string(max_from * 180.0 / M_PI) + ", to_deg=" +
        std::to_string(max_to * 180.0 / M_PI) + ", delta_deg=" +
        std::to_string(*max_delta * 180.0 / M_PI) + ", limit_deg=" +
        std::to_string(max_joint_step_rad_ * 180.0 / M_PI);
      return false;
    }
    return timeParameterizeCartesianTrajectory(
      phase, start, velocity_scaling, acceleration_scaling,
      trajectory, error);
  }

  bool buildCompletePlanAttempt(
    const SprayPath & path, const moveit::core::RobotState & current_state,
    int branch_index, CompletePlanAttempt * attempt)
  {
    // Use the independently selected transfer speed through the reviewed safe-entry stop.
    // The cavity spray trajectory switches to its lower process speed below.
    move_group_->setMaxVelocityScalingFactor(approach_velocity_scaling_);
    move_group_->setMaxAccelerationScalingFactor(approach_acceleration_scaling_);
    move_group_->clearPoseTargets();
    move_group_->setStartState(current_state);
    const auto * joint_model_group = move_group_->getRobotModel()->getJointModelGroup(
      planning_group_);
    const auto * joint6_model = move_group_->getRobotModel()->getJointOfVariable("joint6");
    if (!joint_model_group || !joint6_model) {
      attempt->error = "MoveIt joint model is unavailable for pre-entry IK";
      return false;
    }
    const auto & joint6_bounds = move_group_->getRobotModel()->getVariableBounds("joint6");
    if (!joint6_bounds.position_bounded_) {
      attempt->error = "joint6 must have bounded limits for complete process branch selection";
      return false;
    }
    const auto joint6_seeds = revoluteIkSeedSchedule(
      current_state.getVariablePosition("joint6"),
      joint6_bounds.min_position_, joint6_bounds.max_position_,
      static_cast<std::size_t>(complete_plan_attempts_));
    if (joint6_seeds.empty()) {
      attempt->error = "no in-bounds joint6 seed is available for pre-entry IK";
      return false;
    }
    const std::size_t seed_index = std::min(
      static_cast<std::size_t>(std::max(branch_index, 0)), joint6_seeds.size() - 1U);
    attempt->pre_entry_ik_seed_joint6 = joint6_seeds[seed_index];
    moveit::core::RobotState ik_state(current_state);
    ik_state.setVariablePosition("joint6", attempt->pre_entry_ik_seed_joint6);
    ik_state.update();
    if (!ik_state.setFromIK(
        joint_model_group, path.pre_entry_flange_pose, flange_link_, planning_time_sec_ /
        static_cast<double>(complete_plan_attempts_)) ||
      !ik_state.satisfiesBounds(joint_model_group))
    {
      attempt->error = "MoveIt could not resolve pre-entry IK from joint6 seed_deg=" +
        std::to_string(attempt->pre_entry_ik_seed_joint6 * 180.0 / M_PI);
      return false;
    }
    attempt->pre_entry_ik_joint6 = ik_state.getVariablePosition("joint6");
    // Plan to the explicit IK state. A free pose goal can choose a different winding and
    // put the later +/-180 degree process sweep across the bounded joint6 limit.
    if (!move_group_->setJointValueTarget(ik_state)) {
      attempt->error = "resolved pre-entry IK state is outside the planning group bounds";
      return false;
    }
    RCLCPP_INFO(
      node_->get_logger(),
      "Complete branch %d joint6 seed/resolved: %.3f/%.3f deg",
      branch_index + 1,
      attempt->pre_entry_ik_seed_joint6 * 180.0 / M_PI,
      attempt->pre_entry_ik_joint6 * 180.0 / M_PI);
    const auto result = move_group_->plan(attempt->approach);
    move_group_->clearPoseTargets();
    if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS ||
      attempt->approach.trajectory_.joint_trajectory.points.empty()) {
      attempt->error = "MoveIt could not find a collision-free path to pre-entry";
      return false;
    }

    std::size_t approach_max_point = 0;
    double approach_max_from = 0.0;
    double approach_max_to = 0.0;
    if (!jointStepsWithinLimit(
        attempt->approach.start_state_, attempt->approach.trajectory_,
        &attempt->max_approach_joint_delta, &attempt->max_approach_joint_name,
        &approach_max_point, &approach_max_from, &approach_max_to)) {
      attempt->error = "approach path has an unsafe absolute joint step; joint=" +
        attempt->max_approach_joint_name + ", point=" +
        std::to_string(approach_max_point) + ", from_deg=" +
        std::to_string(approach_max_from * 180.0 / M_PI) + ", to_deg=" +
        std::to_string(approach_max_to * 180.0 / M_PI) + ", delta_deg=" +
        std::to_string(attempt->max_approach_joint_delta * 180.0 / M_PI) +
        ", limit_deg=" + std::to_string(max_joint_step_rad_ * 180.0 / M_PI);
      return false;
    }
    attempt->approach_joint_travel = trajectoryJointTravel(
      attempt->approach.start_state_, attempt->approach.trajectory_);
    if (!std::isfinite(attempt->approach_joint_travel)) {
      attempt->error = "approach path has invalid joint travel";
      return false;
    }

    moveit_msgs::msg::RobotState pre_entry_start = attempt->approach.start_state_;
    if (!updateStartStateFromTrajectory(
        attempt->approach.trajectory_, &pre_entry_start, &attempt->error))
    {
      attempt->error = "MoveIt pre-entry plan: " + attempt->error;
      return false;
    }

    const std::vector<geometry_msgs::msg::Pose> entry_waypoints = {
      path.cavity_entry_flange_pose};
    move_group_->setMaxVelocityScalingFactor(approach_velocity_scaling_);
    move_group_->setMaxAccelerationScalingFactor(approach_acceleration_scaling_);
    double entry_max_delta = 0.0;
    std::string entry_max_joint;
    if (!computeCheckedCartesianTrajectory(
        "safe-entry", pre_entry_start, entry_waypoints, &attempt->entry,
        approach_velocity_scaling_, approach_acceleration_scaling_,
        &entry_max_delta, &entry_max_joint, &attempt->error))
    {
      return false;
    }

    moveit_msgs::msg::RobotState entry_start = pre_entry_start;
    if (!updateStartStateFromTrajectory(attempt->entry, &entry_start, &attempt->error)) {
      attempt->error = "safe-entry trajectory: " + attempt->error;
      return false;
    }
    attempt->first_layer_point_count = firstLayerPointCount(path);
    if (attempt->first_layer_point_count == 0 ||
      attempt->first_layer_point_count > path.points.size())
    {
      attempt->error = "cannot identify the first contiguous spray layer";
      return false;
    }
    std::vector<geometry_msgs::msg::Pose> first_layer_waypoints;
    first_layer_waypoints.reserve(attempt->first_layer_point_count);
    for (std::size_t index = 0; index < attempt->first_layer_point_count; ++index) {
      first_layer_waypoints.push_back(path.points[index].flange_pose);
    }
    move_group_->setMaxVelocityScalingFactor(velocity_scaling_);
    move_group_->setMaxAccelerationScalingFactor(acceleration_scaling_);
    double first_layer_max_delta = 0.0;
    std::string first_layer_max_joint;
    if (!computeCheckedCartesianTrajectory(
        "first spray layer", entry_start, first_layer_waypoints, &attempt->first_layer,
        velocity_scaling_, acceleration_scaling_,
        &first_layer_max_delta, &first_layer_max_joint, &attempt->error))
    {
      return false;
    }

    moveit_msgs::msg::RobotState first_layer_end = entry_start;
    if (!updateStartStateFromTrajectory(
        attempt->first_layer, &first_layer_end, &attempt->error))
    {
      attempt->error = "first spray layer trajectory: " + attempt->error;
      return false;
    }

    double remaining_max_delta = 0.0;
    std::string remaining_max_joint;
    if (attempt->first_layer_point_count < path.points.size()) {
      std::vector<geometry_msgs::msg::Pose> remaining_waypoints;
      remaining_waypoints.reserve(path.points.size() - attempt->first_layer_point_count);
      for (std::size_t index = attempt->first_layer_point_count;
        index < path.points.size(); ++index)
      {
        remaining_waypoints.push_back(path.points[index].flange_pose);
      }
      if (!computeCheckedCartesianTrajectory(
          "remaining spray layers", first_layer_end, remaining_waypoints,
          &attempt->remaining_spray, velocity_scaling_, acceleration_scaling_,
          &remaining_max_delta, &remaining_max_joint, &attempt->error))
      {
        return false;
      }
      TrajectoryCombineResult spray_combine_result;
      if (!plasma_path_executor::combineReviewedTrajectories(
          attempt->first_layer, attempt->remaining_spray, &attempt->spray,
          &spray_combine_result, &attempt->error))
      {
        attempt->error = "cannot combine first and remaining spray trajectories: " +
          attempt->error;
        return false;
      }
    } else {
      attempt->spray = attempt->first_layer;
    }

    attempt->max_joint_delta = entry_max_delta;
    attempt->max_joint_name = entry_max_joint;
    if (first_layer_max_delta > attempt->max_joint_delta) {
      attempt->max_joint_delta = first_layer_max_delta;
      attempt->max_joint_name = first_layer_max_joint;
    }
    if (remaining_max_delta > attempt->max_joint_delta) {
      attempt->max_joint_delta = remaining_max_delta;
      attempt->max_joint_name = remaining_max_joint;
    }
    TrajectoryCombineResult combine_result;
    if (!plasma_path_executor::combineReviewedTrajectories(
        attempt->entry, attempt->spray, &attempt->after_entry,
        &combine_result, &attempt->error))
    {
      attempt->error = "cannot combine reviewed safe-entry and spray trajectories: " +
        attempt->error;
      return false;
    }
    if (combine_result.removed_duplicate_points > 0) {
      RCLCPP_INFO(
        node_->get_logger(),
        "Removed %zu adjacent duplicate joint points while combining safe-entry and spray",
        combine_result.removed_duplicate_points);
    }

    SprayPath first_layer_path = path;
    first_layer_path.points.resize(attempt->first_layer_point_count);
    std::vector<geometry_msgs::msg::Pose> first_layer_retreat_waypoints;
    RetreatWaypointResult first_layer_retreat_geometry;
    if (!buildZeroRotationRetreatWaypoints(
        first_layer_path, &first_layer_retreat_waypoints,
        &first_layer_retreat_geometry, &attempt->error))
    {
      attempt->error = "cannot build first-layer zero-rotation retreat: " + attempt->error;
      return false;
    }
    attempt->first_layer_retreat_waypoint_count = first_layer_retreat_waypoints.size();
    double first_layer_retreat_max_delta = 0.0;
    std::string first_layer_retreat_max_joint;
    if (!computeCheckedCartesianTrajectory(
        "first-layer zero-rotation retreat", first_layer_end,
        first_layer_retreat_waypoints, &attempt->first_layer_retreat,
        velocity_scaling_, acceleration_scaling_,
        &first_layer_retreat_max_delta, &first_layer_retreat_max_joint,
        &attempt->error))
    {
      return false;
    }
    if (first_layer_retreat_max_delta > attempt->max_joint_delta) {
      attempt->max_joint_delta = first_layer_retreat_max_delta;
      attempt->max_joint_name = first_layer_retreat_max_joint;
    }

    moveit_msgs::msg::RobotState retreat_start = entry_start;
    if (!updateStartStateFromTrajectory(attempt->spray, &retreat_start, &attempt->error)) {
      attempt->error = "spray trajectory cannot seed cavity retreat: " + attempt->error;
      return false;
    }
    std::vector<geometry_msgs::msg::Pose> retreat_waypoints;
    RetreatWaypointResult retreat_geometry;
    if (!buildZeroRotationRetreatWaypoints(
        path, &retreat_waypoints, &retreat_geometry, &attempt->error))
    {
      attempt->error = "cannot build direct zero-rotation cavity retreat: " + attempt->error;
      return false;
    }
    attempt->retreat_waypoint_count = retreat_waypoints.size();
    move_group_->setMaxVelocityScalingFactor(velocity_scaling_);
    move_group_->setMaxAccelerationScalingFactor(acceleration_scaling_);
    double retreat_max_delta = 0.0;
    std::string retreat_max_joint;
    if (!computeCheckedCartesianTrajectory(
        "zero-rotation cavity retreat", retreat_start, retreat_waypoints,
        &attempt->retreat, velocity_scaling_, acceleration_scaling_,
        &retreat_max_delta, &retreat_max_joint, &attempt->error))
    {
      return false;
    }
    if (retreat_max_delta > attempt->max_joint_delta) {
      attempt->max_joint_delta = retreat_max_delta;
      attempt->max_joint_name = retreat_max_joint;
    }
    return true;
  }

  void onPlan(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    SprayPath path;
    bool starting_at_safe_entry = false;
    {
      std::scoped_lock lock(mutex_);
      if (executing_ || planning_) {
        response->success = false;
        response->message = executing_ ? "entry motion is active" : "entry planning is already active";
        return;
      }
      if (!path_available_) {
        response->success = false;
        response->message = "no transformed entry target is available";
        return;
      }
    }

    std::string current_state_reason;
    const auto current_state = waitForCurrentRobotState(
      joint_state_wait_sec_, &current_state_reason);
    if (!current_state) {
      std::scoped_lock lock(mutex_);
      response->success = false;
      response->message = current_state_reason +
        "; planning is locked to prevent a false start state";
      state_ = Status::STATE_ERROR;
      message_ = response->message;
      publishStatusLocked();
      return;
    }

    {
      std::scoped_lock lock(mutex_);
      if (executing_ || planning_ || !path_available_) {
        response->success = false;
        response->message = "planner state changed while waiting for fresh joint data";
        return;
      }
      path = path_;
      if (poseFreshLocked()) {
        const PoseError entry_error = poseError(
          latest_flange_pose_, path.cavity_entry_flange_pose);
        starting_at_safe_entry =
          entry_error.translation_m <= pose_tolerance_m_ &&
          entry_error.rotation_rad <= rotation_tolerance_rad_;
      }
      planning_ = true;
      plan_available_ = false;
      retreat_available_ = false;
      clearClearanceLocked();
      state_ = Status::STATE_PLANNING;
      message_ = starting_at_safe_entry ?
        "robot is already at safe entry; MoveIt is re-reviewing the cavity spray branch" :
        "MoveIt is searching for a complete collision-free approach and spray branch";
      publishStatusLocked();
    }

    CompletePlanAttempt accepted_attempt;
    CompletePlanAttempt last_attempt;
    bool complete_plan_found = false;
    int attempts_used = 0;
    for (int attempt_index = 0; attempt_index < complete_plan_attempts_; ++attempt_index) {
      CompletePlanAttempt candidate;
      attempts_used = attempt_index + 1;
      if (buildCompletePlanAttempt(path, *current_state, attempt_index, &candidate)) {
        if (starting_at_safe_entry) {
          std::string start_reason;
          if (!currentJointsMatchReviewedStart(candidate.spray, &start_reason)) {
            candidate.error =
              "spray branch does not preserve the current safe-entry joints: " + start_reason;
            RCLCPP_WARN(
              node_->get_logger(), "Safe-entry spray re-review attempt %d/%d rejected: %s",
              attempts_used, complete_plan_attempts_, candidate.error.c_str());
            last_attempt = std::move(candidate);
            continue;
          }
        }
        if (!complete_plan_found ||
          candidate.approach_joint_travel < accepted_attempt.approach_joint_travel) {
          accepted_attempt = std::move(candidate);
          complete_plan_found = true;
        }
        continue;
      }
      RCLCPP_WARN(
        node_->get_logger(), "Complete planning attempt %d/%d rejected: %s",
        attempts_used, complete_plan_attempts_, candidate.error.c_str());
      last_attempt = std::move(candidate);
    }

    std::scoped_lock lock(mutex_);
    if (!planning_) {
      response->success = false;
      response->message = "complete planning was stopped";
      return;
    }
    planning_ = false;
    if (path_.path_id != path.path_id) {
      response->success = false;
      response->message = "path changed while planning; plan discarded";
      state_ = Status::STATE_READY_TO_PLAN;
      message_ = response->message;
      publishStatusLocked();
      return;
    }
    if (!complete_plan_found) {
      plan_available_ = false;
      state_ = Status::STATE_ERROR;
      message_ = "complete planning failed after " + std::to_string(attempts_used) +
        " branch attempts; last failure: " + last_attempt.error;
      response->success = false;
      response->message = message_;
      publishStatusLocked();
      return;
    }

    plan_ = std::move(accepted_attempt.approach);
    entry_preview_ = std::move(accepted_attempt.entry);
    first_layer_preview_ = std::move(accepted_attempt.first_layer);
    remaining_spray_preview_ = std::move(accepted_attempt.remaining_spray);
    spray_preview_ = std::move(accepted_attempt.spray);
    reviewed_first_layer_retreat_ = std::move(accepted_attempt.first_layer_retreat);
    reviewed_retreat_ = std::move(accepted_attempt.retreat);
    first_layer_point_count_ = accepted_attempt.first_layer_point_count;
    plan_available_ = true;
    retreat_available_ = false;
    planned_path_id_ = path.path_id;
    plan_created_ = std::chrono::steady_clock::now();
    moveit_msgs::msg::DisplayTrajectory display;
    display.model_id = move_group_->getRobotModel()->getName();
    display.trajectory_start = plan_.start_state_;
    if (starting_at_safe_entry) {
      std::string display_start_reason;
      if (!updateStartStateFromTrajectory(
          entry_preview_, &display.trajectory_start, &display_start_reason))
      {
        plan_available_ = false;
        state_ = Status::STATE_ERROR;
        message_ = "cannot prepare safe-entry spray preview start: " + display_start_reason;
        response->success = false;
        response->message = message_;
        publishStatusLocked();
        return;
      }
      state_ = Status::STATE_AT_ENTRY;
      cavity_retreat_context_active_ = true;
      message_ = "robot is already at the verified safe-entry pose; continuous cavity branch "
        "re-reviewed across " + std::to_string(attempts_used) +
        " nearby-IK attempts; spray/retreat max step=" +
        std::to_string(accepted_attempt.max_joint_delta * 180.0 / M_PI) +
        " deg on " + accepted_attempt.max_joint_name +
        "; retreat waypoints=" +
        std::to_string(accepted_attempt.retreat_waypoint_count) +
        "; inspect the continuous all-layer spray and full-retreat sequence in RViz";
      display.trajectory.push_back(spray_preview_);
      display.trajectory.push_back(reviewed_retreat_);
    } else {
      state_ = Status::STATE_PREVIEW_READY;
      cavity_retreat_context_active_ = false;
      message_ = "approach, safe-entry stop, continuous all-layer spray, and direct retreat collision-checked across " +
        std::to_string(attempts_used) + " nearby-IK attempts; approach travel=" +
        std::to_string(accepted_attempt.approach_joint_travel * 180.0 / M_PI) +
        " joint-deg; approach max step=" +
        std::to_string(accepted_attempt.max_approach_joint_delta * 180.0 / M_PI) +
        " deg on " + accepted_attempt.max_approach_joint_name + "; cavity max step=" +
        std::to_string(accepted_attempt.max_joint_delta * 180.0 / M_PI) +
        " deg on " + accepted_attempt.max_joint_name +
        "; pre-entry joint6=" +
        std::to_string(accepted_attempt.pre_entry_ik_joint6 * 180.0 / M_PI) + " deg" +
        "; retreat waypoints=" +
        std::to_string(accepted_attempt.retreat_waypoint_count) +
        "; inspect the continuous approach, all-layer cavity motion, and full retreat in RViz";
      display.trajectory.push_back(plan_.trajectory_);
      display.trajectory.push_back(entry_preview_);
      display.trajectory.push_back(spray_preview_);
      display.trajectory.push_back(reviewed_retreat_);
    }
    display_publisher_->publish(display);
    publishClearanceLocked(path);
    response->success = true;
    response->message = message_;
    publishStatusLocked();
  }

  void onExecute(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    std::string speed_reason;
    if (!setControllerSpeed(approach_velocity_scaling_, &speed_reason)) {
      response->success = false;
      response->message = "cannot set pre-entry controller speed: " + speed_reason;
      std::scoped_lock lock(mutex_);
      state_ = Status::STATE_ERROR;
      message_ = response->message;
      publishStatusLocked();
      return;
    }
    {
      std::scoped_lock lock(mutex_);
      if (!plan_available_ || planned_path_id_ != path_.path_id) {
        response->success = false;
        response->message = "no current previewed plan; plan again";
        return;
      }
      std::string stale_reason;
      if (reviewWindowExpiredLocked(&stale_reason)) {
        response->success = false;
        response->message = stale_reason;
        return;
      }
      if (!path_.execution_permitted && !allow_unvalidated_dry_run_) {
        response->success = false;
        response->message =
          "path execution_permitted=false; hand-eye and TCP acceptance are incomplete";
        return;
      }
      if (!motion_enabled_) {
        response->success = false;
        response->message = "motion_enabled=false; preview only";
        return;
      }
      if (!collisionStageReadyLocked()) {
        response->success = false;
        response->message = "RealMan collision stage 8 is not freshly verified";
        requestCollisionStageLocked();
        return;
      }
      if (!armIdleLocked()) {
        response->success = false;
        response->message = "RealMan is not freshly verified idle; arm_current_status=" +
          std::to_string(arm_status_) + "; execution is locked";
        return;
      }
      if (!poseFreshLocked()) {
        response->success = false;
        response->message = "current Link6 pose is missing or stale; execution is locked";
        return;
      }
      executing_ = true;
      cavity_retreat_context_active_ = false;
      state_ = Status::STATE_EXECUTING;
      message_ = "executing reviewed MoveIt plan to pre-entry at " +
        std::to_string(static_cast<int>(std::lround(approach_velocity_scaling_ * 100.0))) +
        "%";
      publishStatusLocked();
    }

    const auto result = move_group_->execute(plan_);

    std::scoped_lock lock(mutex_);
    if (!executing_) {
      response->success = false;
      response->message = "reviewed pre-entry motion was stopped";
      return;
    }
    executing_ = false;
    if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
      plan_available_ = false;
      state_ = Status::STATE_ERROR;
      message_ = "MoveIt/controller failed while moving to pre-entry";
      response->success = false;
      response->message = message_;
      publishStatusLocked();
      return;
    }
    if (!poseFreshLocked()) {
      plan_available_ = false;
      state_ = Status::STATE_ERROR;
      message_ = "controller reported success but current Link6 pose is missing or stale";
      response->success = false;
      response->message = message_;
      publishStatusLocked();
      return;
    }
    const auto current_flange_pose = latest_flange_pose_;
    const PoseError error = poseError(current_flange_pose, path_.pre_entry_flange_pose);
    if (error.translation_m > pose_tolerance_m_ ||
      error.rotation_rad > rotation_tolerance_rad_) {
      plan_available_ = false;
      state_ = Status::STATE_ERROR;
      message_ = "controller reported success but pre-entry pose verification failed; "
        "translation_error_mm=" + std::to_string(error.translation_m * 1000.0) +
        ", rotation_error_deg=" + std::to_string(error.rotation_rad * 180.0 / M_PI);
      response->success = false;
      response->message = message_;
      publishStatusLocked();
      return;
    }
    // Reaching and verifying a reviewed checkpoint starts the operator's approval
    // window for the next already-reviewed segment.
    plan_created_ = std::chrono::steady_clock::now();
    state_ = Status::STATE_AT_PRE_ENTRY;
    cavity_retreat_context_active_ = false;
    message_ = "pre-entry reached and verified; next reviewed segment approval window reset; "
      "cavity linear-entry confirmation is next";
    response->success = true;
    response->message = message_;
    publishStatusLocked();
  }

  void onExecuteReviewedEntry(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    std::string speed_reason;
    if (!setControllerSpeed(approach_velocity_scaling_, &speed_reason)) {
      response->success = false;
      response->message = "cannot set safe-entry controller speed: " + speed_reason;
      std::scoped_lock lock(mutex_);
      state_ = Status::STATE_ERROR;
      message_ = response->message;
      publishStatusLocked();
      return;
    }
    moveit_msgs::msg::RobotTrajectory reviewed_trajectory;
    {
      std::scoped_lock lock(mutex_);
      if (executing_ || planning_) {
        response->success = false;
        response->message = "planning or execution is already active";
        return;
      }
      if (state_ != Status::STATE_AT_PRE_ENTRY || !plan_available_ ||
        planned_path_id_ != path_.path_id ||
        entry_preview_.joint_trajectory.points.empty())
      {
        response->success = false;
        response->message = "no current reviewed safe-entry trajectory; plan and reach pre-entry again";
        return;
      }
      std::string stale_reason;
      if (reviewWindowExpiredLocked(&stale_reason)) {
        response->success = false;
        response->message = stale_reason;
        return;
      }
      if (!path_.execution_permitted && !allow_unvalidated_dry_run_) {
        response->success = false;
        response->message =
          "path execution_permitted=false; hand-eye and TCP acceptance are incomplete";
        return;
      }
      if (!motion_enabled_) {
        response->success = false;
        response->message = "motion_enabled=false; safe-entry motion is locked";
        return;
      }
      if (!collisionStageReadyLocked()) {
        response->success = false;
        response->message = "RealMan collision stage 8 is not freshly verified";
        requestCollisionStageLocked();
        return;
      }
      if (!armIdleLocked()) {
        response->success = false;
        response->message = "RealMan is not freshly verified idle; arm_current_status=" +
          std::to_string(arm_status_) + "; execution is locked";
        return;
      }
      if (!poseFreshLocked()) {
        response->success = false;
        response->message = "current Link6 pose is missing or stale; execution is locked";
        return;
      }
      const PoseError pre_entry_error = poseError(
        latest_flange_pose_, path_.pre_entry_flange_pose);
      if (pre_entry_error.translation_m > pose_tolerance_m_ ||
        pre_entry_error.rotation_rad > rotation_tolerance_rad_)
      {
        response->success = false;
        response->message = "robot is not at the reviewed pre-entry pose; replan first";
        return;
      }
      reviewed_trajectory = entry_preview_;
      executing_ = true;
      cavity_retreat_context_active_ = true;
      state_ = Status::STATE_EXECUTING;
      message_ = "executing reviewed pre-entry to safe-entry trajectory at " +
        std::to_string(static_cast<int>(std::lround(approach_velocity_scaling_ * 100.0))) +
        "%; rounded tip will remain outside the mouth and plasma is disabled";
      publishStatusLocked();
    }

    std::string start_reason;
    if (!currentJointsMatchReviewedStart(reviewed_trajectory, &start_reason)) {
      std::scoped_lock lock(mutex_);
      if (!executing_) {
        response->success = false;
        response->message = "reviewed safe-entry motion was stopped during start verification";
        return;
      }
      executing_ = false;
      cavity_retreat_context_active_ = false;
      plan_available_ = false;
      state_ = Status::STATE_ERROR;
      message_ = start_reason;
      response->success = false;
      response->message = message_;
      publishStatusLocked();
      return;
    }

    const auto result = move_group_->execute(reviewed_trajectory);
    std::scoped_lock lock(mutex_);
    if (!executing_) {
      response->success = false;
      response->message = "reviewed safe-entry motion was stopped";
      return;
    }
    executing_ = false;
    if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
      plan_available_ = false;
      state_ = Status::STATE_ERROR;
      message_ = "MoveIt/controller failed while moving to the reviewed safe entry";
      response->success = false;
      response->message = message_;
      publishStatusLocked();
      return;
    }
    if (!poseFreshLocked()) {
      plan_available_ = false;
      state_ = Status::STATE_ERROR;
      message_ = "controller reported success but final Link6 pose is missing or stale";
      response->success = false;
      response->message = message_;
      publishStatusLocked();
      return;
    }
    const PoseError entry_error = poseError(
      latest_flange_pose_, path_.cavity_entry_flange_pose);
    if (entry_error.translation_m > pose_tolerance_m_ ||
      entry_error.rotation_rad > rotation_tolerance_rad_)
    {
      plan_available_ = false;
      state_ = Status::STATE_ERROR;
      message_ = "safe entry reached but pose verification failed; translation_error_mm=" +
        std::to_string(entry_error.translation_m * 1000.0) +
        ", rotation_error_deg=" +
        std::to_string(entry_error.rotation_rad * 180.0 / M_PI);
      response->success = false;
      response->message = message_;
      publishStatusLocked();
      return;
    }
    plan_created_ = std::chrono::steady_clock::now();
    state_ = Status::STATE_AT_ENTRY;
    message_ = "safe entry reached and verified; next reviewed segment approval window reset; "
      "inspect the rounded-tip clearance before spray motion";
    response->success = true;
    response->message = message_;
    publishStatusLocked();
  }

  void onExecuteReviewedFirstLayer(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    std::string speed_reason;
    if (!setControllerSpeed(velocity_scaling_, &speed_reason)) {
      response->success = false;
      response->message = "cannot set continuous spray controller speed: " + speed_reason;
      std::scoped_lock lock(mutex_);
      state_ = Status::STATE_ERROR;
      message_ = response->message;
      publishStatusLocked();
      return;
    }

    moveit_msgs::msg::RobotTrajectory reviewed_trajectory;
    geometry_msgs::msg::Pose final_flange_pose;
    {
      std::scoped_lock lock(mutex_);
      if (executing_ || planning_) {
        response->success = false;
        response->message = "planning or execution is already active";
        return;
      }
      if (state_ != Status::STATE_AT_ENTRY || !plan_available_ ||
        planned_path_id_ != path_.path_id || path_.points.empty() ||
        spray_preview_.joint_trajectory.points.empty())
      {
        response->success = false;
        response->message =
          "no current reviewed complete spray trajectory at safe entry";
        return;
      }
      std::string stale_reason;
      if (reviewWindowExpiredLocked(&stale_reason)) {
        response->success = false;
        response->message = stale_reason;
        return;
      }
      if (!path_.execution_permitted && !allow_unvalidated_dry_run_) {
        response->success = false;
        response->message =
          "path execution_permitted=false; hand-eye and TCP acceptance are incomplete";
        return;
      }
      if (!motion_enabled_) {
        response->success = false;
        response->message = "motion_enabled=false; cavity spray motion is locked";
        return;
      }
      if (!collisionStageReadyLocked()) {
        response->success = false;
        response->message = "RealMan collision stage 8 is not freshly verified";
        requestCollisionStageLocked();
        return;
      }
      if (!armIdleLocked()) {
        response->success = false;
        response->message = "RealMan is not freshly verified idle; arm_current_status=" +
          std::to_string(arm_status_) + "; cavity spray motion is locked";
        return;
      }
      if (!poseFreshLocked()) {
        response->success = false;
        response->message = "current Link6 pose is missing or stale; execution is locked";
        return;
      }
      const PoseError entry_error = poseError(
        latest_flange_pose_, path_.cavity_entry_flange_pose);
      if (entry_error.translation_m > pose_tolerance_m_ ||
        entry_error.rotation_rad > rotation_tolerance_rad_)
      {
        response->success = false;
        response->message = "robot is not at the reviewed safe-entry pose; do not enter cavity";
        return;
      }
      reviewed_trajectory = spray_preview_;
      final_flange_pose = path_.points.back().flange_pose;
      executing_ = true;
      cavity_retreat_context_active_ = true;
      state_ = Status::STATE_EXECUTING;
      message_ = "checking current joints against the reviewed complete spray start";
      publishStatusLocked();
    }

    std::string start_reason;
    if (!currentJointsMatchReviewedStart(reviewed_trajectory, &start_reason)) {
      std::scoped_lock lock(mutex_);
      if (!executing_) {
        response->success = false;
        response->message =
          "reviewed complete spray motion was stopped during start verification";
        return;
      }
      executing_ = false;
      cavity_retreat_context_active_ = true;
      plan_available_ = false;
      state_ = Status::STATE_ERROR;
      message_ = start_reason;
      response->success = false;
      response->message = message_;
      publishStatusLocked();
      return;
    }

    {
      std::scoped_lock lock(mutex_);
      if (!executing_) {
        response->success = false;
        response->message = "reviewed complete spray motion was stopped before execution";
        return;
      }
      message_ = "executing all reviewed spray layers continuously at " +
        std::to_string(static_cast<int>(std::lround(velocity_scaling_ * 100.0))) +
        "%; the red stop remains available and plasma is disabled";
      publishStatusLocked();
    }
    const auto result = move_group_->execute(reviewed_trajectory);

    std::scoped_lock lock(mutex_);
    if (!executing_) {
      response->success = false;
      response->message = "reviewed complete spray motion was stopped";
      return;
    }
    executing_ = false;
    if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
      plan_available_ = false;
      state_ = Status::STATE_ERROR;
      message_ = "MoveIt/controller failed while executing the reviewed complete spray path";
      response->success = false;
      response->message = message_;
      publishStatusLocked();
      return;
    }
    if (!poseFreshLocked()) {
      plan_available_ = false;
      state_ = Status::STATE_ERROR;
      message_ = "controller reported success but final spray Link6 pose is stale";
      response->success = false;
      response->message = message_;
      publishStatusLocked();
      return;
    }
    const PoseError final_error = poseError(
      latest_flange_pose_, final_flange_pose);
    if (final_error.translation_m > pose_tolerance_m_ ||
      final_error.rotation_rad > rotation_tolerance_rad_)
    {
      plan_available_ = false;
      state_ = Status::STATE_ERROR;
      message_ = "complete spray path finished but final pose verification failed";
      response->success = false;
      response->message = message_;
      publishStatusLocked();
      return;
    }

    retreat_available_ = !reviewed_retreat_.joint_trajectory.points.empty();
    plan_created_ = std::chrono::steady_clock::now();
    state_ = Status::STATE_COMPLETED;
    message_ = "all reviewed spray layers completed continuously and stopped at zero degrees; "
      "plasma remained disabled; direct retreat is ready";
    response->success = true;
    response->message = message_;
    publishStatusLocked();
  }

  void onExecuteReviewedDryRun(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    std::string speed_reason;
    if (!setControllerSpeed(velocity_scaling_, &speed_reason)) {
      response->success = false;
      response->message = "cannot set cavity controller speed: " + speed_reason;
      std::scoped_lock lock(mutex_);
      state_ = Status::STATE_ERROR;
      message_ = response->message;
      publishStatusLocked();
      return;
    }
    moveit_msgs::msg::RobotTrajectory reviewed_trajectory;
    geometry_msgs::msg::Pose final_flange_pose;
    {
      std::scoped_lock lock(mutex_);
      if (executing_ || planning_) {
        response->success = false;
        response->message = "planning or execution is already active";
        return;
      }
      if (state_ != Status::STATE_AT_FIRST_LAYER || !plan_available_ ||
        planned_path_id_ != path_.path_id || path_.points.empty() ||
        first_layer_point_count_ == 0 ||
        first_layer_point_count_ > path_.points.size() ||
        remaining_spray_preview_.joint_trajectory.points.empty()) {
        response->success = false;
        response->message =
          "no current reviewed remaining-layer trajectory at the first-layer stop";
        return;
      }
      std::string stale_reason;
      if (reviewWindowExpiredLocked(&stale_reason)) {
        response->success = false;
        response->message = stale_reason;
        return;
      }
      if (!path_.execution_permitted && !allow_unvalidated_dry_run_) {
        response->success = false;
        response->message =
          "path execution_permitted=false; hand-eye and TCP acceptance are incomplete";
        return;
      }
      if (!motion_enabled_) {
        response->success = false;
        response->message = "motion_enabled=false; preview only";
        return;
      }
      if (!collisionStageReadyLocked()) {
        response->success = false;
        response->message = "RealMan collision stage 8 is not freshly verified";
        requestCollisionStageLocked();
        return;
      }
      if (!armIdleLocked()) {
        response->success = false;
        response->message = "RealMan is not freshly verified idle; arm_current_status=" +
          std::to_string(arm_status_) + "; execution is locked";
        return;
      }
      if (!poseFreshLocked()) {
        response->success = false;
        response->message = "current Link6 pose is missing or stale; execution is locked";
        return;
      }
      const auto current_flange_pose = latest_flange_pose_;
      const PoseError first_layer_error = poseError(
        current_flange_pose, path_.points[first_layer_point_count_ - 1].flange_pose);
      if (first_layer_error.translation_m > pose_tolerance_m_ ||
        first_layer_error.rotation_rad > rotation_tolerance_rad_) {
        response->success = false;
        response->message = "robot is not at the reviewed first-layer stop; translation_error_mm=" +
          std::to_string(first_layer_error.translation_m * 1000.0) +
          ", rotation_error_deg=" +
          std::to_string(first_layer_error.rotation_rad * 180.0 / M_PI) +
          "; do not start remaining cavity motion";
        return;
      }
      reviewed_trajectory = remaining_spray_preview_;
      final_flange_pose = path_.points.back().flange_pose;
      executing_ = true;
      state_ = Status::STATE_EXECUTING;
      message_ = "checking current joints against the reviewed remaining-layer start";
      publishStatusLocked();
    }

    std::string start_reason;
    if (!currentJointsMatchReviewedStart(reviewed_trajectory, &start_reason)) {
      std::scoped_lock lock(mutex_);
      if (!executing_) {
        response->success = false;
        response->message = "reviewed dry-run was stopped during start verification";
        return;
      }
      executing_ = false;
      plan_available_ = false;
      state_ = Status::STATE_ERROR;
      message_ = start_reason;
      response->success = false;
      response->message = message_;
      publishStatusLocked();
      return;
    }

    {
      std::scoped_lock lock(mutex_);
      if (!executing_) {
        response->success = false;
        response->message = "reviewed dry-run was stopped before execution";
        return;
      }
      message_ = "executing the reviewed remaining spray layers at " +
        std::to_string(static_cast<int>(std::lround(velocity_scaling_ * 100.0))) +
        "%; plasma output is disabled";
      publishStatusLocked();
    }
    const auto result = move_group_->execute(reviewed_trajectory);

    std::scoped_lock lock(mutex_);
    if (!executing_) {
      response->success = false;
      response->message = "reviewed dry-run was stopped";
      return;
    }
    executing_ = false;
    plan_available_ = false;
    if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
      state_ = Status::STATE_ERROR;
      message_ = "MoveIt/controller failed while executing the reviewed remaining layers";
      response->success = false;
      response->message = message_;
      publishStatusLocked();
      return;
    }
    if (!poseFreshLocked()) {
      state_ = Status::STATE_ERROR;
      message_ = "controller reported success but final Link6 pose is missing or stale";
      response->success = false;
      response->message = message_;
      publishStatusLocked();
      return;
    }
    const auto current_flange_pose = latest_flange_pose_;
    const PoseError final_error = poseError(current_flange_pose, final_flange_pose);
    if (final_error.translation_m > pose_tolerance_m_ ||
      final_error.rotation_rad > rotation_tolerance_rad_) {
      state_ = Status::STATE_ERROR;
      message_ = "controller reported success but final spray pose verification failed; "
        "translation_error_mm=" + std::to_string(final_error.translation_m * 1000.0) +
        ", rotation_error_deg=" + std::to_string(final_error.rotation_rad * 180.0 / M_PI);
      response->success = false;
      response->message = message_;
      publishStatusLocked();
      return;
    }
    state_ = Status::STATE_COMPLETED;
    retreat_available_ = true;
    message_ = "reviewed remaining spray layers completed; stopped at zero degrees; "
      "plasma remained disabled and direct retreat is ready";
    response->success = true;
    response->message = message_;
    publishStatusLocked();
  }

  void onRetreatToPreEntry(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    std::string speed_reason;
    if (!setControllerSpeed(velocity_scaling_, &speed_reason)) {
      response->success = false;
      response->message = "cannot set cavity-retreat controller speed: " + speed_reason;
      std::scoped_lock lock(mutex_);
      if (cavity_retreat_context_active_ &&
        (state_ == Status::STATE_STOPPING_IN_CAVITY ||
        state_ == Status::STATE_STOPPED_IN_CAVITY))
      {
        message_ = response->message + "; robot remains stopped inside the cavity";
      } else {
        state_ = Status::STATE_ERROR;
        message_ = response->message;
      }
      response->message = message_;
      publishStatusLocked();
      return;
    }
    bool retreating_from_stopped_pose = false;
    {
      std::scoped_lock lock(mutex_);
      retreating_from_stopped_pose = state_ == Status::STATE_STOPPED_IN_CAVITY;
    }
    if (retreating_from_stopped_pose) {
      onRetreatFromStoppedPose(response);
      return;
    }
    moveit_msgs::msg::RobotTrajectory retreat;
    geometry_msgs::msg::Pose retreat_origin_pose;
    bool retreating_from_first_layer = false;
    {
      std::scoped_lock lock(mutex_);
      if (executing_ || planning_) {
        response->success = false;
        response->message = "planning or execution is already active";
        return;
      }
      retreating_from_first_layer = state_ == Status::STATE_AT_FIRST_LAYER;
      const bool completed_path_retreat = state_ == Status::STATE_COMPLETED &&
        !reviewed_retreat_.joint_trajectory.points.empty();
      const bool first_layer_retreat = retreating_from_first_layer &&
        first_layer_point_count_ > 0 && first_layer_point_count_ <= path_.points.size() &&
        !reviewed_first_layer_retreat_.joint_trajectory.points.empty();
      if (!retreat_available_ || (!completed_path_retreat && !first_layer_retreat))
      {
        response->success = false;
        response->message = "no reviewed retreat is available from the current cavity stop";
        return;
      }
      if (!motion_enabled_) {
        response->success = false;
        response->message = "motion_enabled=false; retreat is locked";
        return;
      }
      if (!collisionStageReadyLocked()) {
        response->success = false;
        response->message = "RealMan collision stage 8 is not freshly verified";
        requestCollisionStageLocked();
        return;
      }
      if (!armIdleLocked()) {
        response->success = false;
        response->message = "RealMan is not freshly verified idle; arm_current_status=" +
          std::to_string(arm_status_) + "; retreat is locked";
        return;
      }
      if (!poseFreshLocked()) {
        response->success = false;
        response->message = "current Link6 pose is missing or stale; retreat is locked";
        return;
      }
      retreat_origin_pose = retreating_from_first_layer ?
        path_.points[first_layer_point_count_ - 1].flange_pose :
        path_.points.back().flange_pose;
      const PoseError final_error = poseError(
        latest_flange_pose_, retreat_origin_pose);
      if (final_error.translation_m > pose_tolerance_m_ ||
        final_error.rotation_rad > rotation_tolerance_rad_)
      {
        response->success = false;
        response->message = "robot is not at the reviewed cavity stop pose; retreat is locked";
        return;
      }
      retreat = retreating_from_first_layer ? reviewed_first_layer_retreat_ : reviewed_retreat_;
      executing_ = true;
      cavity_retreat_context_active_ = true;
      state_ = Status::STATE_EXECUTING;
      message_ = std::string(retreating_from_first_layer ?
        "retreating directly from the first-layer stop along the entry normal at " :
        "retreating directly along zero-degree layer centers and the entry normal at ") +
        std::to_string(static_cast<int>(std::lround(velocity_scaling_ * 100.0))) +
        "%; tool orientation is fixed and plasma is disabled";
      publishStatusLocked();
    }

    std::string start_reason;
    if (!currentJointsMatchReviewedStart(retreat, &start_reason)) {
      std::scoped_lock lock(mutex_);
      if (!executing_) {
        response->success = false;
        response->message = "cavity retreat was stopped during start verification";
        return;
      }
      executing_ = false;
      retreat_available_ = false;
      state_ = Status::STATE_ERROR;
      message_ = start_reason;
      response->success = false;
      response->message = message_;
      publishStatusLocked();
      return;
    }

    const auto result = move_group_->execute(retreat);
    std::scoped_lock lock(mutex_);
    if (!executing_) {
      response->success = false;
      response->message = "cavity retreat was stopped";
      return;
    }
    executing_ = false;
    retreat_available_ = false;
    plan_available_ = false;
    if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
      state_ = Status::STATE_ERROR;
      message_ = "MoveIt/controller failed while retreating to pre-entry";
      response->success = false;
      response->message = message_;
      publishStatusLocked();
      return;
    }
    geometry_msgs::msg::Pose fixed_orientation_pre_entry = path_.pre_entry_flange_pose;
    fixed_orientation_pre_entry.orientation = retreat_origin_pose.orientation;
    const PoseError pre_entry_error = poseError(
      latest_flange_pose_, fixed_orientation_pre_entry);
    if (pre_entry_error.translation_m > pose_tolerance_m_ ||
      pre_entry_error.rotation_rad > rotation_tolerance_rad_)
    {
      state_ = Status::STATE_ERROR;
      message_ = "retreat completed but pre-entry pose verification failed";
      response->success = false;
      response->message = message_;
      publishStatusLocked();
      return;
    }
    state_ = Status::STATE_AT_PRE_ENTRY;
    cavity_retreat_context_active_ = false;
    message_ = "direct zero-rotation cavity retreat completed and pre-entry verified; "
      "plasma remained disabled";
    response->success = true;
    response->message = message_;
    publishStatusLocked();
  }

  void onRetreatFromStoppedPose(
    const std::shared_ptr<std_srvs::srv::Trigger::Response> & response)
  {
    SprayPath path;
    geometry_msgs::msg::Pose stopped_flange_pose;
    {
      std::scoped_lock lock(mutex_);
      if (executing_ || planning_ ||
        state_ != Status::STATE_STOPPED_IN_CAVITY ||
        !cavity_retreat_context_active_ || !path_available_)
      {
        response->success = false;
        response->message = "no verified stopped cavity pose is available for retreat";
        return;
      }
      if (!motion_enabled_) {
        response->success = false;
        response->message = "motion_enabled=false; stopped-pose retreat is locked";
        return;
      }
      if (!path_.execution_permitted && !allow_unvalidated_dry_run_) {
        response->success = false;
        response->message =
          "path execution_permitted=false; stopped-pose retreat is locked";
        return;
      }
      if (!collisionStageReadyLocked()) {
        response->success = false;
        response->message = "RealMan collision stage 8 is not freshly verified";
        requestCollisionStageLocked();
        return;
      }
      if (!stoppedRobotIdleLocked() || !poseFreshLocked() || !jointStateFreshLocked()) {
        response->success = false;
        response->message =
          "robot stop is not yet verified with fresh controller-idle or continuous "
          "joint stability, pose, and joint data";
        return;
      }
      path = path_;
      stopped_flange_pose = latest_flange_pose_;
      planning_ = true;
      plan_available_ = false;
      retreat_available_ = false;
      state_ = Status::STATE_PLANNING;
      message_ = "planning a new collision-checked straight outward retreat from the "
        "actual stopped cavity pose; tool orientation remains fixed";
      publishStatusLocked();
    }

    std::string error;
    const auto current_state = waitForCurrentRobotState(joint_state_wait_sec_, &error);
    moveit_msgs::msg::RobotState start_state;
    if (current_state) {
      moveit::core::robotStateToRobotStateMsg(*current_state, start_state);
      start_state.is_diff = false;
    }
    StoppedRetreatLimits limits;
    limits.max_axis_offset_m = stopped_retreat_max_axis_offset_m_;
    limits.max_outward_distance_m = stopped_retreat_max_distance_m_;
    std::vector<geometry_msgs::msg::Pose> waypoints;
    StoppedRetreatResult geometry;
    moveit_msgs::msg::RobotTrajectory retreat;
    double max_delta = 0.0;
    std::string max_joint;
    bool planned = static_cast<bool>(current_state);
    if (!planned) {
      error = "fresh joint state is unavailable for stopped-pose retreat: " + error;
    } else if (!buildStoppedRetreatWaypoints(
        path, stopped_flange_pose, limits, &waypoints, &geometry, &error))
    {
      planned = false;
    } else if (!computeCheckedCartesianTrajectory(
        "stopped-pose outward retreat", start_state, waypoints, &retreat,
        velocity_scaling_, acceleration_scaling_, &max_delta, &max_joint, &error))
    {
      planned = false;
    }

    {
      std::scoped_lock lock(mutex_);
      if (!planning_ || !cavity_retreat_context_active_) {
        response->success = false;
        response->message = "stopped-pose retreat planning was cancelled by another stop";
        return;
      }
      planning_ = false;
      if (!planned) {
        state_ = Status::STATE_STOPPED_IN_CAVITY;
        message_ = "stopped-pose retreat planning failed; robot remains stopped: " + error;
        response->success = false;
        response->message = message_;
        publishStatusLocked();
        return;
      }
      if (!stoppedRobotIdleLocked() || !poseFreshLocked() ||
        poseError(latest_flange_pose_, stopped_flange_pose).translation_m > pose_tolerance_m_ ||
        poseError(latest_flange_pose_, stopped_flange_pose).rotation_rad > rotation_tolerance_rad_)
      {
        state_ = Status::STATE_STOPPED_IN_CAVITY;
        message_ = "robot moved while the stopped-pose retreat was being planned; plan again";
        response->success = false;
        response->message = message_;
        publishStatusLocked();
        return;
      }
      moveit_msgs::msg::DisplayTrajectory display;
      display.model_id = move_group_->getRobotModel()->getName();
      display.trajectory_start = start_state;
      display.trajectory.push_back(retreat);
      display_publisher_->publish(display);
      executing_ = true;
      state_ = Status::STATE_EXECUTING;
      message_ = "executing collision-checked stopped-pose retreat outward " +
        std::to_string(geometry.outward_distance_m * 1000.0) +
        " mm at fixed tool orientation; max joint step=" +
        std::to_string(max_delta * 180.0 / M_PI) + " deg on " + max_joint;
      publishStatusLocked();
    }

    std::string start_reason;
    if (!currentJointsMatchReviewedStart(retreat, &start_reason)) {
      std::scoped_lock lock(mutex_);
      if (!executing_) {
        response->success = false;
        response->message =
          "stopped-pose cavity retreat was stopped again during start verification";
        return;
      }
      executing_ = false;
      state_ = Status::STATE_STOPPED_IN_CAVITY;
      message_ = "stopped-pose retreat start changed; robot remains stopped: " + start_reason;
      response->success = false;
      response->message = message_;
      publishStatusLocked();
      return;
    }

    const auto result = move_group_->execute(retreat);
    std::scoped_lock lock(mutex_);
    if (!executing_) {
      response->success = false;
      response->message = "stopped-pose cavity retreat was stopped again";
      return;
    }
    executing_ = false;
    plan_available_ = false;
    retreat_available_ = false;
    if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
      state_ = Status::STATE_STOPPING_IN_CAVITY;
      stop_requested_time_ = std::chrono::steady_clock::now();
      message_ = "controller failed during stopped-pose retreat; waiting to verify idle "
        "before another retreat attempt";
      response->success = false;
      response->message = message_;
      publishStatusLocked();
      return;
    }
    if (!poseFreshLocked()) {
      state_ = Status::STATE_STOPPING_IN_CAVITY;
      stop_requested_time_ = std::chrono::steady_clock::now();
      message_ = "stopped-pose retreat returned success but Link6 pose is stale";
      response->success = false;
      response->message = message_;
      publishStatusLocked();
      return;
    }
    const PoseError target_error = poseError(
      latest_flange_pose_, geometry.target_flange_pose);
    if (target_error.translation_m > pose_tolerance_m_ ||
      target_error.rotation_rad > rotation_tolerance_rad_)
    {
      state_ = Status::STATE_STOPPED_IN_CAVITY;
      message_ = "stopped-pose retreat target verification failed; robot remains stopped; "
        "translation_error_mm=" +
        std::to_string(target_error.translation_m * 1000.0) +
        ", rotation_error_deg=" +
        std::to_string(target_error.rotation_rad * 180.0 / M_PI);
      response->success = false;
      response->message = message_;
      publishStatusLocked();
      return;
    }
    cavity_retreat_context_active_ = false;
    clearStopJointStabilityCheckLocked();
    state_ = Status::STATE_AT_PRE_ENTRY;
    message_ = "stopped-pose straight outward retreat completed and verified at the "
      "pre-entry plane; plasma remained disabled";
    response->success = true;
    response->message = message_;
    publishStatusLocked();
  }

  void onStop(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    move_group_->stop();
    std::scoped_lock lock(mutex_);
    executing_ = false;
    planning_ = false;
    plan_available_ = false;
    retreat_available_ = false;
    if (path_available_ && cavity_retreat_context_active_) {
      stop_requested_time_ = std::chrono::steady_clock::now();
      beginStopJointStabilityCheckLocked();
      state_ = Status::STATE_STOPPING_IN_CAVITY;
      message_ = "entry motion stop requested inside the cavity; waiting for fresh idle, "
        "pose, and joint confirmation before enabling straight outward retreat";
    } else {
      clearStopJointStabilityCheckLocked();
      state_ = path_available_ ? Status::STATE_READY_TO_PLAN : Status::STATE_WAITING_PATH;
      message_ = "entry planning/execution stopped outside the cavity; replan is required";
    }
    response->success = true;
    response->message = message_;
    publishStatusLocked();
  }

  void publishStatusLocked()
  {
    Status status;
    status.header.stamp = node_->now();
    status.header.frame_id = base_frame_;
    status.state = state_;
    status.path_id = path_available_ ? path_.path_id : "";
    status.motion_enabled = motion_enabled_;
    status.path_execution_permitted = path_available_ &&
      (path_.execution_permitted || allow_unvalidated_dry_run_);
    status.plan_available = plan_available_;
    status.trajectory_points = plan_available_ ? static_cast<std::int32_t>(
      plan_.trajectory_.joint_trajectory.points.size() +
      entry_preview_.joint_trajectory.points.size() +
      first_layer_preview_.joint_trajectory.points.size() +
      remaining_spray_preview_.joint_trajectory.points.size() +
      reviewed_first_layer_retreat_.joint_trajectory.points.size() +
      reviewed_retreat_.joint_trajectory.points.size()) : 0;
    status.planning_time_sec = plan_available_ ? plan_.planning_time_ : 0.0;
    if (path_available_ && have_flange_pose_) {
      const PoseError error = poseError(latest_flange_pose_, path_.pre_entry_flange_pose);
      status.pre_entry_translation_error_m = error.translation_m;
      status.pre_entry_rotation_error_rad = error.rotation_rad;
    }
    status.message = message_;
    status_publisher_->publish(status);
  }

  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  std::string path_topic_;
  std::string flange_pose_topic_;
  std::string planning_group_;
  std::string flange_link_;
  std::string base_frame_;
  std::string controller_node_name_;
  bool motion_enabled_ = false;
  bool allow_unvalidated_dry_run_ = false;
  double planning_time_sec_ = 10.0;
  int planning_attempts_ = 10;
  int complete_plan_attempts_ = 8;
  double velocity_scaling_ = 0.05;
  double acceleration_scaling_ = 0.05;
  double approach_velocity_scaling_ = 0.10;
  double approach_acceleration_scaling_ = 0.10;
  double max_plan_age_sec_ = 120.0;
  int max_pose_age_ms_ = 500;
  int max_arm_status_age_ms_ = 500;
  int max_joint_state_age_ms_ = 500;
  double joint_state_wait_sec_ = 5.0;
  double planning_position_tolerance_m_ = 0.001;
  double planning_rotation_tolerance_rad_ = 0.017453292519943295;
  double pose_tolerance_m_ = 0.003;
  double rotation_tolerance_rad_ = 0.08726646259971647;
  double cartesian_step_m_ = 0.005;
  double max_joint_step_rad_ = 0.17453292519943295;
  double reviewed_start_joint_tolerance_rad_ = 0.03490658503988659;
  double min_cartesian_fraction_ = 0.99999;
  int required_collision_stage_ = 8;
  int stop_settle_time_ms_ = 500;
  int stop_joint_stability_time_ms_ = 1000;
  double stop_joint_stability_tolerance_rad_ = 0.0008726646259971648;
  double stopped_retreat_max_axis_offset_m_ = 0.02;
  double stopped_retreat_max_distance_m_ = 0.50;

  std::mutex mutex_;
  std::condition_variable joint_state_condition_;
  SprayPath path_;
  bool path_available_ = false;
  geometry_msgs::msg::Pose latest_flange_pose_;
  std::chrono::steady_clock::time_point latest_flange_pose_time_;
  bool have_flange_pose_ = false;
  sensor_msgs::msg::JointState latest_joint_state_;
  std::chrono::steady_clock::time_point latest_joint_state_time_;
  bool have_joint_state_ = false;
  sensor_msgs::msg::JointState stop_joint_anchor_;
  std::chrono::steady_clock::time_point stop_joint_stable_since_;
  bool stop_joint_stability_active_ = false;
  bool have_stop_joint_anchor_ = false;
  Plan plan_;
  moveit_msgs::msg::RobotTrajectory entry_preview_;
  moveit_msgs::msg::RobotTrajectory first_layer_preview_;
  moveit_msgs::msg::RobotTrajectory remaining_spray_preview_;
  moveit_msgs::msg::RobotTrajectory spray_preview_;
  moveit_msgs::msg::RobotTrajectory reviewed_first_layer_retreat_;
  moveit_msgs::msg::RobotTrajectory reviewed_retreat_;
  std::size_t first_layer_point_count_ = 0;
  bool plan_available_ = false;
  bool retreat_available_ = false;
  bool cavity_retreat_context_active_ = false;
  bool executing_ = false;
  bool planning_ = false;
  std::string planned_path_id_;
  std::chrono::steady_clock::time_point plan_created_;
  std::chrono::steady_clock::time_point stop_requested_time_;
  std::uint8_t state_ = Status::STATE_WAITING_PATH;
  std::string message_;
  int collision_stage_ = -1;
  bool have_collision_stage_ = false;
  std::chrono::steady_clock::time_point collision_stage_time_;
  std::chrono::steady_clock::time_point last_collision_stage_request_;
  int arm_status_ = -1;
  bool have_arm_status_ = false;
  std::chrono::steady_clock::time_point arm_status_time_;

  rclcpp::Publisher<Status>::SharedPtr status_publisher_;
  rclcpp::Publisher<moveit_msgs::msg::DisplayTrajectory>::SharedPtr display_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr clearance_publisher_;
  rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr collision_stage_command_publisher_;
  rclcpp::AsyncParametersClient::SharedPtr controller_parameters_;
  rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr collision_stage_subscription_;
  rclcpp::Subscription<rm_ros_interfaces::msg::Armcurrentstatus>::SharedPtr
    arm_status_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscription_;
  rclcpp::Subscription<SprayPath>::SharedPtr path_subscription_;
  rclcpp::CallbackGroup::SharedPtr pose_callback_group_;
  rclcpp::CallbackGroup::SharedPtr operation_callback_group_;
  rclcpp::CallbackGroup::SharedPtr safety_callback_group_;
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr flange_pose_subscription_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr plan_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr execute_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reviewed_entry_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reviewed_first_layer_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reviewed_dry_run_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr retreat_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_service_;
  rclcpp::TimerBase::SharedPtr status_timer_;
};

}  // namespace plasma_path_executor

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("plasma_entry_motion_planner");
  auto planner = std::make_shared<plasma_path_executor::EntryMotionPlanner>(node);
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  planner.reset();
  rclcpp::shutdown();
  return 0;
}
