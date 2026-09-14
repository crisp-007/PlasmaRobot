#ifndef ROS_WORKER_H
#define ROS_WORKER_H

#include <QObject>
#include <QThread>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter_client.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <plasma_robot_interfaces/msg/spray_path.hpp>
#include <plasma_robot_interfaces/msg/path_execution_status.hpp>
#include <plasma_robot_interfaces/msg/entry_motion_status.hpp>
#include <plasma_robot_interfaces/srv/start_spray_path.hpp>
#include <atomic>
#include <array>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "arm_ros_adapter.h"

// ==========================================
// PlasmaGuiNode: 独立的 ROS2 节点类
// ==========================================
class PlasmaGuiNode : public rclcpp::Node
{
public:
    explicit PlasmaGuiNode(const std::string& name = "plasma_gui_node")
        : Node(name, rclcpp::NodeOptions().use_intra_process_comms(false))
    {
        // 创建订阅者
        // 使用 rclcpp::SensorDataQoS() 以匹配传感器数据的 QoS 设置（通常是 Best Effort）
        // 也可以根据实际情况调整 QoS
        auto qos = rclcpp::SensorDataQoS();
        
        m_subscription = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/plasma/camera/colored_points",
            qos,
            std::bind(&PlasmaGuiNode::topic_callback, this, std::placeholders::_1)
        );

        // 2) JointState 订阅（Realman Eco65-B，6 轴）
        m_jointSub = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states",
            rclcpp::SensorDataQoS(),
            [this](sensor_msgs::msg::JointState::ConstSharedPtr msg) {
                if (m_jointCb) m_jointCb(msg);
            }
        );

        // 3) 服务客户端
        m_selfCheckClient    = this->create_client<std_srvs::srv::Trigger>("/arm_self_check");
        m_emergencyStopClient = this->create_client<std_srvs::srv::Trigger>("/arm_emergency_stop");
        m_pathExecutorStartClient =
            this->create_client<plasma_robot_interfaces::srv::StartSprayPath>(
                "/plasma_path_executor/start");
        m_pathExecutorStopClient =
            this->create_client<std_srvs::srv::Trigger>("/plasma_path_executor/stop");
        m_entryPlanClient = this->create_client<std_srvs::srv::Trigger>(
            "/plasma_entry_motion_planner/plan_to_pre_entry");
        m_entryExecuteClient = this->create_client<std_srvs::srv::Trigger>(
            "/plasma_entry_motion_planner/execute_to_pre_entry");
        m_entryReviewedEntryClient = this->create_client<std_srvs::srv::Trigger>(
            "/plasma_entry_motion_planner/execute_reviewed_entry");
        m_entryReviewedFirstLayerClient = this->create_client<std_srvs::srv::Trigger>(
            "/plasma_entry_motion_planner/execute_reviewed_first_layer");
        m_entryReviewedDryRunClient = this->create_client<std_srvs::srv::Trigger>(
            "/plasma_entry_motion_planner/execute_reviewed_dry_run");
        m_entryRetreatClient = this->create_client<std_srvs::srv::Trigger>(
            "/plasma_entry_motion_planner/retreat_to_pre_entry");
        m_entryStopClient = this->create_client<std_srvs::srv::Trigger>(
            "/plasma_entry_motion_planner/stop");

        m_armAdapter = std::make_unique<ArmRosAdapter>(*this);
        m_sprayPathPublisher =
            this->create_publisher<plasma_robot_interfaces::msg::SprayPath>(
                "/plasma/planned_spray_path/camera",
                rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());
        m_pathExecutionStatusSub =
            this->create_subscription<plasma_robot_interfaces::msg::PathExecutionStatus>(
                "/plasma/path_executor/status",
                rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
                [this](plasma_robot_interfaces::msg::PathExecutionStatus::ConstSharedPtr msg) {
                    if (m_pathExecutionStatusCb)
                        m_pathExecutionStatusCb(msg);
                });
        m_entryMotionStatusSub =
            this->create_subscription<plasma_robot_interfaces::msg::EntryMotionStatus>(
                "/plasma/entry_motion/status",
                rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
                [this](plasma_robot_interfaces::msg::EntryMotionStatus::ConstSharedPtr msg) {
                    if (m_entryMotionStatusCb)
                        m_entryMotionStatusCb(msg);
                });
    }

    // ---- 回调接口 ----
    void setCallback(std::function<void(const sensor_msgs::msg::PointCloud2::ConstSharedPtr&)> callback) {
        m_callback = callback;
    }

    void applyCameraPreset(
        int visualPreset,
        std::function<void(bool, const std::string &)> callback)
    {
        if (m_cameraParameterCallback) {
            callback(false, "camera preset update is already in progress");
            return;
        }

        m_cameraParameterClient =
            std::make_shared<rclcpp::AsyncParametersClient>(
                this, "/camera/camera");
        if (!m_cameraParameterClient->service_is_ready()) {
            callback(false, "camera parameter service is not ready");
            return;
        }

        m_cameraParameterSteps = {
            rclcpp::Parameter("depth_module.visual_preset", visualPreset),
            rclcpp::Parameter("enable_color", false),
            rclcpp::Parameter("enable_depth", false),
            rclcpp::Parameter("enable_depth", true),
            rclcpp::Parameter("enable_color", true)
        };
        m_cameraParameterStep = 0;
        m_cameraParameterReadyRetries = 0;
        m_cameraParameterCallback = std::move(callback);
        applyNextCameraParameter();
    }

    void setJointCallback(std::function<void(const sensor_msgs::msg::JointState::ConstSharedPtr&)> cb) {
        m_jointCb = std::move(cb);
    }

    void setPathExecutionStatusCallback(
        std::function<void(
            const plasma_robot_interfaces::msg::PathExecutionStatus::ConstSharedPtr&)> cb) {
        m_pathExecutionStatusCb = std::move(cb);
    }

    void setEntryMotionStatusCallback(
        std::function<void(
            const plasma_robot_interfaces::msg::EntryMotionStatus::ConstSharedPtr&)> cb) {
        m_entryMotionStatusCb = std::move(cb);
    }

    void setMoveJResultCallback(ArmRosAdapter::ResultCallback cb) {
        m_armAdapter->setMoveJResultCallback(std::move(cb));
    }

    void setMoveStopResultCallback(ArmRosAdapter::ResultCallback cb) {
        m_armAdapter->setMoveStopResultCallback(std::move(cb));
    }

    void setMoveLOffsetResultCallback(ArmRosAdapter::ResultCallback cb) {
        m_armAdapter->setMoveLOffsetResultCallback(std::move(cb));
    }

    void setCurrentToolFrameCallback(ArmRosAdapter::TextCallback cb) {
        m_armAdapter->setCurrentToolFrameCallback(std::move(cb));
    }

    void publishMoveJ(const std::array<double, 6> &jointsRad, int speed) {
        m_armAdapter->publishMoveJ(jointsRad, static_cast<std::uint8_t>(speed));
    }

    void publishMoveStop() {
        m_armAdapter->publishMoveStop();
    }

    void publishToolOffset(int toolAxis, double distanceMeters, int speed) {
        m_armAdapter->publishToolOffset(toolAxis, distanceMeters,
                                        static_cast<std::uint8_t>(speed));
    }

    void requestCurrentToolFrame() {
        m_armAdapter->requestCurrentToolFrame();
    }

    bool armMotionReady() const {
        return m_armAdapter && m_armAdapter->moveJReady();
    }

    bool armLinearMotionReady() const {
        return m_armAdapter && m_armAdapter->moveLOffsetReady();
    }

    bool armToolFrameQueryReady() const {
        return m_armAdapter && m_armAdapter->toolFrameQueryReady();
    }

    void publishSprayPath(const plasma_robot_interfaces::msg::SprayPath &path) {
        if (m_sprayPathPublisher)
            m_sprayPathPublisher->publish(path);
    }

    bool pathExecutorStartReady() const {
        return m_pathExecutorStartClient && m_pathExecutorStartClient->service_is_ready();
    }

    bool pathExecutorStopReady() const {
        return m_pathExecutorStopClient && m_pathExecutorStopClient->service_is_ready();
    }

    bool entryPlanReady() const {
        return m_entryPlanClient && m_entryPlanClient->service_is_ready();
    }

    bool entryExecuteReady() const {
        return m_entryExecuteClient && m_entryExecuteClient->service_is_ready();
    }

    bool entryReviewedDryRunReady() const {
        return m_entryReviewedDryRunClient && m_entryReviewedDryRunClient->service_is_ready();
    }

    bool entryReviewedFirstLayerReady() const {
        return m_entryReviewedFirstLayerClient &&
            m_entryReviewedFirstLayerClient->service_is_ready();
    }

    bool entryReviewedEntryReady() const {
        return m_entryReviewedEntryClient && m_entryReviewedEntryClient->service_is_ready();
    }

    bool entryRetreatReady() const {
        return m_entryRetreatClient && m_entryRetreatClient->service_is_ready();
    }

    bool callEntryPlan(std::function<void(bool, const std::string&)> cb) {
        return callTrigger(m_entryPlanClient, std::move(cb));
    }

    bool callEntryExecute(std::function<void(bool, const std::string&)> cb) {
        return callTrigger(m_entryExecuteClient, std::move(cb));
    }

    bool callEntryReviewedDryRun(std::function<void(bool, const std::string&)> cb) {
        return callTrigger(m_entryReviewedDryRunClient, std::move(cb));
    }

    bool callEntryReviewedFirstLayer(
        std::function<void(bool, const std::string&)> cb) {
        return callTrigger(m_entryReviewedFirstLayerClient, std::move(cb));
    }

    bool callEntryReviewedEntry(std::function<void(bool, const std::string&)> cb) {
        return callTrigger(m_entryReviewedEntryClient, std::move(cb));
    }

    bool callEntryRetreat(std::function<void(bool, const std::string&)> cb) {
        return callTrigger(m_entryRetreatClient, std::move(cb));
    }

    bool callEntryStop(std::function<void(bool, const std::string&)> cb) {
        return callTrigger(m_entryStopClient, std::move(cb));
    }

    bool startSprayPath(const std::string &pathId, bool dryRun,
                        bool approachFromConfirmedEntry,
                        std::function<void(bool, const std::string&)> cb) {
        if (!pathExecutorStartReady())
            return false;
        auto request =
            std::make_shared<plasma_robot_interfaces::srv::StartSprayPath::Request>();
        request->path_id = pathId;
        request->dry_run = dryRun;
        request->approach_from_confirmed_entry = approachFromConfirmedEntry;
        m_pathExecutorStartClient->async_send_request(
            request,
            [callback = std::move(cb)](
                rclcpp::Client<plasma_robot_interfaces::srv::StartSprayPath>::SharedFuture future) {
                try {
                    const auto response = future.get();
                    callback(response->accepted, response->message);
                } catch (const std::exception &error) {
                    callback(false, error.what());
                }
            });
        return true;
    }

    bool stopSprayPath(std::function<void(bool, const std::string&)> cb) {
        if (!pathExecutorStopReady())
            return false;
        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
        m_pathExecutorStopClient->async_send_request(
            request,
            [callback = std::move(cb)](
                rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
                try {
                    const auto response = future.get();
                    callback(response->success, response->message);
                } catch (const std::exception &error) {
                    callback(false, error.what());
                }
            });
        return true;
    }

    // ---- 服务调用 ----
    void callEmergencyStop(std::function<void(bool, const std::string&)> cb) {
        auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
        m_emergencyStopClient->async_send_request(req,
            [cb](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
                cb(future.get()->success, future.get()->message);
            });
    }

    void callSelfCheck(std::function<void(bool, const std::string&)> cb) {
        auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
        m_selfCheckClient->async_send_request(req,
            [cb](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
                cb(future.get()->success, future.get()->message);
            });
    }

private:
    void applyNextCameraParameter()
    {
        if (m_cameraParameterStep >= m_cameraParameterSteps.size()) {
            auto callback = std::move(m_cameraParameterCallback);
            m_cameraParameterSteps.clear();
            if (callback)
                callback(true, "camera preset applied and streams restarted");
            return;
        }

        const rclcpp::Parameter parameter =
            m_cameraParameterSteps[m_cameraParameterStep];
        m_cameraParameterClient->set_parameters(
            {parameter},
            [this, parameter](
                std::shared_future<std::vector<
                    rcl_interfaces::msg::SetParametersResult>> future) {
                const auto results = future.get();
                if (results.empty() || !results.front().successful) {
                    const std::string reason = results.empty()
                        ? "empty parameter response"
                        : results.front().reason;
                    const bool cameraStillInitializing =
                        m_cameraParameterStep == 0 &&
                        reason.find("not declared") != std::string::npos &&
                        m_cameraParameterReadyRetries < 15;
                    if (cameraStillInitializing) {
                        ++m_cameraParameterReadyRetries;
                        m_cameraParameterTimer = create_wall_timer(
                            std::chrono::seconds(1),
                            [this]() {
                                m_cameraParameterTimer->cancel();
                                m_cameraParameterTimer.reset();
                                applyNextCameraParameter();
                            });
                        return;
                    }

                    auto callback = std::move(m_cameraParameterCallback);
                    m_cameraParameterSteps.clear();
                    if (callback) {
                        callback(
                            false,
                            "failed to set " + parameter.get_name() +
                                ": " + reason);
                    }
                    return;
                }
                ++m_cameraParameterStep;
                // The L515 V4L2 backend reports a successful parameter update
                // before its UVC sensor has fully stopped or restarted. Sending
                // the next toggle immediately can leave enable_* true while no
                // frames are produced, so serialize the operations with a short
                // settling interval.
                m_cameraParameterTimer = create_wall_timer(
                    std::chrono::seconds(2),
                    [this]() {
                        m_cameraParameterTimer->cancel();
                        m_cameraParameterTimer.reset();
                        applyNextCameraParameter();
                    });
            });
    }

    bool callTrigger(
        const rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr &client,
        std::function<void(bool, const std::string&)> callback) {
        if (!client || !client->service_is_ready())
            return false;
        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
        client->async_send_request(
            request,
            [callback = std::move(callback)](
                rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
                try {
                    const auto response = future.get();
                    callback(response->success, response->message);
                } catch (const std::exception &error) {
                    callback(false, error.what());
                }
            });
        return true;
    }

    void topic_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) const
    {
        if (m_callback) {
            m_callback(msg);
        }
    }

    // ---- 订阅者 ----
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr m_subscription;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr  m_jointSub;
    rclcpp::Subscription<plasma_robot_interfaces::msg::PathExecutionStatus>::SharedPtr
        m_pathExecutionStatusSub;
    rclcpp::Subscription<plasma_robot_interfaces::msg::EntryMotionStatus>::SharedPtr
        m_entryMotionStatusSub;

    // ---- 服务客户端 ----
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr m_selfCheckClient;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr m_emergencyStopClient;
    rclcpp::Client<plasma_robot_interfaces::srv::StartSprayPath>::SharedPtr
        m_pathExecutorStartClient;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr m_pathExecutorStopClient;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr m_entryPlanClient;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr m_entryExecuteClient;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr m_entryReviewedEntryClient;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr m_entryReviewedFirstLayerClient;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr m_entryReviewedDryRunClient;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr m_entryRetreatClient;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr m_entryStopClient;
    std::unique_ptr<ArmRosAdapter> m_armAdapter;
    std::shared_ptr<rclcpp::AsyncParametersClient> m_cameraParameterClient;
    std::vector<rclcpp::Parameter> m_cameraParameterSteps;
    std::size_t m_cameraParameterStep = 0;
    int m_cameraParameterReadyRetries = 0;
    std::function<void(bool, const std::string &)> m_cameraParameterCallback;
    rclcpp::TimerBase::SharedPtr m_cameraParameterTimer;
    rclcpp::Publisher<plasma_robot_interfaces::msg::SprayPath>::SharedPtr
        m_sprayPathPublisher;

    // ---- 回调 ----
    std::function<void(const sensor_msgs::msg::PointCloud2::ConstSharedPtr&)> m_callback;
    std::function<void(const sensor_msgs::msg::JointState::ConstSharedPtr&)>  m_jointCb;
    std::function<void(
        const plasma_robot_interfaces::msg::PathExecutionStatus::ConstSharedPtr&)>
        m_pathExecutionStatusCb;
    std::function<void(
        const plasma_robot_interfaces::msg::EntryMotionStatus::ConstSharedPtr&)>
        m_entryMotionStatusCb;
};

// ==========================================
// RosWorker: 负责 ROS2 线程和执行器管理
// ==========================================
class RosWorker : public QObject
{
    Q_OBJECT
public:
    explicit RosWorker(QObject *parent = nullptr) : QObject(parent), m_paused(false) {
        // 初始化 ROS2 上下文（如果尚未初始化）
        if (!rclcpp::ok()) {
            int argc = 0;
            char **argv = nullptr;
            rclcpp::init(argc, argv);
        }
    }

    ~RosWorker() {
        if (rclcpp::ok()) {
            rclcpp::shutdown();
        }
    }

public slots:
    void start() {
        // 1. 创建节点
        m_node = std::make_shared<PlasmaGuiNode>();

        // 2. 设置回调：收到 ROS 消息后发射 Qt 信号
        m_node->setCallback([this](const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg) {
            if (!m_paused) {
                emit cloudReceived(msg);
            }
        });
        m_node->setJointCallback([this](const sensor_msgs::msg::JointState::ConstSharedPtr& msg) {
            emit jointStateReceived(msg);
        });
        m_node->setPathExecutionStatusCallback(
            [this](const plasma_robot_interfaces::msg::PathExecutionStatus::ConstSharedPtr &msg) {
                if (!msg)
                    return;
                emit pathExecutionStatusReceived(
                    static_cast<int>(msg->state), QString::fromStdString(msg->path_id),
                    msg->current_index, msg->total_points, msg->motion_enabled,
                    msg->path_execution_permitted, msg->driver_ready,
                    msg->entry_pose_valid, msg->plasma_output_enabled,
                    QString::fromStdString(msg->message));
            });
        m_node->setEntryMotionStatusCallback(
            [this](const plasma_robot_interfaces::msg::EntryMotionStatus::ConstSharedPtr &msg) {
                if (!msg)
                    return;
                emit entryMotionStatusReceived(
                    static_cast<int>(msg->state), QString::fromStdString(msg->path_id),
                    msg->motion_enabled, msg->path_execution_permitted,
                    msg->plan_available, msg->trajectory_points,
                    msg->pre_entry_translation_error_m,
                    msg->pre_entry_rotation_error_rad,
                    QString::fromStdString(msg->message));
            });
        m_node->setMoveJResultCallback([this](bool ok) {
            emit moveJResultReceived(ok);
        });
        m_node->setMoveLOffsetResultCallback([this](bool ok) {
            emit moveLOffsetResultReceived(ok);
        });
        m_node->setMoveStopResultCallback([this](bool ok) {
            emit moveStopResultReceived(ok);
        });
        m_node->setCurrentToolFrameCallback([this](const std::string &frameName) {
            emit currentToolFrameReceived(QString::fromStdString(frameName));
        });
        // 3. 创建多线程执行器
        // 允许并行处理回调（如果节点中有多个订阅者或回调组）
        m_executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
        m_executor->add_node(m_node);

        // 4. 开始阻塞运行 (Spin)
        // 注意：此函数会阻塞当前线程（即 m_workerThread），直到 rclcpp::shutdown 被调用
        m_executor->spin();
    }

    void stop() {
        if (rclcpp::ok()) {
            rclcpp::shutdown();
        }
    }

    void setPaused(bool paused) {
        m_paused = paused;
    }

    std::shared_ptr<PlasmaGuiNode> node() const { return m_node; }

signals:
    void cloudReceived(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg);
    void jointStateReceived(const sensor_msgs::msg::JointState::ConstSharedPtr& msg);
    void moveJResultReceived(bool ok);
    void moveLOffsetResultReceived(bool ok);
    void moveStopResultReceived(bool ok);
    void currentToolFrameReceived(const QString &frameName);
    void pathExecutionStatusReceived(int state, const QString &pathId,
                                     int currentIndex, int totalPoints,
                                     bool motionEnabled, bool pathPermitted,
                                     bool driverReady, bool entryPoseValid,
                                     bool plasmaOutputEnabled,
                                     const QString &message);
    void entryMotionStatusReceived(int state, const QString &pathId,
                                   bool motionEnabled, bool pathPermitted,
                                   bool planAvailable, int trajectoryPoints,
                                   double translationError,
                                   double rotationError,
                                   const QString &message);

private:
    std::shared_ptr<PlasmaGuiNode> m_node;
    std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> m_executor;
    std::atomic<bool> m_paused{false};
};

#endif // ROS_WORKER_H
