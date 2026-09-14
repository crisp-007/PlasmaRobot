#ifndef ROS_WORKER_H
#define ROS_WORKER_H

#include <QObject>
#include <QThread>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/trigger.hpp>

// ==========================================
// PlasmaGuiNode: 独立的 ROS2 节点类
// ==========================================
class PlasmaGuiNode : public rclcpp::Node
{
public:
    explicit PlasmaGuiNode(const std::string& name = "plasma_gui_node")
        : Node(name)
    {
        auto qos = rclcpp::SensorDataQoS();

        // 1) 点云订阅
        m_cloudSub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/camera/depth/color/points",
            qos,
            std::bind(&PlasmaGuiNode::cloud_callback, this, std::placeholders::_1)
        );

        // 2) 关节状态订阅（Realman Eco65-B，6 轴）
        m_jointSub = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states",
            rclcpp::SensorDataQoS(),
            std::bind(&PlasmaGuiNode::joint_callback, this, std::placeholders::_1)
        );

        // 3) 自检状态服务客户端
        m_selfCheckClient = this->create_client<std_srvs::srv::Trigger>("/arm_self_check");
    }

    // ---- 回调接口（Qt 世界） ----
    void setCloudCallback(std::function<void(const sensor_msgs::msg::PointCloud2::ConstSharedPtr&)> cb) {
        m_cloudCb = cb;
    }
    void setJointCallback(std::function<void(const sensor_msgs::msg::JointState::ConstSharedPtr&)> cb) {
        m_jointCb = cb;
    }

    // 异步调用自检服务
    void callSelfCheck(std::function<void(bool, const std::string&)> doneCb) {
        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
        m_selfCheckClient->async_send_request(request,
            [this, doneCb](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
                try {
                    auto resp = future.get();
                    if (doneCb) doneCb(resp->success, resp->message);
                } catch (const std::exception &e) {
                    if (doneCb) doneCb(false, std::string("Service call failed: ") + e.what());
                }
            });
    }

    // 异步发送急停服务
    void callEmergencyStop(std::function<void(bool, const std::string&)> doneCb) {
        if (!m_emergencyStopClient) {
            m_emergencyStopClient = this->create_client<std_srvs::srv::Trigger>("/arm_emergency_stop");
        }
        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
        m_emergencyStopClient->async_send_request(request,
            [doneCb](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
                try {
                    auto resp = future.get();
                    if (doneCb) doneCb(resp->success, resp->message);
                } catch (const std::exception &e) {
                    if (doneCb) doneCb(false, std::string("Service call failed: ") + e.what());
                }
            });
    }

private:
    // ---- 点云回调 ----
    void cloud_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) const {
        if (m_cloudCb) m_cloudCb(msg);
    }

    // ---- 关节状态回调 ----
    void joint_callback(const sensor_msgs::msg::JointState::ConstSharedPtr msg) const {
        if (m_jointCb) m_jointCb(msg);
    }

    // ---- 订阅 ----
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr m_cloudSub;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr  m_jointSub;

    // ---- 服务客户端 ----
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr m_selfCheckClient;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr m_emergencyStopClient;

    // ---- Qt 回调 ----
    std::function<void(const sensor_msgs::msg::PointCloud2::ConstSharedPtr&)> m_cloudCb;
    std::function<void(const sensor_msgs::msg::JointState::ConstSharedPtr&)>  m_jointCb;
};

// ==========================================
// RosWorker: 负责 ROS2 线程和执行器管理
// ==========================================
class RosWorker : public QObject
{
    Q_OBJECT
public:
    explicit RosWorker(QObject *parent = nullptr) : QObject(parent) {
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

    /// 获取底层节点指针，供外部直接调用服务
    std::shared_ptr<PlasmaGuiNode> node() const { return m_node; }

public slots:
    void start() {
        m_node = std::make_shared<PlasmaGuiNode>();

        // 点云数据
        m_node->setCloudCallback([this](const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg) {
            emit cloudReceived(msg);
        });

        // 关节状态数据
        m_node->setJointCallback([this](const sensor_msgs::msg::JointState::ConstSharedPtr& msg) {
            emit jointStateReceived(msg);
        });

        m_executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
        m_executor->add_node(m_node);
        m_executor->spin();
    }

    void stop() {
        if (rclcpp::ok()) {
            rclcpp::shutdown();
        }
    }

signals:
    void cloudReceived(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg);
    void jointStateReceived(const sensor_msgs::msg::JointState::ConstSharedPtr& msg);

private:
    std::shared_ptr<PlasmaGuiNode> m_node;
    std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> m_executor;
};

#endif // ROS_WORKER_H
