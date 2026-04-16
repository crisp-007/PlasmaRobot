#ifndef ROS_WORKER_H
#define ROS_WORKER_H

#include <QObject>
#include <QThread>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <atomic>

// ==========================================
// PlasmaGuiNode: 独立的 ROS2 节点类
// ==========================================
class PlasmaGuiNode : public rclcpp::Node
{
public:
    explicit PlasmaGuiNode(const std::string& name = "plasma_gui_node")
        : Node(name, rclcpp::NodeOptions().use_intra_process_comms(true))
    {
        // 创建订阅者
        // 使用 rclcpp::SensorDataQoS() 以匹配传感器数据的 QoS 设置（通常是 Best Effort）
        // 也可以根据实际情况调整 QoS
        auto qos = rclcpp::SensorDataQoS();
        
        m_subscription = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/camera/depth/color/points",
            qos,
            std::bind(&PlasmaGuiNode::topic_callback, this, std::placeholders::_1)
        );
    }

    // 设置回调函数接口，用于将数据传出到 Qt 世界
    void setCallback(std::function<void(const sensor_msgs::msg::PointCloud2::ConstSharedPtr&)> callback) {
        m_callback = callback;
    }

private:
    void topic_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) const
    {
        if (m_callback) {
            m_callback(msg);
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr m_subscription;
    std::function<void(const sensor_msgs::msg::PointCloud2::ConstSharedPtr&)> m_callback;
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

signals:
    void cloudReceived(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg);

private:
    std::shared_ptr<PlasmaGuiNode> m_node;
    std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> m_executor;
    std::atomic<bool> m_paused{false};
};

#endif // ROS_WORKER_H
