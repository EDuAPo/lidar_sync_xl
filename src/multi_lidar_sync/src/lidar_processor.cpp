#include "multi_lidar_sync/lidar_processor.hpp"
#include <rclcpp/rclcpp.hpp>

namespace multi_lidar_sync
{

LidarProcessor::LidarProcessor(
    rclcpp::Node::SharedPtr node,
    const LidarConfig& config)
    : node_(node)
    , config_(config)
    , is_running_(false)
    , frame_sequence_(0)
{
    // 创建缓冲区管理器
    buffer_manager = std::make_shared<LidarBufferManager>();
    
    RCLCPP_INFO(node_->get_logger(),
        "Initialized LiDAR processor: %s (ID: %d, Topic: %s)",
        config_.name.c_str(), config_.id, config_.topic.c_str());
}

LidarProcessor::~LidarProcessor()
{
    stop();
}

bool LidarProcessor::start()
{
    if (is_running_.load())
    {
        return true;
    }
    
    if (!config_.enabled)
    {
        RCLCPP_WARN(node_->get_logger(),
            "LiDAR %s is disabled in config", config_.name.c_str());
        return false;
    }
    
    // 创建订阅者 - 使用 RELIABLE QoS 以兼容 rosbag 播放
    // 同时也兼容 best_effort 的实时数据源
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
    qos.reliability(rclcpp::ReliabilityPolicy::Reliable);
    qos.durability(rclcpp::DurabilityPolicy::Volatile);
    
    sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
        config_.topic,
        qos,
        std::bind(&LidarProcessor::pointCloudCallback, this, std::placeholders::_1));
    
    is_running_.store(true);
    
    RCLCPP_INFO(node_->get_logger(),
        "Started LiDAR processor: %s", config_.name.c_str());
    
    return true;
}

void LidarProcessor::stop()
{
    if (!is_running_.load())
    {
        return;
    }
    
    is_running_.store(false);
    
    // 重置订阅者
    sub_.reset();
    
    RCLCPP_INFO(node_->get_logger(),
        "Stopped LiDAR processor: %s", config_.name.c_str());
}

void LidarProcessor::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    if (!is_running_.load())
    {
        return;
    }
    
    // 构造激光雷达帧
    LidarFrame frame;
    frame.cloud_msg = msg;
    
    // 从消息头获取时间戳
    frame.timestamp_ns = msg->header.stamp.sec * 1000000000ULL + msg->header.stamp.nanosec;
    
    // 分配序号
    frame.sequence = frame_sequence_.fetch_add(1, std::memory_order_relaxed);
    
    frame.valid = true;
    frame.lidar_id = config_.id;
    
    RCLCPP_DEBUG_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
        "LiDAR %s received frame seq=%u ts=%lu ns",
        config_.name.c_str(), frame.sequence, frame.timestamp_ns);
    
    // 写入缓冲区
    if (!buffer_manager->setFrame(frame))
    {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
            "Failed to set frame for LiDAR %s", config_.name.c_str());
    }
}

}  // namespace multi_lidar_sync
