#ifndef MULTI_LIDAR_SYNC__LIDAR_PROCESSOR_HPP_
#define MULTI_LIDAR_SYNC__LIDAR_PROCESSOR_HPP_

#include <memory>
#include <thread>
#include <atomic>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "multi_lidar_sync/lidar_buffer_manager.hpp"
#include "multi_lidar_sync/lidar_types.hpp"

namespace multi_lidar_sync
{

/**
 * @brief 单个激光雷达处理器
 * 
 * 负责单个激光雷达的数据接收和缓冲管理：
 * - 订阅激光雷达话题
 * - 接收点云数据并写入三缓冲区
 * - 提供数据访问接口
 */
class LidarProcessor
{
public:
    /**
     * @brief 构造函数
     * @param node ROS2节点指针
     * @param config 激光雷达配置
     */
    LidarProcessor(
        rclcpp::Node::SharedPtr node,
        const LidarConfig& config);
    
    /**
     * @brief 析构函数
     */
    ~LidarProcessor();
    
    /**
     * @brief 启动激光雷达处理器
     * @return 成功返回true
     */
    bool start();
    
    /**
     * @brief 停止激光雷达处理器
     */
    void stop();
    
    /**
     * @brief 获取激光雷达ID
     */
    int getLidarId() const { return config_.id; }
    
    /**
     * @brief 获取激光雷达名称
     */
    std::string getLidarName() const { return config_.name; }
    
    /**
     * @brief 获取是否启用
     */
    bool isEnabled() const { return config_.enabled; }

    /**
     * @brief 获取雷达坐标系（用于融合）
     */
    std::string getFrameId() const { return config_.frame_id; }

    /**
     * @brief 访问底层配置
     */
    const LidarConfig& getConfig() const { return config_; }
    
    // 缓冲区管理器（公开访问，供同步管理器使用）
    std::shared_ptr<LidarBufferManager> buffer_manager;

private:
    /**
     * @brief 点云数据回调函数
     * @param msg 点云消息
     */
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    
    rclcpp::Node::SharedPtr node_;                                  // ROS2节点
    LidarConfig config_;                                            // 配置
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;  // 订阅者
    std::atomic<bool> is_running_;                                  // 运行状态
    std::atomic<uint32_t> frame_sequence_;                          // 帧序号计数器
};

}  // namespace multi_lidar_sync

#endif  // MULTI_LIDAR_SYNC__LIDAR_PROCESSOR_HPP_
