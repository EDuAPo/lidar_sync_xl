#ifndef MULTI_LIDAR_SYNC__MULTI_LIDAR_PROCESSOR_HPP_
#define MULTI_LIDAR_SYNC__MULTI_LIDAR_PROCESSOR_HPP_

#include <vector>
#include <memory>
#include <thread>
#include <atomic>
#include <mutex>
#include <condition_variable>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "multi_lidar_sync/lidar_processor.hpp"
#include "multi_lidar_sync/lidar_types.hpp"

namespace multi_lidar_sync
{

/**
 * @brief 多激光雷达同步处理器
 * 
 * 核心功能：
 * 1. 管理多个激光雷达处理器
 * 2. 实现时间戳同步算法
 * 3. 判断数据有效性
 * 4. 丢弃不同步的数据
 * 5. 提供同步后的数据接口
 */
class MultiLidarProcessor
{
public:
    /**
     * @brief 构造函数
     * @param node ROS2节点指针
     * @param configs 激光雷达配置列表
     * @param settings 全局设置
     */
    MultiLidarProcessor(
        rclcpp::Node::SharedPtr node,
        const std::vector<LidarConfig>& configs,
        const GlobalSettings& settings);
    
    /**
     * @brief 析构函数
     */
    ~MultiLidarProcessor();
    
    /**
     * @brief 启动所有激光雷达
     * @return 成功返回true
     */
    bool startAll();
    
    /**
     * @brief 停止所有激光雷达
     */
    void stopAll();
    
    /**
     * @brief 获取同步后的数据
     * @param sync_frames 输出同步帧列表
     * @return 如果有同步数据返回true，否则返回false
     */
    bool getSyncData(std::vector<SyncedLidarPacket>& synced_packets);
    
    /**
     * @brief 释放同步数据
     * 必须与 getSyncData() 配对调用
     */
    void releaseSyncData();
    
    /**
     * @brief 获取激光雷达数量
     */
    size_t getLidarCount() const { return lidars_.size(); }
    
    /**
     * @brief 获取诊断信息
     */
    std::string getDiagnostics() const;

private:
    /**
     * @brief 同步主循环（后台线程）
     */
    void syncLoop();
    
    /**
     * @brief 轮询所有激光雷达状态
     */
    void pollLidarStates();
    
    /**
     * @brief 判断是否找到同步集
     * @param tolerance_ns 时间容差（纳秒）
     * @return 找到返回true
     */
    bool findSynchronizedSet(uint64_t tolerance_ns);
    
    /**
     * @brief 获取同步数据
     * @return 成功返回true
     */
    bool acquireData();
    
    /**
     * @brief 释放数据
     */
    void releaseData();
    
    rclcpp::Node::SharedPtr node_;                              // ROS2节点
    GlobalSettings settings_;                                   // 全局设置
    std::vector<std::shared_ptr<LidarProcessor>> lidars_;       // 激光雷达处理器列表
    std::vector<FrameState> frame_states_;                      // 帧状态列表
    std::vector<uint32_t> last_processed_seq_;                  // 上次处理的序号
    std::vector<SyncedLidarPacket> synced_packets_;             // 同步后用于融合的数据
    std::vector<LidarFrame*> locked_buffers_;                   // 当前锁定的底层缓冲指针
    
    std::atomic<bool> is_running_;                              // 运行状态
    std::atomic<bool> is_ready_;                                // 数据就绪标志
    std::thread sync_thread_;                                   // 同步线程
    std::mutex data_access_mutex_;                              // 数据访问互斥锁
    std::mutex ready_mutex_;                                    // 数据就绪互斥锁
    std::condition_variable ready_cv_;                          // 通知就绪/释放
    uint64_t last_sync_delta_ns_ = 0;                           // 最近一次时间跨度
    
    // 统计信息
    std::atomic<uint64_t> sync_count_;                          // 同步成功次数
    std::atomic<uint64_t> drop_count_;                          // 丢弃帧次数
};

}  // namespace multi_lidar_sync

#endif  // MULTI_LIDAR_SYNC__MULTI_LIDAR_PROCESSOR_HPP_
