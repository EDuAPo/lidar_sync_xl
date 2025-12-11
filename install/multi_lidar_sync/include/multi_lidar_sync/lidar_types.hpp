#ifndef MULTI_LIDAR_SYNC__LIDAR_TYPES_HPP_
#define MULTI_LIDAR_SYNC__LIDAR_TYPES_HPP_

#include <memory>
#include <string>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace multi_lidar_sync
{

/**
 * @brief 激光雷达帧数据结构
 */
struct LidarFrame
{
    sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg;  // 点云消息
    uint64_t timestamp_ns;                                // 时间戳（纳秒）
    uint32_t sequence;                                    // 帧序号
    bool valid;                                           // 数据有效标志
    int lidar_id;                                         // 激光雷达ID
    
    LidarFrame()
        : cloud_msg(nullptr)
        , timestamp_ns(0)
        , sequence(0)
        , valid(false)
        , lidar_id(-1)
    {}
};

/**
 * @brief 激光雷达配置
 */
struct LidarConfig
{
    int id;                 // 激光雷达ID
    std::string name;       // 激光雷达名称
    std::string topic;      // 订阅的话题
    std::string frame_id;   // 坐标系ID
    bool enabled;           // 是否启用
    
    LidarConfig()
        : id(-1)
        , name("")
        , topic("")
        , frame_id("")
        , enabled(true)
    {}
};

/**
 * @brief 全局设置
 */
struct GlobalSettings
{
    int lidar_count;                    // 激光雷达数量
    double sync_tolerance_ms;           // 同步容差（毫秒）
    int max_queue_size;                 // 队列大小
    double publish_rate_hz;             // 发布频率
    bool enable_diagnostics;            // 启用诊断
    bool drop_unsync_frames;            // 丢弃不同步帧
    bool publish_fused_cloud;           // 是否发布融合点云
    std::string fused_topic;            // 融合点云话题
    std::string fused_frame_id;         // 融合坐标系
    bool enable_camera_sync;            // 是否推送相机同步信号
    std::string camera_sync_topic;      // 相机同步话题
    double camera_sync_tolerance_ms;    // 相机同步容差
    
    GlobalSettings()
        : lidar_count(5)
        , sync_tolerance_ms(10.0)
        , max_queue_size(10)
        , publish_rate_hz(10.0)
        , enable_diagnostics(true)
        , drop_unsync_frames(true)
        , publish_fused_cloud(false)
        , fused_topic("/fusion/points")
        , fused_frame_id("fusion_base")
        , enable_camera_sync(false)
        , camera_sync_topic("/fusion/camera_sync")
        , camera_sync_tolerance_ms(5.0)
    {}
};

/**
 * @brief 帧状态枚举
 */
enum class FrameStatus
{
    EMPTY,      // 无数据
    NEW,        // 新数据
    STALE       // 旧数据
};

/**
 * @brief 帧状态信息
 */
struct FrameState
{
    uint64_t timestamp;
    uint32_t sequence;
    FrameStatus status;
    
    FrameState()
        : timestamp(0)
        , sequence(0)
        , status(FrameStatus::EMPTY)
    {}
};

/**
 * @brief 同步后的点云包，便于上层融合/相机同步
 */
struct SyncedLidarPacket
{
    int lidar_id;
    std::string frame_id;
    uint64_t timestamp_ns;
    uint32_t sequence;
    sensor_msgs::msg::PointCloud2::SharedPtr cloud;

    SyncedLidarPacket()
        : lidar_id(-1)
        , frame_id("")
        , timestamp_ns(0)
        , sequence(0)
        , cloud(nullptr)
    {}
};

}  // namespace multi_lidar_sync

#endif  // MULTI_LIDAR_SYNC__LIDAR_TYPES_HPP_
