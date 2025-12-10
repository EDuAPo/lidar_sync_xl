#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <limits>
#include <exception>
#include "multi_lidar_sync/multi_lidar_processor.hpp"
#include "multi_lidar_sync/lidar_types.hpp"

namespace multi_lidar_sync
{

class MultiLidarSyncNode : public rclcpp::Node
{
public:
    MultiLidarSyncNode()
        : Node("multi_lidar_sync_node")
    {
        // 加载参数
        loadParameters();
        
        // 创建多激光雷达处理器
        processor_ = std::make_shared<MultiLidarProcessor>(
            shared_from_this(), lidar_configs_, global_settings_);
        
        // 启动所有激光雷达
        if (!processor_->startAll())
        {
            RCLCPP_ERROR(get_logger(), "Failed to start multi-lidar processor!");
            return;
        }

        if (global_settings_.publish_fused_cloud)
        {
            auto qos = rclcpp::QoS(rclcpp::KeepLast(global_settings_.max_queue_size));
            qos.best_effort();
            fused_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
                global_settings_.fused_topic,
                qos);
        }

        if (global_settings_.enable_camera_sync)
        {
            auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
            camera_sync_pub_ = create_publisher<std_msgs::msg::Header>(
                global_settings_.camera_sync_topic,
                qos);
        }
        
        // 创建处理定时器
        auto period = std::chrono::milliseconds(
            static_cast<int>(1000.0 / global_settings_.publish_rate_hz));
        
        processing_timer_ = create_wall_timer(
            period,
            std::bind(&MultiLidarSyncNode::processingCallback, this));
        
        // 创建诊断定时器
        if (global_settings_.enable_diagnostics)
        {
            diagnostics_timer_ = create_wall_timer(
                std::chrono::seconds(5),
                std::bind(&MultiLidarSyncNode::diagnosticsCallback, this));
        }
        
        RCLCPP_INFO(get_logger(), "Multi-LiDAR sync node initialized successfully");
    }
    
    ~MultiLidarSyncNode()
    {
        if (processor_)
        {
            processor_->stopAll();
        }
    }

private:
    void loadParameters()
    {
        // 声明并获取全局参数
        declare_parameter("global_settings.lidar_count", 5);
        declare_parameter("global_settings.sync_tolerance_ms", 10.0);
        declare_parameter("global_settings.max_queue_size", 10);
        declare_parameter("global_settings.publish_rate_hz", 10.0);
        declare_parameter("global_settings.enable_diagnostics", true);
        declare_parameter("global_settings.drop_unsync_frames", true);
        declare_parameter("global_settings.publish_fused_cloud", false);
        declare_parameter("global_settings.fused_topic", "/fusion/points");
        declare_parameter("global_settings.fused_frame_id", "fusion_base");
        declare_parameter("global_settings.enable_camera_sync", false);
        declare_parameter("global_settings.camera_sync_topic", "/fusion/camera_sync");
        declare_parameter("global_settings.camera_sync_tolerance_ms", 5.0);
        
        global_settings_.lidar_count = get_parameter("global_settings.lidar_count").as_int();
        global_settings_.sync_tolerance_ms = get_parameter("global_settings.sync_tolerance_ms").as_double();
        global_settings_.max_queue_size = get_parameter("global_settings.max_queue_size").as_int();
        global_settings_.publish_rate_hz = get_parameter("global_settings.publish_rate_hz").as_double();
        global_settings_.enable_diagnostics = get_parameter("global_settings.enable_diagnostics").as_bool();
        global_settings_.drop_unsync_frames = get_parameter("global_settings.drop_unsync_frames").as_bool();
        global_settings_.publish_fused_cloud = get_parameter("global_settings.publish_fused_cloud").as_bool();
        global_settings_.fused_topic = get_parameter("global_settings.fused_topic").as_string();
        global_settings_.fused_frame_id = get_parameter("global_settings.fused_frame_id").as_string();
        global_settings_.enable_camera_sync = get_parameter("global_settings.enable_camera_sync").as_bool();
        global_settings_.camera_sync_topic = get_parameter("global_settings.camera_sync_topic").as_string();
        global_settings_.camera_sync_tolerance_ms = get_parameter("global_settings.camera_sync_tolerance_ms").as_double();
        
        // 加载激光雷达配置
        for (int i = 0; i < global_settings_.lidar_count; ++i)
        {
            std::string prefix = "lidars." + std::to_string(i) + ".";
            
            declare_parameter(prefix + "id", i);
            declare_parameter(prefix + "name", "lidar_" + std::to_string(i));
            declare_parameter(prefix + "topic", "/lidar_" + std::to_string(i) + "/points");
            declare_parameter(prefix + "frame_id", "lidar_" + std::to_string(i));
            declare_parameter(prefix + "enabled", true);
            
            LidarConfig config;
            config.id = get_parameter(prefix + "id").as_int();
            config.name = get_parameter(prefix + "name").as_string();
            config.topic = get_parameter(prefix + "topic").as_string();
            config.frame_id = get_parameter(prefix + "frame_id").as_string();
            config.enabled = get_parameter(prefix + "enabled").as_bool();
            
            lidar_configs_.push_back(config);
            
            RCLCPP_INFO(get_logger(), "Loaded LiDAR config: %s -> %s",
                config.name.c_str(), config.topic.c_str());
        }
    }
    
    void processingCallback()
    {
        std::vector<SyncedLidarPacket> sync_packets;
        
        // 尝试获取同步数据
        if (processor_->getSyncData(sync_packets))
        {
            // 处理同步后的数据
            processSyncData(sync_packets);
            
            // 释放数据
            processor_->releaseSyncData();
        }
    }
    
    void processSyncData(const std::vector<SyncedLidarPacket>& sync_packets)
    {
        if (sync_packets.empty())
        {
            return;
        }

        RCLCPP_DEBUG(get_logger(), "Processing synchronized frame set with %zu LiDARs",
            sync_packets.size());
        
        // 这里可以添加你的处理逻辑，例如：
        // 1. 点云融合
        // 2. 坐标变换
        // 3. 特征提取
        // 4. 发布处理后的数据
        
        // 示例：打印每个激光雷达的点云信息
        for (const auto& packet : sync_packets)
        {
            if (packet.cloud)
            {
                RCLCPP_DEBUG(get_logger(),
                    "LiDAR %d: %u points, timestamp: %lu ns",
                    packet.lidar_id,
                    packet.cloud->width * packet.cloud->height,
                    packet.timestamp_ns);
            }
        }

        if (global_settings_.enable_camera_sync)
        {
            publishCameraSyncSignal(sync_packets);
        }

        if (global_settings_.publish_fused_cloud)
        {
            publishFusedCloud(sync_packets);
        }
    }
    
    void diagnosticsCallback()
    {
        std::string diagnostics = processor_->getDiagnostics();
        RCLCPP_INFO(get_logger(), "\n%s", diagnostics.c_str());
    }
    
    std::shared_ptr<MultiLidarProcessor> processor_;
    std::vector<LidarConfig> lidar_configs_;
    GlobalSettings global_settings_;
    
    rclcpp::TimerBase::SharedPtr processing_timer_;
    rclcpp::TimerBase::SharedPtr diagnostics_timer_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr fused_cloud_pub_;
    rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr camera_sync_pub_;

    void publishFusedCloud(const std::vector<SyncedLidarPacket>& packets)
    {
        if (!fused_cloud_pub_)
        {
            return;
        }
        auto fused_cloud = buildFusedCloud(packets);
        if (!fused_cloud)
        {
            return;
        }
        last_published_sync_ns_ = fused_cloud->header.stamp.nanoseconds();
        fused_cloud_pub_->publish(*fused_cloud);
    }

    void publishCameraSyncSignal(const std::vector<SyncedLidarPacket>& packets)
    {
        if (!camera_sync_pub_)
        {
            return;
        }
        auto reference_time = computeReferenceTime(packets);
        std_msgs::msg::Header header;
        header.frame_id = global_settings_.fused_frame_id.empty()
            ? packets.front().frame_id
            : global_settings_.fused_frame_id;
        header.stamp = reference_time;
        camera_sync_pub_->publish(header);

        if (last_camera_sync_ns_ != 0)
        {
            double delta_ms = static_cast<double>(reference_time.nanoseconds() - last_camera_sync_ns_) / 1e6;
            if (delta_ms > global_settings_.camera_sync_tolerance_ms)
            {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                    "Camera sync delta %.2f ms exceeds tolerance %.2f ms",
                    delta_ms,
                    global_settings_.camera_sync_tolerance_ms);
            }
        }
        last_camera_sync_ns_ = reference_time.nanoseconds();
    }

    rclcpp::Time computeReferenceTime(const std::vector<SyncedLidarPacket>& packets) const
    {
        uint64_t min_ts = std::numeric_limits<uint64_t>::max();
        for (const auto& packet : packets)
        {
            if (packet.timestamp_ns < min_ts)
            {
                min_ts = packet.timestamp_ns;
            }
        }
        return rclcpp::Time(min_ts, RCL_ROS_TIME);
    }

    sensor_msgs::msg::PointCloud2::SharedPtr buildFusedCloud(
        const std::vector<SyncedLidarPacket>& packets)
    {
        pcl::PointCloud<pcl::PointXYZI> fused;
        bool has_points = false;

        for (const auto& packet : packets)
        {
            if (!packet.cloud)
            {
                continue;
            }
            pcl::PointCloud<pcl::PointXYZI> tmp_cloud;
            try
            {
                pcl::fromROSMsg(*packet.cloud, tmp_cloud);
            }
            catch (const std::exception& e)
            {
                RCLCPP_WARN(get_logger(),
                    "Failed to convert cloud for LiDAR %d: %s",
                    packet.lidar_id,
                    e.what());
                continue;
            }
            fused += tmp_cloud;
            has_points = true;
        }

        if (!has_points)
        {
            return nullptr;
        }

        auto fused_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
        pcl::toROSMsg(fused, *fused_msg);
        auto reference_time = computeReferenceTime(packets);
        fused_msg->header.stamp = reference_time;
        fused_msg->header.frame_id = global_settings_.fused_frame_id.empty()
            ? packets.front().frame_id
            : global_settings_.fused_frame_id;
        return fused_msg;
    }

    uint64_t last_published_sync_ns_ = 0;
    uint64_t last_camera_sync_ns_ = 0;
};

}  // namespace multi_lidar_sync

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<multi_lidar_sync::MultiLidarSyncNode>();
    
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}
