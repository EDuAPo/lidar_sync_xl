#include "multi_lidar_sync/multi_lidar_processor.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <algorithm>
#include <limits>
#include <chrono>

namespace multi_lidar_sync
{

MultiLidarProcessor::MultiLidarProcessor(
    rclcpp::Node::SharedPtr node,
    const std::vector<LidarConfig>& configs,
    const GlobalSettings& settings)
    : node_(node)
    , settings_(settings)
    , is_running_(false)
    , is_ready_(false)
    , sync_count_(0)
    , drop_count_(0)
{
    // 创建激光雷达处理器
    for (const auto& config : configs)
    {
        if (config.enabled)
        {
            auto processor = std::make_shared<LidarProcessor>(node_, config);
            lidars_.push_back(processor);
        }
    }
    
    // 初始化状态向量
    frame_states_.resize(lidars_.size());
    last_processed_seq_.assign(lidars_.size(), std::numeric_limits<uint32_t>::max());
    synced_packets_.resize(lidars_.size());
    locked_buffers_.resize(lidars_.size(), nullptr);
    
    RCLCPP_INFO(node_->get_logger(),
        "Initialized MultiLidarProcessor with %zu LiDARs", lidars_.size());
}

MultiLidarProcessor::~MultiLidarProcessor()
{
    stopAll();
}

bool MultiLidarProcessor::startAll()
{
    if (lidars_.empty())
    {
        RCLCPP_ERROR(node_->get_logger(), "No LiDARs configured!");
        return false;
    }
    
    // 启动所有激光雷达
    for (auto& lidar : lidars_)
    {
        if (!lidar->start())
        {
            RCLCPP_ERROR(node_->get_logger(),
                "Failed to start LiDAR: %s", lidar->getLidarName().c_str());
            return false;
        }
    }
    
    // 启动同步线程
    is_running_.store(true);
    sync_thread_ = std::thread(&MultiLidarProcessor::syncLoop, this);
    
    RCLCPP_INFO(node_->get_logger(), "Started all LiDARs and sync thread");
    return true;
}

void MultiLidarProcessor::stopAll()
{
    if (!is_running_.load())
    {
        return;
    }
    
    is_running_.store(false);
    
    // 等待同步线程结束
    if (sync_thread_.joinable())
    {
        sync_thread_.join();
    }
    
    // 停止所有激光雷达
    for (auto& lidar : lidars_)
    {
        lidar->stop();
    }
    
    RCLCPP_INFO(node_->get_logger(), "Stopped all LiDARs");
}

void MultiLidarProcessor::syncLoop()
{
    // 将毫秒转换为纳秒
    uint64_t tolerance_ns = static_cast<uint64_t>(settings_.sync_tolerance_ms * 1000000);
    const auto idle_sleep = std::chrono::microseconds(
        static_cast<int64_t>(std::max(500.0, settings_.sync_tolerance_ms * 500.0)));
    
    while (is_running_.load())
    {
        // 如果上一批数据还未被消费，等待释放
        if (is_ready_.load(std::memory_order_acquire))
        {
            std::unique_lock<std::mutex> ready_lock(ready_mutex_);
            ready_cv_.wait_for(ready_lock, idle_sleep, [this]() {
                return !is_ready_.load(std::memory_order_acquire) || !is_running_.load();
            });
            continue;
        }

        // 阶段1：轮询所有激光雷达状态
        pollLidarStates();
        
        // 阶段2：判断是否找到同步集
        if (findSynchronizedSet(tolerance_ns))
        {
            // 阶段3：尝试获取同步数据
            if (acquireData())
            {
                sync_count_.fetch_add(1, std::memory_order_relaxed);
                is_ready_.store(true, std::memory_order_release);
                ready_cv_.notify_all();
            }
            else
            {
                drop_count_.fetch_add(1, std::memory_order_relaxed);
            }
        }

        // 短暂休眠，避免CPU占用过高，同时允许更快响应新数据
        std::this_thread::sleep_for(idle_sleep);
    }
}

void MultiLidarProcessor::pollLidarStates()
{
    for (size_t i = 0; i < lidars_.size(); ++i)
    {
        uint64_t ts;
        uint32_t seq;
        
        // 获取最新元数据
        if (lidars_[i]->buffer_manager->getLatestMetadata(ts, seq))
        {
            frame_states_[i].timestamp = ts;
            frame_states_[i].sequence = seq;
            
            // 判断是否为新数据
            int32_t diff = static_cast<int32_t>(seq - last_processed_seq_[i]);
            if (diff > 0)
            {
                frame_states_[i].status = FrameStatus::NEW;
            }
            else
            {
                frame_states_[i].status = FrameStatus::STALE;
            }
        }
        else
        {
            frame_states_[i].status = FrameStatus::EMPTY;
        }
    }
}

bool MultiLidarProcessor::findSynchronizedSet(uint64_t tolerance_ns)
{
    uint64_t min_ts = 0;
    uint64_t max_ts = 0;
    bool at_least_one_new = false;
    
    for (size_t i = 0; i < frame_states_.size(); ++i)
    {
        const auto& state = frame_states_[i];
        
        // 1. 必须所有激光雷达都有数据
        if (state.status == FrameStatus::EMPTY)
        {
            return false;
        }
        
        // 2. 必须至少有一个新帧
        if (state.status == FrameStatus::NEW)
        {
            at_least_one_new = true;
        }
        
        // 3. 收集时间戳
        if (i == 0)
        {
            min_ts = max_ts = state.timestamp;
        }
        else
        {
            if (state.timestamp < min_ts) min_ts = state.timestamp;
            if (state.timestamp > max_ts) max_ts = state.timestamp;
        }
    }
    
    // 4. 如果没有新帧，不处理
    if (!at_least_one_new)
    {
        return false;
    }
    
    last_sync_delta_ns_ = max_ts - min_ts;

    // 5. 检查时间戳容差
    if (last_sync_delta_ns_ > tolerance_ns)
    {
        if (settings_.drop_unsync_frames)
        {
            // 时间不同步，丢弃
            RCLCPP_DEBUG_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                "Frames not synchronized. Delta: %lu ns", last_sync_delta_ns_);
            return false;
        }
    }
    
    return true;
}

bool MultiLidarProcessor::acquireData()
{
    // 阻塞式锁定
    std::vector<LidarFrame*> current_buffers(lidars_.size());
    bool all_acquired = true;
    
    // 尝试获取所有激光雷达的缓冲区
    for (size_t i = 0; i < lidars_.size(); ++i)
    {
        LidarFrame* buf = lidars_[i]->buffer_manager->getReadBuffer();
        
        // 检查是否仍然是决策时的那一帧
        if (buf == nullptr || buf->sequence != frame_states_[i].sequence)
        {
            // 发生竞态，回滚
            if (buf != nullptr)
            {
                lidars_[i]->buffer_manager->releaseReadBuffer();
            }
            
            for (size_t j = 0; j < i; ++j)
            {
                lidars_[j]->buffer_manager->releaseReadBuffer();
            }
            
            all_acquired = false;
            break;
        }
        
        current_buffers[i] = buf;
    }
    
    if (!all_acquired)
    {
        return false;
    }
    
    // 成功获取所有数据
    for (size_t i = 0; i < lidars_.size(); ++i)
    {
        last_processed_seq_[i] = current_buffers[i]->sequence;
        frame_states_[i].status = FrameStatus::STALE;
    }

    {
        std::lock_guard<std::mutex> lock(data_access_mutex_);
        locked_buffers_ = current_buffers;
        if (synced_packets_.size() != lidars_.size())
        {
            synced_packets_.resize(lidars_.size());
        }

        for (size_t i = 0; i < lidars_.size(); ++i)
        {
            auto& packet = synced_packets_[i];
            packet.lidar_id = lidars_[i]->getLidarId();
            packet.frame_id = lidars_[i]->getFrameId();
            packet.timestamp_ns = current_buffers[i]->timestamp_ns;
            packet.sequence = current_buffers[i]->sequence;
            packet.cloud = current_buffers[i]->cloud_msg;
        }
    }

    return true;
}

void MultiLidarProcessor::releaseData()
{
    {
        std::lock_guard<std::mutex> lock(data_access_mutex_);
        is_ready_.store(false, std::memory_order_release);
        
        // 释放所有缓冲区
        for (size_t i = 0; i < lidars_.size(); ++i)
        {
            if (locked_buffers_[i] != nullptr)
            {
                lidars_[i]->buffer_manager->releaseReadBuffer();
                locked_buffers_[i] = nullptr;
            }
        }
        
        for (auto& packet : synced_packets_)
        {
            packet.cloud.reset();
            packet.sequence = 0;
            packet.timestamp_ns = 0;
        }
    }
    ready_cv_.notify_all();
}

bool MultiLidarProcessor::getSyncData(std::vector<SyncedLidarPacket>& sync_packets)
{
    if (!is_ready_.load(std::memory_order_acquire))
    {
        return false;
    }

    std::lock_guard<std::mutex> lock(data_access_mutex_);
    if (!is_ready_.load(std::memory_order_relaxed))
    {
        return false;
    }
    sync_packets = synced_packets_;
    return true;
}

void MultiLidarProcessor::releaseSyncData()
{
    releaseData();
}

std::string MultiLidarProcessor::getDiagnostics() const
{
    std::ostringstream oss;
    oss << "Multi-LiDAR Sync Diagnostics:\n";
    oss << "  LiDAR Count: " << lidars_.size() << "\n";
    oss << "  Sync Count: " << sync_count_.load() << "\n";
    oss << "  Drop Count: " << drop_count_.load() << "\n";
    oss << "  Sync Rate: ";
    
    uint64_t total = sync_count_.load() + drop_count_.load();
    if (total > 0)
    {
        double rate = 100.0 * sync_count_.load() / total;
        oss << rate << "%\n";
    }
    else
    {
        oss << "N/A\n";
    }
    oss << "  Last Time Spread (ms): "
        << static_cast<double>(last_sync_delta_ns_) / 1e6 << "\n";
    
    return oss.str();
}

}  // namespace multi_lidar_sync
