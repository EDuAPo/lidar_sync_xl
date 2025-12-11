#include "multi_lidar_sync/lidar_buffer_manager.hpp"
#include <iostream>

namespace multi_lidar_sync
{

LidarBufferManager::LidarBufferManager()
    : writing_index_(0)
    , ready_index_(-1)
    , reading_index_(-1)
    , last_read_seq_(std::numeric_limits<uint32_t>::max())
{
    // 初始化所有缓冲区
    for (size_t i = 0; i < BUFFER_COUNT; ++i)
    {
        buffers_[i].cloud_msg = nullptr;
        buffers_[i].timestamp_ns = 0;
        buffers_[i].sequence = 0;
        buffers_[i].valid = false;
    }
}

LidarBufferManager::~LidarBufferManager()
{
    // 清理所有缓冲区
    for (size_t i = 0; i < BUFFER_COUNT; ++i)
    {
        cleanupBuffer(&buffers_[i]);
    }
}

bool LidarBufferManager::setFrame(const LidarFrame& frame)
{
    // 1. 获取当前写入槽
    int current_writing = writing_index_.load(std::memory_order_relaxed);
    LidarFrame* buffer = &buffers_[current_writing];
    
    // 2. 清理该槽位中可能存在的旧数据
    cleanupBuffer(buffer);
    
    // 3. 填充新数据
    buffer->cloud_msg = frame.cloud_msg;
    buffer->timestamp_ns = frame.timestamp_ns;
    buffer->sequence = frame.sequence;
    buffer->valid = frame.valid;
    buffer->lidar_id = frame.lidar_id;
    
    // 4. "发布"这个缓冲区：原子地将 ready_index_ 设置为刚写完的索引
    ready_index_.store(current_writing, std::memory_order_release);
    
    // 5. 寻找下一个写入槽（既不是 ready 也不是 reading）
    int current_reading = reading_index_.load(std::memory_order_relaxed);
    int next_writing = -1;
    
    for (int i = 0; i < 3; ++i)
    {
        if (i != current_writing && i != current_reading)
        {
            next_writing = i;
            break;
        }
    }
    
    if (next_writing != -1)
    {
        writing_index_.store(next_writing, std::memory_order_relaxed);
    }
    
    return true;
}

bool LidarBufferManager::getLatestMetadata(uint64_t& timestamp_ns, uint32_t& sequence)
{
    // 使用 acquire 语义加载索引
    int current_ready = ready_index_.load(std::memory_order_acquire);
    
    if (current_ready == -1)
    {
        return false;  // 没有数据
    }
    
    // 安全读取元数据
    timestamp_ns = buffers_[current_ready].timestamp_ns;
    sequence = buffers_[current_ready].sequence;
    
    // 检查数据是否有效 (不再检查 sequence == 0，因为序号从 0 开始是合法的)
    if (!buffers_[current_ready].valid)
    {
        return false;
    }
    
    return true;
}

LidarFrame* LidarBufferManager::getReadBuffer()
{
    // 1. 尝试在短时间内获取读取锁，避免长期占用写线程
    if (!reading_mutex_.try_lock_for(std::chrono::microseconds(200)))
    {
        return nullptr;
    }
    
    // 2. 获取当前准备好的索引
    int current_ready = ready_index_.load(std::memory_order_acquire);
    
    // 3. 检查是否有数据
    if (current_ready == -1)
    {
        reading_mutex_.unlock();
        return nullptr;
    }
    
    LidarFrame* buffer = &buffers_[current_ready];
    
    // 4. 检查缓冲区是否有效
    if (!buffer->valid || buffer->cloud_msg == nullptr)
    {
        reading_mutex_.unlock();
        return nullptr;
    }
    
    // 5. 检查是否是已读过的旧数据
    if (!isSequenceNew(buffer->sequence))
    {
        reading_mutex_.unlock();
        return nullptr;
    }
    
    // 6. 声明正在读取这个缓冲区
    reading_index_.store(current_ready, std::memory_order_relaxed);
    last_read_seq_ = buffer->sequence;
    
    // 返回时 reading_mutex_ 仍被持有
    return buffer;
}

void LidarBufferManager::releaseReadBuffer()
{
    int current_reading = reading_index_.load(std::memory_order_relaxed);
    
    if (current_reading == -1)
    {
        // 没有读者但可能未持锁，尝试无害释放
        if (reading_mutex_.try_lock())
        {
            reading_mutex_.unlock();
        }
        return;
    }
    
    // 释放读取声明
    reading_index_.store(-1, std::memory_order_relaxed);
    
    // 释放锁
    reading_mutex_.unlock();
}

void LidarBufferManager::cleanupBuffer(LidarFrame* buffer)
{
    buffer->cloud_msg = nullptr;
    buffer->timestamp_ns = 0;
    buffer->sequence = 0;
    buffer->valid = false;
    buffer->lidar_id = -1;
}

bool LidarBufferManager::isSequenceNew(uint32_t sequence) const
{
    // 利用有符号差值判断回绕后的新旧
    return static_cast<int32_t>(sequence - last_read_seq_) > 0;
}

}  // namespace multi_lidar_sync
