#ifndef MULTI_LIDAR_SYNC__LIDAR_BUFFER_MANAGER_HPP_
#define MULTI_LIDAR_SYNC__LIDAR_BUFFER_MANAGER_HPP_

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <limits>
#include "multi_lidar_sync/lidar_types.hpp"

namespace multi_lidar_sync
{

/**
 * @brief 三缓冲区管理器
 * 
 * 实现无锁的三缓冲区机制，用于生产者-消费者模式：
 * - 写入线程（生产者）持续写入新数据
 * - 读取线程（消费者）获取最新数据进行处理
 * - 保证数据一致性，避免数据竞争
 */
class LidarBufferManager
{
public:
    /**
     * @brief 构造函数
     * 初始化三个缓冲区
     */
    LidarBufferManager();
    
    /**
     * @brief 析构函数
     * 清理所有缓冲区资源
     */
    ~LidarBufferManager();
    
    /**
     * @brief 写入新帧数据（生产者调用）
     * @param frame 激光雷达帧数据
     * @return 成功返回true，失败返回false
     */
    bool setFrame(const LidarFrame& frame);
    
    /**
     * @brief 无锁获取最新帧的元数据
     * @param timestamp_ns 输出时间戳
     * @param sequence 输出序号
     * @return 成功返回true，无数据返回false
     */
    bool getLatestMetadata(uint64_t& timestamp_ns, uint32_t& sequence);
    
    /**
     * @brief 获取可读缓冲区（消费者调用）
     * 此函数会持有内部锁，直到调用 releaseReadBuffer()
     * @return 成功返回指针，失败返回nullptr
     */
    LidarFrame* getReadBuffer();
    
    /**
     * @brief 释放读缓冲区（消费者调用）
     * 必须与 getReadBuffer() 配对使用
     */
    void releaseReadBuffer();

private:
    static constexpr size_t BUFFER_COUNT = 3;
    
    LidarFrame buffers_[BUFFER_COUNT];          // 三个缓冲区
    std::atomic<int> writing_index_;            // 当前写入索引
    std::atomic<int> ready_index_;              // 准备好的索引
    std::atomic<int> reading_index_;            // 当前读取索引
    uint32_t last_read_seq_;                    // 上次读取的序号
    std::timed_mutex reading_mutex_;            // 读取互斥锁（允许短暂等待）
    
    /**
     * @brief 清理缓冲区
     * @param buffer 要清理的缓冲区指针
     */
    void cleanupBuffer(LidarFrame* buffer);

    /**
     * @brief 判断序号是否为新帧，考虑 32 位回绕
     */
    bool isSequenceNew(uint32_t sequence) const;
};

}  // namespace multi_lidar_sync

#endif  // MULTI_LIDAR_SYNC__LIDAR_BUFFER_MANAGER_HPP_
