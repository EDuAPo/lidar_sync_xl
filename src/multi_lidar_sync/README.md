# Multi-LiDAR Synchronization Node

ROS2 节点，用于多路激光雷达数据的时间同步采集和处理。

## 功能特性

- ✅ 支持多路激光雷达（默认5路，可配置）
- ✅ 三缓冲区机制，无锁高效数据交换
- ✅ 时间戳同步算法，可配置容差
- ✅ 数据有效性验证
- ✅ 自动丢弃不同步数据
- ✅ 最新帧优先处理
- ✅ 各激光雷达独立采集，互不影响
- ✅ 实时诊断和统计信息

## 系统架构

```
┌─────────────────┐
│  LiDAR 1 Topic  │──┐
└─────────────────┘  │
┌─────────────────┐  │    ┌──────────────────┐
│  LiDAR 2 Topic  │──┼───▶│ LidarProcessor   │
└─────────────────┘  │    │  (Triple Buffer) │
┌─────────────────┐  │    └──────────────────┘
│  LiDAR 3 Topic  │──┤              │
└─────────────────┘  │              ▼
┌─────────────────┐  │    ┌──────────────────────┐
│  LiDAR 4 Topic  │──┼───▶│ MultiLidarProcessor  │
└─────────────────┘  │    │  (Sync Algorithm)    │
┌─────────────────┐  │    └──────────────────────┘
│  LiDAR 5 Topic  │──┘              │
└─────────────────┘                 ▼
                          ┌──────────────────┐
                          │  Synchronized    │
                          │  Data Output     │
                          └──────────────────┘
```

## 编译与安装

### 依赖项

- ROS2 (Humble/Iron/Rolling)
- PCL (Point Cloud Library)
- sensor_msgs
- pcl_conversions

### 编译

```bash
cd /home/zgw/Desktop/algo/lidar_sync_ws
colcon build --packages-select multi_lidar_sync
source install/setup.bash
```

## 使用方法

### 1. 配置激光雷达

编辑配置文件 `config/lidar_config.yaml`：

```yaml
global_settings:
  lidar_count: 5
  sync_tolerance_ms: 10.0      # 时间同步容差
  drop_unsync_frames: true     # 丢弃不同步帧

lidars:
  - id: 0
    name: "lidar_front"
    topic: "/lidar_front/points"
    enabled: true
```

### 2. 启动节点

```bash
ros2 launch multi_lidar_sync multi_lidar_sync.launch.py
```

或使用自定义配置：

```bash
ros2 launch multi_lidar_sync multi_lidar_sync.launch.py config_file:=/path/to/your/config.yaml
```

### 3. 查看诊断信息

节点会每5秒输出诊断信息：

```
Multi-LiDAR Sync Diagnostics:
  LiDAR Count: 5
  Sync Count: 1234
  Drop Count: 56
  Sync Rate: 95.67%
```

## 核心算法

### 三缓冲区机制

每个激光雷达使用三个缓冲区：
- **写入缓冲区**：生产者（ROS回调）写入新数据
- **就绪缓冲区**：最新完成的数据
- **读取缓冲区**：消费者（同步线程）读取数据

### 同步算法

1. **轮询阶段**：检查所有激光雷达最新帧状态
2. **决策阶段**：判断是否满足同步条件
   - 所有激光雷达都有数据
   - 至少有一路新数据
   - 时间戳在容差范围内
3. **获取阶段**：锁定并获取同步帧集
4. **处理阶段**：处理同步数据
5. **释放阶段**：释放所有缓冲区

### 数据丢弃策略

- 不满足同步条件的数据自动丢弃
- 只使用最新帧，旧帧自动覆盖
- 保证数据一致性和实时性

## API 接口

### MultiLidarProcessor

```cpp
// 启动所有激光雷达
bool startAll();

// 停止所有激光雷达
void stopAll();

// 获取同步数据的快照（包含fusion所需元数据）
bool getSyncData(std::vector<SyncedLidarPacket>& synced_packets);

// 释放同步数据
void releaseSyncData();

// 获取诊断信息
std::string getDiagnostics() const;
```

## 扩展开发

在 `processSyncData()` 函数中添加你的处理逻辑：

```cpp
void processSyncData(const std::vector<SyncedLidarPacket>& sync_packets)
{
    // 你的处理代码
    // 例如：点云融合、特征提取、目标检测等
    
  for (const auto& packet : sync_packets)
    {
    auto cloud_msg = packet.cloud;
        // ... 处理 ...
    }
}
```

## 性能优化建议

1. 调整 `sync_tolerance_ms` 以平衡同步率和延迟
2. 使用 `best_effort` QoS 以减少网络延迟
3. 根据硬件性能调整 `publish_rate_hz`
4. 启用诊断监控同步效率

## 故障排查

### 同步率低

- 检查激光雷达时钟是否同步
- 增大 `sync_tolerance_ms`
- 检查网络延迟

### 丢帧严重

- 检查CPU负载
- 优化处理代码
- 增大缓冲区数量

### 数据不更新

- 检查话题名称是否正确
- 确认激光雷达正在发布数据
- 查看ROS日志

## 许可证

MIT License

## 作者

Your Name <your_email@example.com>
