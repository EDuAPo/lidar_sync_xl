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
- pcl_ros

安装依赖（Ubuntu + ROS 2 Humble）：

```bash
sudo apt-get update
sudo apt-get install -y ros-humble-pcl-ros ros-humble-pcl-conversions
```

### 编译

```bash
cd /home/zgw/Desktop/algo/lidar_sync_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select multi_lidar_sync
source install/setup.bash
```

---

## 🚀 快速上手（新手必读）

本节介绍如何用 **2 个终端** 完成多激光雷达同步验证。

### 终端布局建议

| 终端编号 | 用途 |
|---------|------|
| 终端 1 | 运行同步节点 |
| 终端 2 | 播放 rosbag 数据 |

---

### 步骤 1：启动同步节点（终端 1）

```bash
# 进入工作空间
cd /home/zgw/Desktop/algo/lidar_sync_ws

# 加载 ROS 2 环境
source /opt/ros/humble/setup.bash
source install/setup.bash

# 启动同步节点
ros2 launch multi_lidar_sync multi_lidar_sync.launch.py
```

**预期输出**：
```
[multi_lidar_sync_node-1] [INFO] Loaded LiDAR config: iv_points_front_left -> /iv_points_front_left
[multi_lidar_sync_node-1] [INFO] Loaded LiDAR config: iv_points_front_right -> /iv_points_front_right
...
[multi_lidar_sync_node-1] [INFO] Started all LiDARs and sync thread
[multi_lidar_sync_node-1] [INFO] Multi-LiDAR sync node initialized successfully
```

每 5 秒会打印诊断信息，开始时 `Sync Count: 0`（还没收到数据）。

---

### 步骤 2：播放 rosbag 数据（终端 2）

```bash
# 加载 ROS 2 环境
source /opt/ros/humble/setup.bash

# 播放 rosbag（替换为你的 bag 路径）
ros2 bag play /media/zgw/5fed7169-56a0-4d89-8970-3db49acc85dc/1205/rosbag2_2025_12_05-16_38_27 \
  --rate 1.0 \
  --topics /iv_points_front_left /iv_points_front_right /iv_points_front_mid /iv_points_rear_left /iv_points_rear_right
```

**说明**：
- `--rate 1.0`：按原速播放，可改为 `0.5`（半速）或 `2.0`（倍速）
- `--topics`：只播放激光雷达话题，节省带宽

---

### 步骤 3：观察同步结果（终端 1）

播放 bag 后，终端 1 的诊断信息会更新：

```
Multi-LiDAR Sync Diagnostics:
  LiDAR Count: 5
  Sync Count: 10          ← 成功同步的帧数
  Drop Count: 6           ← 丢弃的帧数
  Sync Rate: 62.5%        ← 同步成功率
  Last Time Spread (ms): 219.784   ← 各路激光雷达时间戳差值
```

- **Sync Count > 0** 表示同步链路正常工作
- **Time Spread** 反映各路激光雷达时间戳的最大差异

---

### 步骤 4：停止节点

```bash
# 终端 1：按 Ctrl+C 停止同步节点
# 终端 2：按 Ctrl+C 停止 bag 播放
```

---

## 📋 常用调试命令

### 查看 rosbag 信息

```bash
ros2 bag info /path/to/your/rosbag
```

### 查看当前话题列表

```bash
ros2 topic list
```

### 查看话题频率

```bash
ros2 topic hz /iv_points_front_left
```

### 查看话题内容（单条消息）

```bash
ros2 topic echo /iv_points_front_left --once
```

### 查看节点参数

```bash
ros2 param list /multi_lidar_sync_node
ros2 param get /multi_lidar_sync_node global_settings.sync_tolerance_ms
```

---

## ⚙️ 配置文件说明

配置文件位于 `config/params.yaml`，主要参数：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `global_settings.lidar_count` | 5 | 激光雷达数量 |
| `global_settings.sync_tolerance_ms` | 1000.0 | 时间同步容差（毫秒），各路时间戳差值超过此值则丢弃 |
| `global_settings.publish_rate_hz` | 10.0 | 同步数据处理频率 |
| `global_settings.enable_diagnostics` | true | 是否打印诊断信息 |
| `global_settings.drop_unsync_frames` | true | 是否丢弃不同步帧 |
| `global_settings.publish_fused_cloud` | false | 是否发布融合点云 |
| `global_settings.fused_topic` | /fusion/points | 融合点云话题 |

### 修改配置示例

如果同步率太低，可以增大容差：

```bash
# 编辑配置文件
vim src/multi_lidar_sync/config/params.yaml

# 修改 sync_tolerance_ms 为更大的值，如 500.0 或 1000.0

# 重新编译安装配置
colcon build --packages-select multi_lidar_sync
source install/setup.bash

# 重新启动节点
ros2 launch multi_lidar_sync multi_lidar_sync.launch.py
```

---

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

---

## 🚗 实车激光雷达实时数据接入

本节介绍如何在真实车辆上使用本节点接收多路激光雷达实时数据。

### 前置条件

| 项目 | 要求 |
|------|------|
| 激光雷达驱动 | 每路激光雷达需有 ROS 2 驱动，发布 `sensor_msgs/msg/PointCloud2` |
| 网络连接 | 激光雷达与计算平台在同一网络，或通过以太网直连 |
| 时间同步 | 建议使用 PTP/GPS 硬件同步各传感器时钟 |

### 常见激光雷达驱动

| 品牌 | ROS 2 驱动包 | 安装命令 |
|------|-------------|----------|
| Velodyne | `ros-humble-velodyne` | `sudo apt install ros-humble-velodyne` |
| Ouster | `ros-humble-ouster-ros` | `sudo apt install ros-humble-ouster-ros` |
| Livox | `livox_ros_driver2` | 从 GitHub 源码编译 |
| Robosense | `rslidar_sdk` | 从 GitHub 源码编译 |
| 禾赛 Hesai | `hesai_ros_driver` | 从 GitHub 源码编译 |

---

### 步骤 1：启动激光雷达驱动（每路一个终端或 launch 文件）

以 5 路激光雷达为例，假设各驱动已配置好，分别发布：
- `/iv_points_front_left`
- `/iv_points_front_right`
- `/iv_points_front_mid`
- `/iv_points_rear_left`
- `/iv_points_rear_right`

```bash
# 终端 A：启动前左雷达驱动（示例，具体命令参考厂商文档）
ros2 launch your_lidar_driver front_left.launch.py

# 终端 B：启动前右雷达驱动
ros2 launch your_lidar_driver front_right.launch.py

# ... 以此类推
```

> 💡 **建议**：将所有驱动整合到一个 launch 文件中统一管理。

---

### 步骤 2：确认话题正常发布

```bash
# 查看所有激光雷达话题
ros2 topic list | grep iv_points

# 检查某路话题频率
ros2 topic hz /iv_points_front_left

# 预期输出：average rate: 10.0 Hz（取决于激光雷达帧率）
```

---

### 步骤 3：修改配置文件（如果话题名不同）

编辑 `config/params.yaml`，将话题名改为实际驱动发布的名称：

```yaml
multi_lidar_sync_node:
  ros__parameters:
    lidars:
      0:
        topic: "/your_actual_front_left_topic"  # 修改为实际话题名
        ...
```

修改后重新编译：

```bash
colcon build --packages-select multi_lidar_sync
source install/setup.bash
```

---

### 步骤 4：启动同步节点

```bash
cd /home/zgw/Desktop/algo/lidar_sync_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch multi_lidar_sync multi_lidar_sync.launch.py
```

---

### 步骤 5：验证同步效果

观察诊断输出：

```
Multi-LiDAR Sync Diagnostics:
  LiDAR Count: 5
  Sync Count: 150        ← 持续增长表示正常
  Drop Count: 12
  Sync Rate: 92.6%       ← 实车通常比 rosbag 高
  Last Time Spread (ms): 3.2   ← 硬件同步后应 < 10ms
```

---

### 时间同步问题排查

如果 `Last Time Spread` 很大（> 100ms），说明各激光雷达时钟未对齐：

| 问题 | 解决方案 |
|------|----------|
| 时间戳差异大 | 检查是否启用 PTP/GPS 硬件同步 |
| 使用主机时间 | 在驱动配置中改用传感器内部时间戳 |
| 网络延迟 | 使用专用以太网，避免与其他流量混用 |
| 临时方案 | 增大 `sync_tolerance_ms` 容差值 |

---

### （可选）发布 TF 外参

点云融合需要各激光雷达到车辆基准系的变换关系。使用配置文件中的标定数据发布静态 TF：

```bash
# 示例：发布 front_left 激光雷达的外参
ros2 run tf2_ros static_transform_publisher \
  2.048 0.5695 0.698 \
  0 0 0.37 \
  base_link iv_points_front_left
```

或在 launch 文件中添加：

```python
from launch_ros.actions import Node

Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    arguments=['2.048', '0.5695', '0.698', '0', '0', '0.37', 'base_link', 'iv_points_front_left']
)
```

---

### 完整实车启动流程（汇总）

```bash
# ===== 终端 1：启动激光雷达驱动 =====
ros2 launch your_lidar_driver all_lidars.launch.py

# ===== 终端 2：启动同步节点 =====
cd /home/zgw/Desktop/algo/lidar_sync_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch multi_lidar_sync multi_lidar_sync.launch.py

# ===== 终端 3（可选）：启动 RViz 可视化 =====
rviz2 -d /path/to/your/config.rviz
```

---

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

zgw <zgwjlu@gmail.com>
