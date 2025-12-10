# 快速开始指南

## 项目结构

```
lidar_sync_ws/
├── src/
│   └── multi_lidar_sync/
│       ├── include/multi_lidar_sync/     # 头文件
│       │   ├── lidar_types.hpp           # 数据类型定义
│       │   ├── lidar_buffer_manager.hpp  # 三缓冲区管理器
│       │   ├── lidar_processor.hpp       # 单激光雷达处理器
│       │   └── multi_lidar_processor.hpp # 多激光雷达同步器
│       ├── src/                          # 源文件
│       │   ├── lidar_buffer_manager.cpp
│       │   ├── lidar_processor.cpp
│       │   ├── multi_lidar_processor.cpp
│       │   └── multi_lidar_sync_node.cpp # ROS2节点主程序
│       ├── config/                       # 配置文件
│       │   └── lidar_config.yaml
│       ├── launch/                       # 启动文件
│       │   └── multi_lidar_sync.launch.py
│       ├── CMakeLists.txt
│       ├── package.xml
│       └── README.md
└── workspace.sh                          # 快捷脚本

```

## 一键操作

### 1. 编译项目

```bash
cd /home/zgw/Desktop/algo/lidar_sync_ws
./workspace.sh build
```

### 2. 运行节点

```bash
./workspace.sh run
```

### 3. 调试模式

```bash
./workspace.sh test
```

### 4. 清理编译产物

```bash
./workspace.sh clean
```

## 手动操作

### 编译

```bash
cd /home/zgw/Desktop/algo/lidar_sync_ws
colcon build --packages-select multi_lidar_sync
source install/setup.bash
```

### 运行

```bash
ros2 launch multi_lidar_sync multi_lidar_sync.launch.py
```

### 自定义配置运行

```bash
ros2 launch multi_lidar_sync multi_lidar_sync.launch.py \
    config_file:=/path/to/custom_config.yaml
```

## 配置说明

编辑 `src/multi_lidar_sync/config/lidar_config.yaml`:

```yaml
global_settings:
  lidar_count: 5                    # 激光雷达数量
  sync_tolerance_ms: 10.0           # 同步容差(毫秒)
  max_queue_size: 10                # ROS队列大小
  publish_rate_hz: 10.0             # 处理频率
  enable_diagnostics: true          # 启用诊断
  drop_unsync_frames: true          # 丢弃不同步帧

lidars:
  - id: 0
    name: "lidar_front"
    topic: "/lidar_front/points"    # 订阅话题
    frame_id: "lidar_front"         # 坐标系
    enabled: true                   # 是否启用
```

## 核心功能

### 1. 三缓冲区机制
- 无锁设计，高性能
- 生产者-消费者模式
- 自动覆盖旧数据

### 2. 时间同步算法
- 轮询所有激光雷达状态
- 判断时间戳是否在容差内
- 自动丢弃不同步数据
- 只处理最新帧

### 3. 数据验证
- 检查数据有效性
- 防止重复处理
- 竞态条件检测

## 集成示例

在 `multi_lidar_sync_node.cpp` 的 `processSyncData()` 函数中添加你的处理逻辑：

```cpp
void processSyncData(const std::vector<LidarFrame*>& sync_frames)
{
    // 示例1: 访问点云数据
    for (size_t i = 0; i < sync_frames.size(); ++i)
    {
        auto cloud_msg = sync_frames[i]->cloud_msg;
        uint64_t timestamp = sync_frames[i]->timestamp_ns;
        
        // 转换为PCL点云
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*cloud_msg, *cloud);
        
        // 你的处理代码...
    }
    
    // 示例2: 合并所有点云
    // pcl::PointCloud<pcl::PointXYZ>::Ptr merged = mergePointClouds(sync_frames);
    
    // 示例3: 发布处理结果
    // publisher_->publish(result_msg);
}
```

## 调试技巧

### 查看话题列表

```bash
ros2 topic list
```

### 查看话题信息

```bash
ros2 topic info /lidar_front/points
ros2 topic hz /lidar_front/points
```

### 查看节点日志

```bash
ros2 run multi_lidar_sync multi_lidar_sync_node --ros-args --log-level debug
```

### 监控同步率

节点每5秒输出诊断信息：
```
Multi-LiDAR Sync Diagnostics:
  LiDAR Count: 5
  Sync Count: 1234
  Drop Count: 56
  Sync Rate: 95.67%
```

## 常见问题

### Q: 同步率很低怎么办？
A: 增大 `sync_tolerance_ms` 或检查激光雷达时钟同步。

### Q: 如何修改激光雷达数量？
A: 修改 `config/lidar_config.yaml` 中的 `lidar_count` 并添加相应配置。

### Q: 如何添加自定义处理？
A: 在 `processSyncData()` 函数中添加你的代码。

### Q: 支持哪些点云格式？
A: 支持 `sensor_msgs/PointCloud2`，兼容所有标准ROS2激光雷达。

## 性能优化

1. **调整同步容差**: 根据实际延迟调整 `sync_tolerance_ms`
2. **优化处理频率**: 根据CPU负载调整 `publish_rate_hz`
3. **使用Release编译**: `--cmake-args -DCMAKE_BUILD_TYPE=Release`
4. **QoS策略**: 代码中已设置 `best_effort`，减少延迟

## 下一步

1. 根据你的激光雷达修改配置文件
2. 编译并测试基本功能
3. 在 `processSyncData()` 中实现你的处理逻辑
4. 根据需要添加发布器和服务

## 技术支持

详细文档请参考 `src/multi_lidar_sync/README.md`
