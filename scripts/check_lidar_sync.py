#!/usr/bin/env python3
"""
LiDAR 时间戳同步检测工具
用于验证车载多 LiDAR 的时间同步情况
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import PointCloud2
from collections import defaultdict
import time
import sys

class LidarSyncChecker(Node):
    def __init__(self, topics):
        super().__init__('lidar_sync_checker')
        
        self.topics = topics
        self.latest_stamps = {}
        self.frame_count = defaultdict(int)
        self.sync_history = []
        self.start_time = time.time()
        
        # 自动检测 QoS
        # 先尝试 RELIABLE + TRANSIENT_LOCAL（rosbag 常用）
        # 如果真实 LiDAR 使用 BEST_EFFORT，可以修改
        qos_reliable = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        
        qos_best_effort = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )
        
        # 根据参数选择 QoS
        qos = qos_best_effort if '--best-effort' in sys.argv else qos_reliable
        qos_name = "BEST_EFFORT" if '--best-effort' in sys.argv else "RELIABLE"
        
        self.get_logger().info(f"Using QoS: {qos_name}")
        
        for topic in self.topics:
            self.create_subscription(
                PointCloud2,
                topic,
                lambda msg, t=topic: self.callback(msg, t),
                qos
            )
            self.get_logger().info(f"Subscribed to: {topic}")
        
        self.timer = self.create_timer(1.0, self.print_status)
        
    def callback(self, msg, topic):
        stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        self.latest_stamps[topic] = stamp_ns
        self.frame_count[topic] += 1
        
    def print_status(self):
        elapsed = time.time() - self.start_time
        
        print(f"\n{'='*60}")
        print(f"[{elapsed:.1f}s] LiDAR Sync Status")
        print(f"{'='*60}")
        
        # 检查各话题接收情况
        for topic in self.topics:
            count = self.frame_count.get(topic, 0)
            hz = count / elapsed if elapsed > 0 else 0
            status = "✓" if count > 0 else "✗"
            print(f"  {status} {topic}: {count} frames ({hz:.1f} Hz)")
        
        # 计算时间戳同步情况
        if len(self.latest_stamps) == len(self.topics):
            stamps = list(self.latest_stamps.values())
            min_ts = min(stamps)
            max_ts = max(stamps)
            spread_ms = (max_ts - min_ts) / 1_000_000
            
            self.sync_history.append(spread_ms)
            
            print(f"\n  Time Spread: {spread_ms:.3f} ms")
            
            # 显示各 LiDAR 相对时间
            print(f"\n  Relative timestamps:")
            for topic in sorted(self.topics):
                ts = self.latest_stamps.get(topic, 0)
                delta_ms = (ts - min_ts) / 1_000_000
                print(f"    {topic}: +{delta_ms:.3f} ms")
            
            # 统计
            if len(self.sync_history) > 5:
                avg_spread = sum(self.sync_history[-10:]) / len(self.sync_history[-10:])
                max_spread = max(self.sync_history)
                min_spread = min(self.sync_history)
                print(f"\n  Statistics (last {len(self.sync_history)} samples):")
                print(f"    Avg spread: {avg_spread:.3f} ms")
                print(f"    Max spread: {max_spread:.3f} ms")
                print(f"    Min spread: {min_spread:.3f} ms")
                
                # 同步质量评估
                if avg_spread < 10:
                    quality = "优秀 ✓✓✓"
                elif avg_spread < 50:
                    quality = "良好 ✓✓"
                elif avg_spread < 100:
                    quality = "一般 ✓"
                else:
                    quality = "较差 ✗ (建议检查PTP时间同步)"
                print(f"    Sync Quality: {quality}")
        else:
            missing = set(self.topics) - set(self.latest_stamps.keys())
            print(f"\n  Waiting for topics: {missing}")

def main():
    print("=" * 60)
    print("LiDAR 时间戳同步检测工具")
    print("=" * 60)
    print("\n用法:")
    print("  默认 QoS (RELIABLE):     python3 check_lidar_sync.py")
    print("  BEST_EFFORT QoS:         python3 check_lidar_sync.py --best-effort")
    print("\n按 Ctrl+C 退出\n")
    
    rclpy.init()
    
    # 默认 LiDAR 话题列表（可根据实际情况修改）
    topics = [
        '/iv_points_front_left',
        '/iv_points_front_right',
        '/iv_points_front_mid',
        '/iv_points_rear_left',
        '/iv_points_rear_right'
    ]
    
    node = LidarSyncChecker(topics)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n\n检测结束")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
