#!/bin/bash

# Multi-LiDAR Sync Workspace Build and Run Script

WORKSPACE_DIR="/home/zgw/Desktop/algo/lidar_sync_ws"

echo "========================================="
echo "  Multi-LiDAR Sync Workspace Manager"
echo "========================================="
echo ""

cd "$WORKSPACE_DIR"

case "$1" in
    build)
        echo "Building workspace..."
        colcon build --packages-select multi_lidar_sync --cmake-args -DCMAKE_BUILD_TYPE=Release
        echo ""
        echo "Build complete! Source the workspace with:"
        echo "  source install/setup.bash"
        ;;
    
    clean)
        echo "Cleaning workspace..."
        rm -rf build install log
        echo "Clean complete!"
        ;;
    
    run)
        echo "Sourcing workspace..."
        source "$WORKSPACE_DIR/install/setup.bash"
        echo "Launching multi-lidar sync node..."
        ros2 launch multi_lidar_sync multi_lidar_sync.launch.py
        ;;
    
    test)
        echo "Running in test mode with verbose output..."
        source "$WORKSPACE_DIR/install/setup.bash"
        ros2 run multi_lidar_sync multi_lidar_sync_node --ros-args --log-level debug
        ;;
    
    *)
        echo "Usage: $0 {build|clean|run|test}"
        echo ""
        echo "Commands:"
        echo "  build  - Build the workspace"
        echo "  clean  - Clean build artifacts"
        echo "  run    - Run the multi-lidar sync node"
        echo "  test   - Run with debug logging"
        echo ""
        echo "Example:"
        echo "  $0 build"
        echo "  $0 run"
        exit 1
        ;;
esac
