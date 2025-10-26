#!/bin/bash

# 测试修复后的 yolo_launch 包

echo "=== 测试修复后的 yolo_launch 包 ==="

# 设置环境
echo "设置 ROS 2 环境..."
source /opt/ros/foxy/setup.bash
source /home/admin-pc/qyw/yolo_ros2/yolo_ros2_ws/install/setup.bash

echo "环境设置完成！"

echo ""
echo "检查可用的启动文件："
ros2 launch yolo_launch --help

echo ""
echo "测试基本 YOLO 启动："
echo "ros2 launch yolo_launch yolov8_launch.py"

echo ""
echo "测试距离检测节点启动："
echo "ros2 launch yolo_launch yolov8_distance_node.launch.py"

echo ""
echo "测试 RealSense + YOLO 启动："
echo "ros2 launch yolo_launch yolov8_realsense_launch.py"

echo ""
echo "=== 测试完成 ==="
