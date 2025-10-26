#!/bin/bash

# 修复 yolo_launch 包问题后的测试脚本

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
echo "尝试启动 RealSense YOLO 系统："
echo "ros2 launch yolo_launch yolov8_realsense_launch.py"

echo ""
echo "或者启动基本的 YOLO 节点："
echo "ros2 launch yolo_launch yolov8_launch.py"

echo ""
echo "=== 测试完成 ==="
