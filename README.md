# YOLOv8 ROS2 目标检测功能包

## 概述

本ROS2功能包集成了YOLOv8目标检测模型，能够对图像流进行实时目标检测。它订阅相机图像话题，使用YOLOv8进行推理，并发布边界框信息和可视化图像。

## 主要功能：
• 使用YOLOv8进行实时目标检测

• 可配置模型参数、话题和性能设置

• 检测结果可视化

• FPS监控

• 自定义边界框消息类型

## 环境要求

• ROS2版本：Foxy、Humble 都以测试成功

• Python：3.8+

• PyTorch：1.12+（如使用GPU需安装CUDA版本）

• Ultralytics YOLOv8：pip install ultralytics (可以参考requirements)

• OpenCV：4.5+

• ROS2包：sensor_msgs、std_msgs

# 安装步骤

## 克隆代码库
git clone https://github.com/qyw23AI/yolo_ros2_ws.git

## 构建功能包
cd ~/你的工作空间
colcon build --packages-select yolov8_ros2

## 配置工作空间环境
source install/setup.bash


### 配置参数

在config/yolov8_config.yaml中修改以下参数来自定义功能：

核心参数

参数名 默认值 描述

weight_path yolov8n.pt YOLOv8模型权重文件路径

image_topic /camera/camera/image_raw 输入图像话题

pub_topic /yolov8/BoundingBoxes 边界框输出话题

camera_frame "" 相机坐标系ID

conf 0.5 置信度阈值（0.0-1.0）

visualize False 启用本地可视化窗口

use_cpu True 强制使用CPU（禁用GPU加速）

### 启动文件参数

启动时可覆盖以下参数：
ros2 launch yolov8_ros2 yolov8_launch.py \
    config_file:=/路径/到/自定义配置.yaml \
    image_topic:=/你的/相机/话题


### 使用方法

1. 启动检测节点：
ros2 launch yolov8_ros2 yolov8_launch.py


2. 查看检测结果：
• 边界框信息：/yolov8/BoundingBoxes

• 可视化图像：/yolov8/detection_image

3. 启用可视化（在配置中设置visualize: True）：
将打开显示检测结果的窗口


### 自定义配置指南

1. 模型选择

修改weight_path使用不同的YOLOv8模型：
weight_path: "yolov8s.pt"  # 更小更快的模型
weight_path: "yolov8x.pt"  # 更大更精确的模型
weight_path: "/路径/到/自定义模型.pt"  # 自定义训练模型


2. 话题配置

修改话题以匹配您的系统：
image_topic: "/camera/color/image_raw"  # Intel RealSense相机
pub_topic: "/自定义检测结果"  # 自定义输出话题


3. 性能调优

调整速度/精度平衡：
conf: 0.25  # 降低阈值以检测更多目标（提高召回率）
use_cpu: false  # 如有GPU则启用加速


4. 可视化选项

visualize: true  # 显示本地OpenCV窗口
camera_frame: "camera_link"  # 设置消息的坐标系ID


文件结构


yolov8_ros2/
├── launch/
│   └── yolov8_launch.py          # 启动文件
├── config/
│   └── yolov8_config.yaml        # 配置文件
├── yolov8_ros2/
│   └── yolov8_node.py            # 主要节点代码
└── package.xml                   # 功能包配置


故障排除

• CUDA不可用：如无GPU，设置use_cpu: true

• 无检测结果：降低置信度阈值（conf）

• 帧率过低：使用更小模型（yolov8n.pt）或启用GPU

• 未收到图像：确认image_topic与发布者匹配

如果要使用conda 下的yolo环境，需要指明，可以在setup.cfg中指出如下：
### [build_scripts]
executable = /home/v/miniconda3/envs/yolov8/bin/python
### 或者使用export PYTHONPATH="/home/v/miniconda3/envs/yolov8/lib/python3.10/site-packages:"$PYTHONPATH ,指明python包路径后，再运行。
### 或者直接再install/yolov8_ros/lib/yolov8/你的节点 中修改，在第一行添加上解释器路径，如#!/usr/bin/python3


许可证

Apache License 2.0

最后更新：2025年10月25日  
ROS2功能包版本：1.0.0