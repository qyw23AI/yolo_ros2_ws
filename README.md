# YOLOv8 ROS2 节点

该软件包提供了一个基于Ultralytics YOLOv8的ROS2目标检测节点。它订阅图像话题，执行目标检测，并发布边界框和检测结果图像。

## 功能特点

- 使用YOLOv8进行实时目标检测
- 发布带有类名和概率的边界框
- 发布带有叠加边界框的检测结果图像
- 通过ROS2参数进行可配置
- 支持GPU加速（如果可用）

## 环境要求

- ROS2 Humble（或更新版本）
- Python 3.8+
- PyTorch 1.10+
- OpenCV
- Ultralytics YOLOv8
- pyrealsense2（用于Intel RealSense相机）

## 安装步骤

### 1. 创建ROS2工作空间（如果尚未创建）

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

### 2. 克隆本仓库

```bash
git clone https://github.com/yourusername/yolov8_ros2.git
```

### 3. 安装依赖项

```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

### 4. 安装Python依赖

```bash
pip3 install ultralytics torch opencv-python pyrealsense2
```

### 5. 构建软件包

```bash
colcon build --symlink-install
```

### 6.  source工作空间

```bash
source install/setup.bash
```

## 使用方法

### 1. 启动YOLOv8节点

```bash
ros2 launch yolov8_ros2 yolov8_launch.py
```

### 2. 使用自定义配置启动

```bash
ros2 launch yolov8_ros2 yolov8_launch.py config_file:=/path/to/your/config.yaml
```

### 3. 使用自定义参数运行

```bash
ros2 run yolov8_ros2 yolov8_node --ros-args -p weight_path:=yolov8s.pt -p conf:=0.6 -p visualize:=true
```

## 配置说明

配置文件（`config/yolov8_config.yaml`）允许您设置以下参数：

- `weight_path`: YOLOv8模型权重路径（默认值："yolov8n.pt"）
- `conf`: 检测置信度阈值（默认值：0.5）
- `visualize`: 启用/禁用可视化窗口（默认值：true）
- `image_topic`: 输入图像话题（默认值："/camera/color/image_raw"）
- `pub_topic`: 输出边界框话题（默认值："/yolov8/BoundingBoxes"）
- `camera_frame`: 相机坐标系ID（默认值："camera_color_optical_frame"）
- `use_cpu`: 强制使用CPU（默认值：false）

## 话题说明

### 订阅话题

- `/camera/color/image_raw` (sensor_msgs/Image): 用于目标检测的输入图像

### 发布话题

- `/yolov8/BoundingBoxes` (yolov8_ros2_msgs/BoundingBoxes): 检测到的边界框
- `/yolov8/detection_image` (sensor_msgs/Image): 带有叠加边界框的图像

## 消息类型

### BoundingBox.msg

```
float64 xmin
float64 ymin
float64 xmax
float64 ymax
string class_name
float64 probability
```

### BoundingBoxes.msg

```
std_msgs/BoundingBox[] bounding_boxes
std_msgs/Header header
std_msgs/Header image_header
```

## 性能优化

- 节点会自动使用可用的GPU，否则回退到CPU
- 为获得更好的性能，请使用较小的模型（例如yolov8n.pt）或启用GPU加速
- 推理速度取决于模型大小、图像分辨率和硬件能力

## 故障排除

### CUDA内存不足

如果遇到CUDA内存不足错误：
1. 使用较小的模型（例如yolov8n.pt而不是yolov8x.pt）
2. 降低输入图像分辨率
3. 在配置文件中设置`use_cpu: true`启用CPU使用

### 未接收到图像

- 检查图像话题是否正确
- 确保相机正在发布图像
- 使用`ros2 topic list`验证话题名称

### 可视化窗口不显示

- 确保配置文件中设置了`visualize: true`
- 检查OpenCV是否正确安装
- 验证节点是否有显示访问权限（远程运行时设置DISPLAY变量）

## 许可证

该软件包以Apache-2.0许可证发布。

## 致谢

- [Ultralytics YOLOv8](https://github.com/ultralytics/ultralytics)
- [ROS2](https://docs.ros.org/en/humble/index.html)
