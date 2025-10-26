# yolo_launch 包 libexec 错误修复说明

## 问题分析

您遇到的错误：
```
SubstitutionFailure: package 'yolo_launch' found at '/home/admin-pc/qyw/yolo_ros2/yolo_ros2_ws/install/yolo_launch', but libexec directory '/home/admin-pc/qyw/yolo_ros2/yolo_ros2_ws/install/yolo_launch/lib/yolo_launch' does not exist
```

## 根本原因

1. **构建类型不匹配**：`yolo_launch` 包使用 `ament_cmake` 构建类型
2. **错误的包引用**：启动文件试图从 `yolo_launch` 包中执行节点
3. **libexec 目录缺失**：`ament_cmake` 包不会创建 `libexec` 目录

## 修复方案

### 1. 修复启动文件中的包引用

**问题**：启动文件试图从 `yolo_launch` 包执行节点
```python
# 错误的配置
Node(
    package='yolo_launch',  # ❌ 错误：yolo_launch 是 ament_cmake 包
    executable='yolov8_distance_node',  # ❌ 错误：可执行文件不存在
    ...
)
```

**修复**：从正确的包中执行节点
```python
# 正确的配置
Node(
    package='yolov8_ros2',  # ✅ 正确：yolov8_ros2 是 ament_python 包
    executable='yolov8_depth_node',  # ✅ 正确：可执行文件存在
    ...
)
```

### 2. 已修复的文件

#### yolov8_distance_node.launch.py
- ✅ 修改包名：`yolo_launch` → `yolov8_ros2`
- ✅ 修改可执行文件名：`yolov8_distance_node` → `yolov8_depth_node`
- ✅ 添加参数化配置
- ✅ 移除不存在的配置文件引用

#### yolov8_realsense_launch.py（新建）
- ✅ 创建完整的 RealSense + YOLO 启动文件
- ✅ 支持参数化配置
- ✅ 正确的包引用

## 解决步骤

### 步骤 1：重新构建工作空间
```bash
cd /home/admin-pc/qyw/yolo_ros2/yolo_ros2_ws
source /opt/ros/foxy/setup.bash
colcon build
```

### 步骤 2：设置环境
```bash
source install/setup.bash
```

### 步骤 3：测试启动文件
```bash
# 测试基本 YOLO 启动
ros2 launch yolo_launch yolov8_launch.py

# 测试距离检测节点启动
ros2 launch yolo_launch yolov8_distance_node.launch.py

# 测试 RealSense + YOLO 启动
ros2 launch yolo_launch yolov8_realsense_launch.py
```

## 可用的启动文件

1. **yolov8_launch.py**
   - 启动基本的 YOLO 检测节点
   - 订阅图像话题进行检测

2. **yolov8_distance_node.launch.py**
   - 启动 YOLO 深度检测节点
   - 支持距离计算功能

3. **yolov8_realsense_launch.py**
   - 启动完整的 RealSense + YOLO 系统
   - 包含相机和检测节点

## 参数配置

### YOLO 检测参数
- `weight_path`: YOLO 模型路径（默认：/home/admin-pc/qyw/yolov8n.pt）
- `conf`: 置信度阈值（默认：0.5）
- `visualize`: 是否显示可视化窗口（默认：true）
- `use_cpu`: 是否使用CPU推理（默认：true）
- `camera_frame`: 相机坐标系（默认：camera_link）

### RealSense 相机参数
- `width`: 图像宽度（默认：640）
- `height`: 图像高度（默认：480）
- `fps`: 帧率（默认：30）

## 使用示例

### 基本启动
```bash
ros2 launch yolo_launch yolov8_distance_node.launch.py
```

### 自定义参数启动
```bash
ros2 launch yolo_launch yolov8_distance_node.launch.py \
    weight_path:=/path/to/your/model.pt \
    conf:=0.6 \
    visualize:=false
```

### 完整系统启动
```bash
ros2 launch yolo_launch yolov8_realsense_launch.py
```

## 验证修复

运行测试脚本验证修复：
```bash
cd /home/admin-pc/qyw/yolo_ros2/yolo_ros2_ws
chmod +x test_fixed_launch.sh
./test_fixed_launch.sh
```

## 故障排除

### 1. 如果仍然出现 libexec 错误
```bash
# 清理构建目录
rm -rf build/ install/ log/
colcon build
```

### 2. 如果找不到可执行文件
```bash
# 检查可执行文件
ros2 pkg executables yolov8_ros2
```

### 3. 如果参数文件错误
```bash
# 检查配置文件
ros2 param list /yolov8_depth_node
```

现在 `yolo_launch` 包应该可以正常工作了！
