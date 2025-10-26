# yolo_launch 包修复说明

## 问题描述

您遇到的错误：
```
launch.substitutions.substitution_failure.SubstitutionFailure: package 'yolo_launch' found at '/home/admin-pc/qyw/yolo_ros2/yolo_ros2_ws/install/yolo_launch', but libexec directory '/home/admin-pc/qyw/yolo_ros2/yolo_ros2_ws/install/yolo_launch/lib/yolo_launch' does not exist
```

## 问题原因

1. **路径问题**：错误信息显示路径中有重复的 `yolo_ros2` 目录
2. **依赖缺失**：`yolo_launch` 包的 `package.xml` 文件缺少必要的依赖声明
3. **构建类型**：包使用 `ament_cmake` 但缺少正确的依赖配置

## 已修复的内容

### 1. 修复 package.xml 文件

**文件位置**：`/home/admin-pc/qyw/yolo_ros2/yolo_ros2_ws/src/yolov8_ros2/yolo_launch/package.xml`

**修复内容**：
```xml
<buildtool_depend>ament_cmake</buildtool_depend>

<exec_depend>launch</exec_depend>
<exec_depend>launch_ros</exec_depend>
<exec_depend>yolov8_ros2</exec_depend>
<exec_depend>realsense_publisher</exec_depend>
```

### 2. 创建新的启动文件

**文件位置**：`/home/admin-pc/qyw/yolo_ros2/yolo_ros2_ws/src/yolov8_ros2/yolo_launch/launch/yolov8_realsense_launch.py`

**功能**：
- 启动 RealSense 相机节点
- 启动 YOLO 深度检测节点
- 支持参数化配置

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

# 测试 RealSense + YOLO 启动
ros2 launch yolo_launch yolov8_realsense_launch.py
```

## 可用的启动文件

1. **yolov8_launch.py** - 基本的 YOLO 节点启动
2. **yolov8_realsense_launch.py** - RealSense + YOLO 完整系统启动
3. **yolov8_distance_node.launch.py** - 距离检测节点启动

## 参数配置

### RealSense 相机参数
- `width`: 图像宽度（默认：640）
- `height`: 图像高度（默认：480）
- `fps`: 帧率（默认：30）

### YOLO 检测参数
- `weight_path`: YOLO 模型路径（默认：/home/admin-pc/qyw/yolov8n.pt）
- `conf`: 置信度阈值（默认：0.5）
- `visualize`: 是否显示可视化窗口（默认：true）
- `use_cpu`: 是否使用CPU推理（默认：true）

## 使用示例

### 基本启动
```bash
ros2 launch yolo_launch yolov8_realsense_launch.py
```

### 自定义参数启动
```bash
ros2 launch yolo_launch yolov8_realsense_launch.py \
    width:=1280 \
    height:=720 \
    fps:=15 \
    conf:=0.6
```

## 故障排除

### 1. 如果仍然出现 libexec 错误
```bash
# 清理构建目录
rm -rf build/ install/ log/
colcon build
```

### 2. 如果找不到包
```bash
# 检查包是否正确安装
ros2 pkg list | grep yolo_launch

# 检查启动文件
ros2 launch yolo_launch --help
```

### 3. 如果依赖问题
```bash
# 检查依赖
rosdep check --from-paths src --ignore-src
```

## 验证修复

运行以下命令验证修复是否成功：

```bash
cd /home/admin-pc/qyw/yolo_ros2/yolo_ros2_ws
./test_launch_fix.sh
```

如果看到可用的启动文件列表，说明修复成功！
