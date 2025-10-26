from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 获取包共享目录
    pkg_dir = get_package_share_directory('yolo_launch')
    
    # 声明启动参数
    weight_path_arg = DeclareLaunchArgument(
        'weight_path',
        default_value='/home/admin-pc/qyw/yolov8n.pt',
        description='Path to YOLO model weights'
    )
    
    conf_arg = DeclareLaunchArgument(
        'conf',
        default_value='0.5',
        description='YOLO confidence threshold'
    )
    
    visualize_arg = DeclareLaunchArgument(
        'visualize',
        default_value='true',
        description='Enable visualization'
    )
    
    use_cpu_arg = DeclareLaunchArgument(
        'use_cpu',
        default_value='true',
        description='Use CPU for YOLO inference'
    )
    
    camera_frame_arg = DeclareLaunchArgument(
        'camera_frame',
        default_value='camera_link',
        description='Camera frame ID'
    )
    
    # 创建节点 - 从正确的包中执行
    yolo_detector_node = Node(
        package='yolov8_ros2',  # 修改为正确的包名
        executable='yolov8_distance_node',  # 修改为正确的可执行文件名
        name='yolov8_distance_node',
        output='screen',
        parameters=[
            {
                'weight_path': LaunchConfiguration('weight_path'),
                'conf_threshold': LaunchConfiguration('conf'),
                'visualize': LaunchConfiguration('visualize'),
                'use_cpu': LaunchConfiguration('use_cpu'),
                'camera_frame': LaunchConfiguration('camera_frame'),
                'image_width': 640,
                'image_height': 480,
                'fps': 30,
                'max_distance': 10.0,
                'pub_detection_topic': '/realsense_yolo/detections',
                'pub_image_topic': '/realsense_yolo/detection_image'
            }
        ]
    )
    
    # 创建启动描述
    return LaunchDescription([
        weight_path_arg,
        conf_arg,
        visualize_arg,
        use_cpu_arg,
        camera_frame_arg,
        yolo_detector_node
    ])