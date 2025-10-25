from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    """生成启动描述"""
    
    # 获取包的共享目录
    pkg_share = get_package_share_directory('yolov8_ros2')
    
    # 声明启动参数
    config_file = LaunchConfiguration('config_file')
    image_topic = LaunchConfiguration('image_topic')
    
    declare_config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(pkg_share, 'config', 'yolov8_config.yaml'),
        description='Path to the configuration file'
    )
    
    declare_image_topic_arg = DeclareLaunchArgument(
        'image_topic',
        default_value='/camera/camera/image_raw',
        description='Input image topic name'
    )
    
    # 创建YOLOv8节点
    yolov8_node = Node(
        package='yolov8_ros2',
        executable='yolov8_node',
        name='yolov8_ros2_node',
        output='screen',
        parameters=[config_file],
        remappings=[
            ('/camera/camera/image_raw', image_topic)
        ]
    )
    
    # 创建启动描述
    return LaunchDescription([
        declare_config_file_arg,
        declare_image_topic_arg,
        yolov8_node
    ])