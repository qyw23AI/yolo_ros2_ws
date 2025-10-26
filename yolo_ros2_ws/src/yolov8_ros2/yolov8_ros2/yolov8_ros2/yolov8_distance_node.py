#!/home/admin-pc/anaconda3/envs/yolov8/bin/python

import sys
import pyrealsense2 as rs
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from ultralytics import YOLO
from time import time

# ROS2消息类型导入
from std_msgs.msg import Header
from sensor_msgs.msg import Image
from yolov8_ros2_msgs.msg import BoundingBox, BoundingBoxes
from cv_bridge import CvBridge


class RealSenseYoloDistanceDetector(Node):
    def __init__(self):
        """
        RealSense相机与YOLOv8目标距离检测类
        实现深度图像与RGB图像对齐，目标检测，距离计算和可视化显示
        """
        super().__init__('yolo_detect_distance')
        
        # 声明并获取参数
        self.declare_parameter('weight_path', 'yolov8n.pt')
        self.declare_parameter('conf_threshold', 0.5)
        self.declare_parameter('visualize', False)
        self.declare_parameter('camera_frame', 'camera_color_optical_frame')
        self.declare_parameter('pub_detection_topic', '/realsense_yolo/detections')
        self.declare_parameter('pub_image_topic', '/realsense_yolo/detection_image')
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)
        self.declare_parameter('fps', 30)
        self.declare_parameter('max_distance', 10.0)
        self.declare_parameter('use_cpu', True)
        
        # 获取参数值
        weight_path = self.get_parameter('weight_path').get_parameter_value().string_value
        self.conf_threshold = self.get_parameter('conf_threshold').get_parameter_value().double_value
        self.visualize = self.get_parameter('visualize').get_parameter_value().bool_value
        self.camera_frame = self.get_parameter('camera_frame').get_parameter_value().string_value
        self.detection_pub_name = self.get_parameter('pub_detection_topic').get_parameter_value().string_value
        self.visualization_pub_name = self.get_parameter('pub_image_topic').get_parameter_value().string_value
        self.image_width = self.get_parameter('image_width').get_parameter_value().integer_value
        self.image_height = self.get_parameter('image_height').get_parameter_value().integer_value
        self.fps = self.get_parameter('fps').get_parameter_value().integer_value
        self.max_distance = self.get_parameter('max_distance').get_parameter_value().double_value
        use_cpu = self.get_parameter('use_cpu').get_parameter_value().bool_value
        
        # 确定使用的计算设备
        self.device = 'cuda' if not use_cpu else 'cpu'
        
        # 初始化YOLO模型
        self.model = YOLO(weight_path)
        self.model.fuse()  # 融合模型层以提高推理速度
        self.model.conf = self.conf_threshold
        self.model.to(self.device)
        
        # 初始化RealSense相机
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        
        # 配置RealSense流
        self.config.enable_stream(rs.stream.color, self.image_width, self.image_height, rs.format.bgr8, self.fps)
        self.config.enable_stream(rs.stream.depth, self.image_width, self.image_height, rs.format.z16, self.fps)
        
        # 创建对齐对象（深度图对齐到彩色图）
        self.align = rs.align(rs.stream.color)
        
        # 启动RealSense相机
        self.profile = self.pipeline.start(self.config)
        
        # 获取深度传感器的比例因子
        self.depth_sensor = self.profile.get_device().first_depth_sensor()
        self.depth_scale = self.depth_sensor.get_depth_scale()
        self.get_logger().info(f"深度比例系数: {self.depth_scale}")
        
        # 创建ROS2发布者
        self.detection_pub = self.create_publisher(BoundingBoxes, self.detection_pub_name, 10)
        self.visualization_pub = self.create_publisher(Image, self.visualization_pub_name, 10)
        
        # 初始化CV Bridge
        self.bridge = CvBridge()
        
        self.get_logger().info("RealSense YOLO距离检测器初始化完成")
        
        # 启动主循环
        self.timer = self.create_timer(1.0/self.fps, self.run)
    
    def run(self):
        """
        主循环函数，持续获取图像帧并进行目标检测和距离计算
        """
        try:
            # 等待下一组连贯的帧
            frames = self.pipeline.wait_for_frames()
            
            # 对齐深度帧到彩色帧
            aligned_frames = self.align.process(frames)
            
            # 获取对齐后的帧
            color_frame = aligned_frames.get_color_frame()
            depth_frame = aligned_frames.get_depth_frame()
            
            if not color_frame or not depth_frame:
                return
            
            # 将帧转换为numpy数组
            color_image = np.asanyarray(color_frame.get_data()) 
            depth_image = np.asanyarray(depth_frame.get_data())
            
            # 使用YOLO模型进行目标检测
            results = self.model(color_image, show=False, conf=self.conf_threshold)
            
            # 处理检测结果并计算距离
            self.process_detections(results, color_image, depth_image, color_frame)
            
        except Exception as e:
            self.get_logger().error(f"处理帧时出错: {str(e)}")
    
    def process_detections(self, results, color_image, depth_image, color_frame):
        """
        处理检测结果并计算目标距离
        
        Args:
            results: YOLO检测结果
            color_image: 彩色图像
            depth_image: 深度图像
            color_frame: 彩色帧对象
        """
        # 绘制检测结果，将YOLO检测到的边界框和标签绘制在图像上
        frame = results[0].plot()
        
        # 创建边界框消息容器，用于ROS消息发布
        bounding_boxes = BoundingBoxes()
        header = Header()
        header.stamp = self.get_clock().now().to_msg()  # 设置时间戳为当前时间
        header.frame_id = self.camera_frame  # 设置坐标系ID
        bounding_boxes.header = header  # 设置消息头部
        bounding_boxes.image_header = header  # 设置图像头部
        
        # 遍历所有检测到的目标（每个result代表一个检测到的对象）
        for i, result in enumerate(results[0].boxes):
            # 提取边界框坐标 (xmin, ymin, xmax, ymax)
            # xyxy格式表示: [左上角x, 左上角y, 右下角x, 右下角y]
            xmin = (result.xyxy[0][0].item())  # 左上角x坐标
            ymin = (result.xyxy[0][1].item())  # 左上角y坐标
            xmax = (result.xyxy[0][2].item())  # 右下角x坐标
            ymax = (result.xyxy[0][3].item())  # 右下角y坐标
            
            # 计算边界框中心点坐标，用于获取该目标中心位置的深度值
            center_x = int((xmin + xmax) / 2)  # 中心点x坐标
            center_y = int((ymin + ymax) / 2)  # 中心点y坐标
            
            # 确保中心点在图像范围内，防止索引越界
            # max(0, x)确保不小于0，min(x, max_index)确保不大于最大索引
            center_x = max(0, min(center_x, depth_image.shape[1] - 1))  # 限制x坐标在图像宽度范围内
            center_y = max(0, min(center_y, depth_image.shape[0] - 1))  # 限制y坐标在图像高度范围内
            
            # 获取深度值（原始深度值通常以毫米为单位，乘以深度比例系数得到米）
            # depth_image[center_y, center_x]获取中心点的深度值
            # self.depth_scale是深度比例系数，用于将深度值转换为实际距离（米）
            depth_value = depth_image[center_y, center_x] * self.depth_scale
            
            # 只处理有效深度值且在最大距离范围内的目标
            # 0 < depth_value确保深度值有效（非零），depth_value < self.max_distance确保在设定的最大距离内
            if 0 < depth_value < self.max_distance:
                # 创建单个边界框消息对象
                bounding_box = BoundingBox()
                # 设置边界框的四个坐标值
                bounding_box.xmin = xmin  # 左上角x坐标
                bounding_box.ymin = ymin  # 左上角y坐标
                bounding_box.xmax = xmax  # 右下角x坐标
                bounding_box.ymax = ymax  # 右下角y坐标
                # 设置检测到的物体类别名称
                bounding_box.class_name = results[0].names[result.cls.item()]  # 根据类别索引获取类别名称
                # 设置检测置信度
                bounding_box.probability = result.conf.item()  # 获取该检测的置信度分数
                
                # 添加距离信息到类名中（此变量在此处定义但未被使用）
                class_name = results[0].names[result.cls.item()]  # 再次获取类别名称
                distance_text = f"{class_name} ({depth_value:.2f}m)"  # 构造包含距离的文本
                
                # 在图像上绘制距离信息
                # 在边界框上方显示距离值，格式化为保留两位小数的米单位
                cv2.putText(frame, f'{depth_value:.2f}m', 
                           (int(xmin), int(ymin) - 10),  # 文本位置（在边界框上方10像素）
                           cv2.FONT_HERSHEY_SIMPLEX,  # 字体类型
                           0.6,  # 字体大小
                           (0, 255, 0),  # 字体颜色（绿色BGR格式）
                           2)  # 字体粗细
                
                # 将处理好的边界框添加到边界框列表中
                bounding_boxes.bounding_boxes.append(bounding_box)
        
        # 发布检测结果（包含所有检测到的目标信息）
        self.detection_pub.publish(bounding_boxes)
        
        # 计算并显示FPS（每秒帧数）
        # 检查results[0]是否有speed属性（推理时间信息）
        if hasattr(results[0], 'speed'):
            # 计算FPS：1000.0除以推理时间（因为推理时间以毫秒为单位）
            fps = 1000.0 / results[0].speed['inference']
            # 在图像上绘制FPS信息
            cv2.putText(frame, f'FPS: {int(fps)}', (20, 50),  # 在图像左上角显示FPS
                       cv2.FONT_HERSHEY_SIMPLEX,  # 字体类型
                       0.6,  # 字体大小
                       (0, 255, 0),  # 字体颜色（绿色）
                       2,  # 字体粗细
                       cv2.LINE_AA)  # 抗锯齿线型
            
        
        # 发布可视化图像（带边界框和距离标注的图像）
        self.publish_image(frame, color_frame)
        
        # 如果启用可视化，则在OpenCV窗口中显示检测结果
        if self.visualize:
            cv2.imshow('RealSense YOLO Distance Detection', frame)
            cv2.waitKey(1)
    
    def publish_image(self, imgdata, color_frame):
        """
        发布带检测框和距离信息的图像消息
        
        Args:
            imgdata: 图像数据（numpy数组）
            color_frame: 彩色帧对象
        """
        try:
            # 使用CV Bridge转换图像
            image_msg = self.bridge.cv2_to_imgmsg(imgdata, encoding='bgr8')
            
            # 设置消息头
            image_msg.header.stamp = self.get_clock().now().to_msg()
            image_msg.header.frame_id = self.camera_frame
            
            # 发布图像消息
            self.visualization_pub.publish(image_msg)
        except Exception as e:
            self.get_logger().error(f"发布图像时出错: {str(e)}")
    
    def destroy_node(self):
        """
        关闭函数，清理资源
        """
        self.get_logger().info("关闭RealSense相机...")
        self.pipeline.stop()
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    """
    主函数：初始化ROS节点并启动RealSense YOLO距离检测器
    """
    rclpy.init(args=args)
    
    try:
        # 创建检测器节点
        detector = RealSenseYoloDistanceDetector()
        
        # 运行节点
        rclpy.spin(detector)
        
    except Exception as e:
        detector.get_logger().error(f"发生错误: {str(e)}")
    finally:
        # 清理资源
        detector.destroy_node()
        rclpy.shutdown()
        detector.get_logger().info("程序结束")


if __name__ == "__main__":
    """
    程序入口点
    """
    print("Python version:", sys.version)
    main()