
# -*- coding : utf-8 -*-

# 导入必要的库
import cv2              # OpenCV库，用于图像处理
import torch            # PyTorch深度学习框架
import rclpy            # ROS2 Python客户端库
import numpy as np      # 数值计算库
from ultralytics import YOLO    # YOLOv8目标检测模型
from time import time   # 时间处理
from rclpy.node import Node     # ROS2节点基类

# ROS2消息类型导入
from std_msgs.msg import Header          # 标准消息头
from sensor_msgs.msg import Image        # 图像消息
from yolov8_ros2_msgs.msg import BoundingBox, BoundingBoxes  # 自定义边界框消息


class YoloDectNode(Node):
    def __init__(self):
        """
        YOLOv8目标检测类的初始化函数
        负责加载模型参数、订阅图像话题、发布检测结果
        """
        super().__init__('yolov8_ros2_node')  # 初始化ROS2节点

        # 声明并获取参数
        self.declare_parameter('weight_path', 'yolov8n.pt')
        self.declare_parameter('image_topic', '/camera/color/image_raw')
        self.declare_parameter('pub_topic', '/yolov8/BoundingBoxes')
        self.declare_parameter('camera_frame', '')
        self.declare_parameter('conf', 0.5)
        self.declare_parameter('visualize', False)
        self.declare_parameter('use_cpu', True)

        # 获取参数值
        weight_path = self.get_parameter('weight_path').get_parameter_value().string_value
        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        pub_topic = self.get_parameter('pub_topic').get_parameter_value().string_value
        self.camera_frame = self.get_parameter('camera_frame').get_parameter_value().string_value
        conf = self.get_parameter('conf').get_parameter_value().double_value
        self.visualize = self.get_parameter('visualize').get_parameter_value().bool_value
        use_cpu = self.get_parameter('use_cpu').get_parameter_value().bool_value

        # 确定使用的计算设备（CPU或GPU）
        if use_cpu:
            self.device = 'cpu'
        else:
            # 检查CUDA是否可用
            if torch.cuda.is_available():
                self.device = 'cuda'
            else:
                self.device = 'cpu'
                self.get_logger().warning('CUDA not available, using CPU instead')

        # 加载YOLOv8模型
        self.get_logger().info(f'Loading YOLOv8 model from {weight_path}')
        self.model = YOLO(weight_path)
        self.model.to(self.device)  # 将模型移动到指定设备
        self.model.fuse()  # 融合模型层以提高推理速度

        # 设置模型置信度阈值
        self.model.conf = float(conf)
        self.color_image = None      # 初始化图像变量
        self.getImageStatus = False     # 图像接收状态标志

        # 用于存储不同类别颜色的字典
        self.classes_colors = {}

        # 订阅图像话题
        # queue_size=1: 消息队列大小为1，避免延迟
        self.color_sub = self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            1)
        self.color_sub  # 防止未使用变量警告

        # 创建发布者用于发布检测结果
        self.position_pub = self.create_publisher(
            BoundingBoxes,
            pub_topic,
            1)

        self.image_pub = self.create_publisher(
            Image,
            '/yolov8/detection_image',
            1)

        # 创建定时器，定期检查图像接收状态
        self.timer = self.create_timer(2.0, self.check_image_status)

        self.get_logger().info('YOLOv8 ROS2 node initialized successfully')

    def check_image_status(self):
        """检查图像接收状态的定时器回调函数"""
        if not self.getImageStatus:
            self.get_logger().info("Waiting for image...")

    def image_callback(self, image):
        """
        图像回调函数，处理接收到的图像消息
        
        Args:
            image (sensor_msgs.msg.Image): 接收到的ROS2图像消息
        """
        # 初始化边界框消息
        self.boundingBoxes = BoundingBoxes()
        self.boundingBoxes.header = image.header      # 复用图像消息的时间戳和坐标系
        self.boundingBoxes.image_header = image.header
        self.getImageStatus = True                    # 设置图像接收状态为True

        # 将ROS2图像消息转换为numpy数组
        # np.frombuffer: 从字节数据创建numpy数组
        # reshape: 重新调整数组形状为(height, width, channels)
        self.color_image = np.frombuffer(image.data, dtype=np.uint8).reshape(
            image.height, image.width, -1)

        # 转换颜色空间：BGR转RGB（因为ROS使用BGR，而YOLO使用RGB）
        self.color_image = cv2.cvtColor(self.color_image, cv2.COLOR_BGR2RGB)

        # 使用YOLO模型进行目标检测
        # show=False: 不显示检测结果（在回调函数中处理显示）
        results = self.model(self.color_image, show=False, conf=self.model.conf)

        # 处理检测结果并显示
        self.dectshow(results, image.height, image.width)

        # 等待3毫秒，允许OpenCV处理窗口事件
        cv2.waitKey(3)

    def dectshow(self, results, height, width):
        """
        处理检测结果并可视化显示
        
        Args:
            results: YOLO检测结果
            height (int): 图像高度
            width (int): 图像宽度
        """
        # 绘制检测结果（边界框、标签、置信度）
        self.frame = results[0].plot()
        
        # 计算并显示FPS（每秒帧数）
        if hasattr(results[0], 'speed') and 'inference' in results[0].speed:
            inference_time = results[0].speed['inference']
            fps = 1000.0 / inference_time  # 将毫秒转换为FPS
            # 在图像左上角显示FPS信息
            cv2.putText(self.frame, f'FPS: {int(fps)}', (20,50), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2, cv2.LINE_AA)
            self.get_logger().debug(f'Inference time: {inference_time:.2f}ms, FPS: {fps:.1f}')

        # 遍历所有检测到的目标
        for result in results[0].boxes:
            # 创建边界框消息
            boundingBox = BoundingBox()
            
            # 提取边界框坐标并转换为整数
            boundingBox.xmin = float(result.xyxy[0][0].item())
            boundingBox.ymin = float(result.xyxy[0][1].item())
            boundingBox.xmax = float(result.xyxy[0][2].item())
            boundingBox.ymax = float(result.xyxy[0][3].item())
                        
            # 获取类别名称和置信度
            boundingBox.class_name = results[0].names[result.cls.item()]  # 类别名称
            boundingBox.probability = result.conf.item()             # 置信度
            
            # 将边界框添加到边界框列表中
            self.boundingBoxes.bounding_boxes.append(boundingBox)
        
        # 发布检测结果
        self.position_pub.publish(self.boundingBoxes)
        
        # 发布带检测框的图像
        self.publish_image(self.frame, height, width)

        # 如果启用可视化，则显示检测结果
        if self.visualize:
            cv2.imshow('YOLOv8', self.frame)

    def publish_image(self, imgdata, height, width):
        """
        发布带检测框的图像消息
        
        Args:
            imgdata: 图像数据（numpy数组）
            height (int): 图像高度
            width (int): 图像宽度
        """
        # 创建图像消息
        image_temp = Image()
        
        # 设置消息头
        header = Header()
        header.stamp = self.get_clock().now().to_msg()  # 设置当前时间戳
        header.frame_id = self.camera_frame             # 设置坐标系
        
        # 设置图像属性
        image_temp.height = height                   # 图像高度
        image_temp.width = width                     # 图像宽度
        image_temp.encoding = 'bgr8'                 # 图像编码格式（BGR 8位）
        image_temp.data = np.array(imgdata).tobytes()  # 将numpy数组转换为字节数据
        image_temp.header = header                   # 设置消息头
        image_temp.step = width * 3                  # 每行字节数（width * channels）
        
        # 发布图像消息
        self.image_pub.publish(image_temp)


def main(args=None):
    """
    主函数：初始化ROS2节点并启动YOLO检测
    """
    # 初始化ROS2
    rclpy.init(args=args)
    
    # 创建YOLO检测节点
    yolo_dect_node = YoloDectNode()
    
    try:
        # 循环处理ROS2回调
        rclpy.spin(yolo_dect_node)
    except KeyboardInterrupt:
        yolo_dect_node.get_logger().info('Node interrupted by user')
    finally:
        # 清理资源
        cv2.destroyAllWindows()
        yolo_dect_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    """
    程序入口点
    """
   
    print("PyTorch version:", torch.__version__)
    print("CUDA available:", torch.cuda.is_available())
    main()