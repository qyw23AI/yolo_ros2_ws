#coding=utf-8
"""
RealSense深度相机工具函数模块
功能：提供相机信息获取和文件路径创建的辅助函数
作者：根据您的项目信息填写
版本：1.0
"""

import datetime
import pyrealsense2 as rs
import os
import cv2
import numpy as np
def get_depth_camera_info(profile):
    """
    获取深度相机的内参信息和深度值比例因子
    
    通过RealSense配置信息获取深度传感器的详细参数，包括相机内参和深度值转换比例。
    这些信息对于深度数据的正确解析和三维重建至关重要。
    
    Args:
        profile (rs.pipeline_profile): 已启动的RealSense管道配置对象，包含设备信息
        
    Returns:
        None: 直接打印输出相机信息，不返回具体值
        
    Raises:
        RuntimeError: 如果无法获取深度传感器或流配置时可能抛出异常
        
    Example:
        >>> profile = pipeline.start(config)
        >>> get_depth_camera_info(profile)
        Depth Scale is: 0.001
        Width: 640
        Height: 480
        ...
    """
    # 获取深度传感器设备对象
    depth_sensor = profile.get_device().first_depth_sensor()
    
    # 获取深度流配置信息
    depth_stream = profile.get_stream(rs.stream.depth)
    
    # 获取深度相机的内参信息（相机标定参数）
    depth_intrinsics = depth_stream.as_video_stream_profile().get_intrinsics()
    
    # 获取深度值比例因子：将原始深度值转换为米所需的乘数
    # 典型值约为0.001，表示深度单位是毫米
    depth_scale = depth_sensor.get_depth_scale()
    
    # 输出深度比例因子（深度值到米的转换系数）
    print("Depth Scale is: ", depth_scale)
    
    # 输出深度相机内参详细信息
    print("Depth intrinsics:")
    print(f"Width: {depth_intrinsics.width}")  # 图像宽度（像素）
    print(f"Height: {depth_intrinsics.height}")  # 图像高度（像素）
    print(f"PPX (principal point x): {depth_intrinsics.ppx}")  # 主点x坐标（光学中心）
    print(f"PPY (principal point y): {depth_intrinsics.ppy}")  # 主点y坐标（光学中心）
    print(f"FX (focal length x): {depth_intrinsics.fx}")  # x轴焦距（像素单位）
    print(f"FY (focal length y): {depth_intrinsics.fy}")  # y轴焦距（像素单位）
    print(f"Distortion model: {depth_intrinsics.model}")  # 畸变模型类型
    print(f"Distortion coefficients: {depth_intrinsics.coeffs}")  # 畸变系数矩阵[6](@ref)

def create_camera_save_path(save_path=None):
    """
    创建并返回D455相机图像和深度信息的保存路径
    
    根据当前时间生成唯一的文件夹结构，避免数据覆盖。支持自定义保存路径，
    默认使用当前工作目录。自动创建不存在的目录。
    
    Args:
        save_path (str, optional): 自定义保存根路径。默认为None，使用当前工作目录
        
    Returns:
        tuple: 包含两个元素的元组：
            - color_path (str): 彩色图像保存路径
            - depth_path (str): 深度数据保存路径
            
    Raises:
        OSError: 如果目录创建失败可能抛出权限错误
        
    Example:
        >>> color_path, depth_path = create_camera_save_path()
        >>> print(color_path)
        /home/user/2024_10_03_14_30_00/rgb
    """
    # 如果未提供保存路径，使用当前工作目录
    if save_path is None:
        save_path = os.getcwd()
    
    # 生成基于当前时间的时间戳字符串，格式：年_月_日_时_分_秒
    # 替换冒号为下划线以确保文件名兼容性
    time_path = f"{datetime.datetime.now():%Y_%m_%d_%H_%M_%S}".replace(":", "_")
    
    # 构建彩色图像和深度数据的完整保存路径
    color_path = os.path.join(save_path, time_path, 'rgb')  # 彩色图像保存路径
    depth_path = os.path.join(save_path, time_path, 'depth')  # 深度数据保存路径
    
    # 创建目录（如果不存在），exist_ok=True避免目录已存在时抛出异常
    os.makedirs(color_path, exist_ok=True)
    os.makedirs(depth_path, exist_ok=True)
    
    return color_path, depth_path

#coding=utf-8
"""
RealSense D455深度相机数据采集主程序
功能：实时捕获彩色和深度图像，支持手动保存和可视化
依赖：pyrealsense2, opencv-python, numpy
作者：根据您的项目信息填写
版本：1.0
"""

# ==================== 全局配置参数 ====================
saved_count = 0       # 已保存图像计数器，从0开始递增
extend_num = 3        # 文件扩展名长度（未使用，可删除或用于其他用途）
width = 640           # 图像采集宽度（像素）
height = 480          # 图像采集高度（像素）
fps = 30              # 采集帧率（帧/秒）

# ==================== 初始化相机和保存路径 ====================
# 创建按时间戳命名的保存目录
color_path, depth_path = create_camera_save_path()

# 创建RealSense管道和配置对象
pipeline = rs.pipeline()
config = rs.config()

# 配置深度流：Z16格式表示16位深度数据，单位毫米
config.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)

# 配置彩色流：BGR8格式表示24位彩色图像（OpenCV标准格式）
config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)

# 启动管道并获取配置信息
profile = pipeline.start(config)

# 打印相机详细信息（内参和深度比例因子）
get_depth_camera_info(profile)

# ==================== 主采集循环 ====================
try:
    # 持续运行直到用户退出
    while True:
        # 等待获取一组连贯的帧（深度+彩色）
        frames = pipeline.wait_for_frames()
        
        # 从帧集中提取深度帧和彩色帧
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        
        # 将帧数据转换为numpy数组供OpenCV处理
        depth_image = np.asanyarray(depth_frame.get_data())  # 深度图（原始数据，单位毫米）
        color_image = np.asanyarray(color_frame.get_data())  # 彩色图像（BGR格式）
        
        # ==================== 深度数据处理 ====================
        # 获取深度比例因子（将原始深度值转换为米）
        depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()
        
        # 将深度图像转换为物理距离（米单位）
        depth_image_in_meters = depth_image * depth_scale
        
        # 将无效深度值（0值，通常表示测距失败）设置为NaN
        # NaN值在后续处理中容易被识别和过滤
        depth_image_in_meters[depth_image == 0] = np.nan
        
        # ==================== 可视化处理 ====================
        # 创建深度图的伪彩色图：将深度值缩放到0-255并应用彩虹色图
        # alpha=0.03调整对比度，使深度差异更明显
        depth_colormap = cv2.applyColorMap(
            cv2.convertScaleAbs(depth_image, alpha=0.03), 
            cv2.COLORMAP_JET
        )
        
        # 水平拼接彩色图像和深度伪彩色图用于并排显示
        images = np.hstack((color_image, depth_colormap))
        
        # 创建自适应窗口并显示合成图像
        cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('RealSense', images)
        
        # ==================== 键盘事件处理 ====================
        # 等待键盘输入（1毫秒超时，非阻塞）
        key = cv2.waitKey(1) & 0xFF  # 取低8位确保跨平台兼容性
        
        # 's'键：保存当前帧
        if key & 0xFF == ord('s'):
            saved_count += 1  # 递增计数器
            
            print(f"{saved_count} 已保存图像至 {color_path} 和 {depth_path}")
            
            # 保存彩色图像为PNG格式（无损压缩）
            cv2.imwrite(os.path.join(color_path, "{}.png".format(saved_count)), color_image)
            
            # 保存深度数据为NPY格式（保留原始精度，单位为米）
            # NPY格式适合存储数值数据，便于后续分析
            np.save(os.path.join(depth_path, "{}.npy".format(saved_count)), depth_image_in_meters)
        
        # 'q'键或ESC键：退出程序
        elif key & 0xFF == ord('q') or key == 27:
            print("👋 用户退出程序")
            cv2.destroyAllWindows()
            break

finally:
    # ==================== 资源清理 ====================
    # 确保管道停止并释放相机资源（即使发生异常也会执行）
    pipeline.stop()
    print("相机资源已释放，程序安全退出")