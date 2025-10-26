#coding=utf-8
"""
Intel RealSense 深度图与RGB图像对齐处理脚本
主要功能：实现深度相机数据的采集、深度图与RGB图像的对齐处理，并支持实时显示和文件回放。
特性：
  - 支持实时相机或bag文件回放两种数据源
  - 提供两种对齐方式：深度图对齐到RGB图像，或RGB图像对齐到深度图
  - 同时输出对齐和未对齐版本的结果用于对比
依赖库：pyrealsense2, numpy, opencv-python
作者：根据您的项目信息填写
版本：1.0
修改日志：
  - 2025-10-03：初始版本，添加详细注释
"""

import pyrealsense2 as rs
import numpy as np
import cv2

# 全局配置变量
USE_ROS_BAG = 0  # 数据源选择：0-使用实时相机，1-使用录制的bag文件
ALIGN_WAY = 1    # 对齐方式选择：0-彩色图像对齐到深度图，1-深度图对齐到彩色图像

def Align_version(frames, align, show_pic=0):
    """
    使用RealSense SDK内置对齐功能处理深度图与RGB图像对齐[1,5](@ref)
    
    通过rs.align对象实现几何对齐，确保深度像素与RGB像素一一对应，适用于精确的RGB-D应用。
    
    Args:
        frames (rs.frameset): 从管道获取的原始帧集合
        align (rs.align): 对齐对象，已配置对齐方式（深度流或彩色流）
        show_pic (int, optional): 是否显示对齐后的图像。默认0-不显示，1-显示
    
    Returns:
        tuple: 包含以下元素的元组：
            - color_image_aligned (numpy.ndarray): 对齐后的RGB图像（H, W, 3）
            - depth_image_aligned (numpy.ndarray): 对齐后的深度图（H, W），单位为毫米
            - depth_colormap_aligned (numpy.ndarray): 深度图的伪彩色图，用于可视化
    """
    # 处理帧对齐：将深度帧和彩色帧对齐到同一坐标系
    aligned_frames = align.process(frames)
    
    # 从对齐后的帧集中提取深度帧和彩色帧
    depth_frame_aligned = aligned_frames.get_depth_frame()
    color_frame_aligned = aligned_frames.get_color_frame()
    
    # 将帧数据转换为OpenCV可处理的numpy数组
    color_image_aligned = np.asanyarray(color_frame_aligned.get_data())
    depth_image_aligned = np.asanyarray(depth_frame_aligned.get_data())
    
    # 如果使用bag文件，需将BGR格式转换为RGB格式（RealSense bag文件存储为BGR）
    if USE_ROS_BAG:
        color_image_aligned = cv2.cvtColor(color_image_aligned, cv2.COLOR_BGR2RGB)
    
    # 生成深度图的伪彩色图：将深度值缩放到0-255范围并应用彩虹色图
    # alpha=0.05用于调整深度值的缩放比例，增强可视化效果
    depth_colormap_aligned = cv2.applyColorMap(
        cv2.convertScaleAbs(depth_image_aligned, alpha=0.05), 
        cv2.COLORMAP_JET
    )
    
    # 水平拼接RGB图像和深度伪彩色图，用于并排显示
    images_aligned = np.hstack((color_image_aligned, depth_colormap_aligned))
    
    # 根据参数决定是否显示对齐结果
    if show_pic:
        cv2.imshow('aligned_images', images_aligned)
    
    return color_image_aligned, depth_image_aligned, depth_colormap_aligned

def Unalign_version(frames, show_pic=0):
    """
    未对齐版本：通过简单的图像缩放实现深度图与RGB图像的尺寸匹配
    
    注意：这种方法仅调整图像尺寸，不进行几何对齐，适用于快速预览或尺寸不敏感的场景。
    
    Args:
        frames (rs.frameset): 从管道获取的原始帧集合
        show_pic (int, optional): 是否显示未对齐的图像。默认0-不显示，1-显示
    
    Returns:
        tuple: 包含以下元素的元组：
            - color_image (numpy.ndarray): 原始或调整后的RGB图像
            - depth_image (numpy.ndarray): 原始或调整后的深度图
            - depth_colormap (numpy.ndarray): 深度图的伪彩色图
    """
    # 从帧集中提取深度帧和彩色帧（未对齐）
    depth_frame = frames.get_depth_frame()
    color_frame = frames.get_color_frame()
    
    # 如果使用实时相机且配置了红外流，提取左右红外图像（用于立体深度计算）
    if not USE_ROS_BAG:
        left_frame = frames.get_infrared_frame(1)  # 左红外摄像头
        right_frame = frames.get_infrared_frame(2) # 右红外摄像头
        
        # 添加安全检查，确保帧存在
        if left_frame and right_frame:
            left_image = np.asanyarray(left_frame.get_data())
            right_image = np.asanyarray(right_frame.get_data())
            # 显示红外图像（如果启用）
            if show_pic:
                cv2.imshow('left_images', left_image)
                cv2.imshow('right_images', right_image)
        else:
            print("警告：未获取到红外帧数据")
    
    # 将彩色帧和深度帧转换为numpy数组
    color_image = np.asanyarray(color_frame.get_data())
    depth_image = np.asanyarray(depth_frame.get_data())
    
    # 打印图像尺寸信息，用于调试
    print("color:", color_image.shape)
    print("depth:", depth_image.shape)
    
    # 如果使用bag文件，可能需要调整图像尺寸：因为录制的RGB和深度图分辨率可能不同
    if USE_ROS_BAG:
        # 转换颜色空间（BGR转RGB）
        color_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)
        # 根据对齐方式调整图像尺寸
        if ALIGN_WAY:  # 深度图对齐到彩色图：将深度图缩放到彩色图尺寸
            depth_image = cv2.resize(depth_image, (color_image.shape[1], color_image.shape[0]))
        else:          # 彩色图对齐到深度图：将彩色图缩放到深度图尺寸
            color_image = cv2.resize(color_image, (depth_image.shape[1], depth_image.shape[0]))
    
    # 生成深度图的伪彩色图用于可视化
    depth_colormap = cv2.applyColorMap(
        cv2.convertScaleAbs(depth_image, alpha=0.05), 
        cv2.COLORMAP_JET
    )
    
    # 水平拼接RGB图像和深度伪彩色图
    images = np.hstack((color_image, depth_colormap))
    if show_pic:
        cv2.imshow('images', images)
    
    return color_image, depth_image, depth_colormap


if __name__ == "__main__":
    """
    主函数：配置RealSense管道，启动流式传输，并循环处理帧数据
    """
    # 创建RealSense管道和配置对象
    pipeline = rs.pipeline()
    config = rs.config()
    
    # 配置数据源：bag文件或实时相机流
    if USE_ROS_BAG:
        # 从bag文件加载数据（用于回放或调试）
        config.enable_device_from_file("666.bag")
    else:
        # 配置实时流参数：
        # - 彩色流：640x480分辨率，BGR格式，30帧/秒
        # - 深度流：640x480分辨率，Z16格式，30帧/秒
        # - 红外流：左右两个红外摄像头，用于深度计算
        # config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 15)
        # config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 15)
        # config.enable_stream(rs.stream.infrared, 1, 640, 480, rs.format.y8, 15)
        # config.enable_stream(rs.stream.infrared, 2, 640, 480, rs.format.y8, 15)
        
        #让SDK自动选择可用的配置，但是保证启动深度传感器，rgb传感器
        config.enable_all_streams()



    # 创建对齐对象：根据ALIGN_WAY决定对齐到彩色流或深度流
    if ALIGN_WAY:
        way = rs.stream.color  # 深度图对齐到彩色图像
    else:
        way = rs.stream.depth  # 彩色图像对齐到深度图
    align = rs.align(way)
    
    # 启动管道并获取设备配置
    profile = pipeline.start(config)
    
    # 获取深度传感器的深度比例系数（将深度值转换为米）
    depth_sensor = profile.get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()
    print("深度比例系数 scale:", depth_scale)  # 典型值约0.001，表示深度单位是毫米
    
    try:
        # 主循环：持续处理帧数据
        while True:
            # 等待下一组连贯的帧（深度、彩色、红外等）
            frames = pipeline.wait_for_frames()
            
            # 处理对齐版本和未对齐版本的图像
            color_image_aligned, depth_image_aligned, depth_colormap_aligned = Align_version(frames, align, show_pic=1)
            color_image, depth_image, depth_colormap = Unalign_version(frames, show_pic=1)
            
            # 可选：打印对齐后深度图的物理距离（例如中心点距离）
            # print(depth_image_aligned * depth_scale)  # 转换为米
            
            # 检测键盘输入，按'q'或ESC键退出循环
            key = cv2.waitKey(1)
            if key & 0xFF == ord('q') or key == 27:
                cv2.destroyAllWindows()
                break
    finally:
        # 确保管道停止并释放资源
        pipeline.stop()

