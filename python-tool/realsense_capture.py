#coding=utf-8
"""
Intel RealSense 自动图像采集工具
主要功能：使用Intel RealSense相机进行高质量图像采集，适用于YOLO训练数据收集
特性：
  - 支持命令行参数配置，灵活调整采集参数
  - 自动图像清晰度检测，避免保存模糊图像
  - 多种采集模式：单张保存、连续采集模式
  - 实时显示采集状态和图像质量指标
依赖库：pyrealsense2, numpy, opencv-python, pathlib, argparse
版本：1.0
修改日志：
  - 2025-10-03：添加详细注释，优化代码可读性
"""

import pyrealsense2 as rs
import numpy as np
import cv2
import os
import time
import argparse
from pathlib import Path

class RealSenseCapture:
    """
    RealSense相机图像采集类
    
    提供完整的图像采集功能，包括相机初始化、图像质量检测、多模式采集和文件保存。
    适用于机器学习训练数据的大规模采集。
    
    Attributes:
        output_dir (str): 图像保存目录路径
        max_count (int): 最大采集数量
        start_count (int): 起始编号
        save_count (int): 当前保存计数
        image_format (str): 图像保存格式
        resolution (str): 采集分辨率
        fps (int): 采集帧率
        pipeline (rs.pipeline): RealSense管道对象
        config (rs.config): 相机配置对象
        frame_count (int): 帧计数器
        current_frame (numpy.ndarray): 当前帧图像
        window_name (str): 显示窗口名称
    """
    
    def __init__(self, output_dir=None, max_count=100, start_count=0, image_format='jpg', resolution='1280x720', fps=30):
        """
        初始化RealSense采集器
        
        Args:
            output_dir (str, optional): 图像保存目录。默认为用户下载目录下的realsense_capture文件夹
            max_count (int, optional): 最大采集图像数量。默认100张
            start_count (int, optional): 起始编号。默认从0开始
            image_format (str, optional): 图像保存格式。支持jpg、jpeg、png
            resolution (str, optional): 采集分辨率格式为'宽度x高度'。默认1280x720
            fps (int, optional): 采集帧率。默认30fps
        
        Raises:
            Exception: 相机初始化失败时抛出异常
        """
        # 设置输出目录：如果未指定则使用默认下载目录
        if output_dir is None:
            self.output_dir = str(Path.home() / "下载" / "realsense_capture")
        else:
            self.output_dir = output_dir
        
        # 创建保存目录（如果不存在）
        os.makedirs(self.output_dir, exist_ok=True)
        
        # 初始化采集参数
        self.max_count = max_count
        self.start_count = start_count
        self.save_count = start_count  # 当前保存计数器
        self.image_format = image_format.lower()  # 统一转为小写
        self.resolution = resolution
        self.fps = fps
        
        # 验证图像格式合法性，不支持则使用默认jpg格式
        if self.image_format not in ['jpg', 'jpeg', 'png']:
            print(f"⚠️  不支持的图像格式: {self.image_format}, 使用默认格式: jpg")
            self.image_format = 'jpg'
        
        # 解析分辨率字符串为宽度和高度数值
        width, height = map(int, resolution.split('x'))
        
        # 初始化RealSense管道和配置对象
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        
        # 配置彩色流参数：分辨率、格式和帧率
        self.config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
        
        # 启动管道并获取配置信息
        pipeline_profile = self.pipeline.start(self.config)
        
        # 获取相机设备并优化参数以减少图像模糊
        device = pipeline_profile.get_device()
        color_sensor = device.first_color_sensor()
        
        # 设置相机自动曝光（如果支持）
        if color_sensor.supports(rs.option.enable_auto_exposure):
            color_sensor.set_option(rs.option.enable_auto_exposure, 1)
        
        # 设置其他优化参数以获得更清晰的图像
        if color_sensor.supports(rs.option.exposure):
            color_sensor.set_option(rs.option.exposure, 166)  # 曝光值
        if color_sensor.supports(rs.option.gain):
            color_sensor.set_option(rs.option.gain, 16)  # 增益值
        if color_sensor.supports(rs.option.auto_exposure_priority):
            color_sensor.set_option(rs.option.auto_exposure_priority, 0)  # 关闭自动曝光优先级
        
        # 初始化帧计数和当前帧变量
        self.frame_count = 0
        self.current_frame = None
        self.window_name = 'RealSense Capture'  # 显示窗口名称
        
        # 打印配置信息
        print(f"     采集配置:")
        print(f"   - 保存路径: {self.output_dir}")
        print(f"   - 最大数量: {self.max_count}")
        print(f"   - 起始编号: {self.start_count}")
        print(f"   - 图像格式: {self.image_format}")
        print(f"   - 分辨率: {self.resolution}")
        print(f"   - 帧率: {self.fps}")
    
    def save_frame(self):
        """
        保存当前帧到文件
        
        执行图像清晰度检测，只有清晰度达到阈值的图像才会被保存。
        文件名包含时间戳和序列号，便于后续管理。
        
        Returns:
            bool: 保存成功返回True，失败返回False
        """
        if self.current_frame is None:
            print(" 无可用帧可保存")
            return False
        
        # 检查是否达到最大保存数量
        if self.save_count >= self.max_count + self.start_count:
            print(f"  已达到最大保存数量 {self.max_count}")
            return False
        
        # 计算图像清晰度（拉普拉斯方差）
        sharpness = self.calculate_sharpness(self.current_frame)
        if sharpness < 70:  # 清晰度阈值，低于此值认为图像模糊
            print(f"图像模糊 (清晰度: {sharpness:.1f})，跳过保存")
            return False
        
        # 生成包含时间戳和序列号的文件名
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        filename = f"frame_{timestamp}_{self.save_count:06d}.{self.image_format}"
        save_path = os.path.join(self.output_dir, filename)
        
        # 根据图像格式设置保存参数（JPEG格式可设置质量参数）
        save_params = []
        if self.image_format in ['jpg', 'jpeg']:
            save_params = [cv2.IMWRITE_JPEG_QUALITY, 95]  # JPEG质量设置为95%
        
        # 保存图像到文件
        if save_params:
            success = cv2.imwrite(save_path, self.current_frame, save_params)
        else:
            success = cv2.imwrite(save_path, self.current_frame)
        
        # 处理保存结果
        if success:
            current_total = self.save_count - self.start_count + 1
            print(f"✅ 已保存: {filename} ({current_total}/{self.max_count}, 清晰度: {sharpness:.1f})")
            self.save_count += 1
            return True
        else:
            print(f"❌ 保存失败: {filename}")
            return False
    
    def calculate_sharpness(self, image):
        """
        计算图像清晰度（基于拉普拉斯算子的方差）
        
        清晰度是图像质量的重要指标，方差值越高表示图像越清晰。
        该方法将彩色图像转换为灰度图后计算拉普拉斯算子的方差。
        
        Args:
            image (numpy.ndarray): 输入图像，可以是彩色或灰度图
            
        Returns:
            float: 图像清晰度值（拉普拉斯方差）
        """
        # 如果输入是彩色图像，转换为灰度图
        if len(image.shape) == 3:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        else:
            gray = image
        
        # 计算拉普拉斯算子的方差作为清晰度指标
        return cv2.Laplacian(gray, cv2.CV_64F).var()
    
    def mouse_callback(self, event, x, y, flags, param):
        """
        鼠标事件回调函数
        
        实现鼠标左键点击保存当前帧的功能，便于交互式操作。
        
        Args:
            event: OpenCV鼠标事件类型
            x (int): 鼠标点击的x坐标
            y (int): 鼠标点击的y坐标
            flags: 事件标志
            param: 回调参数
        """
        if event == cv2.EVENT_LBUTTONDOWN:  # 左键点击事件
            print(f"🖱️  鼠标点击位置: ({x}, {y}) - 保存图像")
            self.save_frame()
    
    def run(self):
        """
        主运行循环
        
        控制整个采集流程，包括图像获取、显示、用户交互和文件保存。
        支持多种操作模式：单张保存、连续采集和手动选择。
        """
        print("🎥 RealSense采集开始")
        print("⌨️  操作说明:")
        print("   - 按 [ENTER] 键: 保存当前帧")
        print("   - 按 [SPACE] 键: 切换连续采集模式")
        print("   - 按 [ESC] 键: 退出程序")
        print("   - 左键点击窗口: 保存当前帧")
        print("-" * 50)
        
        # 创建固定大小的显示窗口并设置鼠标回调
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.setMouseCallback(self.window_name, self.mouse_callback)
        cv2.resizeWindow(self.window_name, 800, 600)  # 设置窗口大小
        
        # 初始化采集模式参数
        continuous_mode = False  # 连续采集模式标志
        last_save_time = 0  # 上次保存时间
        save_interval = 0.5  # 连续模式保存间隔（秒）
        
        try:
            # 主循环：持续运行直到达到最大采集数量或用户中断
            while (self.save_count - self.start_count) < self.max_count:
                # 等待获取一组连贯的帧
                frames = self.pipeline.wait_for_frames()
                color_frame = frames.get_color_frame()  # 提取彩色帧
                
                # 如果未获取到彩色帧，跳过本次循环
                if not color_frame:
                    continue
                
                # 将帧数据转换为numpy数组并备份当前帧
                self.current_frame = np.asanyarray(color_frame.get_data())
                color_image = self.current_frame.copy()  # 创建副本用于显示
                
                # 在图像上叠加状态信息文本
                current_progress = self.save_count - self.start_count
                status_text = f"已保存: {current_progress}/{self.max_count}"
                mode_text = "连续模式" if continuous_mode else "单张模式"
                progress_percent = f"进度: {current_progress/self.max_count*100:.1f}%"
                sharpness_value = f"清晰度: {self.calculate_sharpness(color_image):.1f}"
                
                # 使用OpenCV绘制文本信息（英文显示）
                cv2.putText(color_image, status_text, (10, 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.putText(color_image, mode_text, (10, 60), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
                cv2.putText(color_image, progress_percent, (10, 90), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
                cv2.putText(color_image, sharpness_value, (10, 120), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                cv2.putText(color_image, "ENTER/鼠标:保存 SPACE:切换 ESC:退出", (10, 450), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                
                # 在图像中心绘制十字标记，便于对焦和构图
                cv2.drawMarker(color_image, (color_image.shape[1]//2, color_image.shape[0]//2), 
                              (0, 0, 255), cv2.MARKER_CROSS, 20, 2)
                
                # 更新窗口显示
                cv2.imshow(self.window_name, color_image)
                
                # 检测键盘输入（非阻塞方式）
                key = cv2.waitKey(1) & 0xFF
                
                # 处理回车键（单张保存）
                if key == 13 or key == 10:  # Enter键（不同系统可能编码不同）
                    self.save_frame()
                
                # 处理空格键（切换连续采集模式）
                elif key == 32:  # Space键
                    continuous_mode = not continuous_mode
                    mode_status = "启用" if continuous_mode else "禁用"
                    print(f"🔄 连续采集模式: {mode_status}")
                
                # 处理ESC键（退出程序）
                elif key == 27:  # ESC键
                    print("👋 用户中断采集")
                    break
                
                # 连续模式处理：按时间间隔自动保存
                current_time = time.time()
                if (continuous_mode and 
                    (self.save_count - self.start_count) < self.max_count and
                    current_time - last_save_time >= save_interval):
                    self.save_frame()
                    last_save_time = current_time
                        
        finally:
            # 确保资源正确释放
            self.pipeline.stop()
            cv2.destroyAllWindows()
            final_count = self.save_count - self.start_count
            print(f"✅ 采集完成，已保存 {final_count}/{self.max_count} 张图像")
            print(f"📁 文件保存位置: {self.output_dir}")

def parse_arguments():
    """
    解析命令行参数
    
    提供灵活的命令行接口，允许用户自定义采集参数而不需要修改代码。
    
    Returns:
        argparse.Namespace: 包含所有解析后参数的命名空间对象
    """
    parser = argparse.ArgumentParser(
        description='RealSense自动采集工具 - 用于YOLO训练数据收集',
        epilog='使用示例: python realsense_capture.py -o /保存路径 -m 500 -s 100'
    )
    
    # 定义所有命令行参数
    parser.add_argument(
        '-o', '--output-dir',
        type=str,
        default=None,
        help='图像保存路径（默认: ~/"下载"/realsense_capture）'
    )
    
    parser.add_argument(
        '-m', '--max-count',
        type=int,
        default=5000,
        help='最大采集图像数量（默认: 5000）'
    )
    
    parser.add_argument(
        '-s', '--start-count',
        type=int,
        default=0,
        help='起始编号（默认: 0）'
    )
    
    parser.add_argument(
        '-f', '--format',
        type=str,
        default='jpg',
        choices=['jpg', 'png', 'jpeg'],
        help='图像保存格式（默认: jpg，可选: jpg, png）'
    )
    
    parser.add_argument(
        '-r', '--resolution',
        type=str,
        default='1280x720',
        choices=['640x480', '1280x720', '1920x1080'],
        help='采集分辨率（默认: 1280x720）'
    )
    
    parser.add_argument(
        '--fps',
        type=int,
        default=15,
        choices=[15, 30, 60],
        help='采集帧率（默认: 15）'
    )
    
    return parser.parse_args()

def main():
    """
    主函数
    
    程序入口点，负责参数解析、采集器初始化和异常处理。
    """
    # 解析命令行参数
    args = parse_arguments()
    
    # 打印程序标题和配置信息
    print("=" * 60)
    print(" RealSense数据采集工具")
    print("=" * 60)
    print(f" 保存路径: {args.output_dir or '默认下载目录'}")
    print(f" 采集数量: {args.max_count}")
    print(f" 起始编号: {args.start_count}")
    print(f" 图像格式: {args.format}")
    print(f" 分辨率: {args.resolution}")
    print(f"⚡ 帧率: {args.fps} FPS")
    print("=" * 60)
    
    try:
        # 创建采集器实例并启动采集
        capture = RealSenseCapture(
            output_dir=args.output_dir,
            max_count=args.max_count,
            start_count=args.start_count,
            image_format=args.format,
            resolution=args.resolution,
            fps=args.fps
        )
        capture.run()
    except Exception as e:
        # 异常处理：提示用户检查相机连接或参数设置
        print(f"错误: {e}")
        print("💡 请检查RealSense相机连接或参数设置")

if __name__ == "__main__":
    main()