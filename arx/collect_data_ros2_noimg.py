#!/usr/bin/env python3
# -- coding: UTF-8 --
"""
ROS2版本的数据采集脚本 - 双臂版本（相机使用RealSense直接获取）
依赖：pip install opencv-python pyrealsense2
用法：python3 collect_data_ros2.py [--your_args]

copy 原始代码
copy2 相机可视化、相机未打卡退出脚本、最后检查视频文件是否存在、按键终止采集
当前代码 相机可视化、相机未打卡退出脚本、最后检查视频文件是否存在、按键终止采集、日志、静止检测
"""
import os
import time
import numpy as np
import h5py
import argparse
import dm_env
import collections
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from arx5_arm_msg.msg import RobotStatus
import cv2
import pyrealsense2 as rs
from pathlib import Path
import logging
import av

# 颜色代码
class Colors:
    RESET = '\033[0m'
    BOLD = '\033[1m'
    RED = '\033[91m'
    GREEN = '\033[92m'
    YELLOW = '\033[93m'
    BLUE = '\033[94m'
    MAGENTA = '\033[95m'
    CYAN = '\033[96m'
    WHITE = '\033[97m'
    GRAY = '\033[90m'

# 自定义日志类
class ColorLogger:
    def __init__(self, episode_idx, log_dir="./logs"):
        self.episode_idx = episode_idx
        self.log_dir = log_dir
        os.makedirs(log_dir, exist_ok=True)
        self.log_file = os.path.join(log_dir, f'episode_{episode_idx}.log')
        self.file_handler = open(self.log_file, 'w', encoding='utf-8')
        
    def _log(self, level, message, color=None):
        """内部日志方法"""
        timestamp = time.strftime('%Y-%m-%d %H:%M:%S', time.localtime())
        # 文件输出（无颜色）
        file_message = f"{timestamp} - {level} - {message}"
        print(file_message, file=self.file_handler, flush=True)
        
        # 控制台输出（有颜色）
        if color:
            console_message = f"{color}{timestamp} - {level} - {message}{Colors.RESET}"
        else:
            console_message = file_message
        print(console_message, flush=True)
    
    def info(self, message):
        self._log("INFO", message, Colors.WHITE)
    
    def success(self, message):
        self._log("SUCCESS", message, Colors.GREEN)
    
    def warning(self, message):
        self._log("WARNING", message, Colors.YELLOW)
    
    def error(self, message):
        self._log("ERROR", message, Colors.RED)
    
    def debug(self, message):
        self._log("DEBUG", message, Colors.GRAY)
    
    def cyan(self, message):
        self._log("INFO", message, Colors.CYAN)
    
    def blue(self, message):
        self._log("INFO", message, Colors.BLUE)
    
    def magenta(self, message):
        self._log("INFO", message, Colors.MAGENTA)
    
    def close(self):
        self.file_handler.close()

# 添加日志配置
def setup_logging(episode_idx, dataset_dir):
    if not os.path.exists(dataset_dir) or not dataset_dir.startswith('/home/kai/data/'):
        log_dir="/home/kai/arx_collect_data/logs"
    else:
        log_dir = dataset_dir.replace('/home/kai/data/', '/home/kai/arx_collect_data/logs/')
    # # 创建当天日期的日志文件夹
    # log_dir = os.path.join(log_dir, time.strftime('%Y-%m-%d', time.localtime()))
    """设置日志配置，同时输出到控制台和文件"""
    logger = ColorLogger(episode_idx, log_dir)
    logger.info(f"日志文件: {logger.log_file}")
    return logger

def encode_video_frames(images: np.ndarray, dst: Path, fps: int, vcodec: str = "libx264",
                        pix_fmt: str = "yuv420p", g: int = 2, crf: int = 23, fast_decode: int = 0,
                        log_level: int = av.logging.ERROR, overwrite: bool = False) -> bytes:
    if vcodec not in {"h264", "hevc", "libx264", "libx265", "libsvtav1"}:
        raise ValueError(f"Unsupported codec {vcodec}")
    video_path = Path(dst)
    video_path.parent.mkdir(parents=True, exist_ok=overwrite)
    if (vcodec in {"libsvtav1", "hevc", "libx265"}) and pix_fmt == "yuv444p":
        pix_fmt = "yuv420p"
    h, w, _ = images[0].shape
    options = {}
    for k, v in {"g": g, "crf": crf}.items():
        if v is not None:
            options[k] = str(v)
    if fast_decode:
        key = "svtav1-params" if vcodec == "libsvtav1" else "tune"
        options[key] = f"fast-decode={fast_decode}" if vcodec == "libsvtav1" else "fastdecode"
    if log_level is not None:
        logging.getLogger("libav").setLevel(log_level)
    with av.open(str(video_path), "w") as out:
        stream = out.add_stream(vcodec, fps, options=options)
        stream.pix_fmt, stream.width, stream.height = pix_fmt, w, h
        for i, img in enumerate(images):
            frame = av.VideoFrame.from_ndarray(img, format="rgb24")
            for pkt in stream.encode(frame):
                out.mux(pkt)
            if (i + 1) % 100 == 0 or i == len(images) - 1:
                global logger
                logger.blue(f"编码第 {i+1} 帧")
        for pkt in stream.encode():
            out.mux(pkt)
    if log_level is not None:
        av.logging.restore_default_callback()
    if not video_path.exists():
        raise OSError(f"Video encoding failed: {video_path}")

def create_video_from_images(images, output_path, fps=30, codec="libx264", quality=23):
    if not images:
        raise ValueError("没有图片数据")
    global logger
    logger.cyan(f"开始编码视频，编码器: {codec}  CRF: {quality}")
    encode_video_frames(np.asarray(images), Path(output_path), fps=fps, vcodec=codec, crf=quality, overwrite=True)
    logger.success(f"视频已保存到: {output_path}")

# 自定义消息结构（根据你的话题数据）
class ArmStatus:
    def __init__(self):
        self.header = None
        self.end_pos = []
        self.joint_pos = []      # 关节位置
        self.joint_vel = []      # 关节速度
        self.joint_cur = []      # 关节电流（对应effort）
        self.receive_time = None  # 接收时间戳
        self.msg_timestamp = None  # 消息产生时间戳（从header.stamp）
        self.seq = 0  # 序列号，用于检测是否是新数据

# -------------- save_data --------------
def save_data(args, timesteps, actions, dataset_path):
    global logger
    data_size = len(actions)
    # 准备数据字典，包含所有需要的数据
    data_dict = {
        '/observations/qpos': [],
        '/observations/qvel': [],
        '/observations/effort': [],
        # '/observations/images/cam_high': [],
        # '/observations/images/cam_left_wrist': [],
        # '/observations/images/cam_right_wrist': [],
        '/action': []
    }

    # 从timesteps中提取数据
    while actions:
        action = actions.pop(0)
        ts = timesteps.pop(0)
        
        data_dict['/observations/qpos'].append(ts.observation['qpos'])
        data_dict['/observations/qvel'].append(ts.observation['qvel'])
        data_dict['/observations/effort'].append(ts.observation['effort'])
        
        # 保存图像数据
        # data_dict['/observations/images/cam_high'].append(ts.observation['images'][args.camera_names[0]])
        # data_dict['/observations/images/cam_left_wrist'].append(ts.observation['images'][args.camera_names[1]])
        # data_dict['/observations/images/cam_right_wrist'].append(ts.observation['images'][args.camera_names[2]])
        
        data_dict['/action'].append(action)

    t0 = time.time()
    print('[save_data] 开始保存HDF5文件...')
    
    with h5py.File(dataset_path + '.hdf5', 'w', rdcc_nbytes=1024**2*2) as root:
        # 设置根属性
        root.attrs['sim'] = False
        root.attrs['compress'] = False
        
        # 创建observations组
        obs = root.create_group('observations')
        
        # 创建数据集 - 关节数据
        obs.create_dataset('qpos', (data_size, 14), dtype='float32')
        obs.create_dataset('qvel', (data_size, 14), dtype='float32')
        obs.create_dataset('effort', (data_size, 14), dtype='float32')
        
        # 创建images子组
        # images_group = obs.create_group('images')
        
        # 获取图像尺寸
        # img_height = args.camera_height
        # img_width = args.camera_width
        
        # 创建图像数据集
        # images_group.create_dataset('cam_high', (data_size, img_height, img_width, 3), dtype='uint8')
        # images_group.create_dataset('cam_left_wrist', (data_size, img_height, img_width, 3), dtype='uint8')
        # images_group.create_dataset('cam_right_wrist', (data_size, img_height, img_width, 3), dtype='uint8')
        
        # 创建action数据集
        root.create_dataset('action', (data_size, 14), dtype='float32')
        
        # 写入数据
        for name, arr in data_dict.items():
            root[name][...] = np.array(arr)
        
    logger.success(f'✓ HDF5保存完成: {time.time() - t0:.1f}秒')
    logger.success(f'  文件路径: {dataset_path}.hdf5')
    logger.success(f'  数据帧数: {data_size}')

# -------------- save_videos --------------
def save_videos(args, timesteps_copy, dataset_dir, episode_idx):
    """
    将三个相机的图像分别保存为视频
    timesteps_copy: timesteps的副本（因为原始的已经被pop掉了）
    dataset_dir: 数据集目录（例如：./data/task_name）
    episode_idx: episode编号
    """
    global logger
    logger.info(f">>> 开始生成视频...")
    t0 = time.time()
    
    if len(timesteps_copy) == 0:
        logger.error("没有图像数据，无法生成视频")
        return
    
    # 获取第一帧以确定视频尺寸
    first_obs = timesteps_copy[0].observation
    first_images = first_obs['images']
    
    # 获取图像尺寸
    sample_img = first_images[args.camera_names[0]]
    h, w = sample_img.shape[:2]
    
    # 设置视频参数
    fps = args.frame_rate
    
    # 为每个相机创建视频写入器
    video_writers = {}
    video_paths = {}
    
    for cam_name in args.camera_names:
        # 创建视频目录：{dataset_dir}/videos/{cam_name}/
        video_dir = os.path.join(dataset_dir, 'video', cam_name)
        os.makedirs(video_dir, exist_ok=True)
        
        # 视频文件路径
        video_path = os.path.join(video_dir, f'episode_{episode_idx}.mp4')
        video_paths[cam_name] = video_path
        images = []
        for ts in timesteps_copy:
            images.append(ts.observation['images'][cam_name])
        create_video_from_images(images, video_path, fps)
        
        logger.blue(f"  创建视频: {cam_name} -> {video_path}")
    
    logger.success(f'✓ 视频保存完成: {time.time() - t0:.1f}秒')
    logger.success(f'  分辨率: {w}x{h}')
    logger.success(f'  帧数: {len(timesteps_copy)}, 帧率: {fps}fps')
    
    # 打印每个视频的路径
    for cam_name in args.camera_names:
        logger.success(f'  [{cam_name}] {video_paths[cam_name]}')

# -------------- RosOperator (ROS2版本) --------------
class RosOperator(Node):
    def __init__(self, args):
        global logger
        super().__init__('record_episodes')
        self.args = args
        self.stop_flag = False
        self.init_deques()
        self.init_subscribers()
        self.init_cameras()

    def init_deques(self):
        # 相机最新帧缓存
        self.latest_img_left = None
        self.latest_img_right = None
        self.latest_img_front = None
        # 双机械臂最新状态缓存
        self.latest_arm_left_status = None
        self.latest_arm_right_status = None
        # 序列号计数器（分别为左臂和右臂）
        self._arm_left_seq = 0
        self._arm_right_seq = 0
        self._last_used_left_seq = -1
        self._last_used_right_seq = -1

    def init_cameras(self):
        """初始化RealSense相机"""
        global logger
        logger.info(f">>> 初始化RealSense相机...")
        
        # 初始化三个RealSense管道和配置
        self.pipeline_left = rs.pipeline()
        self.pipeline_right = rs.pipeline()
        self.pipeline_front = rs.pipeline()
        
        self.config_left = rs.config()
        self.config_right = rs.config()
        self.config_front = rs.config()
        
        # 通过序列号启用设备
        self.config_left.enable_device(self.args.camera_left_serial)
        self.config_right.enable_device(self.args.camera_right_serial)
        self.config_front.enable_device(self.args.camera_front_serial)
        
        # 配置相机流（使用彩色图像）
        width = self.args.camera_width
        height = self.args.camera_height
        fps = 60
        
        for config in [self.config_left, self.config_right, self.config_front]:
            config.enable_stream(rs.stream.color, width, height, rs.format.rgb8, fps)
        
        # 启动相机
        try:
            self.pipeline_left.start(self.config_left)
            logger.success(f"✓ 左侧相机已打开 (序列号: {self.args.camera_left_serial})")
        except Exception as e:
            logger.error(f"错误：无法打开左侧相机 (序列号: {self.args.camera_left_serial}): {e}")
            # 结束程序
            self.cleanup_cameras()
            exit(1)
        
        try:
            self.pipeline_right.start(self.config_right)
            logger.success(f"✓ 右侧相机已打开 (序列号: {self.args.camera_right_serial})")
        except Exception as e:
            logger.error(f"错误：无法打开右侧相机 (序列号: {self.args.camera_right_serial}): {e}")
            # 结束程序
            self.cleanup_cameras()
            exit(1)
        
        try:
            self.pipeline_front.start(self.config_front)
            logger.success(f"✓ 前置相机已打开 (序列号: {self.args.camera_front_serial})")
        except Exception as e:
            logger.error(f"错误：无法打开前置相机 (序列号: {self.args.camera_front_serial}): {e}")
            # 结束程序
            self.cleanup_cameras()
            exit(1)
        
        # 预热相机：丢弃最初的几帧，让相机稳定
        logger.info(f">>> 相机预热中...")
        for i in range(30):  # 丢弃前30帧
            try:
                self.pipeline_left.wait_for_frames(timeout_ms=1000)
                self.pipeline_right.wait_for_frames(timeout_ms=1000)
                self.pipeline_front.wait_for_frames(timeout_ms=1000)
            except:
                pass
        logger.success(f"✓ 相机预热完成")
        
        # 初始化第一帧
        logger.info(f">>> 初始化相机第一帧...")
        for i in range(10):  # 最多尝试10次
            self.read_cameras_nonblocking()
            if (self.latest_img_left is not None and 
                self.latest_img_right is not None and 
                self.latest_img_front is not None):
                logger.success(f"✓ 相机第一帧就绪")
                break
            time.sleep(0.1)
        
        if self.latest_img_left is None or self.latest_img_right is None or self.latest_img_front is None:
            logger.error("警告：部分相机未能获取第一帧")
            # 结束程序
            self.cleanup_cameras()
            exit(1)

    # ---------- 相机读取函数（非阻塞模式）----------
    def read_cameras_nonblocking(self):
        """尝试从三个相机快速读取，使用poll避免阻塞"""
        try:
            # 使用poll_for_frames非阻塞读取
            frames_left = self.pipeline_left.poll_for_frames()
            frames_right = self.pipeline_right.poll_for_frames()
            frames_front = self.pipeline_front.poll_for_frames()
            
            # 更新可用的帧
            if frames_left:
                color_frame = frames_left.get_color_frame()
                if color_frame:
                    self.latest_img_left = np.asanyarray(color_frame.get_data())
            
            if frames_right:
                color_frame = frames_right.get_color_frame()
                if color_frame:
                    self.latest_img_right = np.asanyarray(color_frame.get_data())
            
            if frames_front:
                color_frame = frames_front.get_color_frame()
                if color_frame:
                    self.latest_img_front = np.asanyarray(color_frame.get_data())
                    
        except Exception as e:
            pass  # 静默失败，下次继续尝试

    # ---------- 回调函数 ----------
    def arm_left_status_callback(self, msg):
        global logger
        # 左机械臂状态回调
        arm_msg = ArmStatus()
        arm_msg.header = msg.header
        arm_msg.joint_pos = list(msg.joint_pos)
        arm_msg.joint_vel = list(msg.joint_vel)  # 添加关节速度
        arm_msg.joint_cur = list(msg.joint_cur)  # 添加关节电流（effort）
        arm_msg.receive_time = time.time()
        arm_msg.msg_timestamp = arm_msg.receive_time
        
        # 更新序列号
        self._arm_left_seq += 1
        arm_msg.seq = self._arm_left_seq
        
        # 调试计数器
        if not hasattr(self, '_callback_left_count'):
            self._callback_left_count = 0
            self._last_print_time_left = time.time()
        self._callback_left_count += 1
        
        # 每秒打印一次回调频率
        if time.time() - self._last_print_time_left >= 1.0:
            logger.magenta(f"[左臂回调] 接收频率: {self._callback_left_count}Hz")
            self._callback_left_count = 0
            self._last_print_time_left = time.time()
        
        self.latest_arm_left_status = arm_msg
    
    def arm_right_status_callback(self, msg):
        global logger
        # 右机械臂状态回调
        arm_msg = ArmStatus()
        arm_msg.header = msg.header
        arm_msg.joint_pos = list(msg.joint_pos)
        arm_msg.joint_vel = list(msg.joint_vel)  # 添加关节速度
        arm_msg.joint_cur = list(msg.joint_cur)  # 添加关节电流（effort）
        arm_msg.receive_time = time.time()
        arm_msg.msg_timestamp = arm_msg.receive_time
        
        # 更新序列号
        self._arm_right_seq += 1
        arm_msg.seq = self._arm_right_seq
        
        # 调试计数器
        if not hasattr(self, '_callback_right_count'):
            self._callback_right_count = 0
            self._last_print_time_right = time.time()
        self._callback_right_count += 1
        
        # 每秒打印一次回调频率
        if time.time() - self._last_print_time_right >= 1.0:
            logger.magenta(f"[右臂回调] 接收频率: {self._callback_right_count}Hz")
            self._callback_right_count = 0
            self._last_print_time_right = time.time()
        
        self.latest_arm_right_status = arm_msg

    def init_subscribers(self):
        # 机械臂状态使用BEST_EFFORT
        arm_qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # 订阅双机械臂状态 - 使用 arx5_arm_msg/msg/RobotStatus
        self.create_subscription(RobotStatus, self.args.arm_left_topic, self.arm_left_status_callback, arm_qos_profile)
        self.create_subscription(RobotStatus, self.args.arm_right_topic, self.arm_right_status_callback, arm_qos_profile)

    # ---------- 主采集循环 ----------
    def process(self):
        global logger
        timesteps, actions = [], []
        count = 0
        # 计算每帧的时间间隔
        frame_interval = 1.0 / self.args.frame_rate
        
        # 预热阶段：等待双机械臂数据填充
        logger.info(f">>> 等待双机械臂数据接收中...")
        warmup_time = 0
        max_warmup = 10  # 最多等待10秒
        
        while warmup_time < max_warmup and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            # 检查双机械臂数据是否就绪
            left_ready = self.latest_arm_left_status is not None
            right_ready = self.latest_arm_right_status is not None
            
            if left_ready and right_ready:
                logger.success(f"✓ 双机械臂数据就绪")
                break
            elif left_ready:
                logger.blue(f"  左臂就绪，等待右臂... {int(warmup_time)}s")
            elif right_ready:
                logger.blue(f"  右臂就绪，等待左臂... {int(warmup_time)}s")
            else:
                if int(warmup_time) != int(warmup_time - 0.1):  # 每秒打印一次
                    logger.blue(f"  等待中... {int(warmup_time)}s")
            
            warmup_time += 0.1
        
        if warmup_time >= max_warmup:
            logger.warning(f">>> 警告：等待超时，机械臂数据可能未就绪")
        
        logger.cyan(f">>> 采集已启动...")
        
        # --- 数据检测变量 (左右臂分离) ---
        last_qpos_left, last_qpos_right = None, None
        consecutive_unchanged_count_left, consecutive_unchanged_count_right = 0, 0
        UNCHANGED_THRESHOLD = 100 # 连续100帧无变化则发出警告

        try:
            while (count < self.args.max_timesteps) and rclpy.ok() and not self.stop_flag:
                # 记录循环开始时间，用于精确控制帧率
                loop_start_time = time.time()
                
                # 先读取相机帧（不依赖ROS）
                self.read_cameras_nonblocking()
                
                # 在获取数据前立即多次调用spin_once以确保获取最新的机械臂数据
                # 由于机械臂以1000Hz发布，短时间内多次spin可以获取最新状态
                for _ in range(20):  # 连续调用20次，确保获取最新数据
                    rclpy.spin_once(self, timeout_sec=0.001)  # 增加timeout
                
                # 立即获取帧（此时数据最新）
                result = self.get_frame()

                logger.debug(f"{count}")
                if not result:
                    logger.warning(f"获取帧失败，跳过... (左臂:{self.latest_arm_left_status is not None} 右臂:{self.latest_arm_right_status is not None} 相机L:{self.latest_img_left is not None} R:{self.latest_img_right is not None} F:{self.latest_img_front is not None})")
                    time.sleep(frame_interval)
                    continue

                count += 1
                (img_front, img_left, img_right, arm_left_status, arm_right_status) = result
                
                # --- 分别检测左右臂的 qpos 数据是否连续不变 ---
                # 左臂
                current_qpos_left = np.array(arm_left_status.joint_pos)
                if last_qpos_left is not None:
                    # 计算关节位置变化量
                    pos_diff_left = np.abs(current_qpos_left - last_qpos_left)
                    # 如果所有关节变化量都小于阈值，视为静止
                    if np.all(pos_diff_left < 0.001):  # 阈值为0.001弧度
                        consecutive_unchanged_count_left += 1
                    else:
                        consecutive_unchanged_count_left = 0
                else:
                    consecutive_unchanged_count_left = 0
                
                if consecutive_unchanged_count_left >= UNCHANGED_THRESHOLD:
                    logger.warning(f"警告: 左臂 'position' 数据已经连续 {consecutive_unchanged_count_left} 帧没有明显变化。")
                last_qpos_left = current_qpos_left

                # 右臂
                current_qpos_right = np.array(arm_right_status.joint_pos)
                if last_qpos_right is not None:
                    # 计算关节位置变化量
                    pos_diff_right = np.abs(current_qpos_right - last_qpos_right)
                    # 如果所有关节变化量都小于阈值，视为静止
                    if np.all(pos_diff_right < 0.001):  # 阈值为0.001弧度
                        consecutive_unchanged_count_right += 1
                    else:
                        consecutive_unchanged_count_right = 0
                else:
                    consecutive_unchanged_count_right = 0
                
                if consecutive_unchanged_count_right >= UNCHANGED_THRESHOLD:
                    logger.warning(f"警告: 右臂 'position' 数据已经连续 {consecutive_unchanged_count_right} 帧没有明显变化。")
                last_qpos_right = current_qpos_right
                # --- 检测结束 ---

                ####### display images for debug #######
                display_img_front = cv2.cvtColor(img_front, cv2.COLOR_RGB2BGR)
                display_img_left = cv2.cvtColor(img_left, cv2.COLOR_RGB2BGR)
                display_img_right = cv2.cvtColor(img_right, cv2.COLOR_RGB2BGR)
                combined_img = np.hstack((display_img_left, display_img_front, display_img_right))
                h,w,_ = combined_img.shape
                resized_img = cv2.resize(combined_img, (1280, int(1280/w*h)))
                cv2.imshow('Camera Views (Left | Front | Right)', resized_img)
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q') or key == ord(' '):
                    logger.info(f">>> 采集被用户中断 (按键)")
                    self.stop_flag = True

                image_dict = {self.args.camera_names[0]: img_front,
                            self.args.camera_names[1]: img_left,
                            self.args.camera_names[2]: img_right}
                obs = collections.OrderedDict()
                obs['images'] = image_dict
                # 拼接双臂关节数据：左臂7个关节 + 右臂7个关节 = 14维
                obs['qpos'] = np.concatenate([np.array(arm_left_status.joint_pos), 
                                            np.array(arm_right_status.joint_pos)])
                obs['qvel'] = np.concatenate([np.array(arm_left_status.joint_vel), 
                                            np.array(arm_right_status.joint_vel)])
                obs['effort'] = np.concatenate([np.array(arm_left_status.joint_cur), 
                                            np.array(arm_right_status.joint_cur)])
                
                timesteps.append(dm_env.TimeStep(dm_env.StepType.MID, None, None, obs))
                actions.append(np.concatenate([np.array(arm_left_status.joint_pos), 
                                            np.array(arm_right_status.joint_pos)]))
                # 每隔一定帧数打印进度
                if count % 50 == 0 or count <= 5:
                    logger.cyan(f"已采集: {count}/{self.args.max_timesteps} 帧")
                
                # 精确控制帧率：计算剩余时间并sleep
                elapsed_time = time.time() - loop_start_time
                sleep_time = max(0, frame_interval - elapsed_time)
                if sleep_time > 0:
                    time.sleep(sleep_time)
        except KeyboardInterrupt:
            logger.info(f">>> 采集被用户中断")

        logger.info(f">>> 采集结束，共 {len(actions)} 帧，开始保存...")
        return timesteps, actions

    # ---------- get_frame 从缓存获取 ----------
    def get_frame(self):
        global logger
        try:
            # 检查双机械臂状态是否可用
            if self.latest_arm_left_status is None:
                logger.warning("左机械臂状态不可用")
                return False
            if self.latest_arm_right_status is None:
                logger.warning("右机械臂状态不可用")
                return False
            
            # 检查相机图像是否可用
            if self.latest_img_left is None:
                logger.warning("左侧相机图像不可用")
                return False
            if self.latest_img_right is None:
                logger.warning("右侧相机图像不可用")
                return False
            if self.latest_img_front is None:
                logger.warning("前置相机图像不可用")
                return False

            # 复制最新的左机械臂状态
            arm_left_status = ArmStatus()
            arm_left_status.header = self.latest_arm_left_status.header
            arm_left_status.joint_pos = list(self.latest_arm_left_status.joint_pos)
            arm_left_status.joint_vel = list(self.latest_arm_left_status.joint_vel)
            arm_left_status.joint_cur = list(self.latest_arm_left_status.joint_cur)
            arm_left_status.receive_time = self.latest_arm_left_status.receive_time
            arm_left_status.msg_timestamp = self.latest_arm_left_status.msg_timestamp
            arm_left_status.seq = self.latest_arm_left_status.seq
            
            # 复制最新的右机械臂状态
            arm_right_status = ArmStatus()
            arm_right_status.header = self.latest_arm_right_status.header
            arm_right_status.joint_pos = list(self.latest_arm_right_status.joint_pos)
            arm_right_status.joint_vel = list(self.latest_arm_right_status.joint_vel)
            arm_right_status.joint_cur = list(self.latest_arm_right_status.joint_cur)
            arm_right_status.receive_time = self.latest_arm_right_status.receive_time
            arm_right_status.msg_timestamp = self.latest_arm_right_status.msg_timestamp
            arm_right_status.seq = self.latest_arm_right_status.seq
            
            # 检查左臂是否是新数据
            is_new_left = (arm_left_status.seq != self._last_used_left_seq)
            seq_diff_left = arm_left_status.seq - self._last_used_left_seq
            self._last_used_left_seq = arm_left_status.seq
            
            # 检查右臂是否是新数据
            is_new_right = (arm_right_status.seq != self._last_used_right_seq)
            seq_diff_right = arm_right_status.seq - self._last_used_right_seq
            self._last_used_right_seq = arm_right_status.seq
            
            # 显示关节位置信息
            current_time = time.time()
            delay_left = (current_time - arm_left_status.receive_time) * 1000
            delay_right = (current_time - arm_right_status.receive_time) * 1000
            
            # 记录统计信息
            if not hasattr(self, '_delay_sum_left'):
                self._delay_sum_left = 0
                self._delay_sum_right = 0
                self._delay_count = 0
            
            self._delay_sum_left += delay_left
            self._delay_sum_right += delay_right
            self._delay_count += 1
            avg_delay_left = self._delay_sum_left / self._delay_count
            avg_delay_right = self._delay_sum_right / self._delay_count
            
            joint_pos_left_str = ', '.join([f'{x:.3f}' for x in arm_left_status.joint_pos])
            joint_pos_right_str = ', '.join([f'{x:.3f}' for x in arm_right_status.joint_pos])
            new_flag_left = f"✓新" if is_new_left else f"✗旧"
            new_flag_right = f"✓新" if is_new_right else f"✗旧"
            
            logger.info(f"左臂: {new_flag_left} seq:{arm_left_status.seq}(+{seq_diff_left}) joints=[{joint_pos_left_str}] | 延迟: {delay_left:.1f}ms (平均: {avg_delay_left:.1f}ms)")
            logger.info(f"右臂: {new_flag_right} seq:{arm_right_status.seq}(+{seq_diff_right}) joints=[{joint_pos_right_str}] | 延迟: {delay_right:.1f}ms (平均: {avg_delay_right:.1f}ms)")
            
            if not is_new_left:
                logger.warning(f"警告：左臂数据未更新，可能丢帧")
            if not is_new_right:
                logger.warning(f"警告：右臂数据未更新，可能丢帧")
            if delay_left > 200:
                logger.warning(f"警告：左臂数据延迟过高: {delay_left:.1f}ms")
            if delay_right > 200:
                logger.warning(f"警告：右臂数据延迟过高: {delay_right:.1f}ms")
            

            # 复制最新的图像
            img_left = self.latest_img_left.copy()
            img_right = self.latest_img_right.copy()
            img_front = self.latest_img_front.copy()
            
            return (img_front, img_left, img_right, arm_left_status, arm_right_status)
        except Exception as e:
            logger.error(f"get_frame异常: {e}")
            return False

    def cleanup_cameras(self):
        """释放相机资源"""
        global logger
        logger.info(f">>> 释放RealSense相机资源...")
        # 分别尝试停止每个相机，避免一个失败影响其他
        if hasattr(self, 'pipeline_left'):
            try:
                self.pipeline_left.stop()
                logger.success(f"✓ 左侧相机已释放")
            except Exception as e:
                logger.warning(f"警告：释放左侧相机时出错: {e}")
        
        if hasattr(self, 'pipeline_right'):
            try:
                self.pipeline_right.stop()
                logger.success(f"✓ 右侧相机已释放")
            except Exception as e:
                logger.warning(f"警告：释放右侧相机时出错: {e}")
        
        if hasattr(self, 'pipeline_front'):
            try:
                self.pipeline_front.stop()
                logger.success(f"✓ 前置相机已释放")
            except Exception as e:
                logger.warning(f"警告：释放前置相机时出错: {e}")
        
        logger.success(f"✓ 相机资源释放完成")

# -------------- 参数解析 --------------
def get_arguments():
    parser = argparse.ArgumentParser()
    parser.add_argument('--dataset_dir', type=str, default="./data")
    parser.add_argument('--task_name', type=str, default="aloha_mobile_dummy")
    parser.add_argument('--episode_idx', type=int, default=0)
    parser.add_argument('--max_timesteps', type=int, default=500)
    parser.add_argument('--camera_names', nargs='+', default=['cam_high', 'cam_left_wrist', 'cam_right_wrist'])
    # RealSense相机参数（序列号）
    parser.add_argument('--camera_front_serial', type=str, default='213722070209', help='前置相机序列号')
    parser.add_argument('--camera_left_serial', type=str, default='213722070377', help='左侧相机序列号')
    parser.add_argument('--camera_right_serial', type=str, default='213522071788', help='右侧相机序列号')
    parser.add_argument('--camera_width', type=int, default=640, help='相机宽度')
    parser.add_argument('--camera_height', type=int, default=480, help='相机高度')
    # ROS2参数 - 双机械臂话题
    parser.add_argument('--arm_left_topic', default='/arm_slave_l_status', help='左机械臂状态话题')
    parser.add_argument('--arm_right_topic', default='/arm_slave_r_status', help='右机械臂状态话题')
    parser.add_argument('--frame_rate', type=int, default=30)
    # 视频生成参数
    parser.add_argument('--save_video', action='store_true', help='是否生成视频（默认否）')
    return parser.parse_args()
    
# -------------- main --------------
def main():
    args = get_arguments()
    
    # 初始化日志系统，使用episode_idx命名
    global logger
    logger = setup_logging(args.episode_idx, args.dataset_dir)
    
    # 初始化ROS2
    rclpy.init()
    
    ros_operator = None
    try:
        ros_operator = RosOperator(args)
        timesteps, actions = ros_operator.process()

        if len(actions) == 0:
            logger.error("未采集到任何数据，放弃保存。")
            return

        dataset_dir = os.path.join(args.dataset_dir, args.task_name)
        os.makedirs(dataset_dir, exist_ok=True)
        dataset_path = os.path.join(dataset_dir, f"episode_{args.episode_idx}")
        
        # 如果需要生成视频，先保存timesteps副本（因为save_data会pop掉原始数据）
        timesteps_copy = list(timesteps) if args.save_video else None
        
        # 保存HDF5数据
        save_data(args, timesteps, actions, dataset_path)
        logger.success(f">>> 已保存至：{dataset_path}.hdf5")
        
        # 生成视频
        if args.save_video and timesteps_copy:
            save_videos(args, timesteps_copy, dataset_dir, args.episode_idx)
            # 检查视频是否都存在
            for cam_name in args.camera_names:
                video_dir = os.path.join(dataset_dir, 'video', cam_name)
                if not os.path.exists(os.path.join(video_dir, f'episode_{args.episode_idx}.mp4')):
                    logger.error(f"视频生成失败: {cam_name}")
    finally:
        # 清理相机资源
        if ros_operator is not None:
            ros_operator.cleanup_cameras()

        cv2.destroyAllWindows()
        cv2.waitKey(1)
        logger.success(f"✓ OpenCV窗口已关闭")

        # 清理ROS2资源
        if rclpy.ok():
            if ros_operator is not None:
                ros_operator.destroy_node()
            rclpy.shutdown()
        
        # 关闭日志文件
        logger.close()

if __name__ == '__main__':
    main()