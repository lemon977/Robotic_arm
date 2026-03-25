#!/usr/bin/env python
# -- coding: UTF-8 --
"""
=============================================================
机器人数据采集脚本
核心功能：
1. ROS话题订阅（相机、机械臂、底盘数据）
2. 实时数据采集 + 帧同步
3. 空格键终止采集（非root权限）
4. 数据保存为HDF5格式 + 导出MP4视频
依赖：pip install pynput av opencv-python h5py rospy dm_env
用法：python collect_data.py [--your_args]
=============================================================
"""

# ====================== 1. 导入依赖库（分组排序，便于学习） ======================
# 系统基础库
import os
import time
import threading
import collections
from collections import deque

# 数据处理库
import numpy as np
import h5py
import argparse
import dm_env

# ROS相关库
import rospy
from sensor_msgs.msg import JointState, Image
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import cv2

# 视频编码库
import av
import logging
from pathlib import Path

# 键盘监听库（空格键终止）
from pynput import keyboard

# ====================== 2. 视频编码模块（将图像序列编码为MP4） ======================
def encode_video_frames(images: np.ndarray, dst: Path, fps: int, vcodec: str = "libx264",
                        pix_fmt: str = "yuv420p", g: int = 2, crf: int = 23, fast_decode: int = 0,
                        log_level: int = av.logging.ERROR, overwrite: bool = False) -> bytes:
    """
    底层视频编码函数：使用PyAV将numpy图像数组编码为视频
    :param images: 图像序列 (N, H, W, 3)
    :param dst: 视频保存路径
    :param fps: 视频帧率
    :param vcodec: 视频编码器
    其他参数：视频编码配置（画质、关键帧等）
    """
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
                print(f"编码第 {i+1} 帧")
        for pkt in stream.encode():
            out.mux(pkt)
    if log_level is not None:
        av.logging.restore_default_callback()
    if not video_path.exists():
        raise OSError(f"Video encoding failed: {video_path}")

def create_video_from_images(images, output_path, fps=30, codec="libx264", quality=23):
    """
    上层视频封装函数：调用底层编码，对外提供简单接口
    """
    if not images:
        raise ValueError("没有图片数据")
    print(f"开始编码视频，编码器: {codec}  CRF: {quality}")
    encode_video_frames(np.asarray(images), Path(output_path), fps=fps, vcodec=codec, crf=quality, overwrite=True)
    print(f"视频已保存到: {output_path}")

# ====================== 3. 数据保存模块（将采集数据写入HDF5 + 导出视频） ======================
def save_data(args, timesteps, actions, dataset_path):
    """
    保存采集的所有数据：
    1. 机械臂状态（位置/速度/力矩）、动作、底盘速度 → HDF5文件
    2. 相机图像 → MP4视频（不保存图像到HDF5，节省空间）
    :param args: 命令行参数
    :param timesteps: 观测数据序列
    :param actions: 动作数据序列
    :param dataset_path: 保存路径（不含后缀）
    """
    data_size = len(actions)
    # 初始化数据字典：存储非图像类数据
    data_dict = {k: [] for k in [
        '/observations/qpos', '/observations/qvel', '/observations/effort',
        '/action', '/base_action']}
    
    # 初始化视频图像缓存
    video_images = {cam: [] for cam in args.camera_names}
    assert args.export_video, "请设置 --export_video 参数以启用视频导出功能"

    # 遍历数据，填充数据字典和视频缓存
    while actions:
        action = actions.pop(0)
        ts   = timesteps.pop(0)
        # 填充机械臂状态数据
        for k in ['qpos', 'qvel', 'effort']:
            data_dict[f'/observations/{k}'].append(ts.observation[k])
        # 填充动作和底盘数据
        data_dict['/action'].append(action)
        data_dict['/base_action'].append(ts.observation['base_vel'])
        # 处理相机图像：仅用于视频编码
        for cam in args.camera_names:
            img = ts.observation['images'][cam]
            if args.export_video:
                video_img = img if img.shape[2] == 3 else cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                if video_img.dtype != np.uint8:
                    video_img = (video_img * 255).astype(np.uint8) if video_img.max() <= 1.0 else video_img.astype(np.uint8)
                video_images[cam].append(video_img)

    # ====================== 保存HDF5文件 ======================
    t0 = time.time()
    with h5py.File(dataset_path + '.hdf5', 'w', rdcc_nbytes=1024**2*2) as root:
        root.attrs['sim'], root.attrs['compress'] = False, False
        obs = root.create_group('observations')
        # 创建数据集：机械臂状态
        for k in ['qpos', 'qvel', 'effort']:
            obs.create_dataset(k, (data_size, 14))
        # 创建数据集：动作、底盘动作
        root.create_dataset('action', (data_size, 14))
        root.create_dataset('base_action', (data_size, 2))
        # 写入所有数据
        for name, arr in data_dict.items():
            root[name][...] = arr
    print(f'\033[32m\nSaving: {time.time() - t0:.1f} secs. %s \033[0m\n' % dataset_path)

    # ====================== 导出MP4视频 ======================
    if args.export_video and data_size:
        print('\033[33m\n开始导出视频...\033[0m')
        video_dir = os.path.join(os.path.dirname(dataset_path), "video")
        os.makedirs(video_dir, exist_ok=True)
        # 为每个相机导出独立视频
        for cam in args.camera_names:
            if video_images[cam]:
                try:
                    cam_dir = os.path.join(video_dir, cam)
                    os.makedirs(cam_dir, exist_ok=True)
                    episode_idx = os.path.basename(dataset_path).split('_')[-1]
                    video_path = os.path.join(cam_dir, f"episode_{episode_idx}.mp4")
                    print(f"导出摄像头 {cam} 的视频: {video_path}")
                    create_video_from_images(video_images[cam], video_path,
                                             fps=args.video_fps, codec=args.video_codec, quality=args.video_quality)
                    print(f'\033[32m✅ 摄像头 {cam} 视频导出完成: {video_path}\033[0m')
                except Exception as e:
                    print(f'\033[31m❌ 摄像头 {cam} 视频导出失败: {e}\033[0m')
            else:
                print(f'\033[33m⚠️  摄像头 {cam} 没有图片数据，跳过视频导出\033[0m')
        print(f'\033[32m\n视频导出完成: {time.time() - t0:.1f} secs\033[0m')

# ====================== 4. ROS核心操作类（数据采集、话题订阅、帧同步） ======================
class RosOperator:
    """
    ROS操作核心类：
    1. 初始化ROS节点 + 订阅所有话题
    2. 缓存实时数据（双端队列）
    3. 帧同步（保证所有数据时间戳一致）
    4. 数据采集主循环
    5. 空格键终止监听
    """
    def __init__(self, args):
        self.args = args                  # 命令行参数
        self.stop_flag = False            # 终止标志（空格键触发）
        self.bridge = CvBridge()          # OpenCV-ROS图像转换
        self.init_deques()                # 初始化数据缓存队列
        self.init_ros()                   # 初始化ROS订阅

    def init_deques(self):
        """初始化双端队列：缓存所有ROS话题数据，限制最大长度防止内存溢出"""
        # 图像队列
        self.img_left_deque  = deque()
        self.img_right_deque = deque()
        self.img_front_deque = deque()
        # 深度图像队列
        self.img_left_depth_deque  = deque()
        self.img_right_depth_deque = deque()
        self.img_front_depth_deque = deque()
        # 主/从机械臂队列
        self.master_arm_left_deque  = deque()
        self.master_arm_right_deque = deque()
        self.puppet_arm_left_deque  = deque()
        self.puppet_arm_right_deque = deque()
        # 底盘队列
        self.robot_base_deque = deque()

    def keyboard_listener(self):
        """后台线程：监听空格键，设置终止标志"""
        def on_press(key):
            if key == keyboard.Key.space:
                self.stop_flag = True
                print("\033[35m>>> 空格键被按下，即将终止采集并保存...\033[0m")
                return False  # 停止监听
        with keyboard.Listener(on_press=on_press) as listener:
            listener.join()

    # ====================== ROS话题回调函数（数据入队） ======================
    def img_left_callback(self, msg):
        if len(self.img_left_deque) >= 2000: self.img_left_deque.popleft()
        self.img_left_deque.append(msg)
    def img_right_callback(self, msg):
        if len(self.img_right_deque) >= 2000: self.img_right_deque.popleft()
        self.img_right_deque.append(msg)
    def img_front_callback(self, msg):
        if len(self.img_front_deque) >= 2000: self.img_front_deque.popleft()
        self.img_front_deque.append(msg)
    def img_left_depth_callback(self, msg):
        if len(self.img_left_depth_deque) >= 2000: self.img_left_depth_deque.popleft()
        self.img_left_depth_deque.append(msg)
    def img_right_depth_callback(self, msg):
        if len(self.img_right_depth_deque) >= 2000: self.img_right_depth_deque.popleft()
        self.img_right_depth_deque.append(msg)
    def img_front_depth_callback(self, msg):
        if len(self.img_front_depth_deque) >= 2000: self.img_front_depth_deque.popleft()
        self.img_front_depth_deque.append(msg)
    def master_arm_left_callback(self, msg):
        if len(self.master_arm_left_deque) >= 2000: self.master_arm_left_deque.popleft()
        self.master_arm_left_deque.append(msg)
    def master_arm_right_callback(self, msg):
        if len(self.master_arm_right_deque) >= 2000: self.master_arm_right_deque.popleft()
        self.master_arm_right_deque.append(msg)
    def puppet_arm_left_callback(self, msg):
        if len(self.puppet_arm_left_deque) >= 2000: self.puppet_arm_left_deque.popleft()
        self.puppet_arm_left_deque.append(msg)
    def puppet_arm_right_callback(self, msg):
        if len(self.puppet_arm_right_deque) >= 2000: self.puppet_arm_right_deque.popleft()
        self.puppet_arm_right_deque.append(msg)
    def robot_base_callback(self, msg):
        if len(self.robot_base_deque) >= 2000: self.robot_base_deque.popleft()
        self.robot_base_deque.append(msg)

    def init_ros(self):
        """初始化ROS节点，订阅所有需要的话题"""
        rospy.init_node('record_episodes', anonymous=True)
        # 订阅彩色图像
        rospy.Subscriber(self.args.img_left_topic, Image, self.img_left_callback, queue_size=1000, tcp_nodelay=True)
        rospy.Subscriber(self.args.img_right_topic, Image, self.img_right_callback, queue_size=1000, tcp_nodelay=True)
        rospy.Subscriber(self.args.img_front_topic, Image, self.img_front_callback, queue_size=1000, tcp_nodelay=True)
        # 订阅深度图像（可选）
        if self.args.use_depth_image:
            rospy.Subscriber(self.args.img_left_depth_topic, Image, self.img_left_depth_callback, queue_size=1000, tcp_nodelay=True)
            rospy.Subscriber(self.args.img_right_depth_topic, Image, self.img_right_depth_callback, queue_size=1000, tcp_nodelay=True)
            rospy.Subscriber(self.args.img_front_depth_topic, Image, self.img_front_depth_callback, queue_size=1000, tcp_nodelay=True)
        # 订阅主/从机械臂状态
        rospy.Subscriber(self.args.master_arm_left_topic, JointState, self.master_arm_left_callback, queue_size=1000, tcp_nodelay=True)
        rospy.Subscriber(self.args.master_arm_right_topic, JointState, self.master_arm_right_callback, queue_size=1000, tcp_nodelay=True)
        rospy.Subscriber(self.args.puppet_arm_left_topic, JointState, self.puppet_arm_left_callback, queue_size=1000, tcp_nodelay=True)
        rospy.Subscriber(self.args.puppet_arm_right_topic, JointState, self.puppet_arm_right_callback, queue_size=1000, tcp_nodelay=True)
        # 订阅底盘里程计
        rospy.Subscriber(self.args.robot_base_topic, Odometry, self.robot_base_callback, queue_size=1000, tcp_nodelay=True)

    def process(self):
        """
        数据采集主循环：
        1. 启动键盘监听线程
        2. 循环获取同步帧数据
        3. 数据异常检测（机械臂数据停滞）
        4. 封装观测/动作数据
        """
        timesteps, actions = [], []
        count = 0
        rate = rospy.Rate(self.args.frame_rate)
        print("\033[36m>>> 采集已启动，按【空格键】可提前终止并保存...\033[0m")

        # 机械臂数据停滞检测变量
        last_qpos_left, last_qpos_right = None, None
        consecutive_unchanged_count_left, consecutive_unchanged_count_right = 0, 0
        UNCHANGED_THRESHOLD = 100  # 连续100帧无变化报警

        # 启动后台线程：监听空格键
        threading.Thread(target=self.keyboard_listener, daemon=True).start()

        # 主采集循环
        while (count < self.args.max_timesteps + 1) and not rospy.is_shutdown() and not self.stop_flag:
            result = self.get_frame()
            if not result:
                rate.sleep()
                continue
            count += 1
            # 解包同步后的所有数据
            (img_front, img_left, img_right, img_front_depth, img_left_depth, img_right_depth,
             puppet_arm_left, puppet_arm_right, master_arm_left, master_arm_right, robot_base) = result

            # 构建图像观测字典
            image_dict = {self.args.camera_names[0]: img_front,
                          self.args.camera_names[1]: img_left,
                          self.args.camera_names[2]: img_right}
            obs = collections.OrderedDict()
            obs['images'] = image_dict
            # 深度图像（可选）
            if self.args.use_depth_image:
                obs['images_depth'] = {self.args.camera_names[0]: img_front_depth,
                                       self.args.camera_names[1]: img_left_depth,
                                       self.args.camera_names[2]: img_right_depth}
            # 拼接机械臂状态数据（左臂+右臂）
            obs['qpos'] = np.concatenate((puppet_arm_left.position, puppet_arm_right.position))
            obs['qvel'] = np.concatenate((puppet_arm_left.velocity, puppet_arm_right.velocity))
            obs['effort'] = np.concatenate((puppet_arm_left.effort, puppet_arm_right.effort))
            # 底盘速度（可选）
            obs['base_vel'] = [robot_base.twist.twist.linear.x, robot_base.twist.twist.angular.z] if self.args.use_robot_base else [0.0, 0.0]

            # ====================== 机械臂数据停滞检测 ======================
            # 左臂检测
            current_qpos_left = puppet_arm_left.position
            if last_qpos_left is not None and np.array_equal(current_qpos_left, last_qpos_left):
                consecutive_unchanged_count_left += 1
            else:
                consecutive_unchanged_count_left = 0
            if consecutive_unchanged_count_left >= UNCHANGED_THRESHOLD:
                print(f"\033[33m⚠️ 警告: 左臂 'position' 数据已经连续 {consecutive_unchanged_count_left} 帧没有变化。\033[0m")
            last_qpos_left = current_qpos_left

            # 右臂检测
            current_qpos_right = puppet_arm_right.position
            if last_qpos_right is not None and np.array_equal(current_qpos_right, last_qpos_right):
                consecutive_unchanged_count_right += 1
            else:
                consecutive_unchanged_count_right = 0
            if consecutive_unchanged_count_right >= UNCHANGED_THRESHOLD:
                print(f"\033[33m⚠️ 警告: 右臂 'position' 数据已经连续 {consecutive_unchanged_count_right} 帧没有变化。\033[0m")
            last_qpos_right = current_qpos_right

            # 第一帧：初始化StepType.FIRST
            if count == 1:
                timesteps.append(dm_env.TimeStep(dm_env.StepType.FIRST, None, None, obs))
                continue
            # 正常帧：StepType.MID
            timesteps.append(dm_env.TimeStep(dm_env.StepType.MID, None, None, obs))
            # 拼接动作数据（左臂+右臂）
            left_action = np.concatenate((puppet_arm_left.position[:6], [master_arm_left.position[6]]))
            right_action = np.concatenate((puppet_arm_right.position[:6], [master_arm_right.position[6]]))
            actions.append(np.concatenate((left_action, right_action)))
            
            print("Frame data: ", count)
            rate.sleep()

        print(f"\n>>> 采集结束，共 {len(actions)} 帧，开始保存...")
        return timesteps, actions

    def get_frame(self):
        """
        核心：帧同步函数
        1. 检查所有队列是否有数据
        2. 以最小时间戳为基准，同步所有数据
        3. 弹出过期数据，返回同步后的一帧数据
        """
        # 检查图像队列是否为空
        if len(self.img_left_deque) == 0 or len(self.img_right_deque) == 0 or len(self.img_front_deque) == 0:
            return False
        # 检查深度图像队列（可选）
        if self.args.use_depth_image and (len(self.img_left_depth_deque) == 0 or len(self.img_right_depth_deque) == 0 or len(self.img_front_depth_deque) == 0):
            return False
        
        # 获取最小时间戳（同步基准）
        frame_time = min([self.img_left_deque[-1].header.stamp.to_sec(),
                          self.img_right_deque[-1].header.stamp.to_sec(),
                          self.img_front_deque[-1].header.stamp.to_sec()])
        if self.args.use_depth_image:
            frame_time = min(frame_time,
                             self.img_left_depth_deque[-1].header.stamp.to_sec(),
                             self.img_right_depth_deque[-1].header.stamp.to_sec(),
                             self.img_front_depth_deque[-1].header.stamp.to_sec())
        
        # 检查所有数据时间戳是否达标
        for dq in [self.img_left_deque, self.img_right_deque, self.img_front_deque,
                   self.master_arm_left_deque, self.master_arm_right_deque,
                   self.puppet_arm_left_deque, self.puppet_arm_right_deque]:
            if not dq or dq[-1].header.stamp.to_sec() < frame_time:
                return False
        if self.args.use_depth_image:
            for dq in [self.img_left_depth_deque, self.img_right_depth_deque, self.img_front_depth_deque]:
                if not dq or dq[-1].header.stamp.to_sec() < frame_time:
                    return False
        if self.args.use_robot_base and (not self.robot_base_deque or self.robot_base_deque[-1].header.stamp.to_sec() < frame_time):
            return False

        # 弹出过期数据，返回当前帧数据
        def pop(dq):
            while dq[0].header.stamp.to_sec() < frame_time:
                dq.popleft()
            return dq.popleft()

        # 图像转换（ROS → OpenCV）
        img_left  = self.bridge.imgmsg_to_cv2(pop(self.img_left_deque), 'passthrough')
        img_right = self.bridge.imgmsg_to_cv2(pop(self.img_right_deque), 'passthrough')
        img_front = self.bridge.imgmsg_to_cv2(pop(self.img_front_deque), 'passthrough')
        # 机械臂数据
        master_arm_left  = pop(self.master_arm_left_deque)
        master_arm_right = pop(self.master_arm_right_deque)
        puppet_arm_left  = pop(self.puppet_arm_left_deque)
        puppet_arm_right = pop(self.puppet_arm_right_deque)
        
        # 深度图像（可选，边缘填充）
        img_left_depth = img_right_depth = img_front_depth = None
        if self.args.use_depth_image:
            img_left_depth  = cv2.copyMakeBorder(self.bridge.imgmsg_to_cv2(pop(self.img_left_depth_deque), 'passthrough'), 40, 40, 0, 0, cv2.BORDER_CONSTANT, value=0)
            img_right_depth = cv2.copyMakeBorder(self.bridge.imgmsg_to_cv2(pop(self.img_right_depth_deque), 'passthrough'), 40, 40, 0, 0, cv2.BORDER_CONSTANT, value=0)
            img_front_depth = cv2.copyMakeBorder(self.bridge.imgmsg_to_cv2(pop(self.img_front_depth_deque), 'passthrough'), 40, 40, 0, 0, cv2.BORDER_CONSTANT, value=0)
        # 底盘数据（可选）
        robot_base = None
        if self.args.use_robot_base:
            robot_base = pop(self.robot_base_deque)
        
        return (img_front, img_left, img_right, img_front_depth, img_left_depth, img_right_depth,
                puppet_arm_left, puppet_arm_right, master_arm_left, master_arm_right, robot_base)

# ====================== 5. 命令行参数解析模块 ======================
def get_arguments():
    """
    解析所有运行参数：路径、话题名、帧率、视频配置等
    所有参数均有默认值，可直接运行
    """
    parser = argparse.ArgumentParser()
    # 数据保存配置
    parser.add_argument('--dataset_dir', type=str, default="./data")
    parser.add_argument('--task_name', type=str, default="aloha_mobile_dummy")
    parser.add_argument('--episode_idx', type=int, default=0)
    parser.add_argument('--max_timesteps', type=int, default=500)
    # 相机名称配置
    parser.add_argument('--camera_names', nargs='+', default=['cam_high', 'cam_left_wrist', 'cam_right_wrist'])
    # ROS话题名称配置
    parser.add_argument('--img_front_topic', default='/camera_f/color/image_raw')
    parser.add_argument('--img_left_topic', default='/camera_l/color/image_raw')
    parser.add_argument('--img_right_topic', default='/camera_r/color/image_raw')
    parser.add_argument('--img_front_depth_topic', default='/camera_f/depth/image_raw')
    parser.add_argument('--img_left_depth_topic', default='/camera_l/depth/image_raw')
    parser.add_argument('--img_right_depth_topic', default='/camera_r/depth/image_raw')
    parser.add_argument('--master_arm_left_topic', default='/master/joint_left')
    parser.add_argument('--master_arm_right_topic', default='/master/joint_right')
    parser.add_argument('--puppet_arm_left_topic', default='/puppet/joint_left')
    parser.add_argument('--puppet_arm_right_topic', default='/puppet/joint_right')
    parser.add_argument('--robot_base_topic', default='/odom')
    # 功能开关
    parser.add_argument('--use_robot_base', type=bool, default=False)
    parser.add_argument('--use_depth_image', type=bool, default=False)
    # 采集/视频参数
    parser.add_argument('--frame_rate', type=int, default=30)
    parser.add_argument('--export_video', action='store_true', help='是否导出视频')
    parser.add_argument('--video_fps', type=int, default=30)
    parser.add_argument('--video_codec', choices=['libx264', 'libx265', 'libsvtav1'], default='libx264')
    parser.add_argument('--video_quality', type=int, default=23, help='CRF 越小质量越高')
    return parser.parse_args()

# ====================== 6. 主函数（程序入口逻辑） ======================
def main():
    """程序主流程：解析参数 → 启动采集 → 保存数据"""
    args = get_arguments()
    # 初始化ROS操作类
    ros_operator = RosOperator(args)
    # 启动数据采集
    timesteps, actions = ros_operator.process()

    # 无数据则不保存
    if len(actions) == 0:
        print("\033[31m\n未采集到任何数据，放弃保存。\033[0m")
        return

    # 创建保存路径
    dataset_dir = os.path.join(args.dataset_dir, args.task_name)
    os.makedirs(dataset_dir, exist_ok=True)
    dataset_path = os.path.join(dataset_dir, f"episode_{args.episode_idx}")
    # 保存数据
    save_data(args, timesteps, actions, dataset_path)
    print("\033[32m>>> 已保存至：", dataset_path + ".hdf5\033[0m")

# ====================== 脚本执行入口 ======================
if __name__ == '__main__':
    main()