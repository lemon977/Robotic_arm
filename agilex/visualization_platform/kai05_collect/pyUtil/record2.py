#!/usr/bin/env python
# -- coding: UTF-8 --
"""
record.py —— ALOHA 数据采集脚本（保留14维限制版）
适配/puppet/joint_left/right单臂7维发布，强制拼接为14维，保留原14维硬编码限制
仅需指定：--dataset_dir 和 --episode_idx
所有其他参数已硬编码为合理默认值
"""
import os
import sys
import time
import re
import numpy as np
import h5py
import argparse
import dm_env
import collections
from collections import deque
import rospy
from sensor_msgs.msg import JointState, Image
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import cv2
import threading
import logging
from pathlib import Path
from collect_machine2nas import copy2nas
import datetime
import json

# formatted_date = ""
# try:
#     with open('/home/agilex/kai05_collect/config/config.json', 'r') as f:
#         data = json.load(f)
#         formatted_date = data['date']
#         print('collect_machine2nas ', formatted_date)
# except Exception as e:
#     # 既然没有文件那么说明是第一次运行，当前时间和当前日期保持一致
# 获取当前本地时间
current_time = datetime.datetime.now()
# 格式化为 两位年份+两位月份+两位日期（如260213）
formatted_date = current_time.strftime("%y%m%d")

# -------------------------- 日志配置 --------------------------
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',
    handlers=[
        logging.StreamHandler(sys.stdout),
        logging.FileHandler(f'/home/agilex/kai05_collect/log/collect_data_{formatted_date}.log', mode='a', encoding='utf-8')
    ]
)
logger = logging.getLogger(__name__)

# -------------- 核心工具函数：强制7维处理（单臂）--------------
def force_7d(arr):
    """
    将数组强制处理为7维，不足补0，超出截断
    :param arr: 输入的关节数据数组（list/np.ndarray）
    :return: 严格7维的np.ndarray
    """
    if arr is None or len(arr) == 0:
        return np.zeros(7, dtype=np.float64)
    arr_np = np.array(arr, dtype=np.float64)
    if len(arr_np) < 7:
        # 不足7维，补0至7维
        return np.pad(arr_np, (0, 7 - len(arr_np)), mode='constant', constant_values=0)
    elif len(arr_np) > 7:
        # 超出7维，截断前7维
        return arr_np[:7]
    else:
        # 刚好7维，直接返回
        return arr_np

# -------------- 视频导出（完全未修改）--------------
def encode_video_frames(images: np.ndarray, dst: Path, fps: int, vcodec: str = "libx264",
                        pix_fmt: str = "yuv420p", g: int = 2, crf: int = 23, fast_decode: int = 0,
                        log_level: int = logging.ERROR, overwrite: bool = False) -> None:
    try:
        if vcodec not in {"h264", "hevc", "libx264", "libx265", "libsvtav1"}:
            raise ValueError(f"不支持的编码器 {vcodec}")
        video_path = Path(dst)
        video_path.parent.mkdir(parents=True, exist_ok=True)
        if (vcodec in {"libsvtav1", "hevc", "libx265"}) and pix_fmt == "yuv444p":
            pix_fmt = "yuv420p"
        h, w, _ = images[0].shape
        options = {k: str(v) for k, v in {"g": g, "crf": crf}.items() if v is not None}
        if fast_decode:
            key = "svtav1-params" if vcodec == "libsvtav1" else "tune"
            options[key] = f"fast-decode={fast_decode}" if vcodec == "libsvtav1" else "fastdecode"
        import av
        av_logger = logging.getLogger("libav")
        av_logger.setLevel(log_level)
        with av.open(str(video_path), "w") as out:
            stream = out.add_stream(vcodec, fps, options=options)
            stream.pix_fmt, stream.width, stream.height = pix_fmt, w, h
            for i, img in enumerate(images):
                frame = av.VideoFrame.from_ndarray(img, format="rgb24")
                for pkt in stream.encode(frame):
                    out.mux(pkt)
                if (i + 1) % 100 == 0 or i == len(images) - 1:
                    logger.info(f"视频编码进度：第 {i+1}/{len(images)} 帧")
            for pkt in stream.encode():
                out.mux(pkt)
        av.logging.restore_default_callback()
        if not video_path.exists():
            raise OSError(f"视频编码失败：文件未生成 {video_path}")
        logger.info(f"视频编码完成：{video_path}")
    except Exception as e:
        logger.error(f"视频编码异常：{str(e)}", exc_info=True)
        raise

def create_video_from_images(images, output_path, fps=30, codec="libx264", quality=23):
    if not isinstance(images, (list, np.ndarray)) or len(images) == 0:
        raise ValueError("图片数据为空，无法生成视频")
    logger.info(f"开始生成视频 | 编码器: {codec} | CRF: {quality} | 输出路径: {output_path}")
    encode_video_frames(
        np.asarray(images), 
        Path(output_path), 
        fps=fps, 
        vcodec=codec, 
        crf=quality, 
        overwrite=True
    )
    # print("采集任务完成")
    logger.info(f"视频保存完成：{output_path}")

# -------------- 数据保存（完全未修改，保留14维硬编码）--------------
def save_data(args, timesteps, actions, dataset_path):
    hdf5_path = None
    try:
        data_size = len(actions)
        if data_size == 0:
            logger.warning("无有效数据，跳过保存")
            return
        data_dict = {
            '/observations/qpos': [],
            '/observations/qvel': [],
            '/observations/effort': [],
            '/action': [],
            '/base_action': []
        }
        video_images = {cam: [] for cam in args.camera_names}
        while actions:
            action = actions.pop(0)
            ts = timesteps.pop(0)
            for k in ['qpos', 'qvel', 'effort']:
                data_dict[f'/observations/{k}'].append(ts.observation[k])
            data_dict['/action'].append(action)
            data_dict['/base_action'].append(ts.observation['base_vel'])
            for cam in args.camera_names:
                img = ts.observation['images'][cam]
                if args.export_video:
                    if img.shape[2] != 3:
                        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                    if img.dtype != np.uint8:
                        img = (img * 255).astype(np.uint8) if img.max() <= 1.0 else img.astype(np.uint8)
                    video_images[cam].append(img)
        t0 = time.time()
        dataset_path = Path(dataset_path).expanduser()
        hdf5_path = dataset_path.with_suffix('.hdf5')
        with h5py.File(hdf5_path, 'w', rdcc_nbytes=1024**2*2) as root:
            root.attrs['sim'], root.attrs['compress'] = False, False
            obs = root.create_group('observations')
            # 保留原14维硬编码限制
            for k in ['qpos', 'qvel', 'effort']:
                obs.create_dataset(k, (data_size, 14), data=data_dict[f'/observations/{k}'])
            root.create_dataset('action', (data_size, 14), data=data_dict['/action'])
            root.create_dataset('base_action', (data_size, 2), data=data_dict['/base_action'])
        logger.info(f"HDF5数据保存完成 | 耗时: {time.time() - t0:.1f}s | 路径: {hdf5_path}")
        if args.export_video:
            logger.info("开始导出视频...")
            video_dir = dataset_path.parent / "video"
            video_dir.mkdir(exist_ok=True)
            episode_idx = dataset_path.name.split('_')[-1]
            for cam in args.camera_names:
                cam_imgs = video_images[cam]
                if not cam_imgs:
                    logger.warning(f"摄像头 {cam} 无图像数据，跳过视频导出")
                    continue
                cam_dir = video_dir / cam
                cam_dir.mkdir(exist_ok=True)
                video_path = cam_dir / f"episode_{episode_idx}.mp4"
                try:
                    create_video_from_images(cam_imgs, str(video_path), 
                                             fps=args.video_fps, codec=args.video_codec, quality=args.video_quality)
                    logger.info(f"✅ 摄像头 {cam} 视频导出完成: {video_path}")
                except Exception as e:
                    logger.error(f"❌ 摄像头 {cam} 视频导出失败: {str(e)}", exc_info=True)
            logger.info(f"视频导出总耗时: {time.time() - t0:.1f}s")
            sys.stdout.flush()
    except Exception as e:
        logger.error(f"数据保存异常：{str(e)}", exc_info=True)
        if hdf5_path and os.path.exists(hdf5_path):
            try:
                # 移除hdf5文件
                os.remove(hdf5_path)
                # 移除视频文件
                video_dir = dataset_path.parent / "video"
                if video_dir.exists() and video_dir.is_dir():
                    episode_idx = dataset_path.name.split('_')[-1]
                    for cam in args.camera_names:
                        video_path = video_dir / cam / f"episode_{episode_idx}.mp4"
                        if video_path.exists():
                            os.remove(video_path)
                logger.info(f"已删除损坏的HDF5文件: {hdf5_path}")
            except Exception as del_e:
                logger.warning(f"删除损坏的HDF5文件失败: {str(del_e)}")
        raise

# -------------- ROS操作类（核心修改：拼接前强制7维处理）--------------
class RosOperator:
    def __init__(self, args):
        self.args = args
        # self.stop_flag = False
        self.bridge = CvBridge()
        self.pid = os.getpid()
        # logger.info(f"采集进程启动 | PID: {self.pid}")
        self.deques = {
            'img_left': deque(maxlen=2000),
            'img_right': deque(maxlen=2000),
            'img_front': deque(maxlen=2000),
            'img_left_depth': deque(maxlen=2000),
            'img_right_depth': deque(maxlen=2000),
            'img_front_depth': deque(maxlen=2000),
            'master_arm_left': deque(maxlen=2000),
            'master_arm_right': deque(maxlen=2000),
            'puppet_arm_left': deque(maxlen=2000),
            'puppet_arm_right': deque(maxlen=2000),
            'robot_base': deque(maxlen=2000)
        }
        self.init_ros()
        # self.stop_thread = threading.Thread(target=self.stop_file_listener, daemon=True)
        # self.stop_thread.start()
        # self.no_save_thread = threading.Thread(target=self.no_save_listener, daemon=True)
        # self.no_save_thread.start()
        # self.heartbeat_thread = threading.Thread(target=self.heartbeat, daemon=True)
        # self.heartbeat_thread.start()
        # self.no_save_flag = False

    # def heartbeat(self):
    #     while not self.stop_flag and not rospy.is_shutdown():
    #         logger.info(f"采集进程存活 | PID: {self.pid} | 当前帧计数: {getattr(self, 'count', 0)}")
    #         time.sleep(5)

    # def stop_file_listener(self):
    #     stop_file = Path(f"./stop{self.args.stop}.py").resolve()
    #     logger.info(f"开始监控停止信号 | 检测路径: {stop_file}")
    #     while not self.stop_flag and not rospy.is_shutdown():
    #         if stop_file.exists():
    #             self.stop_flag = True
    #             logger.warning(f"检测到停止信号（stop{self.args.stop}.py存在），即将终止采集并保存数据...")
    #             try:
    #                 stop_file.unlink()
    #                 logger.info(f"已删除停止信号文件: {stop_file}")
    #             except Exception as e:
    #                 logger.warning(f"删除stop{self.args.stop}.py失败: {str(e)}")
    #             break
    #         time.sleep(0.1)
    
    # def no_save_listener(self):
    #     no_save_file = Path(f"./no_save{self.args.stop}.py").resolve()
    #     logger.info(f"开始监控不保存信号 | 检测路径: {no_save_file}")
    #     while not self.stop_flag and not rospy.is_shutdown():
    #         if no_save_file.exists():
    #             self.stop_flag = True
    #             logger.warning(f"检测到不保存信号（no_save{self.args.stop}.py存在），即将终止采集且不保存数据...")
    #             self.no_save_flag = True
    #             try:
    #                 no_save_file.unlink()
    #                 logger.info(f"已删除不保存信号文件: {no_save_file}")
    #             except Exception as e:
    #                 logger.warning(f"删除no_save{self.args.stop}.py失败: {str(e)}")
    #             break  
    #         time.sleep(0.1) 

    def _generic_callback(self, dq_name, msg):
        self.deques[dq_name].append(msg)

    def init_ros(self):
        try:
            rospy.init_node('record_episodes', anonymous=True)
            logger.info("ROS节点初始化成功")
            rospy.Subscriber(self.args.img_left_topic, Image, lambda msg: self._generic_callback('img_left', msg), queue_size=1000, tcp_nodelay=True)
            rospy.Subscriber(self.args.img_right_topic, Image, lambda msg: self._generic_callback('img_right', msg), queue_size=1000, tcp_nodelay=True)
            rospy.Subscriber(self.args.img_front_topic, Image, lambda msg: self._generic_callback('img_front', msg), queue_size=1000, tcp_nodelay=True)
            if self.args.use_depth_image:
                rospy.Subscriber(self.args.img_left_depth_topic, Image, lambda msg: self._generic_callback('img_left_depth', msg), queue_size=1000, tcp_nodelay=True)
                rospy.Subscriber(self.args.img_right_depth_topic, Image, lambda msg: self._generic_callback('img_right_depth', msg), queue_size=1000, tcp_nodelay=True)
                rospy.Subscriber(self.args.img_front_depth_topic, Image, lambda msg: self._generic_callback('img_front_depth', msg), queue_size=1000, tcp_nodelay=True)
            rospy.Subscriber(self.args.master_arm_left_topic, JointState, lambda msg: self._generic_callback('master_arm_left', msg), queue_size=1000, tcp_nodelay=True)
            rospy.Subscriber(self.args.master_arm_right_topic, JointState, lambda msg: self._generic_callback('master_arm_right', msg), queue_size=1000, tcp_nodelay=True)
            rospy.Subscriber(self.args.puppet_arm_left_topic, JointState, lambda msg: self._generic_callback('puppet_arm_left', msg), queue_size=1000, tcp_nodelay=True)
            rospy.Subscriber(self.args.puppet_arm_right_topic, JointState, lambda msg: self._generic_callback('puppet_arm_right', msg), queue_size=1000, tcp_nodelay=True)
            if self.args.use_robot_base:
                rospy.Subscriber(self.args.robot_base_topic, Odometry, lambda msg: self._generic_callback('robot_base', msg), queue_size=1000, tcp_nodelay=True)
        except rospy.ROSException as e:
            logger.error(f"ROS节点初始化失败：{str(e)}", exc_info=True)
            raise

    def get_frame(self):
        try:
            required_deques = ['img_left', 'img_right', 'img_front', 
                               'master_arm_left', 'master_arm_right', 
                               'puppet_arm_left', 'puppet_arm_right']
            if any(len(self.deques[dq]) == 0 for dq in required_deques):
                return None
            if self.args.use_depth_image and any(len(self.deques[f'img_{cam}_depth']) == 0 for cam in ['left', 'right', 'front']):
                return None
            frame_time = min(self.deques[dq][-1].header.stamp.to_sec() for dq in ['img_left', 'img_right', 'img_front'])
            if self.args.use_depth_image:
                frame_time = min(frame_time, *[self.deques[f'img_{cam}_depth'][-1].header.stamp.to_sec() for cam in ['left', 'right', 'front']])
            if self.args.use_robot_base:
                frame_time = min(frame_time, self.deques['robot_base'][-1].header.stamp.to_sec())
            check_deques = required_deques.copy()
            if self.args.use_depth_image:
                check_deques += [f'img_{cam}_depth' for cam in ['left', 'right', 'front']]
            if self.args.use_robot_base:
                check_deques.append('robot_base')
            if any(len(self.deques[dq]) == 0 or self.deques[dq][-1].header.stamp.to_sec() < frame_time for dq in check_deques):
                return None
            def pop_deque(dq_name):
                dq = self.deques[dq_name]
                while dq[0].header.stamp.to_sec() < frame_time:
                    dq.popleft()
                return dq.popleft()
            img_left = self.bridge.imgmsg_to_cv2(pop_deque('img_left'), 'passthrough')
            img_right = self.bridge.imgmsg_to_cv2(pop_deque('img_right'), 'passthrough')
            img_front = self.bridge.imgmsg_to_cv2(pop_deque('img_front'), 'passthrough')
            master_arm_left = pop_deque('master_arm_left')
            master_arm_right = pop_deque('master_arm_right')
            puppet_arm_left = pop_deque('puppet_arm_left')
            puppet_arm_right = pop_deque('puppet_arm_right')
            img_left_depth = img_right_depth = img_front_depth = None
            if self.args.use_depth_image:
                for cam in ['left', 'right', 'front']:
                    depth_img = self.bridge.imgmsg_to_cv2(pop_deque(f'img_{cam}_depth'), 'passthrough')
                    depth_img = cv2.copyMakeBorder(depth_img, 40, 40, 0, 0, cv2.BORDER_CONSTANT, value=0)
                    locals()[f'img_{cam}_depth'] = depth_img
            robot_base = pop_deque('robot_base') if self.args.use_robot_base else None
            return (img_front, img_left, img_right, img_front_depth, img_left_depth, img_right_depth,
                    puppet_arm_left, puppet_arm_right, master_arm_left, master_arm_right, robot_base)
        except Exception as e:
            logger.error(f"获取帧数据异常：{str(e)}", exc_info=True)
            return None

    def process(self):
        timesteps, actions = [], []
        self.count = 0
        rate = rospy.Rate(self.args.frame_rate)
        logger.info(f"采集循环启动 | 最大帧数: {self.args.max_timesteps} | 帧率: {self.args.frame_rate}Hz")
        UNCHANGED_THRESHOLD = 100
        last_qpos_left = last_qpos_right = None
        consecutive_unchanged_left = consecutive_unchanged_right = 0
        try:
            while (self.count < self.args.max_timesteps + 1) and not rospy.is_shutdown():
                frame_data = self.get_frame()
                if frame_data is None:
                    rate.sleep()
                    continue
                self.count += 1
                (img_front, img_left, img_right, img_front_depth, img_left_depth, img_right_depth,
                 puppet_arm_left, puppet_arm_right, master_arm_left, master_arm_right, robot_base) = frame_data
                obs = collections.OrderedDict()
                obs['images'] = {
                    self.args.camera_names[0]: img_front,
                    self.args.camera_names[1]: img_left,
                    self.args.camera_names[2]: img_right
                }
                if self.args.use_depth_image:
                    obs['images_depth'] = {
                        self.args.camera_names[0]: img_front_depth,
                        self.args.camera_names[1]: img_left_depth,
                        self.args.camera_names[2]: img_right_depth
                    }

                # ============= 核心修改1：puppet数据强制7维+拼接为14维 =============
                # 对position/velocity/effort分别做7维处理，缺失则补0
                p_left_pos = force_7d(puppet_arm_left.position)
                p_right_pos = force_7d(puppet_arm_right.position)
                p_left_vel = force_7d(getattr(puppet_arm_left, 'velocity', []))
                p_right_vel = force_7d(getattr(puppet_arm_right, 'velocity', []))
                p_left_eff = force_7d(getattr(puppet_arm_left, 'effort', []))
                p_right_eff = force_7d(getattr(puppet_arm_right, 'effort', []))
                # 左→右拼接，严格14维
                obs['qpos'] = np.concatenate((p_left_pos, p_right_pos))
                obs['qvel'] = np.concatenate((p_left_vel, p_right_vel))
                obs['effort'] = np.concatenate((p_left_eff, p_right_eff))

                obs['base_vel'] = [robot_base.twist.twist.linear.x, robot_base.twist.twist.angular.z] if self.args.use_robot_base else [0.0, 0.0]

                # ============= 核心修改2：不变检测用处理后的7维数据 =============
                current_qpos_left = p_left_pos
                if last_qpos_left is not None and np.allclose(current_qpos_left, last_qpos_left):
                    consecutive_unchanged_left += 1
                    if consecutive_unchanged_left >= UNCHANGED_THRESHOLD:
                        logger.warning(f"左臂位置数据连续 {consecutive_unchanged_left} 帧无变化！")
                else:
                    consecutive_unchanged_left = 0
                last_qpos_left = current_qpos_left

                current_qpos_right = p_right_pos
                if last_qpos_right is not None and np.allclose(current_qpos_right, last_qpos_right):
                    consecutive_unchanged_right += 1
                    if consecutive_unchanged_right >= UNCHANGED_THRESHOLD:
                        logger.warning(f"右臂位置数据连续 {consecutive_unchanged_right} 帧无变化！")
                else:
                    consecutive_unchanged_right = 0
                last_qpos_right = current_qpos_right

                if self.count == 1:
                    timesteps.append(dm_env.TimeStep(dm_env.StepType.FIRST, None, None, obs))
                else:
                    timesteps.append(dm_env.TimeStep(dm_env.StepType.MID, None, None, obs))
                    # ============= 核心修改3：Action构造强制7维+拼接为14维 =============
                    m_left_pos = force_7d(master_arm_left.position)
                    m_right_pos = force_7d(master_arm_right.position)
                    # 按原逻辑取前6关节+主臂第7关节，强制7维后拼接
                    left_action = force_7d(np.concatenate((p_left_pos[:6], [m_left_pos[6]])))
                    right_action = force_7d(np.concatenate((p_right_pos[:6], [m_right_pos[6]])))
                    actions.append(np.concatenate((left_action, right_action)))

                logger.debug(f"采集帧 {self.count} 完成")
                rate.sleep()
        except Exception as e:
            logger.error(f"采集循环异常：{str(e)}", exc_info=True)
            raise
        finally:
            if rospy.core.is_initialized():
                rospy.signal_shutdown("采集结束，关闭ROS节点")
                logger.info("ROS节点已关闭")
        logger.info(f"采集循环结束 | 实际采集帧数: {len(actions)}")
        return timesteps, actions

# -------------- 参数解析（完全未修改）--------------
def get_arguments():
    parser = argparse.ArgumentParser(
        description="ALOHA数据采集脚本（简化参数版）\n"
                    "仅需指定两个参数：\n"
                    "  --dataset_dir: 数据存储根目录（默认: ./data）\n"
                    "  --episode_idx: Episode索引（必须指定）",
        formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument('--dataset_dir', type=str, default="./data", 
                        help="数据集保存根目录（默认: ./data）")
    parser.add_argument('--episode_idx', type=int, required=True, 
                        help="Episode索引（必须指定，例如: 0, 1, 2...）")
    args = parser.parse_args()
    
    # ============= 硬编码所有其他参数 =============
    # 任务配置
    args.task_name = "aloha_mobile_dummy"
    args.max_timesteps = 10000
    args.frame_rate = 30  # 30Hz采集频率
    
    # 摄像头配置
    args.camera_names = ['cam_high', 'cam_left_wrist', 'cam_right_wrist']
    args.img_front_topic = '/camera_f/color/image_raw'
    args.img_left_topic = '/camera_l/color/image_raw'
    args.img_right_topic = '/camera_r/color/image_raw'
    
    # 深度图像（默认禁用）
    args.use_depth_image = False
    args.img_front_depth_topic = '/camera_f/depth/image_raw'
    args.img_left_depth_topic = '/camera_l/depth/image_raw'
    args.img_right_depth_topic = '/camera_r/depth/image_raw'
    
    # 机械臂话题
    args.master_arm_left_topic = '/master/joint_left'
    args.master_arm_right_topic = '/master/joint_right'
    args.puppet_arm_left_topic = '/puppet/joint_left'
    args.puppet_arm_right_topic = '/puppet/joint_right'
    
    # 机器人基座（默认禁用）
    args.use_robot_base = False
    args.robot_base_topic = '/odom'
    
    # 视频导出（默认启用）
    args.export_video = True
    args.video_fps = 30
    args.video_codec = 'libx264'
    args.video_quality = 23  # CRF值，越小质量越高
    
    # 其他
    args.stop = 1  # 停止信号文件标识符
    
    # 参数验证
    if args.max_timesteps <= 0:
        logger.error("max_timesteps必须大于0")
        sys.exit(1)
    if not args.camera_names:
        logger.error("camera_names不能为空")
        sys.exit(1)
    
    # logger.info(f"✓ 数据将保存至: {os.path.join(args.dataset_dir, args.task_name, f'episode_{args.episode_idx}.hdf5')}")
    # logger.info(f"✓ 采集参数: {args.max_timesteps}帧 @ {args.frame_rate}Hz | 摄像头: {args.camera_names}")
    # logger.info(f"✓ 视频导出: {'启用' if args.export_video else '禁用'} (codec={args.video_codec}, fps={args.video_fps})")
    logger.info(f'开始采集 episode_{args.episode_idx}')

    return args

# -------------- 主函数（完全未修改）--------------
def main():
    try:
        args = get_arguments()
        ros_operator = RosOperator(args)
        timesteps, actions = ros_operator.process()
        
        # if ros_operator.no_save_flag:
        #     logger.warning("用户取消保存，直接退出进程")
        #     print("用户放弃保存")
        #     sys.exit(2)
        
        if len(actions) == 0:
            logger.error("未采集到任何有效数据，退出保存")
            print("未采集到数据")
            sys.exit(0)
        try:
            # 构建保存路径（使用硬编码的task_name）
            dataset_dir = os.path.join(args.dataset_dir, args.task_name)
            Path(dataset_dir).mkdir(parents=True, exist_ok=True)
            dataset_path = os.path.join(dataset_dir, f"episode_{args.episode_idx}")
            
            save_data(args, timesteps, actions, dataset_path)
            copy2nas(data_dir=dataset_dir, episode_idx=args.episode_idx, cameras=args.camera_names)
            logger.info(f"✓ 采集任务全部完成 | 数据路径: {dataset_path}.hdf5")
        except Exception as e:
            logger.error("copy到nas时出现错误，",str(e))
            # print("脚本执行异常")
            # 需要将保存的数据删除
            raise
        print(f"采集任务完成")
        print(f"   HDF5文件: {dataset_path}.hdf5")
        if args.export_video:
            print(f"   视频目录: {Path(dataset_path).parent / 'video'}")
        
    except KeyboardInterrupt:
        logger.warning("用户手动中断（Ctrl+C）")
        print("用户手动中断")
        sys.exit(1)
    except Exception as e:
        logger.error(f"脚本执行异常：{str(e)}", exc_info=True)
        print(f"{str(e)}")
        print("脚本执行异常")
        sys.exit(1)
    finally:
        logger.info(f"采集进程（PID:{os.getpid()}）即将退出")
        sys.exit(0)

if __name__ == '__main__':
    main()