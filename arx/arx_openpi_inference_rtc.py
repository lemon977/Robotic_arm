#!/usr/bin/env python3
# -*- coding: UTF-8 -*-
"""
ARX-X5 双臂分布式推理（重构版）- 基于agilex_inference的设计
上位机推理 → WebSocket → 本地下位机执行
"""

import argparse
import time
import threading
import json
import numpy as np
import cv2
import os
import signal
import sys
from collections import deque
from typing import Dict, Any, Optional, List
from sensor_msgs.msg import JointState

try:
    import pyrealsense2 as rs
except ImportError:
    print("警告: 未安装pyrealsense2,摄像头功能将不可用")

# ===== ROS2 =====
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Header
# import 

# 导入ARX5自定义消息
try:
    from arx5_arm_msg.msg import RobotStatus, RobotCmd
    print("✓ 成功导入 arx5_arm_msg（RobotStatus + RobotCmd）")
except ImportError:
    print("❌ 无法导入 arx5_arm_msg，请确保已安装 ARX5 消息包")
    from sensor_msgs.msg import JointState
    RobotStatus = JointState
    RobotCmd = JointState  # 防止 NameError，占位
    print("⚠️ 使用 JointState 作为临时替代 RobotCmd")


# ===== openpi_client =====
from openpi_client import image_tools, websocket_client_policy

# ===== 全局变量 =====
CAMERA_NAMES = ["cam_high", "cam_right_wrist", "cam_left_wrist"]
stream_buffer = None
observation_window = deque(maxlen=2)
lang_embeddings = "hang the cloth"
rtc_prev_chunk_lock = threading.Lock()
rtc_prev_chunk = None
delay_buffer = deque(maxlen=20)  # 滑动窗口记录推理往返时间
pred_delay_steps = 0             # 估计的控制步延迟

# 调试绘图相关全局缓存
published_actions_history = []  # list[np.ndarray(shape=(14,))]
observed_qpos_history = []      # list[np.ndarray(shape=(14,))]
inferred_chunks = []            # list[dict(start_step:int, chunk:np.ndarray[chunk,14])]
inferred_chunks_lock = threading.Lock()
shutdown_event = threading.Event()

# ARX5 DH参数
DH_MODIFIED_ARX = np.array([
    [0.0,         0.0,       0.123,    0.0],
    [-np.pi/2.0,  0.0,       0.0,      -172.22/180.0*np.pi],
    [0.0,         0.28503,   0.0,      -102.78/180.0*np.pi],
    [np.pi/2.0,  -0.021984,  0.25075,  0.0],
    [-np.pi/2.0,  0.0,       0.0,      0.0],
    [np.pi/2.0,   0.0,       0.211,    0.0],
], dtype=float)


def _clamp_norm(v, max_norm):
    n = float(np.linalg.norm(v))
    if max_norm <= 0.0 or n <= 1e-12 or n <= max_norm:
        return v
    return v * (max_norm / n)


def fk_arx_modified(q6):
    """6DOF FK,返回 4x4"""
    T = np.eye(4)
    for i in range(6):
        alpha, a, d, theta_off = DH_MODIFIED_ARX[i]
        theta = q6[i] + theta_off
        ct, st = np.cos(theta), np.sin(theta)
        ca, sa = np.cos(alpha), np.sin(alpha)
        Ti = np.array([
            [ct, -st, 0, a],
            [st*ca, ct*ca, -sa, -sa*d],
            [st*sa, ct*sa, ca, ca*d],
            [0, 0, 0, 1]
        ], dtype=float)
        T = T @ Ti
    return T


def position_jacobian_numeric(q6, delta=1e-6):
    J = np.zeros((3, 6))
    p0 = fk_arx_modified(q6)[:3, 3]
    for i in range(6):
        q_plus = q6.copy()
        q_plus[i] += delta
        pi = fk_arx_modified(q_plus)[:3, 3]
        J[:, i] = (pi - p0) / delta
    return J


def dls_step_pos(q6, dp_base, lambda_):
    J = position_jacobian_numeric(q6)
    Jt = J.T
    JJt = J @ Jt + lambda_**2 * np.eye(3)
    x = np.linalg.solve(JJt, dp_base)
    return Jt @ x


def apply_micro_correction(q6, dp_vec, frame, lambda_, step_limit, dq_limit):
    """单步 DLS 末端微调,返回新 q6"""
    if dp_vec is None or np.allclose(dp_vec, 0):
        return q6
    dp_vec = _clamp_norm(dp_vec, step_limit)
    R = fk_arx_modified(q6)[:3, :3]
    dp_base = R @ dp_vec if frame == "tool" else dp_vec
    dq = dls_step_pos(q6, dp_base, lambda_)
    if dq_limit is not None:
        dq = np.clip(dq, -dq_limit, dq_limit)
    return q6 + dq

# def apply_gripper_binary(act: np.ndarray, open_val: float, close_val: float, thresh: float) -> np.ndarray:
#     out = act.copy()
#     out[6]  = close_val if act[6] >= thresh else open_val   # 左爪
#     out[13] = close_val if act[13] >= thresh else open_val  # 右爪
#     return out


class StreamActionBuffer:
    """
    维护 actions 的 chunk 队列，基于agilex_inference的设计
    """
    def __init__(self, max_chunks=10, decay_alpha=0.25, state_dim=14, smooth_method="temporal"):
        self.chunks = deque()                 
        self.max_chunks = max_chunks
        self.lock = threading.Lock()
        self.decay_alpha = float(decay_alpha)
        self.state_dim = state_dim
        self.smooth_method = smooth_method
        # 新时序平滑状态
        self.cur_chunk = deque()              # 当前生效、待发布的动作序列（平滑后的结果）
        self.k = 0                            # 已发布步数计数（用于延时裁剪）
        self.last_action = None               # 记录最后一次成功弹出的动作

    def integrate_new_chunk(self, actions_chunk: np.ndarray, max_k: int, min_m: int = 8):
        with self.lock:
            if actions_chunk is None or len(actions_chunk) == 0:
                return
            max_k = max(0, int(max_k))
            min_m = max(1, int(min_m))
            drop_n = min(self.k, max_k)
            if drop_n >= len(actions_chunk):
                return
            new_chunk = [a.copy() for a in actions_chunk[drop_n:]]

            # 原样模式：仅做延时裁剪，不做重叠平滑
            if str(self.smooth_method).lower() == "raw":
                self.cur_chunk = deque(new_chunk, maxlen=None)
                self.k = 0
                return
            
            # 构建旧序列
            if len(self.cur_chunk) == 0 and self.last_action is not None:
                old_list = [np.asarray(self.last_action, dtype=float).copy() for _ in range(min_m)]
                self.last_action = None
            else:
                old_list = list(self.cur_chunk)
                if len(old_list) > 0 and len(old_list) < min_m:
                    tail = np.asarray(old_list[-1], dtype=float).copy()
                    old_list.extend([tail.copy() for _ in range(min_m - len(old_list))])
                elif len(old_list) == 0:
                    self.cur_chunk = deque(new_chunk, maxlen=None)
                    self.k = 0
                    return
            new_list = list(new_chunk)

            overlap_len = min(len(old_list), len(new_list))
            if overlap_len <= 0:
                self.cur_chunk = deque(new_list, maxlen=None)
                self.k = 0
                return

            if len(old_list) > len(new_list):
                old_list = old_list[:len(new_list)]
                overlap_len = len(new_list)

            if overlap_len == 1:
                w_old = np.array([1.0], dtype=float)
            else:
                w_old = np.linspace(1.0, 0.0, overlap_len, dtype=float)
            w_new = 1.0 - w_old

            smoothed = [
                (w_old[i] * np.asarray(old_list[i], dtype=float) +
                 w_new[i] * np.asarray(new_list[i], dtype=float))
                for i in range(overlap_len)
            ]
            combined = smoothed + new_list[overlap_len:]
            self.cur_chunk = deque([a.copy() for a in combined], maxlen=None)
            self.k = 0

    def has_any(self):
        with self.lock:
            return len(self.cur_chunk) > 0

    def pop_next_action(self) -> np.ndarray | None:
        with self.lock:
            if len(self.cur_chunk) == 0:
                return None
            if len(self.cur_chunk) == 1:
                self.last_action = np.asarray(self.cur_chunk[0], dtype=float).copy()
            act = np.asarray(self.cur_chunk.popleft(), dtype=float)
            self.k += 1
            return act

    def clear(self):
        with self.lock:
            self.cur_chunk.clear()
            self.last_action = None
            self.k = 0


def inference_fn_non_blocking_fast(args, config, policy, ros_operator):
    """
    非阻塞推理线程：不做频率限制
    """
    global stream_buffer, observation_window, lang_embeddings

    # rate = rclpy.create_rate(args.inference_rate)
    rate = ros_operator.create_rate(args.inference_rate)
    consecutive_failures = 0
    max_consecutive_failures = 5

    while rclpy.ok() and not shutdown_event.is_set():
        try:
            time1 = time.time()
            
            # 1) 取当前最新观测
            update_observation_window(args, config, ros_operator)
            print("Get Observation Time", time.time() - time1, "s")
            
            if len(observation_window) == 0:
                continue
                
            latest_obs = observation_window[-1]
            imgs = [
                latest_obs["images"][config["camera_names"][0]],
                latest_obs["images"][config["camera_names"][1]],
                latest_obs["images"][config["camera_names"][2]],
            ]
            
            # BGR->RGB & pad/resize
            imgs = [cv2.cvtColor(im, cv2.COLOR_BGR2RGB) for im in imgs]
            imgs = image_tools.resize_with_pad(np.array(imgs), 224, 224)
            proprio = latest_obs["qpos"]

            # 2) 组装 payload
            payload = {
                "state": proprio,
                "images": {
                    "top_head":  imgs[0].transpose(2, 0, 1),
                    "hand_right": imgs[1].transpose(2, 0, 1),
                    "hand_left":  imgs[2].transpose(2, 0, 1),
                },
                "prompt": lang_embeddings,
            }

            # 3) 推理
            time1 = time.time()
            actions = policy.infer(payload)["actions"]
            print("Inference Time", time.time() - time1, "s")

            # 4) 推到并行缓冲
            if actions is not None and len(actions) > 0:
                max_k = int(getattr(args, "latency_k", 0))
                min_m = int(getattr(args, "min_smooth_steps", 8))
                stream_buffer.integrate_new_chunk(actions, max_k=max_k, min_m=min_m)
                # print(f"[infer] 推入 chunk，长度={len(actions)}")   # ← 加这行
                
                # 记录 chunk 用于调试绘图
                try:
                    step_now = max(len(published_actions_history), len(observed_qpos_history))
                    with inferred_chunks_lock:
                        inferred_chunks.append({
                            "start_step": int(step_now),
                            "chunk": np.asarray(actions, dtype=float).copy()
                        })
                except Exception:
                    pass
                
                consecutive_failures = 0  # 重置失败计数
            elif actions is None:
                print("actions is None")
            elif len(actions) == 0:
                print("len(actions) == 0")

            rate.sleep()

        except Exception as e:
            print(f"[inference_fn_non_blocking_fast] {e}")
            consecutive_failures += 1
            
            # 错误恢复
            if consecutive_failures >= max_consecutive_failures:
                print(f"[推理] 连续失败{consecutive_failures}次，清空缓冲区")
                stream_buffer.clear()
                consecutive_failures = 0
            
            try:
                rate.sleep()
            except:
                time.sleep(0.001)


def inference_fn_non_blocking_rtc(args, config, policy, ros_operator):
    """异步 RTC 推理线程：携带 prev_chunk 与延迟信息，直接推入缓冲。"""
    global stream_buffer, observation_window, rtc_prev_chunk, pred_delay_steps
    rate = ros_operator.create_rate(args.inference_rate)
    chunk_size = config["chunk_size"]
    exec_h = chunk_size if getattr(args, "rtc_execute_horizon", None) is None else args.rtc_execute_horizon
    exec_h = max(1, min(exec_h, chunk_size))

    while rclpy.ok() and not shutdown_event.is_set():
        try:
            update_observation_window(args, config, ros_operator)
            if len(observation_window) == 0:
                rate.sleep()
                continue

            latest_obs = observation_window[-1]
            imgs = [
                latest_obs["images"][config["camera_names"][0]],
                latest_obs["images"][config["camera_names"][1]],
                latest_obs["images"][config["camera_names"][2]],
            ]
            imgs = [cv2.cvtColor(im, cv2.COLOR_BGR2RGB) for im in imgs]
            imgs = image_tools.resize_with_pad(np.array(imgs), 224, 224)
            proprio = latest_obs["qpos"]

            payload = {
                "state": proprio,
                "images": {
                    "top_head": imgs[0].transpose(2, 0, 1),
                    "hand_right": imgs[1].transpose(2, 0, 1),
                    "hand_left": imgs[2].transpose(2, 0, 1),
                },
                "prompt": lang_embeddings,
                "execute_horizon": exec_h,
                "enable_rtc": True,
                "mask_prefix_delay": getattr(args, "rtc_mask_prefix_delay", False),
                "max_guidance_weight": getattr(args, "rtc_max_guidance_weight", 0.5),
            }
            with rtc_prev_chunk_lock:
                pc = np.array(rtc_prev_chunk) if rtc_prev_chunk is not None else None
            if pc is not None:
                payload["prev_action_chunk"] = pc.tolist()
            payload["inference_delay"] = int(max(0, pred_delay_steps))

            t0 = time.time()
            out = policy.infer(payload)
            rtt = time.time() - t0
            _update_delay_buffer(rtt, args.control_frequency)

            actions = out.get("actions", None) if isinstance(out, dict) else None
            if actions is None or len(actions) == 0:
                rate.sleep()
                continue

            with rtc_prev_chunk_lock:
                rtc_prev_chunk = np.asarray(actions, dtype=float)

            stream_buffer.integrate_new_chunk(
                np.asarray(actions, dtype=float),
                max_k=int(getattr(args, "latency_k", 0)),
                min_m=int(getattr(args, "min_smooth_steps", 8)),
            )
        except Exception as e:
            print(f"[inference_fn_non_blocking_rtc] {e}")
        try:
            rate.sleep()
        except Exception:
            time.sleep(0.001)


def _update_delay_buffer(rtt_sec: float, control_rate: float):
    """记录一次推理往返时间，并更新预测延迟步数。"""
    global pred_delay_steps
    if rtt_sec is None or not np.isfinite(rtt_sec):
        return
    delay_buffer.append(float(rtt_sec))
    if len(delay_buffer) == 0:
        pred_delay_steps = 0
        return
    median_rtt = float(np.median(np.asarray(delay_buffer, dtype=float)))
    pred_delay_steps = int(max(0, round(median_rtt * float(control_rate))))


def start_inference_thread(args, config, policy, ros_operator):
    inference_thread = threading.Thread(
        target=inference_fn_non_blocking_fast, 
        args=(args, config, policy, ros_operator)
    )
    inference_thread.daemon = True
    inference_thread.start()
    return inference_thread


def save_debug_plots_on_interrupt(output_dir: str = "./debug_arx5"):
    """
    保存调试图表
    """
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        
        os.makedirs(output_dir, exist_ok=True)

        pub_len = len(published_actions_history)
        obs_len = len(observed_qpos_history)
        x_pub = np.arange(pub_len)
        x_obs = np.arange(obs_len)

        with inferred_chunks_lock:
            chunks_copy = [
                {"start_step": item["start_step"], "chunk": np.asarray(item["chunk"], dtype=float)}
                for item in inferred_chunks
            ]

        for joint_idx in range(14):
            try:
                plt.figure(figsize=(12, 4))

                if len(chunks_copy) > 0:
                    for item in chunks_copy:
                        arr = item["chunk"]
                        if arr.ndim != 2 or arr.shape[1] < 14:
                            continue
                        start_step = int(item["start_step"])
                        xs = np.arange(start_step, start_step + arr.shape[0])
                        ys = arr[:, joint_idx]
                        plt.plot(xs, ys, color='C0', alpha=0.35, label='inferred_chunk' if joint_idx == 0 else "")

                # 画已发布动作轨迹
                if pub_len > 0:
                    pub_arr = np.asarray(published_actions_history, dtype=float)
                    if pub_arr.ndim == 2 and pub_arr.shape[1] >= 14:
                        ys_pub = pub_arr[:, joint_idx]
                        plt.plot(x_pub, ys_pub, color='C1', linewidth=2.0, label='published_action' if joint_idx == 0 else "")

                # 画观测 qpos 轨迹
                if obs_len > 0:
                    obs_arr = np.asarray(observed_qpos_history, dtype=float)
                    if obs_arr.ndim == 2 and obs_arr.shape[1] >= 14:
                        ys_obs = obs_arr[:, joint_idx]
                        plt.plot(x_obs, ys_obs, color='C2', linewidth=2.0, label='observed_qpos' if joint_idx == 0 else "")

                plt.title(f"Joint {joint_idx}")
                plt.xlabel("step")
                plt.ylabel("value")
                if joint_idx == 0:
                    plt.legend(loc='best')
                plt.grid(True, alpha=0.25)
                out_path = os.path.join(output_dir, f"joint_{joint_idx:02d}.png")
                plt.tight_layout()
                plt.savefig(out_path)
                print(f"saved {out_path}")
            except Exception as e:
                print(f"plot joint {joint_idx} failed: {e}")
            finally:
                plt.close()

        print(f"Saved debug plots to {output_dir}")
    except Exception as e:
        print(f"Failed to save debug plots: {e}")


def _on_sigint(signum, frame):
    """SIGINT信号处理"""
    try:
        shutdown_event.set()
    except Exception:
        pass


def update_observation_window(args, config, ros_operator):
    """更新观察窗口"""
    global observation_window
    
    if len(observation_window) == 0:
        # 初始化观察窗口
        observation_window.append({
            "qpos": None,
            "images": {
                config["camera_names"][0]: None,
                config["camera_names"][1]: None,
                config["camera_names"][2]: None,
            },
        })

    # 获取传感器数据
    frame = ros_operator.get_frame()
    if frame is None:
        return
        
    imgs, j_left, j_right = frame
    qpos = ros_operator.get_joint_positions(j_left, j_right)
    
    # JPEG压缩模拟（与训练对齐）
    def jpeg_mapping(img):
        img = cv2.imencode(".jpg", img)[1].tobytes()
        img = cv2.imdecode(np.frombuffer(img, np.uint8), cv2.IMREAD_COLOR)
        return img

    img_front = jpeg_mapping(imgs['cam_high'])
    img_left = jpeg_mapping(imgs['cam_left_wrist'])
    img_right = jpeg_mapping(imgs['cam_right_wrist'])

    observation_window.append({
        "qpos": qpos,
        "images": {
            config["camera_names"][0]: img_front,
            config["camera_names"][1]: img_right,
            config["camera_names"][2]: img_left,
        },
    })
    
    # 记录观测到的 qpos 用于绘图
    try:
        observed_qpos_history.append(np.asarray(qpos, dtype=float).copy())
    except Exception:
        pass


class ARX5ROSController(Node):
    # 导入ARX5自定义消息
    try:
        from arx5_arm_msg.msg import RobotStatus, RobotCmd
        print("✓ 成功导入arx5_arm_msg（RobotStatus + RobotCmd）")
    except ImportError:
        print("❌ 无法导入arx5_arm_msg，请确保已安装ARX5消息包")
        from sensor_msgs.msg import JointState
        RobotStatus = JointState
        RobotCmd = JointState  # 临时占位，防止NameError

    def __init__(self, args):
        super().__init__("arx5_controller")
        self.args = args
        self.bridge = CvBridge()

        self.last_qpos = None
        self.qpos_lock = threading.Lock()

        # 初始化数据缓存
        self.joint_left_deque = deque(maxlen=2000)
        self.joint_right_deque = deque(maxlen=2000)

        # 创建发布器
        self.pub_left = self.create_publisher(RobotStatus, args.joint_cmd_topic_left, 10)
        self.pub_right = self.create_publisher(RobotStatus, args.joint_cmd_topic_right, 10)

        # ===== 发布器：使用 RobotStatus 作为控制命令 =====
        self.RobotStatus = RobotStatus  # 保留类型引用（方便后面使用）

        # 订阅控制命令
        self.create_subscription(
            RobotStatus,  # 确保消息类型正确
            '/arm_master_l_cmd',  # 订阅左臂控制命令
            self.left_arm_command_callback,  # 回调函数
            10
        )

        self.create_subscription(
            RobotStatus,  # 确保消息类型正确
            '/arm_master_r_cmd',  # 订阅右臂控制命令
            self.right_arm_command_callback,  # 回调函数
            10
        )

        # ===== 订阅器：使用 RobotStatus 读取反馈 =====
        self.get_logger().info(f"订阅左臂话题: {args.joint_state_topic_left}")
        self.create_subscription(
            RobotStatus,
            args.joint_state_topic_left,
            self.joint_left_callback,
            10
        )

        self.get_logger().info(f"订阅右臂话题: {args.joint_state_topic_right}")
        self.create_subscription(
            RobotStatus,
            args.joint_state_topic_right,
            self.joint_right_callback,
            10
        )

        # ===== 数据状态标志 =====
        self.data_ready = {
            'joint_left': False,
            'joint_right': False,
            'cameras': False
        }
        self.data_ready_lock = threading.Lock()

        # ===== 初始化摄像头 =====
        self.init_cameras()

    def joint_left_callback(self, msg):
        """处理左臂 RobotStatus 消息"""
        # print(f"[CB] 收到左臂 RobotStatus 消息")
        # print(f"  - joint_pos 长度: {len(msg.joint_pos)}")
        # print(f"  - 关节数据: {msg.joint_pos}")

        self.joint_left_deque.append(msg)
        with self.data_ready_lock:
            self.data_ready['joint_left'] = True

    def joint_right_callback(self, msg):
        """处理右臂 RobotStatus 消息"""
        # print(f"[CB] 收到右臂 RobotStatus 消息")
        # print(f"  - joint_pos 长度: {len(msg.joint_pos)}")
        # print(f"  - 关节数据: {msg.joint_pos}")


        self.joint_right_deque.append(msg)
        with self.data_ready_lock:
            self.data_ready['joint_right'] = True

    def left_arm_command_callback(self, msg):
        """处理左臂控制命令"""
        # print(f"[CB] 收到左臂控制命令")
        # print(f"  - joint_pos 长度: {len(msg.joint_pos)}")
        # print(f"  - 控制命令数据: {msg.joint_pos}")

        # 将左臂控制命令和右臂当前的关节位置合并，发布控制命令
        self.set_joint_positions(np.array(msg.joint_pos + self.joint_right_deque[-1].joint_pos))

    def right_arm_command_callback(self, msg):
        """处理右臂控制命令"""
        # print(f"[CB] 收到右臂控制命令")
        # print(f"  - joint_pos 长度: {len(msg.joint_pos)}")
        # print(f"  - 控制命令数据: {msg.joint_pos}")

        # 将右臂控制命令和左臂当前的关节位置合并，发布控制命令
        self.set_joint_positions(np.array(self.joint_left_deque[-1].joint_pos + msg.joint_pos))

    def get_joint_positions(self, j_left: RobotStatus, j_right: RobotStatus) -> np.ndarray:
        """从 RobotStatus 消息获取 14 维关节位置"""
        # print(f"[DEBUG] 左臂 joint_pos: {j_left.joint_pos}")
        # print(f"[DEBUG] 右臂 joint_pos: {j_right.joint_pos}")

        left = list(j_left.joint_pos)
        right = list(j_right.joint_pos)

        # print(f"[DEBUG] 合并后维度: 左 {len(left)} + 右 {len(right)} = {len(left + right)}")

        q = np.array(left + right, dtype=float)
        with self.qpos_lock:
            self.last_qpos = q.copy()
        return q

        # return np.array(left + right, dtype=float)

    def get_frame(self):
        """获取同步的传感器数据"""
        if len(self.joint_left_deque) == 0 or len(self.joint_right_deque) == 0:
            # print(f"[DEBUG] 队列状态: 左臂{len(self.joint_left_deque)}, 右臂{len(self.joint_right_deque)}")
            return None
        
        # 获取最新的关节数据
        j_left = self.joint_left_deque[-1]  # 使用最新数据
        j_right = self.joint_right_deque[-1]
        
        # print(f"[DEBUG] 获取到关节数据: 左臂{len(j_left.joint_pos)}, 右臂{len(j_right.joint_pos)}")
        
        # 获取摄像头图像
        imgs = self.get_camera_images()
        if len(imgs) != 3:
            print(f"[DEBUG] 摄像头图像数量: {len(imgs)}")
            return None

        return imgs, j_left, j_right

    def init_cameras(self):
        """初始化RealSense摄像头"""
        try:
            import pyrealsense2 as rs
            self.pipelines = {}
            camera_serials = {
                'cam_high': self.args.camera_front_serial,
                'cam_left_wrist': self.args.camera_left_serial, 
                'cam_right_wrist': self.args.camera_right_serial
            }
            
            print("初始化RealSense相机...")
            for cam_name, serial in camera_serials.items():
                pipeline = rs.pipeline()
                config = rs.config()
                config.enable_device(serial)
                config.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)
                pipeline.start(config)
                self.pipelines[cam_name] = pipeline
                print(f"✓ {cam_name} 相机已启动")
                
            # 预热相机
            for i in range(30):
                for pipeline in self.pipelines.values():
                    pipeline.wait_for_frames(timeout_ms=5000)
            print("✓ 相机预热完成")
            
            with self.data_ready_lock:
                self.data_ready['cameras'] = True
                
        except Exception as e:
            print(f"摄像头初始化失败: {e}")
            self.pipelines = {}

    def get_camera_images(self):
        """获取摄像头图像"""
        images = {}
        if not hasattr(self, 'pipelines') or not self.pipelines:
            return images
            
        for cam_name, pipeline in self.pipelines.items():
            try:
                frames = pipeline.wait_for_frames(timeout_ms=1000)
                color_frame = frames.get_color_frame()
                if color_frame:
                    image = np.asanyarray(color_frame.get_data())
                    images[cam_name] = image
            except Exception as e:
                print(f"获取 {cam_name} 图像失败: {e}")
        return images

    def wait_for_data_ready(self, timeout: float = 15.0) -> bool:
        """等待所有传感器数据就绪"""
        print("等待传感器数据就绪...")
        start_time = time.time()
        
        while time.time() - start_time < timeout and rclpy.ok():
            with self.data_ready_lock:
                joints_ready = self.data_ready['joint_left'] and self.data_ready['joint_right']
                cameras_ready = self.data_ready['cameras']
            
            if joints_ready and cameras_ready:
                print("✓ 所有传感器数据就绪!")
                return True
            
            time.sleep(0.5)
        
        print("❌ 等待传感器数据超时!")
        return False

    def set_joint_positions(self, pos: np.ndarray):
        """发布 RobotStatus 控制命令"""
        if not rclpy.ok():
            print("[WARN] ROS 已关闭，跳过发布")
            return

        # print("[DEBUG] set_joint_positions 被调用!")
        # print(f"[DEBUG] 发布的控制命令: {pos}")

        if len(pos) != 14:
            self.get_logger().warn(f"期望14维, 实际 {len(pos)}")
            return

        msg_left = RobotStatus()
        msg_right = RobotStatus()
        msg_left.joint_pos = [float(x) for x in pos[:7]]
        msg_right.joint_pos = [float(x) for x in pos[7:]]
        
        # 发布控制命令到左臂和右臂的命令话题
        self.pub_left.publish(msg_left)
        self.pub_right.publish(msg_right)

        # print(f"[CMD] 发布 RobotStatus 左臂: {msg_left.joint_pos} 右臂: {msg_right.joint_pos}")


    def smooth_return_to_zero(self, duration: float = 3.0):
        """平滑回零功能"""
        print("正在平滑回零...")
        
        frame = self.get_frame()
        if frame is None:
            print("无法获取当前关节位置")
            return False
            
        _, j_left, j_right = frame
        current_pos = self.get_joint_positions(j_left, j_right)
        target_pos = np.zeros(14)
        target_pos[6] = 3.0
        target_pos[13] = 3.0

        
        control_hz = 50.0
        num_steps = int(duration * control_hz)
        
        for step in range(num_steps + 1):
            alpha = step / num_steps
            smooth_alpha = (1 - np.cos(alpha * np.pi)) / 2
            
            interpolated_pos = current_pos * (1 - smooth_alpha) + target_pos * smooth_alpha
            self.set_joint_positions(interpolated_pos)
            
            progress = int(alpha * 100)
            print(f"\r回零进度: {progress}%", end='', flush=True)
            time.sleep(1.0/control_hz)
        
        print("\n✓ 回零完成!")
        
        # === 自动张开夹爪 ===
        open_pos = np.zeros(14)
        open_pos[6] = 5.0
        open_pos[13] = 5.0
        self.set_joint_positions(open_pos)
        # print("已自动张开夹爪")
        
        return True


    def exit_return_to_zero(self, duration: float = 3.0):
        """平滑回零（不依赖相机，仅用缓存）"""
        with self.qpos_lock:
            if self.last_qpos is None:
                print("❌ 无缓存关节角，跳过回零")
                return False
            current_pos = self.last_qpos.copy()

        target_pos = np.zeros(14)
        control_hz = 50.0
        num_steps = int(duration * control_hz)

        for step in range(num_steps + 1):
            alpha = step / num_steps
            smooth_alpha = (1 - np.cos(alpha * np.pi)) / 2
            interp = current_pos * (1 - smooth_alpha) + target_pos * smooth_alpha
            self.set_joint_positions(interp)

            progress = int(alpha * 100)
            print(f"\r回零进度: {progress}%", end='', flush=True)
            time.sleep(1.0 / control_hz)

        print("\n✓ 回零完成!")

        # === 自动张开夹爪 ===
        open_pos = np.zeros(14)
        open_pos[6] = 5.0
        open_pos[13] = 5.0
        self.set_joint_positions(open_pos)
        # print("🟢 已自动张开夹爪")

        return True



    # def smooth_return_to_zero(self, duration: float = 3.0):
    #     """平滑回零功能"""
    #     print("正在平滑回零...")
        
    #     frame = self.get_frame()
    #     if frame is None:
    #         print("无法获取当前关节位置")
    #         return False
            
    #     _, j_left, j_right = frame
    #     current_pos = self.get_joint_positions(j_left, j_right)
    #     target_pos = np.zeros(14)
        
    #     control_hz = 50.0
    #     num_steps = int(duration * control_hz)
        
    #     for step in range(num_steps + 1):
    #         alpha = step / num_steps
    #         smooth_alpha = (1 - np.cos(alpha * np.pi)) / 2
            
    #         interpolated_pos = current_pos * (1 - smooth_alpha) + target_pos * smooth_alpha
    #         self.set_joint_positions(interpolated_pos)
            
    #         progress = int(alpha * 100)
    #         print(f"\r回零进度: {progress}%", end='', flush=True)
    #         time.sleep(1.0/control_hz)
        
    #     print("\n✓ 回零完成!")
    #     return True
    
    # def exit_return_to_zero(self, duration: float = 3.0):
    #     """平滑回零（不依赖相机，仅用缓存）"""
    #     with self.qpos_lock:
    #         if self.last_qpos is None:
    #             print("❌ 无缓存关节角，跳过回零")
    #             return False
    #         current_pos = self.last_qpos.copy()

    #     target_pos = np.zeros(14)
    #     control_hz = 50.0
    #     num_steps = int(duration * control_hz)

    #     for step in range(num_steps + 1):
    #         alpha = step / num_steps
    #         smooth_alpha = (1 - np.cos(alpha * np.pi)) / 2
    #         interp = current_pos * (1 - smooth_alpha) + target_pos * smooth_alpha
    #         self.set_joint_positions(interp)

    #         progress = int(alpha * 100)
    #         print(f"\r回零进度: {progress}%", end='', flush=True)
    #         time.sleep(1.0 / control_hz)

    #     print("\n✓ 回零完成!")
    #     return True
    
    def smooth_goto_position(self, target_pos: np.ndarray, duration: float = 3.0, hz: float = 50.0) -> bool:
        """
        平滑插值到任意 14 维目标位（实时读角，无缓存滞后）
        """
        frame = self.get_frame()
        if frame is None:
            print("❌ 无法获取当前关节角，跳过平滑移动")
            return False

        _, j_left, j_right = frame
        current_pos = self.get_joint_positions(j_left, j_right)   # 实时角
        max_delta = np.max(np.abs(target_pos - current_pos))
        print(f"👉 最大角差: {max_delta:.4f} rad")

        num_steps = int(duration * hz)
        for step in range(num_steps + 1):
            alpha = step / num_steps
            smooth_alpha = (1 - np.cos(alpha * np.pi)) / 2
            interp = current_pos * (1 - smooth_alpha) + target_pos * smooth_alpha
            self.set_joint_positions(interp)

            if step % 50 == 0 or step == num_steps:
                progress = int(alpha * 100)
                print(f"\r平滑移动进度: {progress}%", end='', flush=True)
            time.sleep(1.0 / hz)

        print("\n✓ 平滑移动完成!")
        return True

    def cleanup_cameras(self):
        """释放相机资源"""
        if hasattr(self, 'pipelines'):
            print("释放RealSense相机资源...")
            for cam_name, pipeline in self.pipelines.items():
                try:
                    pipeline.stop()
                except Exception:
                    pass



def get_config(args):
    """获取配置"""
    config = {
        "episode_len": args.max_publish_step,
        "state_dim": 14,
        "chunk_size": args.chunk_size,
        "camera_names": CAMERA_NAMES,
    }
    return config


def model_inference(args, config, ros_operator):
    """主推理循环"""
    global stream_buffer, lang_embeddings, rtc_prev_chunk

    # 加载WebSocket客户端
    policy = websocket_client_policy.WebsocketClientPolicy(args.host, args.port)
    print(f"Server metadata: {policy.get_server_metadata()}")

    max_publish_step = config["episode_len"]


    left0  = [-0.00972748, 0.44651699, 0.81998158, -0.43850613, -0.01087189, -0.08220768, 5.0]
    right0 = [-0.00972748, 0.44651699, 0.81998158, -0.43850613, -0.01087189, -0.08220768, 5.0]
    
    frame = ros_operator.get_frame()
    if frame is None:
        print("❌ 无法获取当前关节角，跳过初始插值")
    else:
        _, j_left, j_right = frame
        current_q = ros_operator.get_joint_positions(j_left, j_right)
        target_q = np.array(left0 + right0)

        print("👉 当前关节角:", current_q)
        print("👉 目标关节角:", target_q)

        # 用通用平滑函数去初始位
        ros_operator.smooth_goto_position(
            target_pos=np.array(left0 + right0),
            duration=3.0,   # 可命令行调
            hz=50.0        # 已 200 Hz
        )
        print("✓ 初始姿态平滑完成")


    # 预热推理
    try:
        print("🔥 预热推理...")
        update_observation_window(args, config, ros_operator)
        print("🔥 观测窗口已更新")
        if len(observation_window) == 0:
            print("❌ 观测窗口仍为空，跳过预热")
        else:
            print("👉 开始调用 policy.infer() 预热...")
            latest_obs = observation_window[-1]
            image_arrs = [
                latest_obs["images"][config["camera_names"][0]],
                latest_obs["images"][config["camera_names"][1]],
                latest_obs["images"][config["camera_names"][2]],
            ]
            image_arrs = [cv2.cvtColor(img, cv2.COLOR_BGR2RGB) for img in image_arrs]
            image_arrs = image_tools.resize_with_pad(np.array(image_arrs), 224, 224)
            proprio = latest_obs["qpos"]
            payload = {
                "state": proprio,
                "images": {
                    "top_head":  image_arrs[0].transpose(2, 0, 1),
                    "hand_right": image_arrs[1].transpose(2, 0, 1),
                    "hand_left":  image_arrs[2].transpose(2, 0, 1),
                },
                "prompt": lang_embeddings,
            }
            _ = policy.infer(payload)["actions"]
            print("✓ 预热推理完成")
    except Exception as e:
        print(f"❌ 预热失败: {e}")
        import traceback
        traceback.print_exc()

    # RTC 预热 prev_chunk，避免第一步回零
    if args.rtc_mode and len(observation_window) > 0:
        latest_obs = observation_window[-1]
        proprio = latest_obs.get("qpos", None)
        if proprio is not None:
            with rtc_prev_chunk_lock:
                rtc_prev_chunk = np.tile(np.asarray(proprio, dtype=float)[None, : config["state_dim"]], (config["chunk_size"], 1))

    # 初始化流缓冲区
    stream_buffer = StreamActionBuffer(
        max_chunks=args.buffer_max_chunks,
        decay_alpha=args.exp_decay_alpha,
        state_dim=config["state_dim"],
        smooth_method="raw" if (args.rtc_mode and getattr(args, "rtc_disable_smoothing", False)) else "temporal",
    )
    
    # 启动推理线程
    if args.rtc_mode:
        inference_thread = threading.Thread(
            target=inference_fn_non_blocking_rtc,
            args=(args, config, policy, ros_operator),
            daemon=True,
        )
        inference_thread.start()
    else:
        inference_thread = start_inference_thread(args, config, policy, ros_operator)

    # 主控制循环
    # rate = rclpy.create_rate(args.control_frequency)
    rate = ros_operator.create_rate(args.control_frequency)
    step = 0
    consecutive_empty_actions = 0
    max_empty_actions = 100
    
    print("开始推理控制循环...")
    
    try:
        print("🌀 主循环开始")
        while rclpy.ok() and step < max_publish_step and not shutdown_event.is_set():
            frame = ros_operator.get_frame()
            if frame is None:
                print("[DEBUG] 无法获取传感器数据，跳过此轮控制")
                rate.sleep()
                continue

            imgs, j_left, j_right = frame
            qpos = ros_operator.get_joint_positions(j_left, j_right)
            observed_qpos_history.append(qpos.copy())

            act = stream_buffer.pop_next_action()
            # import ipdb; ipdb.set_trace()
            if act is not None:
                consecutive_empty_actions = 0
                # print(f"[DEBUG] 获取到的动作: {act}")
                
                if args.use_eef_correction:
                    act = apply_eef_correction(act, qpos, args)

                act = apply_gripper_binary(act)

                ros_operator.set_joint_positions(act)
                published_actions_history.append(act.copy())

                step += 1
                if step % 50 == 0:
                    print(f"[main] step {step}, 缓冲区动作数: {len(stream_buffer.cur_chunk)}")
            else:
                print("[main] 缓冲区空，无动作可发")
                consecutive_empty_actions += 1
                if consecutive_empty_actions >= max_empty_actions:
                    print(f"[main] 连续 {consecutive_empty_actions} 次无动作, 执行安全回零")
                    ros_operator.smooth_return_to_zero(duration=3.0)
                    consecutive_empty_actions = 0

            rate.sleep()
                
    except Exception as e:
        print(f"[main] 主循环异常: {e}")
        import traceback
        traceback.print_exc()
    finally:
        shutdown_event.set()
        if inference_thread.is_alive():
            inference_thread.join(timeout=2.0)
        # print("执行安全回零...")
        # ros_operator.smooth_return_to_zero(duration=3.0)
        ros_operator.cleanup_cameras()
        if args.save_debug_plots:
            save_debug_plots_on_interrupt()
        print("✓ ARX5双臂推理控制器已安全关闭")

    return inference_thread




def apply_eef_correction(act: np.ndarray, qpos: np.ndarray, args) -> np.ndarray:
    """应用末端微调"""
    left0, right0 = qpos[:6], qpos[7:13]
    dl = np.array(args.eef_corr_left)
    dr = np.array(args.eef_corr_right)
    
    left6 = apply_micro_correction(left0, dl, "base",
                                   args.eef_lambda,
                                   args.eef_step_limit_m,
                                   args.eef_joint_step_limit)
    right6 = apply_micro_correction(right0, dr, "base",
                                    args.eef_lambda,
                                    args.eef_step_limit_m,
                                    args.eef_joint_step_limit)
    
    act2 = act.copy()
    act2[:6], act2[7:13] = left6, right6
    return act2

def apply_gripper_binary(act: np.ndarray, close_val: float = 0.0, open_val: float = 5.0, thresh: float = 2.5) -> np.ndarray:
    """应用夹爪二分阈值"""
    act2 = act.copy()
    act2[6] = open_val if act[6] >= thresh else close_val
    act2[13] = open_val if act[13] >= thresh else close_val
    return act2


def main():

    # 处理 Ctrl+C 信号
    def _on_sigint(sig, frame):
        print("接收到终止信号，准备关闭程序...")
        shutdown_event.set()          # 通知所有线程
        sys.exit(0)                   # 强制结束进程

    parser = argparse.ArgumentParser(description="ARX-X5 双臂推理控制器（重构版）")
    
    # =================== 基础话题 ===================
    parser.add_argument("--joint_cmd_topic_left", default="/arm_master_l_status")
    parser.add_argument("--joint_state_topic_left", default="/arm_slave_l_status")
    parser.add_argument("--joint_cmd_topic_right", default="/arm_master_r_status")
    parser.add_argument("--joint_state_topic_right", default="/arm_slave_r_status")



    # =================== 摄像头序列号参数 ===================
    parser.add_argument("--camera_front_serial", type=str, default='152122076864')
    parser.add_argument("--camera_left_serial", type=str, default='405622074586')
    parser.add_argument("--camera_right_serial", type=str, default='347622070706')

    # =================== 推理参数 ===================
    parser.add_argument("--host", default="192.168.10.31")
    parser.add_argument("--port", type=int, default=8000)
    parser.add_argument("--control_frequency", type=float, default=30.0)
    parser.add_argument("--inference_rate", type=float, default=4.0)
    parser.add_argument("--chunk_size", type=int, default=50)
    parser.add_argument("--max_publish_step", type=int, default=10000000)

    # =================== RTC 参数 ===================
    parser.add_argument("--rtc_mode", action="store_true", help="启用实时分块（需要服务端支持）")
    parser.add_argument("--rtc_mask_prefix_delay", action="store_true", help="RTC 时掩蔽延迟前缀")
    parser.add_argument("--rtc_max_guidance_weight", type=float, default=0.5, help="RTC 引导强度上限")
    parser.add_argument("--rtc_execute_horizon", type=int, default=None, help="RTC 执行窗口（默认等于 chunk_size）")
    parser.add_argument("--rtc_disable_smoothing", action="store_true", help="RTC 模式下禁用 chunk 平滑，仅保留延时裁剪")

    # =================== 平滑参数 ===================
    parser.add_argument("--use_temporal_smoothing", action="store_true", default=True)
    parser.add_argument("--latency_k", type=int, default=8)
    parser.add_argument("--min_smooth_steps", type=int, default=10)
    parser.add_argument("--buffer_max_chunks", type=int, default=10)
    parser.add_argument("--exp_decay_alpha", type=float, default=0.25)

    # =================== 二分阈值参数 ===================
    parser.add_argument("--gripper_open",  type=float, default=0.0,   help="夹爪打开位置 (rad)")
    parser.add_argument("--gripper_close", type=float, default=-0.8,  help="夹爪闭合位置 (rad)")
    parser.add_argument("--gripper_thresh",type=float, default=0.5,   help="推理值≥该阈值时视为闭合")

    # =================== 末端微调 ===================
    parser.add_argument("--use_eef_correction", action="store_true")
    parser.add_argument("--eef_corr_left", nargs=3, type=float, default=[0., 0., 0.])
    parser.add_argument("--eef_corr_right", nargs=3, type=float, default=[0., 0., 0.])
    parser.add_argument("--eef_lambda", type=float, default=0.001)
    parser.add_argument("--eef_step_limit_m", type=float, default=0.01)
    parser.add_argument("--eef_joint_step_limit", nargs=6, type=float, default=[0.1]*6)

    # =================== 安全与调试 ===================
    parser.add_argument("--auto_homing", action="store_true", default=True, help="启动时自动回零（默认开启）")
    parser.add_argument("--exit_homing", action="store_true", help="退出时自动回零")
    parser.add_argument("--save_debug_plots", action="store_true")

    args = parser.parse_args()

    # 注册 Ctrl+C 信号处理
    signal.signal(signal.SIGINT, _on_sigint)

    try:
        # 初始化ROS2
        rclpy.init()

        # 创建 ARX5ROSController 实例
        ros_operator = ARX5ROSController(args)

        # ✅ 启动独立线程执行 rclpy.spin()
        spin_thread = threading.Thread(target=rclpy.spin, args=(ros_operator,), daemon=True)
        spin_thread.start()
        print("✓ ROS 回调线程已启动")

        # ========== 等待传感器数据就绪 ==========
        if not ros_operator.wait_for_data_ready(timeout=15.0):
            print("❌ 传感器数据未就绪, 退出")
            return

        # ========== 自动回零（可选） ==========
        if args.auto_homing:
            print("执行自动回零...")
            ros_operator.smooth_return_to_zero(duration=3.0)
            time.sleep(1.0)

        # ========== 用户确认 ==========
        # print("\n" + "=" * 60) #加上这个
        print("等待用户回车确认...")
        input("双臂准备就绪,按回车键开始推理...")
        # print("✅ 用户已确认，进入后续流程")   # ← 把这行加上

        # ========== 获取配置并开始推理 ==========
        print("获取配置...") #加上这个
        config = get_config(args)
        inference_thread = model_inference(args, config, ros_operator) 

    except Exception as e:
        print(f"主程序异常: {e}")
        import traceback
        traceback.print_exc()

    finally:
        # 安全关闭 ROS
        try:
            shutdown_event.set()  # 假设你有设置 shutdown_event
            if rclpy.ok():
                rclpy.shutdown()
            print("✓ ROS2 已安全关闭")

            
        except Exception as e:
            print(f"ROS2 关闭异常: {e}")

        print("程序退出")
        os._exit(0)   # 强制结束所有线程

    # finally:
    #     shutdown_event.set()
    #     if inference_thread.is_alive():
    #         inference_thread.join(timeout=1.0)

    #     # ===== 退出时回零（不依赖相机，仅用缓存） =====
    #     if args.exit_homing:
    #         print("退出时回零...")
    #         ros_operator.exit_return_to_zero(duration=3.0)  # 已用缓存角

    #     ros_operator.cleanup_cameras()
    #     if args.save_debug_plots:
    #         save_debug_plots_on_interrupt()

    #     print("✓ ARX5双臂推理控制器已安全关闭")
    #     os._exit(0)   # 强制结束所有线程

    # finally:
    #     # 1. 先通知推理线程退出
    #     shutdown_event.set()
    #     if inference_thread.is_alive():
    #         inference_thread.join(timeout=2.0)

    #     # 2. 在 ROS 仍然有效的这段时间里做回零
    #     if args.exit_homing:
    #         print("退出时回零...")
    #         # 用不依赖相机的版本即可，但一定在 rclpy.shutdown() 之前
    #         ros_operator.exit_return_to_zero(duration=3.0)

    #     # 3. 回零完成后再清理相机、保存调试图
    #     ros_operator.cleanup_cameras()
    #     if args.save_debug_plots:
    #         save_debug_plots_on_interrupt()

    #     # 4. 最后才真正 shutdown ROS
    #     if rclpy.ok():
    #         rclpy.shutdown()

    #     print("✓ ARX5双臂推理控制器已安全关闭")
    #     # 不要 os._exit(0)，让线程自然结束


if __name__ == "__main__":
    main()



# python3 arx_openpi_inference_smooth.py --host 192.168.10.17 --port 8000 --save_debug_plots
