#!/usr/bin/env python3
# -- coding: UTF-8 --
"""
HDF5数据Replay工具
功能：读取HDF5文件中的qpos数据，通过位置控制replay到双臂机器人
"""

import os
import pdb
import argparse
from bimanual import BimanualArm
import numpy as np
import h5py
import time
import sys
import select
import termios
import tty
from typing import Dict, Any, Optional, List


def check_space_pressed() -> bool:
    """
    检查是否按下了空格键（非阻塞）
    
    返回:
        True: 按下了空格键
        False: 没有按下空格键
    """
    # 使用select检查是否有输入
    if select.select([sys.stdin], [], [], 0)[0]:
        char = sys.stdin.read(1)
        # 空格键的ASCII码是32，字符是' '
        if char == ' ':
            return True
        # 清空剩余输入
        while select.select([sys.stdin], [], [], 0)[0]:
            sys.stdin.read(1)
    return False


def smooth_return_to_zero(
    dual_arm: BimanualArm, 
    current_positions: List[float], 
    duration: float = 3.0, 
    control_hz: float = 50.0
):
    """
    平滑地将机械臂从当前位置回到零位
    
    参数:
        dual_arm: BimanualArm实例
        current_positions: 当前关节位置 (14维)
        duration: 回零持续时间（秒），默认3秒
        control_hz: 控制频率（Hz），默认50Hz
    """
    print("\n\n正在平滑回零...")
    
    # 目标位置（全零）
    target_positions = np.zeros(14)
    target_positions[6] = 4
    target_positions[13] = 4
    current_positions = np.array(current_positions)
    
    # 计算插值步数
    num_steps = int(duration * control_hz)
    dt = 1.0 / control_hz
    
    print(f"回零时间: {duration}s, 控制频率: {control_hz}Hz, 插值步数: {num_steps}")
    
    start_time = time.time()
    
    for step in range(num_steps + 1):
        loop_start = time.time()
        
        # 使用余弦插值，使运动更平滑（慢-快-慢）
        # alpha从0到1，使用余弦函数使变化更平滑
        alpha = step / num_steps
        smooth_alpha = (1 - np.cos(alpha * np.pi)) / 2  # 余弦插值
        
        # 线性插值位置
        interpolated_positions = current_positions * (1 - smooth_alpha) + target_positions * smooth_alpha
        
        # 分离左右臂位置 (前6个是左臂关节，第7个是左gripper，7-12是右臂关节，13是右gripper)
        left_positions = interpolated_positions[:6].tolist()
        left_gripper = float(interpolated_positions[6])
        right_positions = interpolated_positions[7:13].tolist()
        right_gripper = float(interpolated_positions[13])
        
        # 设置关节位置
        dual_arm.left_arm.set_joint_positions(left_positions)
        dual_arm.right_arm.set_joint_positions(right_positions)
        
        # 设置gripper位置
        dual_arm.left_arm.set_catch_pos(left_gripper)
        dual_arm.right_arm.set_catch_pos(right_gripper)
        
        # 显示进度
        progress = int((step / num_steps) * 100)
        print(f"\r回零进度: {progress}% | "
              f"左臂: [{', '.join(f'{x:.3f}' for x in left_positions[:3])}...] G:{left_gripper:.3f} | "
              f"右臂: [{', '.join(f'{x:.3f}' for x in right_positions[:3])}...] G:{right_gripper:.3f}",
              end='', flush=True)
        
        # 控制频率
        elapsed = time.time() - loop_start
        sleep_time = dt - elapsed
        if sleep_time > 0:
            time.sleep(sleep_time)
    
    total_time = time.time() - start_time
    print(f"\n回零完成! 总时间: {total_time:.2f}s")


def smooth_transition_to_first_frame(
    dual_arm: BimanualArm, 
    current_positions: List[float], 
    target_positions: np.ndarray,
    duration: float = 2.0, 
    control_hz: float = 50.0
):
    """
    平滑地从当前位置过渡到第一帧位置
    
    参数:
        dual_arm: BimanualArm实例
        current_positions: 当前关节位置 (14维)
        target_positions: 目标位置（第一帧位置，14维）
        duration: 过渡持续时间（秒），默认2秒
        control_hz: 控制频率（Hz），默认50Hz
    """
    print("\n正在平滑过渡到第一帧...")
    
    current_positions = np.array(current_positions)
    target_positions = np.array(target_positions)
    
    # 计算插值步数
    num_steps = int(duration * control_hz)
    dt = 1.0 / control_hz
    
    print(f"过渡时间: {duration}s, 控制频率: {control_hz}Hz, 插值步数: {num_steps}")
    
    start_time = time.time()
    
    for step in range(num_steps + 1):
        loop_start = time.time()
        
        # 使用余弦插值，使运动更平滑（慢-快-慢）
        alpha = step / num_steps
        smooth_alpha = (1 - np.cos(alpha * np.pi)) / 2  # 余弦插值
        
        # 线性插值位置
        interpolated_positions = current_positions * (1 - smooth_alpha) + target_positions * smooth_alpha
        
        # 分离左右臂位置 (前6个是左臂关节，第7个是左gripper，7-12是右臂关节，13是右gripper)
        left_positions = interpolated_positions[:6].tolist()
        left_gripper = float(interpolated_positions[6])
        right_positions = interpolated_positions[7:13].tolist()
        right_gripper = float(interpolated_positions[13])
        
        # 设置关节位置
        dual_arm.left_arm.set_joint_positions(left_positions)
        dual_arm.right_arm.set_joint_positions(right_positions)
        
        # 设置gripper位置
        dual_arm.left_arm.set_catch_pos(left_gripper)
        dual_arm.right_arm.set_catch_pos(right_gripper)
        
        # 显示进度
        progress = int((step / num_steps) * 100)
        print(f"\r过渡进度: {progress}% | "
              f"左臂: [{', '.join(f'{x:.3f}' for x in left_positions[:3])}...] G:{left_gripper:.3f} | "
              f"右臂: [{', '.join(f'{x:.3f}' for x in right_positions[:3])}...] G:{right_gripper:.3f}",
              end='', flush=True)
        
        # 控制频率
        elapsed = time.time() - loop_start
        sleep_time = dt - elapsed
        if sleep_time > 0:
            time.sleep(sleep_time)
    
    total_time = time.time() - start_time
    print(f"\n过渡完成! 总时间: {total_time:.2f}s\n")


def replay_episode(dual_arm: BimanualArm, hdf5_path: str, replay_hz: float = 50.0, 
                   transition_duration: float = 2.0) -> Optional[np.ndarray]:
    """
    Replay HDF5文件中的episode数据
    
    参数:
        dual_arm: BimanualArm实例
        hdf5_path: HDF5文件路径
        replay_hz: Replay频率（Hz），默认50Hz
        transition_duration: 过渡到第一帧的时间（秒），默认2秒
    
    返回:
        最后一帧的qpos位置，用于后续回零
    """
    print(f"正在加载HDF5文件: {hdf5_path}")
    
    last_qpos = None
    
    # 保存终端设置
    old_settings = None
    try:
        old_settings = termios.tcgetattr(sys.stdin)
    except Exception:
        print("警告：无法保存终端设置，空格键中断功能可能不可用")
    
    try:
        with h5py.File(hdf5_path, 'r') as f:
            # 读取qpos数据
            qpos_data = f['observations']['qpos'][:]
            print(f"数据shape: {qpos_data.shape}")
            print(f"总帧数: {len(qpos_data)}")
            print(f"关节数: {qpos_data.shape[1]}")
            
            if len(qpos_data) == 0:
                print("错误：数据为空")
                return None
            
            # 获取当前机器人位置
            current_qpos = []
            # 左臂关节位置
            left_joint_pos_raw = dual_arm.left_arm.get_joint_positions()
            # 如果返回的包含gripper，只取前6个关节
            if len(left_joint_pos_raw) == 7:
                left_joint_pos = left_joint_pos_raw[:6]
                left_gripper = left_joint_pos_raw[6]
            else:
                left_joint_pos = left_joint_pos_raw[:6]
                left_gripper = 0.0  # 假设值
            current_qpos.extend(left_joint_pos)
            current_qpos.append(left_gripper)
            
            # 右臂关节位置
            right_joint_pos_raw = dual_arm.right_arm.get_joint_positions()
            # 如果返回的包含gripper，只取前6个关节
            if len(right_joint_pos_raw) == 7:
                right_joint_pos = right_joint_pos_raw[:6]
                right_gripper = right_joint_pos_raw[6]
            else:
                right_joint_pos = right_joint_pos_raw[:6]
                right_gripper = 0.0  # 假设值
            current_qpos.extend(right_joint_pos)
            current_qpos.append(right_gripper)
            
            # 调试信息
            print(f"\n调试信息: 左臂原始维度={len(left_joint_pos_raw)}, 右臂原始维度={len(right_joint_pos_raw)}, 当前qpos总维度={len(current_qpos)}")
            
            print(f"当前位置: 左臂[{', '.join(f'{x:.3f}' for x in left_joint_pos[:3])}...] G:{left_gripper:.3f} | "
                  f"右臂[{', '.join(f'{x:.3f}' for x in right_joint_pos[:3])}...] G:{right_gripper:.3f}")
            print(f"目标位置(第一帧): 左臂[{', '.join(f'{x:.3f}' for x in qpos_data[0][:3])}...] G:{qpos_data[0][6]:.3f} | "
                  f"右臂[{', '.join(f'{x:.3f}' for x in qpos_data[0][7:10])}...] G:{qpos_data[0][13]:.3f}")
            
            # 平滑过渡到第一帧
            smooth_transition_to_first_frame(
                dual_arm, 
                current_qpos, 
                qpos_data[0],
                duration=transition_duration,
                control_hz=replay_hz
            )
            
            # 计算时间间隔
            dt = 1.0 / replay_hz
            print(f"Replay频率: {replay_hz} Hz (每帧间隔: {dt*1000:.2f} ms)")
            
            # 开始replay
            print("\n开始Replay...")
            print("提示：按空格键可随时停止replay并归零\n")
            
            # 设置终端为cbreak模式（非阻塞）
            if old_settings is not None:
                tty.setcbreak(sys.stdin.fileno())
            
            start_time = time.time()
            user_interrupted = False  # 标记是否被用户中断
            
            for idx, qpos in enumerate(qpos_data):
                loop_start = time.time()
                
                # 检查是否按下空格键
                if old_settings is not None and check_space_pressed():
                    print(f"\n\n检测到空格键按下！")
                    print(f"Replay已中断（已播放 {idx+1}/{len(qpos_data)} 帧）")
                    user_interrupted = True
                    last_qpos = qpos  # 保存当前位置
                    break
                
                # qpos是14维，前7个是左臂(6关节+1gripper)，后7个是右臂(6关节+1gripper)
                if len(qpos) == 14:
                    # 左臂：前6个是关节，第7个是gripper
                    left_joint_positions = qpos[:6].tolist()
                    left_gripper_pos = float(qpos[6])
                    
                    # 右臂：索引7-12是关节，索引13是gripper
                    right_joint_positions = qpos[7:13].tolist()
                    right_gripper_pos = float(qpos[13])
                    
                    # 设置关节位置
                    dual_arm.left_arm.set_joint_positions(left_joint_positions)
                    dual_arm.right_arm.set_joint_positions(right_joint_positions)
                    
                    # 设置gripper位置
                    dual_arm.left_arm.set_catch_pos(left_gripper_pos)
                    dual_arm.right_arm.set_catch_pos(right_gripper_pos)
                    
                    # 保存最后一帧位置
                    last_qpos = qpos
                    
                    # 显示进度（包含gripper信息）
                    print(f"\r帧 {idx+1}/{len(qpos_data)} | "
                          f"左臂: [{', '.join(f'{x:.3f}' for x in left_joint_positions[:3])}...] G:{left_gripper_pos:.3f} | "
                          f"右臂: [{', '.join(f'{x:.3f}' for x in right_joint_positions[:3])}...] G:{right_gripper_pos:.3f}",
                          end='', flush=True)
                
                else:
                    print(f"\n警告：qpos维度不是14，而是{len(qpos)}")
                    print(f"当前qpos: {qpos}")
                
                # 控制频率
                elapsed = time.time() - loop_start
                sleep_time = dt - elapsed
                if sleep_time > 0:
                    time.sleep(sleep_time)
                else:
                    print(f"\n警告：帧{idx}执行时间({elapsed*1000:.2f}ms)超过目标间隔({dt*1000:.2f}ms)")
            
            total_time = time.time() - start_time
            
            if not user_interrupted:
                print(f"\n\nReplay完成!")
                print(f"总时间: {total_time:.2f}s")
                print(f"平均频率: {len(qpos_data)/total_time:.2f} Hz")
            else:
                print(f"总时间: {total_time:.2f}s")
    
    except FileNotFoundError:
        print(f"错误：文件不存在: {hdf5_path}")
    except KeyError as e:
        print(f"错误：HDF5文件中缺少键: {e}")
    except Exception as e:
        print(f"错误：{e}")
        import traceback
        traceback.print_exc()
    finally:
        # 恢复终端设置
        if old_settings is not None:
            try:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
            except Exception:
                pass
    
    return last_qpos


def parse_args():
    parser = argparse.ArgumentParser(description="HDF5数据replay工具")
    parser.add_argument("--hdf5_path", type=str, help="HDF5文件路径")
    parser.add_argument("--replay_hz", type=float, default=30.0, help="Replay频率(Hz)，默认50")
    parser.add_argument("--transition_duration", type=float, default=2.0, help="过渡到第一帧的时间(秒)，默认2")
    return parser.parse_args()


def main():
    """主函数"""
    exit_if_master_or_slave_running()
    # 解析参数
    args = parse_args()
    # hdf5_path = args.hdf5_path
    hdf5_path = args.hdf5_path if args.hdf5_path.endswith('.hdf5') else args.hdf5_path + '.hdf5'
    if not os.path.exists(hdf5_path):
        print("文件不存在")
        return

    # Replay频率（Hz），来自命令行参数
    replay_hz = args.replay_hz
    
    # 过渡到第一帧的时间（秒），来自命令行参数
    transition_duration = args.transition_duration
    
    # 回零参数
    return_to_zero_duration = 2.0  # 回零持续时间（秒）
    return_to_zero_hz = 50.0  # 回零控制频率（Hz）
    
    # 双臂配置
    left_arm_config: Dict[str, Any] = {
        "can_port": "can1",
        # 根据需要添加其他配置参数
    }
    
    right_arm_config: Dict[str, Any] = {
        "can_port": "can3",
        # 根据需要添加其他配置参数
    }
    
    print("="*60)
    print("HDF5数据Replay工具")
    print("="*60)
    
    # 创建BimanualArm实例

    # init_time = time.time()
    print("\n初始化双臂机器人...")
    global bimanual_arm
    bimanual_arm = BimanualArm(left_arm_config, right_arm_config)
    # pdb.set_trace()
    # 执行replay
    last_qpos = replay_episode(bimanual_arm, hdf5_path, replay_hz, transition_duration)
    
    # 平滑回零
    if last_qpos is not None:
        smooth_return_to_zero(
            bimanual_arm, 
            last_qpos.tolist(), 
            duration=return_to_zero_duration,
            control_hz=return_to_zero_hz
        )
    else:
        print("\n警告：无法获取最后位置，跳过回零")

    #重播前，检测主从遥操是否运行
import subprocess
import sys

def _find_pids(pattern: str) -> list[int]:
    """返回所有匹配 pattern 的 pid 列表（排除 grep 自身）"""
    cmd = f"ps -ef | grep -E '{pattern}' | grep -v grep | awk '{{print $2}}'"
    try:
        out = subprocess.check_output(cmd, shell=True, text=True).strip()
        return [int(p) for p in out.split()] if out else []
    except subprocess.CalledProcessError:
        return []

def exit_if_master_or_slave_running() -> None:
    """
    只要 open_remote_master.launch.py 或 open_remote_slave.launch.py
    任意一个还在跑，就打印三行提示并退出整个脚本。
    """
    master_pids = _find_pids(r'open_remote_master\.launch\.py')
    slave_pids  = _find_pids(r'open_remote_slave\.launch\.py')

    if master_pids or slave_pids:
        for _ in range(3):
            print("检测到主从遥操未关闭，请先关闭后再进行重播")
        sys.exit(1)          # 直接退出 replay123.py

    #按Ctrl+C,机械臂回到零点，并退出程序
import sys
import signal
import time
import numpy as np

# -------------- 回零封装 --------------
def move_both_arms_to_zero(dual_arm: "BimanualArm", duration: float = 3.0, hz: float = 50.0) -> None:
    """
    把主臂（左）和从臂（右）同时平滑拉回零位。
    与 smooth_return_to_zero 逻辑一致，但不再依赖最后一帧数据。
    """
    print("\n\nCtrl+C 触发，正在平滑回零...")
    current = []
    # 左臂 6 关节 + gripper
    left_j  = dual_arm.left_arm.get_joint_positions()
    if len(left_j) == 7:
        current.extend(left_j)
    else:
        current.extend(left_j[:6])
        current.append(0.0)
    # 右臂 6 关节 + gripper
    right_j = dual_arm.right_arm.get_joint_positions()
    if len(right_j) == 7:
        current.extend(right_j)
    else:
        current.extend(right_j[:6])
        current.append(0.0)

    current = np.array(current)
    target  = np.zeros(14)
    steps   = int(duration * hz)
    dt      = 1.0 / hz

    for step in range(steps + 1):
        alpha        = step / steps
        smooth_alpha = (1 - np.cos(alpha * np.pi)) / 2
        interp       = current * (1 - smooth_alpha) + target * smooth_alpha

        left_pos , left_g  = interp[:6].tolist(), float(interp[6])
        right_pos, right_g = interp[7:13].tolist(), float(interp[13])

        dual_arm.left_arm.set_joint_positions(left_pos)
        dual_arm.right_arm.set_joint_positions(right_pos)
        dual_arm.left_arm.set_catch_pos(left_g)
        dual_arm.right_arm.set_catch_pos(right_g)
        time.sleep(dt)
    print("回零完成，即将退出程序。")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        # ======== Ctrl+C 入口 ========
        # 确保 bimanual_arm 实例已创建（main 里第一段就会生成）
        # 把 bimanual_arm 做成全局引用即可
        move_both_arms_to_zero(bimanual_arm, duration=3.0, hz=50.0)
        sys.exit(0)
    except Exception as e:
        print(f"\n程序异常: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)

