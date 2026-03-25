#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
=============================================================
机器人数据校验脚本
核心功能：
1. 自动检测：日期文件夹、主机用户路径、所有任务子目录
2. 数据校验：HDF5数据集 与 三路相机视频 数量/编号是否一致
3. 异常清理：交互式删除**跨通道不完整**的episode数据（缺失视频/HDF5）
4. 辅助功能：日志输出、批量检查、缺失编号检测
支持参数：
 --date MMDD   指定检查的日期（4位，如0319）
 --all         批量检查所有匹配日期的文件夹
 --log         启用日志文件（保存在脚本目录）
=============================================================
"""
from math import e
# 系统交互/文件操作
import os
import sys
# 日期时间处理
import datetime
# 正则表达式（匹配episode文件名）
import re
# 获取当前登录用户
import getpass
# 命令行参数解析
import argparse
# 文件删除/移动
import shutil
# 面向对象路径处理
from pathlib import Path

# ====================== 全局常量配置 ======================
# 自动获取当前日期（月日，如0319），用于匹配数据文件夹
DATE = datetime.datetime.now().strftime("%m%d")
# 全局日志文件对象（初始为空）
LOG_FILE = None
# 固定相机名称（与采集脚本一一对应）
CAMERA_NAMES = ['cam_high', 'cam_left_wrist', 'cam_right_wrist']

# ====================== 日志功能模块 ======================
class TeeOutput:
    """
    日志核心类：实现【终端+日志文件】双输出
    作用：替代系统默认输出，所有print同时打印到屏幕和文件
    """
    def __init__(self, *files):
        # 接收多个文件对象（终端+日志文件）
        self.files = files
    
    def write(self, data):
        # 写入数据到所有输出流
        for f in self.files:
            f.write(data)
            f.flush()  # 立即刷新，不缓存
    
    def flush(self):
        # 刷新所有流
        for f in self.files:
            f.flush()

def setup_logging(fd_name: str):
    """
    初始化日志文件
    :param fd_name: 日期文件夹名称，用于日志命名
    :return: 日志文件路径
    """
    global LOG_FILE
    # 获取脚本所在目录（日志固定存在这里）
    script_dir = Path(__file__).parent
    # 日志文件名：check_日期_时间.log
    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    log_filename = f"check_{fd_name}_{timestamp}.log"
    log_path = script_dir / log_filename
    
    # 打开日志文件
    LOG_FILE = open(log_path, 'w', encoding='utf-8')
    # 重定向标准输出：同时打印到终端+日志
    sys.stdout = TeeOutput(sys.__stdout__, LOG_FILE)
    print(f"日志文件: {log_path}")
    return log_path

def close_logging():
    """关闭日志文件，恢复默认输出"""
    global LOG_FILE
    if LOG_FILE:
        sys.stdout = sys.__stdout__  # 恢复终端默认输出
        LOG_FILE.close()
        LOG_FILE = None

# ====================== 路径自动检测模块 ======================
def detect_user_home_candidates():
    """
    自动检测机器人数据根目录（适配不同主机：kai/agilex/当前用户）
    作用：无需手动写死路径，自动匹配可用的data文件夹
    :return: 存在的data路径列表
    """
    # 获取当前登录用户名
    user = os.environ.get("USER") or getpass.getuser() or os.path.basename(os.path.expanduser("~"))
    # 候选路径（覆盖常用机器人主机配置）
    candidates = [f"/home/{user}/data", "/home/kai/data", "/home/agilex/data"]
    # 去重+过滤：只保留真实存在的文件夹
    seen = set()
    existing = []
    for p in candidates:
        if p not in seen:
            seen.add(p)
            if os.path.isdir(p):
                existing.append(p)
    return existing

def find_folders_with_date(base_dir=None, date_str=DATE):
    """
    核心：查找包含指定日期的所有数据文件夹
    :param base_dir: 根目录（为空则自动检测）
    :param date_str: 日期字符串（如0319）
    :return: 匹配的文件夹列表（按修改时间倒序）
    """
    # 根目录：手动指定 / 自动检测
    base_dirs = [base_dir] if base_dir else detect_user_home_candidates()
    results = []
    for root in base_dirs:
        if not os.path.isdir(root):
            continue
        # 遍历根目录下所有文件夹
        for item in os.listdir(root):
            item_path = os.path.join(root, item)
            if os.path.isdir(item_path) and date_str in item:
                try:
                    mtime = os.path.getmtime(item_path)  # 获取修改时间
                except OSError:
                    mtime = 0
                results.append((root, item, mtime))
    # 按修改时间降序排列（最新的在前）
    results.sort(key=lambda x: x[2], reverse=True)
    return results

def find_folder_with_date(base_dir=None, date_str=DATE):
    """兼容旧代码：返回单个最佳匹配的日期文件夹"""
    matches = find_folders_with_date(base_dir, date_str)
    if matches:
        root, name, _ = matches[0]
        return root, name
    return None, None

# ====================== 任务目录枚举模块 ======================
def list_task_dirs(base_dir: str):
    """
    自动枚举所有需要检查的【任务子目录】
    规则：目录下有 video文件夹 或 .hdf5文件
    作用：不写死任务名（aloha_mobile_dummy等），自动扫描
    :param base_dir: 日期根目录
    :return: 所有任务目录的绝对路径列表
    """
    candidates = []
    if not os.path.isdir(base_dir):
        return candidates
    try:
        # 遍历日期目录下的一级子文件夹
        for name in os.listdir(base_dir):
            p = os.path.join(base_dir, name)
            if not os.path.isdir(p):
                continue
            # 判断条件：存在video目录 或 存在hdf5文件
            has_video = os.path.isdir(os.path.join(p, 'video'))
            has_hdf5 = any(f.endswith('.hdf5') for f in os.listdir(p) if os.path.isfile(os.path.join(p, f)))
            if has_video or has_hdf5:
                candidates.append(p)
    except Exception:
        pass

    # 兜底逻辑：如果日期目录本身就是任务目录，直接加入
    if not candidates:
        try:
            has_video = os.path.isdir(os.path.join(base_dir, 'video'))
            has_hdf5 = any(f.endswith('.hdf5') for f in os.listdir(base_dir) if os.path.isfile(os.path.join(base_dir, f)))
            if has_video or has_hdf5:
                candidates.append(base_dir)
        except Exception:
            pass
    return candidates

# ====================== 编号缺失检测模块 ======================
def find_missing_numbers(directory: str, ext: str):
    """
    检测文件夹内 episode_数字.后缀 的**连续编号缺失**
    例：episode_0.mp4, episode_2.mp4 → 缺失1
    :param directory: 文件夹路径
    :param ext: 文件后缀（hdf5/mp4）
    :return: 缺失的编号列表 / None（目录不存在）
    """
    if not os.path.isdir(directory):
        return None
    try:
        files = os.listdir(directory)
    except Exception:
        files = []

    nums = []
    # 正则匹配：episode_数字.后缀（忽略大小写）
    pat = re.compile(rf'^episode_(\d+)\.{re.escape(ext)}$', re.IGNORECASE)
    for f in files:
        m = pat.match(f)
        if m:
            try:
                nums.append(int(m.group(1)))
            except ValueError:
                pass
    # 无文件 → 返回空列表
    if not nums:
        return []
    # 计算0~最大编号之间的缺失值
    max_num = max(nums)
    full = set(range(max_num + 1))
    return sorted(full - set(nums))

# ====================== 数据清理核心模块 ======================
# 正则：匹配所有episode文件（hdf5/mp4）
EP_RE = re.compile(r"^episode_(\d+)\.(hdf5|mp4)$", re.IGNORECASE)

def _collect_episodes(task_dir: str):
    """
    【核心数据统计】收集任务目录下所有episode的完整信息
    :param task_dir: 任务目录路径
    :return: 字典结构：
        {
            0: {'hdf5': Path对象, 'mp4': {'cam_high': Path对象...}},
            1: {...}
        }
    """
    episodes = {}
    task = Path(task_dir)
    hdf5_dir = task
    cam_root = task / 'video'
    # 获取所有相机文件夹
    cam_dirs = [p for p in (cam_root.glob('cam_*') if cam_root.is_dir() else []) if p.is_dir()]
    
    # 1. 收集所有hdf5文件
    for h5 in hdf5_dir.glob('episode_*.hdf5'):
        m = EP_RE.match(h5.name)
        if not m:
            continue
        ep = int(m.group(1))
        episodes.setdefault(ep, {})['hdf5'] = h5
    
    # 2. 收集所有相机视频文件
    for cam in cam_dirs:
        cam_name = cam.name
        for mp4 in cam.glob('episode_*.mp4'):
            m = EP_RE.match(mp4.name)
            if not m:
                continue
            ep = int(m.group(1))
            episodes.setdefault(ep, {}).setdefault('mp4', {})[cam_name] = mp4
    return episodes

def _parse_delete_set(text: str):
    """
    解析用户输入的删除编号（支持：1,3,5-8）
    :param text: 用户输入字符串
    :return: 要删除的编号集合
    """
    text = text.strip()
    if not text:
        return set()
    out = set()
    # 按逗号/空格分割
    for part in re.split(r"[,\s]+", text):
        if not part:
            continue
        if '-' in part:
            # 处理范围：5-8
            a,b = part.split('-',1)
            if a.isdigit() and b.isdigit():
                ai, bi = int(a), int(b)
                if ai <= bi:
                    out.update(range(ai, bi+1))
                else:
                    out.update(range(bi, ai+1))
        elif part.isdigit():
            # 处理单个数字
            out.add(int(part))
    return out

def _gather_delete_list(episodes: dict, delete_set: set, task_dir: str = None):
    """
    【预览删除】生成待删除文件列表（不实际删除）
    :param episodes: 收集的所有episode数据
    :param delete_set: 要删除的编号集合
    :param task_dir: 任务目录（用于打印相对路径）
    :return: 待删除文件列表（编号+路径+相对路径）
    """
    to_delete = []
    for ep in sorted(delete_set):
        info = episodes.get(ep)
        if not info:
            print(f"[WARN] episode_{ep} 不存在，跳过")
            continue

        # 添加hdf5文件
        h5 = info.get('hdf5')
        if h5 and h5.exists():
            rel = os.path.relpath(str(h5), task_dir) if task_dir else str(h5)
            print(f"[DELETE-PREVIEW] {rel}")
            to_delete.append((ep, h5, rel))

        # 添加所有相机视频
        for cam, mp4 in info.get('mp4', {}).items():
            if mp4 and mp4.exists():
                rel = os.path.relpath(str(mp4), task_dir) if task_dir else str(mp4)
                print(f"[DELETE-PREVIEW] {rel}")
                to_delete.append((ep, mp4, rel))

    return to_delete

def _execute_delete_list(to_delete: list, episodes: dict = None):
    """
    【执行删除】永久删除文件
    :param to_delete: 待删除列表
    :param episodes: 数据字典（删除后同步更新）
    :return: 已删除文件列表
    """
    deleted = []
    ep_counts = {}
    # 统计每个episode的待删文件数
    for ep, p, rel in to_delete:
        ep_counts.setdefault(ep, 0)
        ep_counts[ep] += 1

    for ep, p, rel in to_delete:
        try:
            p.unlink()  # 删除文件
            print(f"[DELETED] {rel}")
            deleted.append(rel)
            # 删除完成后，从数据字典中移除该episode
            ep_counts[ep] -= 1
            if episodes is not None and ep_counts[ep] <= 0:
                episodes.pop(ep, None)
        except Exception as e:
            print(f"[ERROR] 删除 {rel} 失败: {e}")
    return deleted

def _print_deleted_summary(deleted_files: list):
    """打印删除完成的摘要日志"""
    print("\n=== 已删除文件摘要 ===")
    if deleted_files:
        print(f"已删除文件 ({len(deleted_files)}个):")
        for f in deleted_files:
            print(f"  - {f}")
    else:
        print("已删除文件: 无")

def cleanup_auto(task_dir: str):
    """
    【自动清理主逻辑】
    1. 检测跨通道不完整数据（某episode缺失HDF5/视频）
    2. 交互式确认删除
    3. 仅删除不完整数据，不修改编号
    """
    # 收集所有episode数据
    episodes = _collect_episodes(task_dir)
    if not episodes:
        print("[INFO] 未发现任何 episode 文件，跳过。")
        return

    # 统计各通道（HDF5+相机）存在的episode编号
    hset = set()
    cam_sets = {}
    for ep, info in episodes.items():
        if info.get('hdf5'):
            hset.add(ep)
        for cam, p in info.get('mp4', {}).items():
            cam_sets.setdefault(cam, set()).add(ep)

    # 无数据直接退出
    if not hset and not cam_sets:
        print("[INFO] 无 hdf5 或 mp4 记录，跳过。")
        return

    # 计算【所有通道都存在】的episode（交集）
    channel_sets = []
    if hset:
        channel_sets.append(hset)
    for s in cam_sets.values():
        if s:
            channel_sets.append(s)
    if not channel_sets:
        print("[INFO] 无可用通道数据，跳过。")
        return

    # 完整episode = 所有通道都有数据
    keep = set.intersection(*channel_sets)
    all_eps = set(episodes.keys())
    # 不完整episode = 需要删除
    delete_set = sorted(all_eps - keep)

    # 无异常数据 → 直接退出
    if not delete_set:
        return

    # ====================== 打印异常信息 ======================
    print("\n=== 自动清理检查 ===")
    print(f"可用通道数: {len(channel_sets)}，总 episode: {len(all_eps)}，将删除不完整: {len(delete_set)}")
    print("[跨通道差异] 以下 episode 在部分通道缺失：")
    preview = " ".join(map(str, delete_set)) if len(delete_set) <= 200 else f"{delete_set[0]}..{delete_set[-1]} (共{len(delete_set)}个)"
    print(preview)

    # 检测连续编号缺失
    remaining_eps = sorted(episodes.keys())
    seq_missing = []
    if remaining_eps:
        expected = list(range(max(remaining_eps) + 1))
        seq_missing = sorted(set(expected) - set(remaining_eps))
    if seq_missing:
        print("[顺序编号缺失] 在 0..max 范围内缺失以下编号：")
        print(", ".join(map(str, seq_missing)))

    # ====================== 确认并删除 ======================
    to_delete = _gather_delete_list(episodes, set(delete_set), task_dir=task_dir)
    if not to_delete:
        print("[INFO] 未发现可删除的文件，退出。")
        return
    confirm = input("是否永久删除上述文件？(yes/no)：").strip().lower()
    if confirm != 'yes':
        print("[CANCEL] 已取消删除。")
        return

    # 执行删除
    deleted_files = _execute_delete_list(to_delete, episodes=episodes)
    _print_deleted_summary(deleted_files)
    if deleted_files:
        print(f"[DONE] 已永久删除 {len(deleted_files)} 个文件，流程结束。")
    else:
        print("[INFO] 未删除任何文件，流程结束。")

# ====================== 数据校验核心模块 ======================
# 全局路径（在主函数中动态赋值）
DATA_PATH = ""   # 任务目录（存放hdf5）
VIDEO_PATH = ""  # 视频目录（任务目录下的video）

def check_data():
    """
    【数据校验主函数】
    1. 统计HDF5和三路视频的文件数量
    2. 检测跨通道缺失（某episode只有视频/只有HDF5）
    3. 检测连续编号缺失
    """
    # 路径校验
    if not os.path.isdir(DATA_PATH):
        print(f"数据路径不存在: {DATA_PATH}")
        exit(1)
    
    # 统计所有文件
    hdf5_files = [f for f in os.listdir(DATA_PATH) if f.endswith('.hdf5')]
    cam_high_dir = os.path.join(VIDEO_PATH, 'cam_high')
    cam_left_dir = os.path.join(VIDEO_PATH, 'cam_left_wrist')
    cam_right_dir = os.path.join(VIDEO_PATH, 'cam_right_wrist')
    
    # 三路视频文件统计
    cam_high_videos = [f for f in os.listdir(cam_high_dir) if f.endswith('.mp4')] if os.path.isdir(cam_high_dir) else []
    cam_left_wrist_videos = [f for f in os.listdir(cam_left_dir) if f.endswith('.mp4')] if os.path.isdir(cam_left_dir) else []
    cam_right_wrist_videos = [f for f in os.listdir(cam_right_dir) if f.endswith('.mp4')] if os.path.isdir(cam_right_dir) else []

    # ====================== 数量统计 ======================
    print("=== 数量统计 ===")
    print(f"HDF5: {len(hdf5_files)}，cam_high: {len(cam_high_videos)}，cam_left_wrist: {len(cam_left_wrist_videos)}，cam_right_wrist: {len(cam_right_wrist_videos)}")
    all_counts = [len(hdf5_files), len(cam_high_videos), len(cam_left_wrist_videos), len(cam_right_wrist_videos)]
    # 判断数量是否完全一致
    if len(set(all_counts)) == 1:
        print("结论：数量一致✅")
    else:
        print("结论：数量不一致❌（详见下方缺失编号）")

    # ====================== 缺失检测 ======================
    episodes_map = _collect_episodes(DATA_PATH)
    if episodes_map:
        hset = set()
        cam_sets = {}
        found_cams = set()
        for ep, info in episodes_map.items():
            if info.get('hdf5'):
                hset.add(ep)
            for cam, p in info.get('mp4', {}).items():
                cam_sets.setdefault(cam, set()).add(ep)
                found_cams.add(cam)
        eps_union = set(episodes_map.keys())

        # 1. 跨通道缺失（和全集对比）
        print("\n=== 跨通道差异检测 ===")
        miss_h_vs_all = sorted(eps_union - hset)
        if miss_h_vs_all:
            print("[HDF5] 缺少：" + ", ".join(map(str, miss_h_vs_all)))

        for cam in sorted(found_cams):
            s = cam_sets.get(cam, set())
            miss_cam_vs_all = sorted(eps_union - s)
            if miss_cam_vs_all:
                print(f"[VIDEO:{cam}] 缺少：" + ", ".join(map(str, miss_cam_vs_all)))

        # 2. 连续编号缺失
        print("\n=== 顺序编号缺失检测 ===")
        h_miss_nums = find_missing_numbers(DATA_PATH, 'hdf5')
        if h_miss_nums:
            print("[HDF5] 缺少：" + ", ".join(map(str, h_miss_nums)))

        for cam in CAMERA_NAMES:
            cam_dir = os.path.join(VIDEO_PATH, cam)
            miss_nums = find_missing_numbers(cam_dir, 'mp4')
            tag = f"[VIDEO:{cam}]"
            if miss_nums is None:
                print(f"{tag} 目录不存在或无法读取: {cam_dir}")
            elif miss_nums:
                print(f"{tag} 缺少：" + ", ".join(map(str, miss_nums)))

# ====================== 命令行参数解析 ======================
def parse_args():
    """解析脚本启动参数"""
    parser = argparse.ArgumentParser(description="检查 HDF5 与三路相机视频数量/编号一致性")
    parser.add_argument("-d", "--date", help="指定日期(4位MMDD)，例如 0319；不指定则使用当天")
    parser.add_argument("-a", "--all", action="store_true", help="匹配多个目录时，全部检查；否则交互选择")
    parser.add_argument("--log", action="store_true", help="启用日志文件输出（脚本目录）")
    return parser.parse_args()

# ====================== 主执行流程 ======================
if __name__ == "__main__":
    # 1. 解析参数
    args = parse_args()
    target_date = args.date if args.date else DATE
    # 日期格式校验（必须4位MMDD）
    if args.date and not re.fullmatch(r"\d{4}", args.date or ""):
        print("日期格式应为 4 位 MMDD，例如 0319")
        exit(1)

    def run_check_for(root, name, enable_log=False):
        """
        执行单个日期文件夹的检查+清理
        :param root: 数据根目录
        :param name: 日期文件夹名
        :param enable_log: 是否开启日志
        """
        base_dir = os.path.join(root, name)
        # 获取所有任务子目录
        task_dirs = list_task_dirs(base_dir)
        if not task_dirs:
            print(f"\n[WARN] {base_dir} 下未发现任务目录，跳过。")
            return
        
        # 遍历检查每个任务目录
        for task_dir in task_dirs:
            global DATA_PATH, VIDEO_PATH
            DATA_PATH = task_dir
            VIDEO_PATH = os.path.join(DATA_PATH, 'video')
            rel_task = os.path.relpath(task_dir, base_dir)
            
            try:
                # 打印分割线+目录信息
                print("\n====================================================")
                print(f"数据根目录: {root}")
                print(f"目标日期: {target_date}")
                print(f"日期目录: {name}")
                print(f"任务子目录: {rel_task if rel_task != '.' else '(根目录)'}")
                # 初始化日志
                if enable_log:
                    setup_logging(name)
                print("====================================================\n")

                # 执行数据校验
                check_data()
                print("\n数据检查完成。\n")
                # 执行自动清理
                cleanup_auto(task_dir)
                
            except KeyboardInterrupt:
                print("\n[INFO] 用户取消操作。")
            finally:
                # 关闭日志
                close_logging()

    # 2. 查找所有匹配日期的文件夹
    matches = find_folders_with_date(date_str=target_date)
    if not matches:
        print(f"未找到包含日期 {target_date} 的数据文件夹")
        print("自动检测的根目录：", detect_user_home_candidates())
        exit(1)

    # 3. 执行检查（单文件夹/批量/交互选择）
    if len(matches) == 1:
        root, name, _ = matches[0]
        run_check_for(root, name, args.log)
    else:
        if args.all:
            # 批量检查所有
            for root, name, _ in matches:
                run_check_for(root, name, args.log)
        else:
            # 交互式选择
            print(f"发现 {len(matches)} 个匹配目录：")
            for i, (root, name, mtime) in enumerate(matches, 1):
                ts = datetime.datetime.fromtimestamp(mtime).strftime('%Y-%m-%d %H:%M:%S') if mtime else '未知'
                print(f"  {i}. {os.path.join(root, name)}  （修改时间：{ts}）")
            while True:
                sel = input("请输入编号，a=全部 / q=退出：").strip().lower()
                if sel in ('q', 'quit', 'exit'):
                    print("已退出。")
                    exit(0)
                if sel in ('a', 'all'):
                    for root, name, _ in matches:
                        run_check_for(root, name, args.log)
                    break
                if sel.isdigit():
                    idx = int(sel)
                    if 1 <= idx <= len(matches):
                        root, name, _ = matches[idx-1]
                        run_check_for(root, name, args.log)
                        break
                print("输入无效，请重试。")