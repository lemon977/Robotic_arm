import subprocess
import threading
import re
import os
from typing import List, Dict, Optional
import time
import sys
# 获取当前文件的目录（pyUtil/）
current_dir = os.path.dirname(os.path.abspath(__file__))
# 获取上级目录（即包含pyUtil的目录）
parent_dir = os.path.dirname(current_dir)
# 将上级目录加入Python的模块搜索路径
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)
from pyUtil.host_data_manager import add_collect_data
'''
脚本监听与输出捕获模块
capture_command_output - 监听ROS脚本输出，解析机械臂状态变化
print_collect_status - 解析采集脚本输出，打印关键状态
capture_collect_output - 采集脚本输出捕获函数
parse_position: 机械臂浮动判断
is_small_floating - 机械臂微小浮动判断函数
extract_position_array - rostopic echo 输出解析函数
check_pid_alive - 检测PID是否存活函数
monitor_command_background - ROS脚本监控线程
start_target_command - ROS脚本启动方法
monitor_collect_background - 采集脚本监控线程
'''

# py_last_printed_state = None
# status_map = {
#         "采集任务完成": "🟢 采集任务完成",
#         "用户手动中断": "🟡 用户手动中断采集",
#         "脚本执行异常": "🔴 脚本执行异常",
#         "未采集到数据": "🟠 未采集到任何数据",
#         "用户放弃保存": "🟠 用户放弃保存采集数据"
# }

# 监听脚本内容
def capture_command_output(commands_status, index: int, status_lock: threading.Lock, arm_float_eps: float):
    """
    该函数在独立线程中运行，持续监听指定ROS脚本的终端输出，并根据输出内容解析机械臂状态变化
        :param commands_status: 脚本命令状态列表
        :param index: 当前脚本下标
        :param status_lock: 状态锁
        :param arm_float_eps: 机械臂浮动阈值
    处理流程：
        1. 持续读取脚本输出行
        2. 如果当前脚本处于监听状态且不是主从臂脚本，则打印输出内容
        3. 如果是主臂或从臂脚本，尝试从输出行中提取 position 数组，并解析为位置列表
        4. 根据当前位置与上一次位置的关系，判断机械臂状态为 "zero"、"floating" 或 "moving"，并更新状态
    """
    process = commands_status[index]["process"]
    try:
        for line in iter(process.stdout.readline, ''):
            if not line:
                break

            line = line.strip()

            # 如果在监听，那么输出内容  3,4静默启动不监听
            if commands_status[index].get("listening", False) and index not in [3, 4]:
                print(f"[脚本输出 - {commands_status[index]['name']}] {line}")

            if index in [5, 6]:  # 左臂或右臂
                pos_str = extract_position_array(line)
                if not pos_str:
                    continue
                curr_pos = parse_position(pos_str)
                if not curr_pos:
                    continue
            
                with status_lock:
                    # 获取之前的状态
                    # prev_state = commands_status[index].get("state")
                    last_pos = commands_status[index].get("last_position")
                    if all(abs(v) < 1e-6 for v in curr_pos):
                        commands_status[index]["state"] = "zero"
                    elif last_pos and is_small_floating(last_pos, curr_pos, arm_float_eps):
                        commands_status[index]["state"] = "floating"
                    else:
                        commands_status[index]["state"] = "moving"

                    # 更新last_position
                    commands_status[index]["last_position"] = curr_pos
                    # 只有状态变化才打印语义
                    # if commands_status[index]["state"] != prev_state:
                    #     print(f"[{commands_status[index]['name']}] 状态变化: {prev_state} -> {commands_status[index]['state']}")
            
            # # ===== 检测 CAN 初始化完成 =====
            # if index == 0 and "所有 CAN 接口已成功重命名并激活" in line:
            #     print("✅ 检测到 CAN 初始化完成（index=0 语义完成）")

    except Exception as e:
        print(f"⚠️ 读取脚本 {commands_status[index]['name']} stdout 异常: {e}")


# ========== 采集脚本输出捕获函数 ==========
# def print_collect_status(line: str, idx: int):
#     """
#     根据采集脚本终端输出内容，打印关键状态（只打印一次）
#     :param line: 采集脚本输出行
#     :param idx: 采集脚本下标
#     """
#     global py_last_printed_state
#     print("last_state_py_collect", py_last_printed_state)
#     line = line.strip()
#     if not line:
#         return

#     for keyword, msg in status_map.items():
#         if keyword in line:
#             # 防止重复打印
#             if py_last_printed_state != keyword:
#                 print(f"[解释终端状态 - 数采脚本-{idx} ] {msg}")
#                 py_last_printed_state = keyword
#             return
        

# ========= 采集脚本输出捕获函数 ==========
def capture_collect_output(collect_status, idx: int, collect_history, machine_info):
    """
    该函数在独立线程中运行，持续监听指定采集脚本的终端输出，并根据输出内容解析采集状态变化
        :param collect_status: 采集脚本状态列表
        :param idx: 当前采集脚本下标
        :param collect_history: 采集历史记录
        :param machine_info: 机器信息
    处理流程：
        1. 持续读取采集脚本输出行
        2. 根据输出内容判断采集状态变化，如 "采集任务完成"、"用户手动中断"、"脚本执行异常"、"用户放弃保存" 等，并更新状态
        3. 如果采集完成，且历史记录中没有当前 Episode 索引，则添加到历史记录并调用 add_collect_data 更新数据库
    """
    process = collect_status[idx].get("process")
    # print(f"🟣 采集脚本-{idx} 输出捕获启动，PID={process.pid}")
    try:
        for line in iter(process.stdout.readline, ''):
            if not line:
                break
            
            wrFlag = False

            line = line.strip()
            # 仅仅捕获错误
            if 'copy到nas时出现错误' in line:
                print(line)
            # print(f"[采集脚本输出] {line}")

            # ===== 成功完成 =====
            if "采集任务完成" in line:
                collect_status[idx].update({"state": "success", "color": "green", "msg": line})
                ep_idx = collect_status[idx]['ep_idx']
                if ep_idx not in collect_history['history_list']:
                    collect_history['history_list'].append(ep_idx)
                    collect_history['collect_count'] += 1
                    add_collect_data(machine_info, ep_idx)
                wrFlag = True
                    
            
            # ===== 异常情况 =====
            elif any(word in line for word in ["未采集到数据", "用户手动中断", "脚本执行异常"]):
                collect_status[idx].update({"state": "error", "color": "red", "msg": line})
                wrFlag = True

            elif "用户放弃保存" in line:
                collect_status[idx].update({"state": "error", "color": "orange", "msg": line})
                wrFlag = True

            # if "实际采集帧数" in line:
            #     collect_status[idx]["all_step"] = int(re.search(r'实际采集帧数：(\d+)', line).group(1))
            #     print(f"📊 设置 all_step = {collect_status[idx]['all_step']}")

            # if "视频编码进度" in line:
            #     collect_status[idx]["now_step"] = int(re.search(r'视频编码进度：第 (\d+) 帧', line).group(1))

            # print(f"📊 all_step={all_step}, now_step={now_step}")

            # if "自动生成Episode索引" in line:
            #     collect_status[idx]["ep_idx"] = re.search(r'自动生成Episode索引：(\d+)', line).group(1)
            #     print(f"🎬 采集脚本生成的 Episode 索引: {collect_status[idx]['ep_idx']}")

            if wrFlag:
                with open(f"/home/agilex/kai05_collect/log/collect_data_{machine_info['date']}.log", 'a', encoding='utf-8') as f:
                        f.write(f'{collect_status[idx]["ep_idx"]} {line}\n')

    except Exception as e:
        print(f"⚠️ 读取采集脚本 stdout 失败: {e}")


# ========= 机械臂浮动判断函数 ==========
def parse_position(pos_str: str):
    """
    该函数将 position 数组字符串解析为位置列表
        :param pos_str: 形如 "0.12, -0.34, ..." 的字符串
        :return: [0.12, -0.34, ...] 或 None 如果解析失败
    """
    try:
        return [float(x.strip()) for x in pos_str.split(",")]
    except Exception:
        return None


# ======== 机械臂浮动判断函数，微小浮动 ========== 
def is_small_floating(last, curr, eps):
    """
    该函数判断当前位置与上一次位置的关系，判断机械臂是否处于微小浮动状态
        :param last: 上一次位置列表
        :param curr: 当前位列表表
        :param eps: 浮动阈值，单位与位置数值相同，表示当前位置与上一次位置的最大差值小于该阈值则认为是微小浮动
        :return: True 如果是微小浮动，False 否则
    """
    if not last or not curr or len(last) != len(curr):
        return False

    max_diff = max(abs(c - l) for c, l in zip(curr, last))
    return max_diff < eps


# ========= rostopic echo 输出解析函数 ==========
def extract_position_array(line: str) -> Optional[str]:
    """
    该函数尝试从 rostopic echo 输出的行中提取 position 数组字符串
        :param line: rostopic echo 输出的一行文本
        :return: 形如 "0.12, -0.34, ..." 的字符串，如果未找到 position 数组则返回 None
    """
    line = line.strip()
    if "position" not in line:
        return None

    # 匹配 position: [ ... ]
    match = re.search(r"position:\s*\[([^\]]+)\]", line)
    if not match:
        return None

    return match.group(1)   # 返回 "0.12, -0.34, ..."


# ========= 检测 PID 是否存活函数 ==========
def check_pid_alive(pid: int) -> bool:
    """
    该函数检测指定 PID 是否对应一个存活的进程
        :param pid: 进程 PID
        :return: True 如果 PID 存活，False 否则
    """
    if pid <= 0:
        return False
    try:
        ps = subprocess.run(
            ["ps", "-p", str(pid), "-o", "stat="],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True
        )
        if ps.returncode != 0:
            return False
        stat = ps.stdout.strip().upper()
        # Z = zombie
        if not stat or stat.startswith("Z"):
            return False
        return True
    except Exception as e:
        print(f"⚠️ PID {pid} 检测失败: {e}")
        return False


# ========== ROS脚本监控线程 ==========
def monitor_command_background(commands_status: List[Dict], status_lock: threading.Lock, arm_float_eps: float, command_monitor_freq: int):
    """
    该函数在独立线程中运行，持续监控 ROS 脚本的启动和存活状态，并根据脚本输出解析机械臂状态变化
        :param commands_status: 脚本命令状态列表
        :param status_lock: 状态锁
        :param arm_float_eps: 机械臂浮动阈值
        :param command_monitor_freq: 监控频率（每秒次数）
    处理流程：
        1. 循环监控 ROS 脚本状态
        2. 如果尚未全部启动且当前运行脚本数小于总脚本数，尝试启动下一个脚本。对于第1-4个脚本，在启动前增加延迟，并在启动第3个脚本时执行夹爪行程设置，第4个脚本启动前清除从臂使能相关进程
    """
    # 总脚本数量
    total = len(commands_status)
    # 所有脚本是否启动标识
    all_started = False

    # 循环监控
    while True:
        # 尝试启动下一个ROS脚本 如果尚未全部启动且当前运行脚本数小于总脚本数
        index = sum(1 for item in commands_status if item["process"] is not None)
        if not all_started and index < total:
            # 如果上一个脚本的alive状态为True，则启动下一个脚本
            if commands_status[index - 1]["alive"] or index == 0:
                # 若脚本为第1,2,3个脚本，那么提高3s延迟
                if index in [1, 2, 3, 4]:
                    time.sleep(2)
                    # # 新版第一个can口激活较慢，叠加延迟
                    # if index == 1:
                    #     print('can口激活')
                    #     time.sleep(5)
                    if index == 3:
                        # 判断夹爪行程是否为100，不为100则会通过此代码进行修改
                        subprocess.run('python3 /home/agilex/kai05_collect/pyUtil/gripper_set100.py', shell=True)
                    # 如果是第4个脚本，优先查询是否有从臂使能进程，有的话直接清除
                    if index in [4]:
                        # print(commands_status[4]["cmd"])
                        subprocess.run(["pkill", "-f", commands_status[4]["cmd"]])
                        subprocess.run(["pkill", "-f", "python3 slave_ros8.py"])
                        time.sleep(2)

                process = start_target_command(
                    commands_status,
                    index
                )

                # if process and hasattr(process, 'pid'):
                if process:
                    print(f"✅ [{commands_status[index]['name']}] 脚本启动成功，PID={process.pid}")
                    with status_lock:
                        commands_status[index].update({"process": process, "pid": process.pid, "alive": True})
                        # 如果开启的脚本不是主从臂脚本，开启监听
                        if index not in [3, 4]:
                            # 开启监听线程
                            threading.Thread(
                                target=capture_command_output,
                                args=(commands_status, index, status_lock, arm_float_eps),
                                daemon=True
                            ).start()
                    if index + 1 == total:
                        all_started = True

        with status_lock:
            for i in range(index):
                # 获取当前PID存活状态
                is_alive = check_pid_alive(commands_status[i]["pid"])
                # 读取上一次存活状态
                old_alive = commands_status[i]["alive"]
                # 如果两次状态不同，说明状态变化并将上一次状态更新
                if is_alive != old_alive and i != 0:
                #     print(f"🔄 ROS脚本状态变化: index={i} {commands_status[i]['name']} PID={commands_status[i]['pid']} 状态 {old_alive} → {is_alive}")
                    commands_status[i]["alive"] = is_alive

        time.sleep(1 / command_monitor_freq)

# ========== ROS脚本启动方法 ==========
def start_target_command(commands_status: List[Dict], index: int):
    """
    该函数尝试启动指定下标的ROS脚本，并返回 subprocess.Popen 对象
        :param commands_status: 脚本命令状态列表
        :param index: 要启动的脚本下标
        :return: subprocess.Popen 对象，如果启动失败则返回 None
    处理流程：
        1. 从 commands_status 中获取要启动的脚本命令
        2. 对于主臂和从臂脚本，静默启动（不输出终端内容），其他脚本正常启动
        3. 如果启动过程中出现异常，捕获并打印错误信息
    """
    try:
        command = commands_status[index]['cmd']
        process = None
        # 对于主臂和从臂脚本，静默启动
        if index in [3, 4]:
            process = subprocess.Popen(command, shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, text=True)
        else:
            process = subprocess.Popen(command, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)    
    except Exception as e:
        print(f"❌ [{commands_status[index]['name']}] 脚本启动异常: {e}")

    return process


# ========== 采集脚本监控线程 ==========
def monitor_collect_background(collect_status: List[Dict], status_lock: threading.Lock, idx, collect_monitor_freq: int):
    """
    该函数在独立线程中运行，持续监控指定采集脚本的存活状态，并根据脚本输出解析采集状态变化
        :param collect_status: 采集脚本状态列表
        :param status_lock: 状态锁
        :param idx: 当前采集脚本下标
        :param collect_monitor_freq: 监控频率（每秒次数）
    处理流程：
        1. 循环监控采集脚本状态
        2. 如果采集脚本进程对象存在，检查其 PID 是否存活
        3. 如果当前存活状态与之前状态不同，说明脚本状态发生变化
            - 如果现在的状态为 False，说明脚本结束了
                - 如果之前状态是 aborted / running，且用户点击的结束采集按钮，则说明脚本正常结束，更新状态为 success
                - 如果之前状态是 aborted / running，且用户点击的取消保存按钮，则说明脚本异常结束，更新状态为 error
    """
    while True:
        with status_lock:
            # 如果采集脚本进程对象存在
            if collect_status[idx]['process']:
                # 检查采集脚本是否存活
                is_alive = check_pid_alive(collect_status[idx]['pid'])
                # 如果当前状态与之前状态不同
                if collect_status[idx]["alive"] != is_alive:
                    # print(f"📝 采集脚本 (PID:{collect_status[idx]['pid']}) 状态变化: {collect_status[idx]['alive']} → {is_alive}")
                    # 如果现在的状态为False
                    if not is_alive:
                        # 如果之前状态是 aborted / running, 用户点击的结束采集按钮 -> 脚本正常结束 → success
                        if collect_status[idx]["state"] in ("aborted", "running") and collect_status[idx]["collect_state"] == "endCollecting":
                            print("采集任务完成 MONITOR")
                            collect_status[idx].update({"state": "success", "color": "green", "msg": "采集任务完成"})
                        # 如果之前状态是 aborted / running, 用户点击的取消保存按钮 -> 脚本异常结束 → error
                        elif collect_status[idx]["state"] in ("aborted", "running") and collect_status[idx]["collect_state"] == "no_save":
                            print("用户放弃保存 MONITOR")
                            collect_status[idx].update({"state": "error", "color": "orange", "msg": "用户放弃保存"})
                        collect_status[idx].update({"collect_state": "none"})
                    collect_status[idx].update({"alive": is_alive})
            else:
                collect_status[idx].update({"collect_state": "none", "pid": -1, "alive": False})
            
        time.sleep(1 / collect_monitor_freq)


