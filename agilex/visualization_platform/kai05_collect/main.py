"""
//页面渲染部分
update_ui - 更新UI数据
open_config_panel - 打开配置面板
save_config - 保存配置文件函数
replay_btn_click - 打开重播页面
on_delete_episode - 打开删除确认弹窗

// 重播切换部分
restart_collect_piper - 重启 主臂会话

// 重播部分
replay_click - 重播按钮
start_replay - 开始重播
stop_replay - 停止重播

// 采集部分
start_collect - 开始采集
end_collect - 结束采集
stop_collect - 放弃保存
"""
import subprocess
import time
import threading
import gradio as gr
from typing import List, Dict
import os
import json
from pathlib import Path

if not os.path.exists('/home/agilex/kai05_collect/log/'):
    os.mkdir('/home/agilex/kai05_collect/log/')

from pyUtil.capture_ros import (
    monitor_command_background,
    monitor_collect_background,
    capture_collect_output
)

from pyUtil.file_manager import (
    read_html_file,
    save_config_file,
    read_config_file,
    open_collect_dir,
    scan_episode_indices,
    command_indicator_html,
    collect_indicator_html,
    get_children,
    delete_data,
    get_dir_idx,
    commit_collect,
    update_indicator_js,
    is_continuous_array,
    check_list_coverage
)

from pyUtil.host_data_manager import ( 
    replay_finished,
    get_replay_data
)

from pyUtil.read_hdf5 import load_hdf5_split_left_right

status_lock = threading.Lock()     # 状态锁

# ========== 监控频率全局配置（可直接修改！） ==========
COMMAND_MONITOR_FREQ = 2    # 6个ROS脚本监听频率：每秒5次（间隔0.2秒）
COLLECT_MONITOR_FREQ = 10    # 采集脚本监听频率：每秒10次（间隔0.1秒）
ARM_FLOAT_EPS = 0.002   # 浮动阈值（单位同 position）
DEBUG_FLAG = False     # 开发模式
LISTEN_INDICES = [0]  # 需要监听的脚本下标
REPLAY_FLAG = False  # 重播标志
REPLAY_FREQUENCY = 5  # 重播频率，每5条重播一次
REPLAY_START = False # 强制重播锁

# ========== 配置文件路径 ==========
# 此脚本所在目录
WORKING_DIR = os.path.dirname(os.path.abspath(__file__))
# 配置文件脚本路径
CONFIG_FILE_PATH = os.path.join(WORKING_DIR, "config", "config.json")
# 总脚本信息
ALL_TASKS = os.path.join(WORKING_DIR, "config", "all_tasks.json")
# html/js路径
HTML_JS_PATH = os.path.join(WORKING_DIR, "static", "indicator_update.js")
# html/css路径
HTML_CSS_PATH = os.path.join(WORKING_DIR, "static", "styles.css")
# 采集脚本路径
COLLECT_SCRIPT_PATH = os.path.join(WORKING_DIR, "pyUtil", "record2.py")
# 重播脚本路径
REPLAY_SCRIPT_PATH = os.path.join(WORKING_DIR, "pyUtil", "replay.py")
# 重播进程对象
REPLAY_PROCESS = None

# ========== 命令数组配置 ==========
# SCRIPT_COMMANDS = [
#     f"echo agx | sudo -S bash -lc '. /home/agilex/miniconda3/etc/profile.d/conda.sh && conda activate aloha && {WORKING_DIR}/config/can_port.sh'",
#     ". /home/agilex/miniconda3/etc/profile.d/conda.sh && conda activate aloha && bash -lc 'roscore'",
#     ". /home/agilex/miniconda3/etc/profile.d/conda.sh && conda activate aloha && bash -lc 'roslaunch realsense2_camera camera_web.launch'",
#     f"python3 {WORKING_DIR}/pyUtil/pipper_ros.py",
#     f"python3 {WORKING_DIR}/pyUtil/slave_ros8.py",
#     ". /home/agilex/miniconda3/etc/profile.d/conda.sh && conda activate aloha && bash -lc 'rostopic echo /puppet/joint_left'",
#     ". /home/agilex/miniconda3/etc/profile.d/conda.sh && conda activate aloha && bash -lc 'rostopic echo /puppet/joint_right'"
# ]

SCRIPT_COMMANDS = [
    f"echo agx | sudo -S bash -lc '{WORKING_DIR}/config/can_config.sh'",
    "bash -lc 'roscore'",
    "bash -lc 'roslaunch realsense2_camera camera_web.launch'",
    f"python3 {WORKING_DIR}/pyUtil/pipper_ros.py",
    f"python3 {WORKING_DIR}/pyUtil/slave_ros8.py",
    "bash -lc 'rostopic echo /puppet/joint_left'",
    "bash -lc 'rostopic echo /puppet/joint_right'"
]

# 当前机器信息
MACHINE_INFO = {
    "machine_id": "",
    "machine_name": "",
    "dataset_dir": "",
    "pre_dataset_dir": "",
    "task": "",
    "max_timesteps": 0,
    "plan_count": 0,
    "collect_count": 0,
    "scheduling": "",
    "version": "",
    "date": "",
    "machine_tasks": [],
    "format_tasks": [],
    "config_file_path": CONFIG_FILE_PATH,
    "all_tasks": ALL_TASKS
}

# 脚本显示名称映射
COMMANDS_DISPLAY_NAMES = ["使能", "ROS", "相机", "主臂", "从臂", "左监听", "右监听"]

# ===== 脚本的启动状态 ===== state -> none 未启动 / moveing 运动中 / zero 全0 / float 微小浮动 / running 运行中
COMMANDS_STATUS = [
    {
        "index": i,
        "name": COMMANDS_DISPLAY_NAMES[i],
        "cmd": SCRIPT_COMMANDS[i],
        "pid": -1,
        "process": None,
        "alive": "none",
        "state": "zero",
        "listen": True if i in LISTEN_INDICES else False,
        "last_position": None
    }
    for i in range(len(SCRIPT_COMMANDS))
]

COLLECTS_STATUS = [
    {
        "index": i,
        "pid": -1,
        "alive": False,
        "process": None,
        "collect_state": "none", # collecting / endCollecting / no_save
        "state": "none",  # none / running / success / aborted / error / no_data （未启动/运行中/保存完成/保存中/保存失败/未采集到数据）
        "color": "gray",
        "msg": "未启动",
        "all_step": -1,
        "now_step": -1,
        "ep_idx": -1,
    }
    for i in range(2)
]

COLLECT_HISTORY = {
    "collect_count": 0,
    "history_list": []
}

REPLAY_HISTORY = {
    "replay_count": 0,
    "history_list": []
}


# ========= 更新UI数据 ==========
def update_ui():
    """
    该函数用于更新前端UI数据，主要包括命令状态、采集状态和采集历史记录。通过加锁确保线程安全。
    处理流程：
        1. 获取当前命令状态和采集状态，去除Process对象以避免序列化失败
        2. 判断当前采集数量是否与历史记录中的数量相等，如果不相等则更新配置文件并同步历史记录
        3. 将处理后的命令状态、采集状态和采集历史记录打包成一个字典，并转换为JSON字符串返回给前端
    """
    global COLLECT_HISTORY
    with status_lock:
        # 将Porocess对象移除，避免序列化失败
        collect_results = [
            { k: v for k, v in item.items() if k != "process" }
            for item in COLLECTS_STATUS
        ]

        command_results = [
            { k: v for k, v in item.items() if k != "process" }
            for item in COMMANDS_STATUS
        ]

        # 判断采集数量是否和采集历史的采集数量相等，不相等进行更新
        if not COLLECT_HISTORY["collect_count"] == MACHINE_INFO['collect_count']:
            save_config_file(MACHINE_INFO, CONFIG_FILE_PATH, True)
            COLLECT_HISTORY.update({'collect_count': MACHINE_INFO['collect_count']})

        data = {
            "command": command_results,
            "collect": collect_results,
            "collect_history": COLLECT_HISTORY
        }
    return json.dumps(data)


# ========= 打开配置面板 ==========
def open_config_panel():
    """
    该函数用于打开配置面板，并将当前机器信息中的任务、调度方式、最大时间步数等参数传递给前端进行显示和选择。
    """
    # read_config_file(MACHINE_INFO)
    update_info = [
        gr.update(),
        gr.update(value=MACHINE_INFO["task"]),   # Dropdown 选中
        gr.update(value=MACHINE_INFO["scheduling"]), # Scheduling 选中
        gr.update(value=MACHINE_INFO["max_timesteps"]), # 只读显示
        gr.update(visible=True)
    ]

    if MACHINE_INFO.get('collect_count', 0) and '250' not in MACHINE_INFO.get("dataset_dir", None):
        update_info.append(gr.update(value='更换新目录开始采集'))
        update_info.append(gr.update(value='继续原目录进行采集'))
    else:
        update_info.append(gr.update())
        update_info.append(gr.update())
            
    return update_info


def delete_data_controller(idx):
    """
    该函数用于删除指定 episode 的数据记录，调用 delete_data 函数并传入当前机器信息、要删除的 episode 索引和采集历史记录。返回删除结果给前端。
        :param idx: 要删除的 episode 索引
    """
    return delete_data(MACHINE_INFO, idx, COLLECT_HISTORY)


# ========= 保存配置文件函数 ==========
def save_config(format_date: str, task: str, scheduling: str, max_timesteps: int):
    """
    该函数用于保存配置文件，更新当前机器信息中的日期、任务、调度方式和最大时间步数等参数，并调用 save_config_file 函数将更新后的机器信息保存到指定路径的配置文件中。同时，如果采集数量为0，则清空采集历史记录。最后返回保存结果和关闭配置面板的指令给前端。
        :param format_date: 格式化后的日期字符串
        :param task: 任务名称
        :param scheduling: 调度方式
        :param max_timesteps: 最大时间步数
    """
    global MACHINE_INFO, COLLECT_HISTORY
    MACHINE_INFO.update({
        "date": format_date,
        "task": task,
        "scheduling": scheduling,
        "max_timesteps": max_timesteps
    })
    msg = save_config_file(MACHINE_INFO, CONFIG_FILE_PATH, collect_history=COLLECT_HISTORY)
    # 修改到一个新的文件目录后清空历史记录
    if MACHINE_INFO.get("collect_count") == 0:
        COLLECT_HISTORY["history_list"].clear()
    return [msg, gr.update(visible=False)]


# ========== 打开重播页面 ==========
def replay_btn_click(replay_popup):
    """
    该函数用于处理重播按钮的点击事件，首先检查当前是否有正在进行的采集任务，如果没有，则扫描指定目录下的 episode 数据，找到可用的 episode 索引并打开重播选择面板供用户选择要重播的 episode。如果当前有采集任务在进行中，则返回提示信息给前端。
        :param replay_popup: 重播选择面板对象
    """
    if COLLECTS_STATUS[0]['collect_state'] == 'none' and COLLECTS_STATUS[1]['collect_state'] == 'none':
        indices = scan_episode_indices(os.path.join(MACHINE_INFO.get('dataset_dir'), 'aloha_mobile_dummy'))

        if not indices:
            return [gr.update(), gr.update(visible=replay_popup.visible), "⚠️ 未找到可用的 episode 数据，请先进行数据采集！"]

        # 转成字符串，避免 Gradio 类型坑
        choices = [str(i) for i in indices]
        default_value = str(indices[-1])   # ⭐ 默认最后一个

        replay_popup.visible = not replay_popup.visible
        return [gr.update(choices=choices, value=default_value), gr.update(visible=replay_popup.visible), f"已打开重播选择面板，请选择要重播的 episode"]
    else:
        return [gr.update(), gr.update(visible=replay_popup.visible), "⚠️ 当前有采集任务在进行中，请结束采集/保存后再进行重播！"]
    

# ========= 删除数据 ==========
def on_delete_episode(idx: int):
    """
    该函数用于处理删除 episode 的事件，首先检查是否有正在进行的重播任务，如果没有，则调用 delete_data_controller 函数删除指定 episode 的数据记录，并返回结果给前端。如果当前有重播任务在进行中，则返回提示信息给前端。
        :param idx: 要删除的 episode 索引
    """
    if idx is None:
        return ["⚠️ 未选择 episode", gr.update()]
    else:
        if REPLAY_FLAG:
            return ["⚠️ 当前有重播任务在进行中，无法删除数据！", gr.update(visible=False)]
        else:
            return ["", gr.update(visible=True)]


# ========= 重启 主臂会话 ==========
def restart_collect_piper(replay_popup):
    """
    该函数用于重启主臂会话，首先检查当前是否有正在进行的重播任务，如果没有，则检查主臂会话进程是否存活，如果未存活则启动主臂会话脚本并更新命令状态。最后切换重播选择面板的显示状态并返回结果给前端。如果当前有重播任务在进行中，则返回提示信息给前端。
        :param replay_popup: 重播选择面板对象
    """
    global COMMANDS_STATUS
    if REPLAY_FLAG:
        return [gr.update(), "当前仍在重播，请重播结束后再关闭此界面"]     

    # 若主臂进程未存活，重启 主臂会话 脚本
    if COMMANDS_STATUS[3]['process'] is None:
        cmd = SCRIPT_COMMANDS[3]
        new_proc = subprocess.Popen(
            cmd,
            shell=True,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            text=True,
            bufsize=1
        )
        if new_proc:
            with status_lock:
                COMMANDS_STATUS[3].update({
                    "name": "主臂",
                    "cmd": cmd,
                    "pid": new_proc.pid,
                    "process": new_proc,
                    "alive": True,
                    "state": "zero"
                })

            print(f"✅ 主臂重启完成 PID={new_proc.pid}")
    replay_popup.visible = not replay_popup.visible
    return [gr.update(visible=replay_popup.visible), f"✅ 已退出重播"]


# =========== 重播按钮 ==========
def replay_click(idx: int):
    """
    该函数用于处理重播按钮的点击事件，首先检查当前是否有正在进行的重播任务，如果没有，则启动重播线程并将重播标志位置为True，最后返回重播开始的提示信息给前端。如果当前有重播任务在进行中，则返回提示信息给前端。
        :param idx: 要重播的 episode 索引
    """
    global REPLAY_FLAG
    # 如果没在重播，那么可以进行重播  1.重播状态为false 2.重播进程为None
    if not REPLAY_FLAG and REPLAY_PROCESS is None:
        # 🔥 启动 重播 线程
        # 将重播标志位置为True
        REPLAY_FLAG = True
        threading.Thread(
            target=start_replay,
            args=(idx, ),
            daemon=True
        ).start()
        
        return f"⭐ 已经开始重播 episode_{idx} 啦"
    else:
        return "⚠️ 当前已有重播任务在进行中，请稍后再试！"


# ========== 开始重播 ==========
def start_replay(idx: int):
    """
    该函数用于开始重播指定 episode 的数据
        :param idx: 要重播的 episode 索引
    处理流程：
        1. 检查当前是否有正在进行的采集任务，如果有，则记录当前主臂位置并杀死主臂会话进程
        2. 读取要重播的 episode 的第一帧位置数据，并将机械臂移动到该位置
        3. 启动重播脚本进行重播，并监控重播进程的状态
        4. 当重播进程结束或者重播标志被置为False时，将机械臂移动回重播前的位置，并调用 replay_finished 函数将重播结果发送给服务器
    """
    global COMMANDS_STATUS, REPLAY_FLAG, REPLAY_PROCESS

    print(f"🔄 请求重播 episode !!!!!!!!!!!!!!!!!!!{idx}")
    try:
        # 1.启动重播模式
        # 若主臂进程存活，将主臂会话切断
        if "pipper_ros" in COMMANDS_STATUS[3]['cmd'] or COMMANDS_STATUS[3]['process'] is not None:
            # 1.1 将当前主臂position记录到日志
            subprocess.run(
                ['python3', f'{WORKING_DIR}/pyUtil/master_arm_position.py']
            )
            time.sleep(1.5)

            # 1.2 杀死主臂进程
            ppids = get_children(COMMANDS_STATUS[3]['pid'])
            print(f"🔍 Piper 子进程 PID 列表: {ppids}")
            for ppid in ppids:
                subprocess.run(["kill", "-SIGINT", str(ppid)])
            COMMANDS_STATUS[3]['process'].terminate()
            COMMANDS_STATUS[3]['process'].wait()
            if not COMMANDS_STATUS[3]['process'].poll():
                print("terminate 关闭采集失败")
            else:
                print("terminate 关闭采集成功")
                COMMANDS_STATUS[3].update({"process": None, "cmd": "", "pid": -1, "alive": False})

        time.sleep(1)
        
        # 2.启动 重播数据脚本
        # 2.1读取所需要重播的hdf5文件第一帧position位置 qpos
        # load_hdf5_data(MACHINE_INFO['dataset_dir'], idx)
        load_hdf5_split_left_right(MACHINE_INFO['dataset_dir'], 'aloha_mobile_dummy', idx)
        # 2.2将机械臂移动到需要重播hdf5的第一帧位置
        subprocess.run(
            ['python3', f'{WORKING_DIR}/pyUtil/arm_hdf5.py']
        )
        # 2.3运行重播代码
        cmd = (
            f"python3 {REPLAY_SCRIPT_PATH} {MACHINE_INFO['dataset_dir']}/aloha_mobile_dummy/episode_{idx}.hdf5"
        )
        
        if REPLAY_FLAG:
            REPLAY_PROCESS = subprocess.Popen(
                cmd,
                shell=True,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,  # ⚠️ 合并 stderr
                text=True,
                bufsize=1
            )

        print(f"重播命令: {cmd}", f"PID={REPLAY_PROCESS.pid}")
        
        while True:
            # 重播进程结束或者重播状态为false
            if REPLAY_PROCESS.poll() is not None or not REPLAY_FLAG:
                # 重播结束后从臂回到重播前记录的主臂位置
                subprocess.run(
                    ['python3', f'{WORKING_DIR}/pyUtil/arm_home.py']
                )
                REPLAY_FLAG = False
                REPLAY_PROCESS = None
                # 发送重播数据给服务器
                replay_finished(MACHINE_INFO, cmd)
                break
    except Exception as e:
        print(f"重播过程中发生错误: {e}")
        stop_replay()  # 确保重播状态和进程被正确重置
        REPLAY_FLAG = False
        REPLAY_PROCESS = None


# ========= 停止重播 ==========
def stop_replay():
    """
    该函数用于停止当前正在进行的重播任务，首先检查当前是否有正在进行的重播任务，如果有，则通过获取重播进程的子进程列表并发送 SIGINT 信号来终止重播进程。最后将重播标志位置为False，并返回重播已停止的提示信息给前端。如果当前没有重播任务在进行中，则返回提示信息给前端。
    """
    global REPLAY_FLAG
    # 如果在重播 1.重播状态为True 2.重播进程非None
    if REPLAY_FLAG and REPLAY_PROCESS is not None:
        print("⏹️ 停止重播请求收到，正在停止重播进程...")
        ppids = get_children(REPLAY_PROCESS.pid)
        print(f"🔍 重播子进程 PID 列表: {ppids}")
        for ppid in ppids:
            subprocess.run(["kill", "-SIGINT", str(ppid)])
        REPLAY_PROCESS.terminate()
        REPLAY_PROCESS.wait()
        if not REPLAY_PROCESS.poll():
            print("terminate 关闭重播失败")
        else:
            print("terminate 关闭重播成功")
        REPLAY_FLAG = False
        return "✅ 重播已停止"
    else:
        return "⚠️ 当前无重播任务在进行中"


# ========== 开始采集 ==========
def start_collect(replay_popup):
    """
    该函数用于开始数据采集，首先检查当前是否有正在进行的重播任务，如果没有，则检查采集脚本是否存在、采集目录是否配置、主臂会话是否启动等条件是否满足。如果满足条件，则找到一个空闲的采集线程并启动采集脚本，同时更新采集状态并启动一个线程来监听采集脚本的输出。最后返回采集已启动的提示信息给前端。如果当前有重播任务在进行中，则返回提示信息给前端。
         :param replay_popup: 重播选择面板对象
    """
    global COLLECTS_STATUS, REPLAY_HISTORY
    if replay_popup.visible:
        return f"❌ 已打开重播页面，禁止采集"
    
    if not os.path.exists(COLLECT_SCRIPT_PATH):
        return f"❌ 采集脚本不存在: {COLLECT_SCRIPT_PATH}"
    
    if not MACHINE_INFO.get("dataset_dir"):
        return "❌ 采集目录未配置，请先配置采集参数！"
    
    if COMMANDS_STATUS[3]['process'] is None or REPLAY_FLAG:
        return "❌ 主臂会话未启动或正在重播中，请先启动主臂会话！"
    
    if REPLAY_START:
        # 不为0且满足重播频率
        print("重播1")
        collect_count = COLLECT_HISTORY['collect_count']
        if (collect_count % REPLAY_FREQUENCY == 0 and collect_count != 0) or ((collect_count + 1) % REPLAY_FREQUENCY == 0 and (COLLECTS_STATUS[0]['collect_state'] == "endCollecting" or COLLECTS_STATUS[1]['collect_state'] == "endCollecting")):
            print("重播2")
            # 查询采集记录是否连续
            if is_continuous_array(MACHINE_INFO):
                print("重播3")    
                # 获取重播记录
                replay_data = get_replay_data(MACHINE_INFO)
                print("当前重播记录", replay_data)
                REPLAY_HISTORY.update({
                    "replay_count": len(replay_data) if isinstance(replay_data, list) else 0,
                    "history_list": replay_data
                })
                print("重播数组",REPLAY_HISTORY)
                # 判断重播次数是否达标
                # if len(replay_data) < collect_count // REPLAY_FREQUENCY:
                #     return "⚠️ 已经采集到重播频率数了，需要先进行重播后才可进行下一次采集"
                
                # 判断是否存在临近重播，需要每REPLAY_FREQUENCY条至少重播一条
                if replay_data:
                    coverage_flag, text = check_list_coverage(replay_data, collect_count, REPLAY_FREQUENCY)
                    if not coverage_flag:
                        return f"⚠️ 需确保每{REPLAY_FREQUENCY}条采集数据中至少有一条被重播了，现在缺少{text[0]}-{text[1]}分段数据"
                else:
                    return f"⚠️ 已经采集到重播频率数了，需要先进行重播后才可进行下一次采集"
                
    idx = -1
    for i in COLLECTS_STATUS:
        # 任何一个线程现在处于采集状态，那么直接返回线程进行中
        if i["collect_state"] == "collecting":
            return "⚠️ 当前有正在采集的线程，请稍后再试"
        # 如果线程空闲
        if i["collect_state"] == "none":
            idx = i["index"]
            break
    
    if idx == -1:
        return "⚠️ 当前线程均在工作中，请稍后再试"
    
    if DEBUG_FLAG:
        print(f'main - 432 - start_collect \n当前使用线程{idx+1}, 双线程状态为 {COLLECTS_STATUS}')
    
    get_dir_idx(MACHINE_INFO, COLLECTS_STATUS, idx)
    
    cmd = (
        f"python3 {COLLECT_SCRIPT_PATH} --dataset_dir {MACHINE_INFO['dataset_dir']} --episode_idx {COLLECTS_STATUS[idx]['ep_idx']}"
    )
    COLLECTS_STATUS[idx].update({"state": "none", "color": "gray", "msg": "未启动"})
    COLLECTS_STATUS[idx]["process"] = subprocess.Popen(
        cmd,
        shell=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,  # ⚠️ 合并 stderr
        text=True,
        bufsize=1
    )

    print(f"采集命令: {cmd}")

    with status_lock:
        COLLECTS_STATUS[idx].update({
            "pid": COLLECTS_STATUS[idx]["process"].pid, 
            "alive": True, 
            "collect_state": "collecting",
            "state": "running", 
            "color": "purple", 
            "msg": f"正在采集 {COLLECTS_STATUS[idx]['ep_idx']} 中"})
        
    # 🔥 启动 stdout 监听线程
    threading.Thread(
        target=capture_collect_output,
        args=(COLLECTS_STATUS, idx, COLLECT_HISTORY, MACHINE_INFO),
        daemon=True
    ).start()

    return f"▶️ 采集脚本已启动，线程{idx+1}，PID={COLLECTS_STATUS[idx]['pid']}"


# ========= 结束采集 ==========
def end_collect():
    """
    该函数用于结束当前正在进行的采集任务，首先检查当前是否有正在进行的采集任务，如果有，则通过获取采集进程的子进程列表并发送 SIGINT 信号来终止采集进程，同时更新采集状态为正在保存数据。最后返回已发送停止信号的提示信息给前端。如果当前没有采集任务在进行中，则返回提示信息给前端。
    """
    global COLLECTS_STATUS
    # 检查是否有在采集的脚本 采集 / 结束 / 取消保存 均 false

    idx = -1 
    for i in COLLECTS_STATUS:
        if i["collect_state"] == "collecting":
            idx = i["index"]
            break
    
    if idx == -1:
        return "⚠️ 当前无在采集的线程，无法停止"

    # 发送结束采集信号
    ppids = get_children(COLLECTS_STATUS[idx]['pid'])
    for ppid in ppids:
        subprocess.run(["kill", "-SIGINT", str(ppid)])

    COLLECTS_STATUS[idx].update({"collect_state": "endCollecting", "state": "aborted", "color": "yellow", "msg": "正在保存数据"})
    # return ["🛑 已发送停止信号", gr.update(visible=True)]
    return "🛑 已发送停止信号，数据保存中"


# ========= 放弃保存 ==========
def stop_collect():
    """
    该函数用于放弃当前正在进行的采集任务的保存，首先检查当前是否有正在进行的采集任务，如果有，则通过获取采集进程的子进程列表并发送 SIGKILL 信号来强制终止采集进程，同时更新采集状态为用户放弃保存。最后返回已发送取消保存信号的提示信息给前端。如果当前没有采集任务在进行中，则返回提示信息给前端。
    """
    global COLLECTS_STATUS

    idx = -1
    for i in COLLECTS_STATUS:
        if i["collect_state"] == "collecting":
            idx = i["index"]
            break
    if idx == -1:
        return "⚠️ 当前无在采集的线程，无法取消保存"
    
    # 发送kill信号
    ppids = get_children(COLLECTS_STATUS[idx]['pid'])
    for ppid in ppids:
        subprocess.run(["kill", "-9", str(ppid)])

    COLLECTS_STATUS[idx].update({"collect_state": "no_save", "state": "error", "color": "orange", "msg": "用户放弃保存"})
    # 写入采集目录
    with open(f"./log/collect_data_{MACHINE_INFO['date']}.log", 'a', encoding='utf-8') as f:
        f.write(f'{COLLECTS_STATUS[idx]["ep_idx"]} 用户放弃保存\n')

    return "❌ 已发送取消保存信号"


# ========== 启动程序 ==========
if __name__ == "__main__":
    # 若nas目录不存在，则挂载nas
    try:
        if os.path.exists('/mnt/nas'):
            if len(os.listdir('/mnt/nas')) == 0:
                subprocess.run(
                    'cd /home/agilex/nas_uploader && echo agx | sudo -S bash nas_mount.sh',
                    shell=True
                )
    except Exception as e:
        print("挂载nas失败，可能是nas_mount.sh脚本执行权限或未存在此脚本")

    # 配置文件初始化本机内容
    read_config_file(MACHINE_INFO)
    css_class = read_html_file(HTML_CSS_PATH)
    # 启动监控线程
    command_monitor_thread = threading.Thread(target=monitor_command_background, args=(COMMANDS_STATUS, status_lock, ARM_FLOAT_EPS, COMMAND_MONITOR_FREQ), daemon=True)
    collect_monitor_thread = threading.Thread(target=monitor_collect_background, args=(COLLECTS_STATUS, status_lock, 0, COLLECT_MONITOR_FREQ), daemon=True)
    collect_monitor_thread2 = threading.Thread(target=monitor_collect_background, args=(COLLECTS_STATUS, status_lock, 1, COLLECT_MONITOR_FREQ), daemon=True)
    command_monitor_thread.start()
    collect_monitor_thread.start()
    collect_monitor_thread2.start()
    print(f"✅ 命令脚本监控线程启动（频率：{COMMAND_MONITOR_FREQ}次/秒）")
    print(f"✅ 采集脚本监控线程启动（频率：{COLLECT_MONITOR_FREQ}次/秒）")

    # 构建Gradio页面
    with gr.Blocks(title="Kai05_collect", css=css_class) as demo:
        gr.Markdown("# 🚨 Kai05_collect操作面板")
        gr.Markdown("## 📝 当前采集条数：0 条，历史采集记录：[]")
        
        # ROS脚本指示灯
        gr.Markdown("### 🟢 命令脚本状态")
        with gr.Row(elem_classes="indicator-row"):
            command_indicators = []
            for i in range(len(COMMANDS_STATUS)):
                indicator = gr.HTML(
                        value=command_indicator_html(COMMANDS_DISPLAY_NAMES[i], i)
                )
                command_indicators.append(indicator)
        
        # 采集脚本指示灯
        gr.Markdown("### 🟢 采集脚本状态")
        with gr.Row(elem_classes="indicator-row"):
            py_indicators = []
            for i in range(len(COLLECTS_STATUS)):
                py_indicator = gr.HTML(
                    value=collect_indicator_html("采集脚本", i)
                )
                py_indicators.append(py_indicator)

        gr.Markdown("### 🎥 相机画面")

        gr.HTML(
            value="""
            <div class="video-row">
                <div class="video-box">左臂画面<img id="cam_l"></div>
                <div class="video-box">中间画面<img id="cam_f"></div>
                <div class="video-box">右臂画面<img id="cam_r"></div>
            </div>
            """
        )
        
        # 隐藏组件：存储状态数据（供JS读取）
        # status_data_store = gr.State(value="{}")
        status_data_store = gr.Textbox(visible=False)

        # 按钮行1：重播/开始/结束
        with gr.Row():
            replay_btn = gr.Button("重播数据", variant="secondary")
            start_collect_btn  = gr.Button("开始采集", variant="primary", elem_id="start_collect_btn")
            end_collect_btn    = gr.Button("结束采集", variant="stop", elem_id="end_collect_btn")

        
        # 按钮行2：配置按钮（关键：先定义config_btn，再绑定事件）
        with gr.Row():
            commit_collect_btn = gr.Button("数据集采集完毕", variant="primary", elem_id="commit_collect_btn")
            stop_collect_btn = gr.Button("放弃保存", variant="secondary", scale=1, elem_id="stop_collect_btn")
            config_btn = gr.Button("配置采集参数", variant="secondary", scale=1, elem_id="config_btn")
            open_dir = gr.Button("打开采集目录", variant="secondary", scale=1)
        
        # 配置面板（默认隐藏）
        config_panel = gr.Column(
            visible=False, 
            elem_id="config_panel"
        )
        with config_panel:
            gr.Markdown("### 📝 采集脚本参数配置")
            
            task_input = gr.Dropdown(
                label="具体任务",
                choices=MACHINE_INFO["format_tasks"] if isinstance(MACHINE_INFO["format_tasks"], list) else [],
                interactive=True
            )
            scheduling = gr.Dropdown(
                label="班次选择",
                choices=[
                    "day_shift",
                    "night_shift"
                ],
                interactive=True
            )
            formatDate = gr.Textbox(
                label="当前时间",
                interactive=False,
                elem_id="textbox",
                value=time.strftime("%y%m%d", time.localtime())
            )
            max_timesteps_input = gr.Textbox(
                label="最大帧数",
                interactive=False,   # 🔒 只能看，不能改
                elem_id="textbox"
            )

            with gr.Row():
                save_config_btn = gr.Button("保存配置", variant="primary")
                cancel_config_btn = gr.Button("取消", variant="secondary")

        # 操作提示框
        op_tips = gr.Textbox(
            label="操作提示",
            value="等待操作...",
            interactive=False,
            elem_id="op_tips"
        )

        # 重播视频弹窗
        # replay_video = gr.Column(
        #     visible=False,
        #     elem_id="replay_video"
        # )
        # with replay_video:
        #     gr.Markdown("## 重播视频")

        #     gr.HTML(
        #         value="""
        #         <div class="replay_video">
        #             <div class="video-box">左臂画面
        #                 <video id="replay_l" controls autoplay>
        #                     <source src="/home/agilex/kai05_data/agilex/flatten_fold/short_sleeve/flatten_fold_weitiao/v8_260214_test_day_shift_8_10000_hdf5/aloha_mobile_dummy/video/cam_high/episode_0.mp4" type="video/mp4">
        #                 </video>
        #             </div>
        #             <div class="video-box">中间画面
        #                 <video id="replay_f" >
        #                     <source src="/home/agilex/kai05_data/agilex/flatten_fold/short_sleeve/flatten_fold_weitiao/v8_260214_test_day_shift_8_10000_hdf5/aloha_mobile_dummy/video/cam_high/episode_0.mp4" type="video/mp4">
        #                 </video>
        #             </div>
        #             <div class="video-box">右臂画面
        #                 <video id="replay_r" >
        #                     <source src="/home/agilex/kai05_data/agilex/flatten_fold/short_sleeve/flatten_fold_weitiao/v8_260214_test_day_shift_8_10000_hdf5/aloha_mobile_dummy/video/cam_high/episode_0.mp4" type="video/mp4">
        #                 </video>
        #             </div>
        #         </div>
        #         """
        #     )

        # 重播数据弹窗（默认隐藏）
        replay_popup = gr.Column(
            visible=False,
            elem_id="replay_popup"
        )

        with replay_popup:
            gr.Markdown("## 📼 已采集数据（HDF5）")

            replay_episode_dropdown = gr.Dropdown(
                label="选择回放 episode",
                choices=[],
                value=None,
                interactive=True
            )

            with gr.Row():
                replay_confirm_btn = gr.Button("▶️ 重播", variant="primary")
                stop_replay_btn = gr.Button("⏹️ 停止重播", variant="stop")

            with gr.Row():    
                delete_episode_btn = gr.Button("🗑️ 删除", variant="secondary")
                close_replay_btn = gr.Button("❌ 退出", variant="stop")

        # 删除按钮 确认框
        sure_delete = gr.Column(
            visible=False,
            elem_id="sure_delete"
        )

        with sure_delete:
            gr.Markdown(value="是否确认删除")

            with gr.Row():
                del_btn_yes = gr.Button('确认')
                del_btn_no = gr.Button("取消")

        sure_commit = gr.Column(
            visible=False,
            elem_id="sure_commit"
        )
        
        with sure_commit:
            gr.Markdown(value="是否确认提交")

            with gr.Row():
                commit_yes = gr.Button("采集完毕")
                commit_no = gr.Button("仍要采集")

        # 提示弹窗
        bar = gr.Slider(
            minimum=0,
            maximum=1,
            value=0,
            label="采集进度",
            interactive=False,
            visible=False
        )

        # gr.Timer(0.1).tick(get_progress, outputs=bar)
        # ========== 绑定回调（核心：修复变量定义顺序 + JS语法） ==========
        # 自动更新UI并打印状态
        demo.load(
            fn=update_ui,
            inputs=[],
            outputs=status_data_store,
            every=1 / COLLECT_MONITOR_FREQ
        )

        # 状态变化时触发前端打印
        status_data_store.change(
            fn=None,
            inputs=[status_data_store],   # ← 注意是 list
            outputs=None,
            js=update_indicator_js(HTML_JS_PATH)
        )


        # 1-1.重播按钮事件
        replay_btn.click(
            fn=lambda: replay_btn_click(replay_popup),
            inputs=[],
            outputs=[replay_episode_dropdown, replay_popup, op_tips]
        )

        # 1-1-1.确认重播按钮
        replay_confirm_btn.click(
            fn=replay_click,
            inputs=replay_episode_dropdown,
            outputs=[op_tips]
        )

        # 1-1-2.停止重播按钮
        stop_replay_btn.click(
            fn=stop_replay,
            inputs=[],
            outputs=op_tips
        )

        # 1-1-3.删除回放数据按钮
        delete_episode_btn.click(
            fn=on_delete_episode,
            inputs=replay_episode_dropdown,
            outputs=[op_tips, sure_delete]
        )

        # 1-1-3-1.确认删除回放
        del_btn_yes.click(
            delete_data_controller,
            inputs=[replay_episode_dropdown],
            outputs=[op_tips, sure_delete, replay_popup, replay_episode_dropdown]
        )

        # 1-1-3-2.取消删除回放
        del_btn_no.click(
            fn=lambda : gr.update(visible=False),
            inputs=None,
            outputs=[sure_delete]
        )

        # 1-1-4. 关闭重播弹窗按钮
        close_replay_btn.click(
            fn=lambda: restart_collect_piper(replay_popup),
            inputs=[],
            outputs=[replay_popup, op_tips]
        )

        # 1-2.开始采集按钮事件
        start_collect_btn.click(fn=lambda: start_collect(replay_popup), inputs=[], outputs=op_tips)

        # 1-3.结束采集按钮事件
        end_collect_btn.click(end_collect, inputs=[], outputs=[op_tips])

        # 2-1 数据集采集完毕按钮事件
        commit_collect_btn.click(
            fn=lambda: gr.update(visible=True),
            inputs=[],
            outputs=[sure_commit]
        )

        # 2-1-1 数据采集完毕二级验证
        commit_yes.click(
            fn=lambda: [commit_collect(MACHINE_INFO, COLLECTS_STATUS), gr.update(visible=False)],
            inputs=[],
            outputs=[op_tips, sure_commit]
        )

        # 2-1-2 数据取消提交
        commit_no.click(
            fn=lambda: ["取消提交", gr.update(visible=False)],
            inputs=[],
            outputs=[op_tips, sure_commit]
        )

        # 2-2放弃保存按钮
        stop_collect_btn.click(
            stop_collect,
            inputs=[],
            outputs=op_tips
        )
        
        # 2-3. 配置按钮事件
        config_btn.click(
            open_config_panel, 
            inputs=[], 
            outputs=[formatDate, task_input, scheduling, max_timesteps_input, config_panel, save_config_btn, cancel_config_btn]
        )

        # 2-3-1. 保存配置
        save_config_btn.click(
            save_config,
            inputs=[formatDate, task_input, scheduling, max_timesteps_input],
            outputs=[op_tips, config_panel]   # 或者直接 outputs=None
        )

        # 2-3-2. 取消配置按钮事件
        cancel_config_btn.click(
            fn=lambda : ("❌ 用户取消配置采集参数", gr.update(visible=False)), 
            inputs=[], 
            outputs=[op_tips, config_panel]
        )

        # 2-4打开采集目录按钮事件
        open_dir.click(
            fn=lambda: open_collect_dir(MACHINE_INFO),
            inputs=[],
            outputs=[op_tips]
        )
        

    # 启动Gradio服务
    demo.queue().launch(
        server_port=7860,
        server_name="0.0.0.0",
        inbrowser=True,
        max_threads=4
    )

    # 等待线程结束
    command_monitor_thread.join()
    collect_monitor_thread.join()
    collect_monitor_thread2.join()
    print("✅ 所有监控线程已停止")
