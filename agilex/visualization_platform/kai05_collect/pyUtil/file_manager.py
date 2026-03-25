import json
import os
import re
from pathlib import Path
import gradio as gr
# import h5py
import subprocess
import sys
import socket
# 获取当前文件的目录（pyUtil/）
current_dir = os.path.dirname(os.path.abspath(__file__))
# 获取上级目录（即包含pyUtil的目录）
parent_dir = os.path.dirname(current_dir)
# 将上级目录加入Python的模块搜索路径
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)
from pyUtil.collect_machine2nas import notice_nas_service
from pyUtil.host_data_manager import (
    get_id,
    get_tasks,
    delete_host_data,
    get_collect_data
)
'''
文件管理模块，包含读取和保存配置文件的函数
update_indicator_js: 更新js
read_html_file: 读取HTML文件内容
command_indicator_html: 生成ROS命令行状态指示器HTML片段
collect_indicator_html: 生成采集命令行状态指示器HTML片段

save_config_file: 保存配置文件到config.json
read_config_file: 读取配置文件内容
get_dir_idx: 获取当前工作的索引
scan_episode_indices: 扫描数据文件夹获取可用的 episode 索引列表
load_hdf5_data: 读取HDF5文件并拆分左右手数据

open_collect_dir: 打开采集目录
get_children: 获取子进程PID列表
delete_data: 确认删除指定 episode 的数据文件
commit_collect: 提交采集结束标识
'''
# ========= 更新指示器JS ==========
def update_indicator_js(html_js_path):
    """
    该函数读取指定路径的HTML/JS文件内容，并返回字符串
        :param html_js_path: HTML/JS文件路径
        :return: 文件内容字符串
    """
    js = read_html_file(html_js_path)
    return js


# ========= 读取HTML文件函数 ==========
def read_html_file(file_path):
    """
    该函数读取指定路径的HTML文件内容，并返回字符串
        :param file_path: HTML文件路径
        :return: 文件内容字符串
    """
    txt = ""
    with open(file_path, 'r', encoding='utf-8') as f:
        txt = f.read()
    return txt


# 命令行状态指示器 HTML 片段
def command_indicator_html(name: str, idx: int):
    """
    该函数生成一个命令行状态指示器的HTML片段，包含一个圆点、命令名称、状态文本和PID显示
        :param name: 命令名称
        :param idx: 指示器下标，用于区分不同指示器的HTML元素ID
    """
    return f"""
    <div id="command_{idx}" class="indicator">
        <span id="command_dot_{idx}" class="dot gray"></span>
        <span class="name">{name}</span>
        <span id="command_text_{idx}" class="text">未启动</span>
        <span id="command_pid_{idx}" class="pid">PID: —</span>
    </div>
    """

# 采集命令行状态指示器 HTML 片段
def collect_indicator_html(name: str, idx: int):
    """
    该函数生成一个采集命令行状态指示器的HTML片段，包含一个圆点、命令名称、状态文本和PID显示
        :param name: 命令名称
        :param idx: 指示器下标，用于区分不同指示器的HTML元素ID
    """
    return f"""
    <div id="collect_{idx}" class="indicator">
        <span id="collect_dot_{idx}" class="dot gray"></span>
        <span class="name">{name}</span>
        <span id="collect_text_{idx}" class="text">未启动</span>
        <span id="collect_pid_{idx}" class="pid">PID: —</span>
    </div>
    """


# ========= 保存配置文件函数 ==========
def save_config_file(machine_info: dict, config_file_path, only_change_count: bool=False, collect_history: dict=None):
    """
    该函数将机器信息保存到指定路径的配置文件中
        :param machine_info: 包含机器信息的字典
        :param config_file_path: 配置文件路径
        :param only_change_count: 如果为 True，则仅更新 collect_count 字段，其他字段保持不变
        :param collect_history: 采集历史记录字典，如果提供则在保存配置后更新历史记录中的 collect_count 和 history_list 字段
    处理流程：
        1. 如果 only_change_count 为 True，则从现有配置文件中读取 collect_count 字段，并更新 machine_info 中的 collect_count 字段
        2. 如果 collect_history 提供，则从 machine_info 中获取 dataset_dir 和 episode 索引，扫描数据目录获取当前 collect_count，并更新 collect_history 中的 collect_count 字段
        3. 将 machine_info 中的相关字段写入配置文件（JSON格式）
    """
    dataset_dir = machine_info.get('dataset_dir')
    # 从更新的配置文件中获取task任务
    if not only_change_count:
        task = machine_info.get("task", None)
        if not task:
            return "❌ 请选择具体任务"
        # 获取具体任务的全名
        task_map = [m for m in machine_info["machine_tasks"] if m.endswith(task)][0]
        dataset_dir = f"/home/{machine_info['machine_name']}/kai05_data/{machine_info['machine_name']}/{task_map}/{machine_info['version']}_{machine_info['date']}_{machine_info['machine_id']}_{machine_info['scheduling']}_{machine_info['plan_count']}_10000_hdf5"
        machine_info.update({
            "dataset_dir": dataset_dir,
            "pre_dataset_dir": dataset_dir
        })
    
    # 扫描文件数量
    collect_count = len(scan_episode_indices(os.path.join(dataset_dir, "aloha_mobile_dummy")))
    machine_info.update({"collect_count": collect_count})
    # 从服务器获取采集记录
    if collect_history:
        collect_data = get_collect_data(machine_info)
        if collect_data:
            collect_history.update({
                'history_list': collect_data
            })

    try:
        with open(config_file_path, "w", encoding="utf-8") as f:
            f.write(json.dumps(machine_info, indent=2, ensure_ascii=False))
        return "✅ 配置保存成功！"
    except Exception as e:
        return "❌ 配置保存失败"


# ========= 读取配置文件函数 ==========
def read_config_file(machine_info: dict):
    """
    该函数读取指定路径的配置文件内容，并更新 machine_info 字典中的相关字段
        :param machine_info: 包含机器信息的字典，函数会更新该字典中的相关字段
    处理流程：
        1. 从 machine_info 中获取配置文件路径 config_file_path
        2. 如果配置文件存在，则读取配置文件内容，并更新 machine_info 中的相关字段（如果配置文件中没有某个字段，则保持 machine_info 中原有的值不变）
        3. 如果配置文件不存在，则尝试通过获取本机IP地址和远程服务器接口获取机器ID和任务列表，并更新 machine_info 中的相关字段
        4. 最后将更新后的 machine_info 保存回配置文件中（如果之前没有配置文件，则会创建新的配置文件）
    """
    config_file_path = machine_info.get('config_file_path')
    # 从配置文件获取上次配置
    if os.path.exists(config_file_path):
        print("配置文件存在，获取上次配置内容")
        with open(config_file_path, 'r') as f:
            data = json.load(f)
            machine_info.update({data_key: data.get(data_key, machine_info.get(data_key, "")) for data_key in machine_info})
    # 如果没有配置文件则进行ip分配生成
    else:
        try:
            # 获取本机ip
            machine_ip = get_local_ip()
            last_one_ip = machine_ip.split('.')[-1]
            # response = req.get(f'http://192.168.9.50:5000/api/get_id?ip={int(last_one_ip)}').json()
            res_data = get_id(last_one_ip)
            # print(response)
            # res_data = response['data']
            # print(res_data)
            if res_data != None:
                machine_info.update({
                    'machine_id': res_data['machine_id'],
                    "machine_name": res_data['machine_name'],
                    "max_timesteps": res_data['max_timesteps']
                })
                print(f"获取id成功 {machine_info['machine_id']}")
        except Exception as e:
            print("当前机器未编入松灵kai05序列")
            return

    all_tasks = machine_info.get("all_tasks")
    machine_id = machine_info.get("machine_id")
    # 更新配置
    # 通过机器id获取能做的项目（远程）
    res_data = None
    update_info = None
    try:
        # response = req.get(f'http://192.168.9.50:5000/api/get_tasks?id={machine_id}').json()
        res_data = get_tasks(machine_id)
    except Exception as e:
        print(f"❌ 远程获取任务列表失败: {e}")
    
    if res_data:
        update_info = res_data
        print("通过远程获取任务列表成功！")
    else:
        # 通过机器id获取能做的项目（本地）
        if os.path.exists(all_tasks):
            with open(all_tasks, 'r') as f:
                data = json.load(f)
                print(data)
                if machine_id in data:
                    update_info = data[machine_id]
                    print("通过本地获取任务列表成功！")
        else:
            return "❌ 未连接网络，且本地all_tasks.json文件不存在，请检查网络或联系管理员获取最新任务列表！"
    
    # print(update_info)

    machine_info.update({data_key: update_info.get(data_key, machine_info.get(data_key, "")) for data_key in update_info})
    machine_info.update({'format_tasks': [m.split("/")[-1] for m in machine_info["machine_tasks"]]})
    
    # print(machine_info , type(machine_info['format_tasks']), type(machine_info))
    # 启动网页后自动先根据上一次的配置文件保存一次配置，若需要修改可通过配置采集参数并保存，这一步考虑的是用户可能离线删除了文件
    save_config_file(machine_info, config_file_path, True)
        
    # print("初始化机器信息", json.dumps(machine_info, indent=2, ensure_ascii=False))
    
    return "✅ 获取具体任务列表成功"


# 获取当前工作的索引
def get_dir_idx(machine_info: dict, collects_status: dict=None, idx: int=0, last_one: bool=False) -> int:
    """
    该函数根据当前数据目录中的文件情况，获取一个可用的 episode 索引，用于新的采集工作
        :param machine_info: 包含机器信息的字典，必须包含 dataset_dir 字段
        :param collects_status: 当前正在采集的状态字典，用于避免分配正在采集的 episode 索引（可选）
        :param idx: 当前采集线程的下标，用于从 collects_status 中获取对应线程的状态（默认0）
        :param last_one: 如果为 True，则直接返回当前数据目录中最大的 episode 索引，适用于查询最后一个索引的场景（默认 False）
        :return: 可用的 episode 索引，如果 last_one 为 True 则返回当前数据目录中最大的 episode 索引
    处理流程：
        1. 从 machine_info 中获取 dataset_dir 字段，确保目录存在
        2. 扫描 dataset_dir 中的 episode 文件，获取当前已存在的 episode 索引列表
        3. 如果 last_one 为 True，则直接返回当前数据目录中最大的 episode 索引
        4. 如果 last_one 为 False，则根据 collects_status 中正在采集的状态，优先分配未被占用的 episode 索引（包括已存在但未被占用的索引，以及新的索引）
    """
    episode_idx = 0
    dataset_dir = os.path.join(machine_info.get("dataset_dir", ""), "aloha_mobile_dummy")
    dataset_dir = Path(dataset_dir).expanduser()
    # 确保目录存在
    Path(dataset_dir).mkdir(parents=True, exist_ok=True)
    episode_indices = scan_episode_indices(dataset_dir)
    # 如果存在索引
    if episode_indices:
        # 如果只是来查询最后一个索引，则直接返回最大索引
        if last_one:
            return max(episode_indices)
        # 获取完整的索引范围，找出缺失的索引
        full = set(range(0, max(episode_indices) + 1))
        missing = sorted(full - set(episode_indices))

        # 如果有缺失的索引，优先分配缺失的索引
        if missing:
            for ep in missing:
                # if ep in (collects_status[0]['ep_idx'], collects_status[1]['ep_idx']):
                #     continue
                # 线程占有此idx并线程不处于空闲状态，说明可能在保存
                if ep in collects_status[0] and collects_status[0]['collect_state'] != "none":
                    continue
                elif ep in collects_status[1] and collects_status[1]['collect_state'] != "none":
                    continue
                else:
                    print(f"分配 episode 索引: {ep}")
                    episode_idx = ep
                    break
        # 如果没有缺失的索引，则分配下一个新的索引
        # 如果当前获取到的 episode 索引为 0，并且 0 已经存在于已采集的索引列表中，则说明 0 已经被占用，需要分配下一个新的索引
        if episode_idx == 0 and 0 in episode_indices:
            episode_idx = max(episode_indices) + 1 if episode_indices else 0
            # print(f"检测到{len(episode_indices)}个episode")
            while True:
                # if episode_idx in (collects_status[0]['ep_idx'], collects_status[1]['ep_idx']):
                #     episode_idx += 1
                if episode_idx == collects_status[0]['ep_idx'] and collects_status[0]['collect_state'] != "none":
                    print(f'{episode_idx} 在 线程1中，并且线程1不为none')
                    episode_idx += 1
                elif episode_idx == collects_status[1]['ep_idx'] and collects_status[1]['collect_state'] != "none":
                    print(f'{episode_idx} 在 线程2中，并且线程2不为none')
                    episode_idx += 1
                else:
                    break
    if last_one:
        return -1
    
    print(f"当前采集 episode 索引: {episode_idx}")
    collects_status[idx].update({"ep_idx": episode_idx})


# 文件扫描函数，获取可用的 episode 索引列表（重播时获取文件列表）
def scan_episode_indices(dataset_dir):
    """
    该函数扫描指定数据目录中的 episode 文件，提取并返回可用的 episode 索引列表
        :param dataset_dir: 数据目录路径，应该包含 episode 文件（例如 episode_0.hdf5、episode_1.hdf5 等）
        :return: 可用的 episode 索引列表，按照升序排序
    """
    pattern = re.compile(r"episode_(\d+)\.hdf5$")
    indices = []
    try:
        for fname in os.listdir(dataset_dir):
            m = pattern.match(fname)
            if m:
                indices.append(int(m.group(1)))
        print(f"file-manager [scan_episode_indices] 扫描到的 episode 索引列表: {indices}")
        return sorted(indices)
    except:
        return indices

# def load_hdf5_data(dataset_dir, episode_idx, return_row_line = 1):
#     """
#     : 从指定数据集目录和episode索引的HDF5文件中读取qpos数据，拆分为左右手数据，并转换为指定列表格式
#     : param dataset_dir: 数据集根目录
#     : param episode_idx: episode索引
#     : return: 左手数据、右手数据（格式与LEFT0/RIGHT0一致，list列表）
#     """
#     qpos = None
#     LEFT_DATA = []
#     RIGHT_DATA = []

#     # 1. 拼接HDF5文件路径（与原代码逻辑一致）
#     dataset_path = os.path.join(dataset_dir, "aloha_mobile_dummy", f'episode_{episode_idx}.hdf5')
    
#     # 2. 校验文件是否存在
#     if not os.path.isfile(dataset_path):
#         print(f'错误：数据集文件不存在！\n路径：{dataset_path}\n')
#         return None, None
    
#     # 3. 沿用原代码逻辑读取HDF5文件（只读模式）
#     with h5py.File(dataset_path, 'r') as root:
#         # 读取核心数据集qpos（关节位置，14个值：前7左、后7右）
#         qpos = root['/observations/qpos'][()]

#     if len(qpos) == 0:
#         print("错误：qpos中无有效数据！")
#         return None, None
    
#     # 4. 核心修改：拆分左右手数据，并转换为指定格式
#     def convert_to_target_format(data):
#         """将numpy数组转换为与LEFT0/RIGHT0一致的Python列表格式"""
#         # 步骤1：拆分前7（左）、后7（右）
#         left_data_np = data[:7]  # 前7个：左手
#         right_data_np = data[7:] # 后7个：右手
        
#         # 步骤2：numpy数组 → Python列表（匹配目标格式）
#         # 保留小数精度，与示例[0, 0.0, 0.0, 0.02, 0.43, 0.0, 0.07]格式对齐
#         left_data_list = left_data_np.tolist()
#         right_data_list = right_data_np.tolist()
#         return left_data_list, right_data_list
    
#     # 5. 提取数据（默认仅返回第一行）
#     for i in range(return_row_line):
#         print(f'第{i}行数据：{qpos[i]}')
#         # 提取第一行数据（索引0），转换为目标格式
#         left_frame, right_frame = convert_to_target_format(qpos[i])
#         LEFT_DATA.append(left_frame)
#         RIGHT_DATA.append(right_frame)

#     # 6.写入记录中心文件，方便查看
#     with open("/home/agilex/kai05_collect/log/hdf5_position.txt", "w") as f:
#         f.write(",".join([str(x) for x in LEFT_DATA]) + "\n")
#         f.write(",".join([str(x) for x in RIGHT_DATA]))
#         print("hdf5第一帧记录成功，文件目录：/home/agilex/kai05_collect/log/hdf5_position.txt")

#     # 7. 返回拆分后的左右手数据（匹配目标格式）
#     return LEFT_DATA, RIGHT_DATA


# ========= 打开采集目录 ==========
def open_collect_dir(machine_info: dict) -> str:
    """
    该函数尝试打开采集目录，并返回操作结果的提示信息字符串
        :param machine_info: 包含机器信息的字典，必须包含 dataset_dir 字段
        :return: 操作结果提示信息字符串，成功时包含已打开的目录路径，失败时包含错误信息
    处理流程：
        1. 从 machine_info 中获取 dataset_dir 字段，并构建完整的采集目录路径
        2. 检查采集目录是否存在，如果不存在则返回错误提示
        3. 如果目录存在，尝试使用系统默认文件管理器打开该目录，并返回成功提示；如果打开过程中出现异常，则捕获并返回错误提示
    """
    dataset_dir = machine_info.get("dataset_dir", "")
    dataset_dir = os.path.join(dataset_dir.replace("~", f"/home/{machine_info.get('machine_name')}"), "aloha_mobile_dummy")
    
    if not dataset_dir or not os.path.exists(dataset_dir):
        return "❌ 采集目录未配置或不存在！"
    
    try:
        subprocess.Popen(["xdg-open", dataset_dir])
        return f"📂 已打开采集目录: {dataset_dir}"
    except Exception as e:
        return f"❌ 打开采集目录失败: {e}"


# 获取本机内网ip
def get_local_ip():
    """
    获取本机内网IP地址（非127.0.0.1）
    """
    try:
        # 创建一个UDP套接字，连接到外部服务器（无需实际连接，仅用于获取本机出口IP）
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # 连接到公共DNS服务器（8.8.8.8是Google的DNS，仅用于探测，不会真的发送数据）
        s.connect(("8.8.8.8", 80))
        # 获取套接字绑定的本机IP
        local_ip = s.getsockname()[0]
    except Exception as e:
        # 异常时返回回环地址
        local_ip = "127.0.0.1"
        print(f"获取内网IP失败：{e}")
    finally:
        s.close()
    return local_ip


# ========= 获取子进程PID ==========
def get_children(ppid):
    """
    该函数获取指定父进程ID（ppid）的所有子进程ID列表
        :param ppid: 父进程ID，整数类型
        :return: 子进程ID列表，包含所有直接子进程的PID
    处理流程：
        1. 初始化一个空列表 children 用于存储子进程ID
        2. 遍历 /proc 目录下的所有项，筛选出名称为数字的项（即进程ID）
        3. 对于每个数字项，尝试打开对应的 /proc/[pid]/stat 文件，读取其中的内容并解析出父进程ID（ppid）
        4. 如果解析出的父进程ID与输入的 ppid 匹配，则将该进程ID添加到 children 列表中
        5. 返回最终的子进程ID列表
    """
    children = []
    for pid in os.listdir("/proc"):
        if pid.isdigit():
            try:
                with open(f"/proc/{pid}/stat") as f:
                    if int(f.read().split()[3]) == ppid:
                        children.append(int(pid))
            except:
                pass
    return children


# ========== 确认删除数据 ==========
def delete_data(machine_info, idx: int, collect_history):
    """
    该函数尝试删除指定 episode 索引的数据文件，并返回操作结果的提示信息字符串以及更新后的 episode 索引列表（供 Gradio 组件更新使用）
        :param machine_info: 包含机器信息的字典，必须包含 dataset_dir 字段
        :param idx: 要删除的 episode 索引，整数类型
        :param collect_history: 采集历史记录字典，包含 history_list 和 collect_count 字段，用于在删除成功后更新历史记录
        :return: 包含操作结果提示信息字符串、Gradio 组件更新对象（用于控制组件显示和更新选项列表）的列表
    处理流程：
        1. 参数校验：确保 idx 是非负整数，避免无效路径
        2. 提取常量，提高可维护性（避免硬编码重复）
        3. 定义所有需要删除的文件路径（使用 pathlib 更优雅）
        4. 初始化删除结果统计
        5. 遍历文件，逐个删除（容错性更强：单个文件失败不影响其他文件）
        6. 构造返回提示信息
        7. 保持原有返回格式，更新 episode 索引列表供 Gradio 组件使用
    """
    # 1. 参数校验：确保 idx 是非负整数，避免无效路径
    idx = int(idx)
    if not isinstance(idx, int) or idx < 0:
        return [f'删除数据 episode_{idx} 失败：索引必须是非负整数', gr.update(visible=False), gr.update(), gr.update()]
    
    # 2. 提取常量，提高可维护性（避免硬编码重复）
    dataset_dir = os.path.join(machine_info.get("dataset_dir", ""), "aloha_mobile_dummy")
    base_dir = Path(dataset_dir).expanduser()  # 自动处理 ~，比手动 replace 更稳妥
    episode_prefix = f"episode_{idx}"
    
    # 3. 定义所有需要删除的文件路径（使用 pathlib 更优雅）
    file_paths = [
        base_dir /  f"{episode_prefix}.hdf5",
        base_dir /  "video" / "cam_high" / f"{episode_prefix}.mp4",
        base_dir /  "video" / "cam_left_wrist" / f"{episode_prefix}.mp4",
        base_dir /  "video" / "cam_right_wrist" / f"{episode_prefix}.mp4"
    ]
    
    # 4. 初始化删除结果统计
    deleted_files = []
    failed_files = []
    
    # 5. 遍历文件，逐个删除（容错性更强：单个文件失败不影响其他文件）
    for file_path in file_paths:
        try:
            # 检查文件是否存在
            if file_path.exists():
                # pathlib.Path.unlink() 用于删除文件，missing_ok=False（默认）：文件不存在会抛异常
                file_path.unlink()
                deleted_files.append(str(file_path))
                print(f"成功删除文件：{file_path}")
            else:
                failed_files.append(f"{file_path}（文件不存在）")
        except Exception as e:
            # 捕获所有可能的异常（权限不足、文件被占用等）
            failed_files.append(f"{file_path}（错误：{str(e)}）")
            print(f"删除文件失败：{file_path}，错误：{str(e)}")
    
    # 6. 构造返回提示信息
    if not failed_files:
        msg = f"删除数据 {episode_prefix} 成功，共删除 {len(deleted_files)} 个文件"
        # 如果删除成功且 idx 在 collect_list 中，则从 collect_list 中移除 idx
        if idx in collect_history["history_list"]:
            collect_history["history_list"].remove(idx)
            collect_history["collect_count"] -= 1
        # 向服务器发送删除操作
        delete_host_data(machine_info, idx)

    else:
        msg = f"删除数据 {episode_prefix} 部分成功（成功 {len(deleted_files)} 个，失败 {len(failed_files)} 个）：\n{'; '.join(failed_files[:3])}"  # 只显示前3个失败项，避免信息过长
    
    # 7. 保持原有返回格式
    indices = scan_episode_indices(os.path.join(machine_info.get('dataset_dir'), 'aloha_mobile_dummy'))
    # 如果删除后没有数据，则将页面关闭
    if len(indices) == 0:
        return [msg, gr.update(visible=False), gr.update(visible=False), gr.update()]
    # 转成字符串，避免 Gradio 类型坑
    choices = [str(i) for i in indices]
    default_value = str(indices[-1])   # ⭐ 默认最后一个
    return [msg, gr.update(visible=False), gr.update(), gr.update(choices=choices, value=default_value)]


# 检查列表是否每个分段都有覆盖（例如每5条一个分段，检查每5条是否都有目标列表中的数字）
def check_list_coverage(replay_list, collect_count, replay_frequency):
    """
    该函数检查给定的 replay_list 是否在 collect_count 范围内，每个 replay_frequency 分段都至少包含 replay_list 中的一个数字
        :param replay_list: 目标列表，包含需要覆盖的数字（字符串类型）
        :param collect_count: 总的数字范围，整数类型，表示从0到collect_count-1的数字
        :param replay_frequency: 分段大小，整数类型，表示每个分段的长度（例如5表示每5条一个分段）
        :return: 一个包含两个元素的列表，第一个元素是布尔值，表示是否每个分段都覆盖了目标列表中的数字；第二个元素是一个列表，如果第一个元素为False，则第二个元素包含第一个未覆盖的分段的起始和结束数字；如果第一个元素为True，则第二个元素为None
    """
    # 将目标列表转为集合，提升查找效率
    target_set = set(replay_list)
    
    # 遍历每一个分段
    current_start = 0
    while current_start <= collect_count - 1:
        # 计算当前分段的结束值（不超过总结束值）
        current_end = min(current_start + replay_frequency - 1, collect_count - 1)
        
        # 检查当前分段内是否有目标列表中的数字
        has_number = False
        for num in range(current_start, current_end + 1):
            if str(num) in target_set:
                has_number = True
                break  # 找到一个就不用继续检查了
        
        # 如果当前分段没有数字，直接返回False
        if not has_number:
            print(f"验证失败：分段 [{current_start}, {current_end}] 中没有目标列表的数字")
            return False, [current_start, current_end]
        
        # 进入下一个分段
        current_start += replay_frequency
    
    # 所有分段都有数字，返回True
    print("验证成功：所有分段都包含目标列表的数字")
    return True, None


# 查询数字列表是否连续
def is_continuous_array(machine_info):
    """
    该函数检查数据目录中的 episode 索引列表是否连续（即没有缺失的索引），用于判断文件是否有丢失现象
        :param machine_info: 包含机器信息的字典，必须包含 dataset_dir 字段
        :return: 布尔值，True表示索引连续，False表示索引不连续（存在丢失现象）
    处理流程：
        1. 从 machine_info 中获取 dataset_dir 字段，并构建完整的数据目录路径
        2. 扫描数据目录中的 episode 文件，获取当前已存在的 episode 索引列表
        3. 将索引列表排序，并检查相邻索引之间的差值是否为1，如果存在差值不为1的情况，则说明索引不连续，返回 False；如果所有相邻索引之间的差值都为1，则说明索引连续，返回 True
    """
    indices = scan_episode_indices(os.path.join(machine_info.get('dataset_dir'), 'aloha_mobile_dummy'))
    sorted_arr = sorted(indices)
    for i in range(len(sorted_arr) - 1):
        if sorted_arr[i+1] - sorted_arr[i] != 1:
            return False
    return True
    

def commit_collect(machine_info, collects_status: dict=None) -> str:
    """
    该函数用于提交采集结束标识，并通知 NAS 服务新的数据目录，返回操作结果的提示信息字符串
        :param machine_info: 包含机器信息的字典，必须包含 dataset_dir、pre_dataset_dir、plan_count 等字段
        :param collects_status: 当前正在采集的状态字典，用于检查是否有正在进行中的采集任务（可选）
        :return: 操作结果提示信息字符串，成功时包含提交成功的消息，失败时包含错误信息或提示用户操作的消息
    处理流程：
        1. 如果 collects_status 提供，则检查是否有正在进行中的采集任务（collect_state 不为 "none"），如果有则返回提示用户结束采集/保存后再提交的消息
        2. 获取当前数据目录中最大的 episode 索引，构建新的数据目录名称（将 pre_dataset_dir 中的 plan_count 替换为 max_idx+1）
        3. 统计当前数据目录中的 episode 文件数量，检查是否存在丢失现象（即最大索引与文件总数不匹配），如果存在丢失现象则返回提示文件丢失的消息
        4. 如果没有丢失现象且 max_idx 不为 -1，则尝试重命名数据目录为新的名称，并更新 machine_info 中的 dataset_dir 字段；如果重命名过程中出现异常，则捕获并返回错误提示
        5. 如果重命名成功，则调用 notice_nas_service 函数通知 NAS 服务新的数据目录，并返回提交成功的消息
    """
    if collects_status:
        for i in collects_status:
            if i["collect_state"] != "none":
                return f"⚠️ 当前线程{int(i['index'])+1}采集任务正在进行中，请结束采集/保存后再进行提交！"
    max_idx = get_dir_idx(machine_info, last_one=True)
    dataset_dir = machine_info.get("dataset_dir")
    pre_dataset_dir = machine_info.get("pre_dataset_dir")
    new_name = pre_dataset_dir.replace(str(machine_info.get('plan_count')), str(max_idx+1))
    # 统计文件完整性
    episode_num = len(scan_episode_indices(os.path.join(dataset_dir, 'aloha_mobile_dummy')))
    if max_idx+1 != episode_num:
        return f"整体文件数和文件索引有丢失现象 最大索引{max_idx}, 文件总数{episode_num}"
    if max_idx == -1:
        return "⚠️ 当前采集目录无有效数据，无法提交！"
    # prompt = [mts for mts in machine_info.get("machine_tasks", []) if machine_info.get("task") in mts]
    prompt = "Flatten and fold the short sleeve"
    try:
        # 重命名之后发送消息
        os.rename(dataset_dir, new_name)
        print('a',machine_info['dataset_dir'])
        machine_info.update({"dataset_dir": new_name})
        print('b',machine_info['dataset_dir'])
        with open("/home/agilex/kai05_collect/config/config.json", 'w', encoding='utf-8') as f:
            f.write(json.dumps(machine_info, indent=2, ensure_ascii=False))
        notice_nas_service(os.path.join(new_name, 'aloha_mobile_dummy'), os.path.join(pre_dataset_dir, 'aloha_mobile_dummy'), max_idx, prompt=prompt, is_last_one=True)
    except Exception as e:
        return "⚠️ 发送消息失败或重命名失败，请检查网络或联系管理员！"
    return "✅ 发送消息成功"


if __name__ == '__main__':
    # 如果启动的是本地测试环境，读取配置文件并发送消息
    if os.path.exists("/home/agilex/kai05_collect/config/config.json"):
        with open("/home/agilex/kai05_collect/config/config.json", 'r') as f:
            machine_info = json.load(f)
    print(commit_collect(machine_info))





