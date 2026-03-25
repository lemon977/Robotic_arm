import gradio as gr
import os
import json
import sys
import datetime
import subprocess
import logging
import threading
import time
import re

from tools.log_setup import setup_logger

from tools.nas_upload import (
    nas_upload,
    nas_select
)

from tools.config_upload import (
    config_upload,
    config_upload_byNet
)

from tools.html_manager import (
    update_ui,
    update_indicator_js
)

from tools.monitor_manager import (
    monitor_process,
    monitor_resets,
    # monitor_timeout
)

from tools.ssh_host import (
    ssh_execute_shell_with_progress
)

current_time = datetime.datetime.now()
# 格式化为 两位年份+两位月份+两位日期（如260213）
formatted_date = current_time.strftime("%y%m%d")
CONFIG_DATA = {}
DEBUG_FLAG = False
COMMANDS_STATUS = {}
LISTEN_INDICES = [0, 1, 2]  # 需要监听的下标
WORKING_DIR = os.path.dirname(os.path.abspath(__file__))
logger = ""
THIS_LOG = []
TEST_STATUS = {
    "machine_id": "001",            # 机器ID，唯一标识一台机器，建议使用数字编号
    "machine_name": "agilex",       # 当前测试的机器名称
    "pid": -1,                      # 测试进程的PID，初始值为-1表示未启动
    "process": None,                # 测试进程对象，包含了测试进程的相关信息，如pid、状态等，可以通过这个对象来监控测试进程的状态并进行相应的操作
    "alive": False,                 # 测试是否正在进行中的标志位，通过监听测试进程的状态来更新
    "start_test": False,            # 测试是否已经开始的标志位，避免重复点击开始测试按钮导致的重复执行测试流程
    "arm_status": "未复位",           # 从臂状态：未复位、正在复位、零位、推理位等
}
MODEL_STATUS = {
    "model_name": "未启动",          # 当前正在运行的模型名称
    "progress": 0,                  # 模型启动进度百分比
    "output": "",                   # 模型启动过程中的输出信息
    "cmd": "",                      # 启动模型的命令
    "run_status": "未启动",          # 模型运行状态：未启动、运行中、复制中、加载中、运行完成、错误等
    "last_status": "",              # 上一次状态，用于判断状态是否发生变化
    "port": "",                     # 模型监听的端口号
    "host": "",                     # SSH连接的主机地址
}
UPDATE_DROP = True
TELL_CHANGE = False


# 日志写入函数
def log_append(level, value):
    """
    该函数用于将日志信息写入日志文件，并根据日志级别打印不同的日志内容。可以根据需要修改日志的格式、级别和处理方式。
    参数:
        - level: 日志级别，例如"success"、"error"、"warning"等。
        - value: 日志内容，可以是字符串或其他类型的数据，表示需要记录的日志信息。
    处理流程:
        - 根据日志级别将日志内容写入日志文件，并使用logger打印日志信息。
    """
    global THIS_LOG
    datetime_str = datetime.datetime.now().strftime("%y%m%d %H:%M:%S")
    THIS_LOG.append(f"{datetime_str}: {value}")
    if level == "success":
        logger.info(f"{value}")
    elif level == "error":
        logger.error(f"{value}")
    elif level == "warning":
        logger.warning(f"{value}")


# 分段显示和隐藏提示信息
def show_tips_then_hide(value):
    """
    该函数用于在前端显示提示信息，并在一定时间后自动隐藏提示信息。可以根据需要修改提示信息的显示方式、持续时间和处理方式。
    参数:
        - value: 要显示的提示信息内容。
    """
    if DEBUG_FLAG:
        print("show_tips_then_hide被触发，value:", value)
    if value != "" and MODEL_STATUS['run_status'] != "未启动":
        datetime_str = (datetime.datetime.now()).strftime("%y%m%d %H:%M:%S")
        THIS_LOG.append(f"{datetime_str}: {value}")
        logger.info(f"{value}")
        yield gr.update(elem_classes="xshow")
        time.sleep(2)
        yield gr.update(elem_classes="", value="")
    else:
        yield gr.update(elem_classes="")


# 启动模型
def start_model(model_name, subdir_name, steps):
    """
    该函数用于启动模型，并通过SSH连接远程主机执行启动命令，同时实时监听命令输出以捕获进度信息和关键日志。可以根据需要修改SSH连接的参数、命令执行方式和输出处理方式。
    参数:
        - model_name: 模型名称，表示需要启动的模型的名称。
        - subdir_name: 子目录名称，表示模型所在的子目录名称。
        - steps: 步长，表示模型运行的步长信息。
    处理流程:
        - 检查当前模型状态，如果已有模型在运行中、加载中、复制中或运行完成，则返回相应的提示信息，避免重复启动。
        - 根据模型名称、子目录名称和步长信息构建启动命令，并更新MODEL_STATUS中的相关信息。
        - 通过SSH连接远程主机执行启动命令，并实时监听命令输出以捕获进度信息和关键日志，同时更新MODEL_STATUS中的运行状态和进度信息。
    """
    global MODEL_STATUS
    
    if not model_name or not subdir_name or not steps:
        return gr.update(value="模型名称、子目录、步长、主机或端口不能为空")      

    run_status = MODEL_STATUS.get("run_status")
    host = MODEL_STATUS.get("host")
    port = MODEL_STATUS.get("port")

    if DEBUG_FLAG: 
        print(f"正在启动模型: {model_name}, 子目录: {subdir_name}, 步长: {steps}, 主机: {host}, 端口: {port}")
    
    print(f"当前模型状态: {run_status}")  

    if "running" in run_status:
        return gr.update(value="已有模型在运行中，请勿重复启动")

    if "loading" in run_status or "copying" in run_status:
        return gr.update(value="已有模型在加载或复制中，请勿重复启动")

    if "finished" in run_status:
        return gr.update(value="已有模型运行完成，如需重新启动请先结束测试")
    
    if "error" in run_status:
        end_machine()
        return gr.update(value=run_status)

    promat = "Flatten and fold the short sleeve, advantage: 1"
    # 通过subdir_name来区分promath和其他模型，执行不同的命令
    if "baseline" in subdir_name:
        promat = "Flatten and fold the short sleeve"
    
    command_parser = {
        "model_path": CONFIG_DATA.get("model_path"),
        "shell_path": CONFIG_DATA.get("shell_path"),
        "host": host,
        "port": port,
        "promat": promat,
        "model_name": model_name,
        "subdir_name": subdir_name,
        "steps": steps
    }
    
    # 启动模型的命令，这里以一个示例命令为例
    # command = f"bash /home/lirui/Kai05-VLA/serve_policy_yaml.sh {model_path}/{model_name}/{subdir_name}/{steps} \"{promat}\" {port}"
    command = CONFIG_DATA.get("start_model_command").format_map(command_parser)
    MODEL_STATUS.update({
        "model_name": f'{subdir_name}_{steps}',
        "cmd": command,
        "run_status": "running",
        "host": host,
        "port": port
    })
    print("执行命令:", command)

    username = CONFIG_DATA.get("username")
    password = CONFIG_DATA.get("password")
    threading.Thread(
        target=ssh_execute_shell_with_progress,
        args=(host, username, password, MODEL_STATUS),
        daemon=True
    ).start()

    return gr.update(value="正在启动模型，请稍候...")
    

# 从本地nas获取模型列表，展示在下拉框中
def chooice_model():
    """
    该函数用于从本地NAS获取模型列表，并将模型列表展示在前端的下拉框中。可以根据需要修改获取模型列表的方式、处理方式和展示方式。
    返回值:
        - 包含模型列表的gr.update对象，用于更新前端下拉框的选项和可见性。
    处理流程:
        - 检查NAS是否挂载成功，如果成功则继续获取模型列表；如果失败则返回空列表并隐藏相关UI元素。
        - 从配置数据中获取模型路径，并列出模型目录下的子目录作为模型列表。
        - 更新前端下拉框的选项为获取到的模型列表，并设置默认选项为第一个模型（如果存在）。
    """
    if nas_select():
        if os.path.exists(CONFIG_DATA.get("model_path")):
            model_list = os.listdir(CONFIG_DATA.get("model_path"))
            log_append("success", f"从NAS获取模型列表成功，已展示模型")
            if DEBUG_FLAG:
                print("模型列表获取成功", model_list)
            return [gr.update(choices=model_list, value=model_list[0] if model_list else None), gr.update(visible=True), gr.update(visible=True)]
                        
    log_append("error", "模型列表获取失败，无法展示模型")
    return [gr.update(choices=[], value=None), gr.update(visible=False), gr.update(visible=False)]


# 模型选择后，获取二级目录列表展示在下拉框中
def chooice_model_2(model_copy, model_name):
    """
    该函数用于在模型选择后，从本地NAS获取对应模型的二级目录列表，并将二级目录列表展示在前端的下拉框中。可以根据需要修改获取二级目录列表的方式、处理方式和展示方式。
    参数:
        - model_copy: 当前选择的模型名称，用于判断是否需要更新二级目录列表。
        - model_name: 需要获取二级目录列表的模型名称。
    处理流程:
        - 从model_copy中提取出模型名称，并与当前选择的模型名称进行比较，判断是否需要更新二级目录列表。
        - 如果需要更新，则根据模型名称构建二级目录路径，并列出二级目录作为二级目录列表。
        - 更新前端下拉框的选项为获取到的二级目录列表，并设置默认选项为第一个二级目录（如果存在）。如果不需要更新，则保持当前二级目录列表不变。
    """
    global UPDATE_DROP
    model_path = os.path.join(CONFIG_DATA.get("model_path"), model_name)

    try:
        model_drop1 = model_copy.rsplit("_", 1)[0].split("_", 1)[1]
        # 如果当前选择的模型与自动匹配的模型不同，说明是手动选择了模型，需要更新二级目录和步长列表；如果相同则说明是自动匹配触发的，不需要更新二级目录和步长列表
        print(model_drop1, model_name)
        if model_name != model_drop1:
            UPDATE_DROP = True
        else:
            UPDATE_DROP = False
            print(f"chooice_model_2 {model_copy}，自动匹配触发，不需要更新二级目录和步长列表")
    except Exception as e:
        pass

    if os.path.exists(model_path):
        subdir_list = [d for d in os.listdir(model_path) if os.path.isdir(os.path.join(model_path, d))]
        if DEBUG_FLAG:
            print(f"chooice_model_2 {model_name}的二级目录列表获取成功", subdir_list)

        if UPDATE_DROP:        
            return [gr.update(value=""), gr.update(choices=subdir_list, value=subdir_list[0] if subdir_list else None)]
        else:
            print(f"chooice_model_2 {model_copy}，自动匹配，只需要更新二级目录列表")
            return [gr.update(), gr.update(choices=subdir_list)]
    
    return [gr.update(), gr.update(choices=[], value=None)]


# 二级目录选择后，获取步长列表展示在下拉框中
def chooice_steps(model_copy, model_name, subdir_name):
    """
    该函数用于在二级目录选择后，从本地NAS获取对应模型和二级目录的步长列表，并将步长列表展示在前端的下拉框中。可以根据需要修改获取步长列表的方式、处理方式和展示方式。
    参数:
        - model_copy: 当前选择的模型名称，用于判断是否需要更新步长列表。
        - model_name: 需要获取步长列表的模型名称。
        - subdir_name: 需要获取步长列表的二级目录名称。
    处理流程:
        - 从model_copy中提取出模型名称和二级目录名称，并与当前选择的模型名称和二级目录名称进行比较，判断是否需要更新步长列表。  
    """
    global UPDATE_DROP
    steps_path = os.path.join(CONFIG_DATA.get("model_path"), model_name, subdir_name)
    print(model_copy)
    
    try:
        model_drop2 = model_copy.rsplit("_", 1)[0]
        model_drop1 = model_drop2.split("_", 1)[1]
        # 如果当前选择的模型与自动匹配的模型不同，说明是手动选择了模型，需要更新二级目录和步长列表；如果相同则说明是自动匹配触发的，不需要更新二级目录和步长列表
        print(model_drop1, model_name)
        if model_name != model_drop1 or subdir_name != model_drop2:
            UPDATE_DROP = True
        else:
            UPDATE_DROP = False
            print(f"chooice_steps {model_copy}，自动匹配触发，不需要更新二级目录和步长列表")
    except Exception as e:
        pass

    if os.path.exists(steps_path):
        steps_list = [f for f in os.listdir(steps_path) if os.path.isdir(os.path.join(steps_path, f))]
        if DEBUG_FLAG:
            print(f"{model_name}/{subdir_name}的步长列表获取成功", steps_list)
    
        if UPDATE_DROP:
            return [gr.update(value=""), gr.update(choices=steps_list, value=steps_list[0] if steps_list else None)]
        else:
            print(f"chooice_steps {model_copy}，自动匹配，只需要更新步长列表")
            return [gr.update(), gr.update(choices=steps_list)]
    
    # print(f"{model_name}/{subdir_name}路径不存在，无法获取步长列表")
    return [gr.update(), gr.update(choices=[], value=None)]


# 步长发生改变时，判断是否需要更新二级目录和步长列表
def change_steps(model_copy, model_name, subdir_name, steps):
    """
    该函数用于在步长发生改变时，判断是否需要更新二级目录和步长列表。可以根据需要修改判断的方式和处理方式。
    参数:
        - model_copy: 当前选择的模型名称，用于判断是否需要更新二级目录和步长列表。
        - model_name: 需要判断的模型名称。
        - subdir_name: 需要判断的二级目录名称。
        - steps: 需要判断的步长信息。
    处理流程:
        - 从model_copy中提取出模型名称、二级目录名称和步长信息。
        - 将提取出的模型名称、二级目录名称和步长信息与当前选择的模型名称、二级目录名称和步长信息进行比较。
        - 如果当前选择的模型名称、二级目录名称或步长信息与提取出的信息不同
    """
    global UPDATE_DROP
    try:
        model_drop2 = model_copy.rsplit("_", 1)[0]
        model_drop1 = model_drop2.split("_", 1)[1]
        model_steps = model_copy.rsplit("_", 1)[1]
        print(model_drop1, model_name, subdir_name, model_drop2, steps, model_steps)
        if model_name != model_drop1 or subdir_name != model_drop2 or model_steps != steps:
            UPDATE_DROP = True
        else:
            UPDATE_DROP = False
            print(f"change_steps {model_copy}，自动匹配触发，不需要更新二级目录和步长列表")
    except Exception as e:
        pass

    if UPDATE_DROP:
        return gr.update(value="")
    else:
        return gr.update()


# 自动匹配实验
def auto_model(model_copy):
    """
    该函数用于根据当前选择的模型名称自动匹配对应的二级目录和步长信息，并将这些信息展示在前端的下拉框中。可以根据需要修改自动匹配的方式、处理方式和展示方式。
    参数:
        - model_copy: 当前选择的模型名称，用于提取模型名称、二级目录名称和步长信息，并与当前选择的模型名称进行比较以判断是否需要更新二级目录和步长列表。
    处理流程:
        - 从model_copy中提取出模型名称、二级目录名称和步长信息。
        - 将提取出的模型名称、二级目录名称和步长信息与当前选择的模型名称进行比较。
        - 如果当前选择的模型名称与提取出的模型名称不同，说明是手动选择了模型，需要更新二级目录和步长列表；如果相同则说明是自动匹配触发的，不需要更新二级目录和步长列表。
        - 根据提取出的模型名称、二级目录名称和步长信息获取对应的二级目录列表和步长列表，并更新前端的下拉框选项和默认选项。
    """
    if model_copy == "":
        return [gr.update(), gr.update(), gr.update()]
    else:
        try:
            model_copy = model_copy.strip()
            # 0228_vla_torch_flatten_fold_weitiao_v1_1991_baseline_50000
            model_drop2 = model_copy.rsplit("_", 1)[0]
            model_drop1 = model_drop2.split("_", 1)[1]
            steps = model_copy.rsplit("_", 1)[1]
            print(f"自动匹配结果: 模型={model_drop1}, 子目录={model_drop2}, 步长={steps}")
            chooice2 = chooice_model_2(model_copy, model_drop1)
            print("chooice2结果:", chooice2)
            chooice3 = chooice_steps(model_copy, model_drop1, model_drop2)
            print("chooice3结果:", chooice3)
            return [gr.update(value=model_drop1), gr.update(choices=chooice2[1]['choices'], value=model_drop2), gr.update(choices=chooice3[1]['choices'], value=steps)]
        except Exception as e:
            print(f"自动匹配失败，可能是模型命名不规范导致的，错误信息: {e}")
            return [gr.update(), gr.update(), gr.update()]


def number_detect(str):
    """
    该函数用于检测输入的字符串是否为整数。可以根据需要修改检测的方式和处理方式。
    参数:
        - str: 需要检测的字符串，表示需要判断是否为整数的输入内容。
    返回值:
        - True，如果输入的字符串是整数；否则False。
    处理流程:
        - 尝试将输入的字符串转换为整数，如果成功则返回True；如果转换失败则捕获ValueError异常并返回False。
    """
    try:
        print(str)
        str = int(str)
        return True
    except ValueError:
        return False
    

def host_change(host):
    """
    该函数用于处理SSH连接主机地址的改变，并根据输入的主机地址格式进行验证和更新。可以根据需要修改主机地址的验证方式、处理方式和返回值。
    参数:
        - host: 输入的主机地址，表示需要验证和更新的SSH连接主机地址。
    返回值:
        - 包含更新结果的gr.update对象，用于更新前端输入框的值和提示信息。
    处理流程:
        - 将输入的主机地址按照点号进行分割，得到一个字符串数组。
        - 使用number_detect函数检测字符串数组中的每个元素是否为整数，以验证主机地址的格式是否正确。
        - 如果主机地址格式正确，并且TELL_CHANGE标志为True，说明是用户修改了主机地址，需要更新MODEL_STATUS中的主机地址并返回修改成功的提示信息；如果TELL_CHANGE标志为False，说明是自动匹配触发的，不需要更新主机地址但需要将TELL_CHANGE标志置为True以便下次修改时能够正确识别。
        - 如果主机地址格式有误，说明输入的主机地址不符合要求，需要将TELL_CHANGE标志置为False，并返回格式有误的提示信息，同时将输入框的值重置为MODEL_STATUS中当前的主机地址。
    """
    global MODEL_STATUS, TELL_CHANGE
    host_array = host.split('.')
    print(host_array)
    number_detect_result = all(number_detect(a) for a in host_array)
    if number_detect_result:
        if TELL_CHANGE:
            MODEL_STATUS.update({"host": host})
            return gr.update(value="ssh连接主机地址修改成功"), gr.update(value=host)
        else:
            TELL_CHANGE = True
            return gr.update(), gr.update()
    else:
        TELL_CHANGE = False
        return gr.update(value="主机地址格式有误，已重置"), gr.update(value=MODEL_STATUS.get('host'))


def port_change(port):
    """
    该函数用于处理SSH连接端口号的改变，并验证输入的端口号是否为整数。可以根据需要修改端口号的验证方式、处理方式和返回值。
    参数:
        - port: 输入的端口号，表示需要验证和更新的SSH连接端口号。
    返回值:
        - 包含更新结果的gr.update对象，用于更新前端输入框的值和提示信息。
    处理流程:
        - 使用number_detect函数检测输入的端口号是否为整数，以验证端口号的格式是否正确。
        - 如果端口号格式正确，并且TELL_CHANGE标志为True，说明是用户修改了端口号，需要更新MODEL_STATUS中的端口号并返回修改成功的提示信息；如果TELL_CHANGE标志为False，说明是自动匹配触发的，不需要更新端口号但需要将TELL_CHANGE标志置为True以便下次修改时能够正确识别。
        - 如果端口号格式有误，说明输入的端口号不符合要求，需要将TELL_CHANGE标志置为False，并返回格式有误的提示信息，同时将输入框的值重置为MODEL_STATUS中当前的端口号。
    """
    global MODEL_STATUS, TELL_CHANGE
    number_detect_result = number_detect(port)
    if number_detect_result:
        if TELL_CHANGE:
            MODEL_STATUS.update({"port": port})
            return gr.update(value="端口号修改成功"), gr.update(value=port)
        else:
            TELL_CHANGE = True
            return gr.update(), gr.update()
    else:
        TELL_CHANGE = False
        return gr.update(value="端口号必须为整数，已重置"), gr.update(value=MODEL_STATUS.get('port'))
        

# 查看日志
def see_log():
    """
    该函数用于查看日志文件，并通过系统默认的文本编辑器打开日志文件。可以根据需要修改日志文件的路径、打开方式和错误处理方式。
    处理流程:
        - 构建日志文件的路径，通常根据当前日期和机器ID来命名日志文件，以便区分不同日期和不同机器的日志。
        - 检查日志文件是否存在，如果不存在则创建日志文件并记录日志文件创建成功的信息；如果存在则直接打开日志文件。
        - 使用subprocess模块调用系统命令来打开日志文件，通常使用xdg-open命令在Linux系统中打开文件。如果打开日志文件失败，则捕获异常并记录错误信息。
    """
    log_path = os.path.join(WORKING_DIR, "logs", f"{CONFIG_DATA.get('machine_id')}_test_{formatted_date}.log")
    if not os.path.exists(log_path):
        print("日志文件不存在，正在创建...")
        os.makedirs(os.path.dirname(log_path), exist_ok=True)
        logger.info("日志文件不存在，已创建新日志文件")
        return 
    
    try:
        subprocess.Popen(f"xdg-open {log_path}", shell=True)
    except Exception as e:
        print(f"打开日志文件失败: {e}")
        logger.error(f"打开日志文件失败: {e}")


# 启动工控机
def start_machine():
    """
    该函数用于启动工控机，并通过subprocess模块执行启动命令，同时实时监听命令输出以捕获启动状态和日志信息。可以根据需要修改启动命令、处理方式和返回值。
    处理流程:
        - 检查工控机是否已经启动，如果已经启动则返回提示信息；如果未启动则继续执行启动流程。
        - 遍历COMMANDS_STATUS列表中的每个命令，检查命令的process属性是否为None，如果为None则说明该命令需要启动。
        - 对于需要启动的命令，使用subprocess.Popen来执行命令，并将返回的进程对象存储在COMMANDS_STATUS中对应的process属性中。
        - 启动命令后，使用线程来实时监听命令的输出，以捕获启动状态和日志信息，并根据监听结果更新COMMANDS_STATUS中的相关信息。
        - 返回提示信息，告知用户工控机正在启动中。
    """
    global COMMANDS_STATUS
    # 判断工控机是否已经启动，未启动则启动，已启动则提示正在运行中
    if any(command['process'] is not None for command in COMMANDS_STATUS[1:]):
        # return gr.update(value="工控机未启动，正在启动...")
    # else:
        return gr.update(value="工控机已在运行中...")

    for i in range(len(COMMANDS_STATUS)):
        print(f"启动{i}——{COMMANDS_STATUS[i]['name']}")
        # 不为None并且命令不为空字符串，说明需要启动
        # 由于can口激活只需要执行一次并激活后不为None但不需要重复执行，所以alive为False说明未启动，且监听时can口结束alive不会被置为False
        if COMMANDS_STATUS[i]['process'] is None:
            print("开始启动")
            if i != 0:
                time.sleep(1.5)
          
            process = subprocess.Popen(
                COMMANDS_STATUS[i]['cmd'], 
                shell=True,
                stdout=subprocess.PIPE if i != 3 else subprocess.DEVNULL,
                stderr=subprocess.PIPE if i != 3 else subprocess.DEVNULL,
                text=True
            )
            
            if process.poll() is None:
                print("启动成功")
                COMMANDS_STATUS[i].update({
                    "pid": process.pid,
                    "process": process,
                })
                # 启动进程后对此进程进行实时监听
                monitor_thread = threading.Thread(target=monitor_process, args=(COMMANDS_STATUS[i],), daemon=True)
                monitor_thread.start()

    return gr.update(value="工控机启动中，请稍候...")


# 更新模型启动进度
def update_progress(last_percentage):
    """
    该函数用于更新模型启动的进度信息，并根据模型的运行状态来控制前端UI元素的显示和更新。可以根据需要修改进度信息的提取方式、UI元素的更新方式和返回值。
    参数:
        - last_percentage: 上一次更新的进度百分比，用于判断是否需要更新
    返回值:
        - 包含UI更新信息的列表，用于更新前端UI元素的显示和内容。
    处理流程:
        - 从MODEL_STATUS中获取当前的进度信息、运行状态和上一次的运行状态。
        - 使用正则表达式从进度信息中提取出当前的进度百分比，并将其转换为整数类型。
        - 根据当前的运行状态和进度百分比来判断是否需要更新UI元素的显示和内容。如果模型正在复制中且进度百分比发生变化，则显示进度条并更新进度值；如果模型运行状态发生变化，则更新显示内容为当前的运行状态；如果模型运行完成，则更新TEST_STATUS中的模型名称和端口信息。
        - 返回包含UI更新信息的列表，用于更新前端UI元素的显示和内容。
    """
    global TEST_STATUS, MODEL_STATUS
    if DEBUG_FLAG:
        print("*"*20, "update_progress被触发", "*"*20)
    progress = MODEL_STATUS.get("progress")
    run_status = MODEL_STATUS.get("run_status")
    last_status = MODEL_STATUS.get("last_status")
    percentage = 0
    gr_update = []
    if progress:
        progress_match = re.search(r'(\d+)%', progress)
        if progress_match:
            percentage = int(progress_match.group(1))
        
    if "finished" in run_status:
        TEST_STATUS.update({
            "model_name": MODEL_STATUS.get("model_name"),
            "port": MODEL_STATUS.get("port")
        })

    if percentage >= 0 and percentage <= 100 and "copying" in run_status and percentage != last_percentage:
        gr_update.append(gr.update(visible=True, value=percentage))
    else:
        gr_update.append(gr.update(visible=False))

    if run_status != last_status:
        # MODEL_STATUS["last_status"] = run_status
        MODEL_STATUS.update({"last_status": run_status})
        if "running" in run_status:
            gr_update.append(gr.update())
        else:
            gr_update.append(gr.update(value=run_status))
    else:
        gr_update.append(gr.update())
    
    return gr_update


# 判断函数
def status_check(num):
    """
    该函数用于检查当前系统的状态，以判断是否满足进行某些操作的条件。可以根据需要修改检查的条件、处理方式和返回值。
    参数:
        - num: 检查的级别，表示需要检查的条件的数量和优先级，数值越大表示需要检查的条件越多且优先级越高。
    返回值:
        - 如果检查不通过，则返回相应的提示信息；如果检查通过，则返回-1表示可以继续进行后续操作。
    处理流程:
        - 根据输入的num值，依次检查工控机启动状态、从臂状态、测试状态、模型启动状态等条件。
        - 如果某个条件不满足，则返回相应的提示信息，告知用户需要先满足该条件才能进行后续操作。
        - 如果所有条件都满足，则返回-1表示可以继续进行后续操作。
    """
    # 判断工控机启动
    if num > 0:
        if not all(c['alive'] for c in COMMANDS_STATUS):
            return "请先启动工控机或等待工控机启动完毕"
    # 判断从臂状态
    if num > 1:
        if TEST_STATUS['arm_status'] == '正在复位':
            return "从臂正在复位，请稍后"
    # 判断测试状态
    if num > 2:
        if TEST_STATUS["alive"] or TEST_STATUS["start_test"]:
            return "当前正在测试，请先结束/中断测试后再复位"        
    # 判断模型是否启动
    if num > 3:
        run_status = MODEL_STATUS['run_status']
        if run_status in ["", "未启动"]:
            return "请先启动模型再进行测试"
    if num > 4:
        if TEST_STATUS['arm_status'] != '推理点':
            return "请先将从臂复位到推理点再进行测试"
        
    return -1
        

# 开始测试
def start_test():
    """
    该函数用于开始测试流程，并通过subprocess模块执行测试命令，同时实时监听测试命令的输出以捕获测试状态和日志信息。可以根据需要修改测试命令、处理方式和返回值。
    处理流程:
        - 检查当前的系统状态，判断是否满足进行测试的条件。如果不满足条件，则返回相应的提示信息；如果满足条件，则继续执行测试流程。
        - 根据配置数据中的测试命令模板，构建实际的测试命令，并使用subprocess.Popen来执行测试命令，并将返回的进程对象存储在TEST_STATUS中。
        - 启动测试命令后，使用线程来实时监听测试命令的输出，以捕获测试状态和日志信息，并根据监听结果更新TEST_STATUS中的相关信息。
        - 返回提示信息，告知用户测试已经开始。
    """
    global TEST_STATUS

    status = status_check(5)
    if status != -1:
        return gr.update(value=status)

    TEST_STATUS["start_test"] = True
    start_command = CONFIG_DATA.get("start_command").format_map(MODEL_STATUS)
    print('命令为', start_command)
    start_command_process = subprocess.Popen(start_command, shell=True)
    TEST_STATUS.update({
        "pid": start_command_process.pid,
        "process": start_command_process,
        "alive": True,
        "start_test": True,
        "arm_status": "未复位"
    })

    # time_out_process = subprocess.Popen(CONFIG_DATA.get("time_out_command"), shell=True)
    # if time_out_process.poll() is None:
    #     threading.Thread(target=monitor_timeout, args=(time_out_process, TEST_STATUS), daemon=True).start()

    # if start_command_process.poll() is None:  # 进程仍在运行
    #     # 启动进程后对此进程进行实时监听
    #     start_command = threading.Thread(target=monitor_process, args=(TEST_STATUS,), daemon=True)
    #     start_command.start()
    return gr.update(value="开始测试")
        

# 中断测试
def stop_test():
    """
    该函数用于中断正在进行的测试流程，并通过subprocess模块终止测试命令的执行，同时更新TEST_STATUS中的相关信息。可以根据需要修改中断的方式、处理方式和返回值。
    处理流程:
        - 检查当前的测试状态，判断是否有测试正在进行中。如果没有测试正在进行中，则返回提示信息；如果有测试正在进行中，则继续执行中断流程。
        - 将TEST_STATUS中的测试标志位置为False，以避免在结束测试流程中被误判为正在测试。
        - 如果TEST_STATUS中的process属性不为None，说明有测试命令正在执行，则使用terminate方法来终止测试命令的执行，并使用subprocess.run来发送SIGINT信号以确保测试命令被正确中断。
    """
    global TEST_STATUS
    # 如果正在测试中
    if TEST_STATUS["alive"] or TEST_STATUS["start_test"]:
        # 先将测试标志位置为False，避免在结束测试流程中被误判为正在测试
        if TEST_STATUS["process"] is not None:
            TEST_STATUS["process"].terminate()
            subprocess.run(f"kill -SIGINT {TEST_STATUS['pid']}", shell=True)
            TEST_STATUS.update({
                "pid": -1,
                "process": None,
                "alive": False,
                "start_test": False
            })
            return gr.update(value="测试已中断")
    else:
        return gr.update(value="未在测试中，无需中断")
    

# 结束测试
def end_machine():
    """
    该函数用于结束正在进行的测试流程，并通过subprocess模块终止测试命令的执行，同时关闭SSH连接并更新MODEL_STATUS和TEST_STATUS中的相关信息。可以根据需要修改结束的方式、处理方式和返回值。
    处理流程:
        - 调用stop_test函数来中断正在进行的测试流程，以确保测试命令被正确中断。
        - 获取MODEL_STATUS中的ssh_client对象，并调用close方法来关闭SSH连接，以释放相关资源。
        - 更新MODEL_STATUS中的模型名称、进度、输出、命令、运行状态和上一次的运行状态等信息，重置为初始状态。
        - 返回提示信息，告知用户测试已经结束。
    """
    global MODEL_STATUS
    # 判断是否在测试中
    try:
        stop_test()
        ssh_client = MODEL_STATUS.get("ssh_client")
        ssh_client.close()
        # 结束模型运行
        MODEL_STATUS.update({
            "model_name": "未启动",
            "progress": 0,
            "output": "",
            "cmd": "",
            "run_status": "",
            "last_status": "",
        })
    except Exception as e:
        pass
    return gr.update(value="已经结束测试啦")
    

# 从臂复位
def reset(value):
    """
    该函数用于执行从臂复位的操作，并通过subprocess模块调用复位命令，同时更新TEST_STATUS中的相关信息以反映复位状态。可以根据需要修改复位命令、处理方式和返回值。
    参数:
        - value: 复位的目标位置，表示需要将从臂复位到哪个位置，例如推理点、初始点等。
    返回值:
        - 包含更新信息的列表，用于更新前端UI元素的显示和内容。
    处理流程:
         - 检查当前的系统状态，判断是否满足进行复位的条件。如果不满足条件，则返回相应的提示信息；如果满足条件，则继续执行复位流程。
        - 更新TEST_STATUS中的arm_status为正在复位，以反映当前的复位状态。
        - 根据配置数据中的复位命令模板，构建实际的复位命令，并使用subprocess.Popen来执行复位命令，并将返回的进程对象存储在reset_process变量中。
        - 如果复位命令成功启动，则使用线程来实时监听复位命令的输出，以捕获复位状态和日志信息，并根据监听结果更新TEST_STATUS中的相关信息；如果复位命令启动失败，则更新TEST_STATUS中的arm_status为未复位，并返回复位失败的提示信息。
        - 返回包含更新信息的列表，用于更新前端UI元素的显示和内容。
    """
    global TEST_STATUS
    gr_update = [gr.update(visible=False), gr.update(visible=False), gr.update(visible=True)]
    
    status = status_check(3)
    if status != -1:
        gr_update.append(gr.update(value=status))
        return gr_update
    
    # 优先修改状态，底下如果失败再改回
    TEST_STATUS.update({"arm_status": "正在复位"})

    # 执行复位脚本
    reset_process = subprocess.Popen(
        f"{CONFIG_DATA.get('reset_command')} {value}",
        shell=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True
    )

    # 复位脚本启动
    if reset_process.poll() is None:
        # 脚本启动通过线程修改状态
        threading.Thread(target=monitor_resets, args=(TEST_STATUS, reset_process, value), daemon=True).start()
        gr_update.append(gr.update(value="正在复位，请稍候..."))
    else:
        # 脚本启动失败，修改状态并提示
        TEST_STATUS.update({"arm_status": "未复位"})
        gr_update.append(gr.update(value="复位失败，请检查复位脚本和从臂状态"))

    return gr_update


# 初始化函数，加载配置文件，配置日志记录器，获取命令状态等
def init():
    """
    该函数用于初始化系统的配置和状态，包括加载配置文件、配置日志记录器、获取命令状态等操作。可以根据需要修改初始化的内容、处理方式和返回值。
     处理流程:
        - 加载配置文件，获取模型列表等初始化操作。
        - 配置日志记录器，根据配置文件中的机器ID来命名日志文件，并设置日志级别为DEBUG或INFO。
        - 获取命令状态，根据配置文件中的命令列表和命令状态列表来构建COMMANDS_STATUS列表，包含每个命令的索引、名称、命令内容、进程ID、进程对象、运行状态、上一次的位置和监听状态等信息。
        - 返回提示信息，告知用户初始化完成。
    """
    global THIS_LOG, COMMANDS_STATUS, logger, MODEL_STATUS, CONFIG_DATA
    # 加载配置文件，获取模型列表等初始化操作
    CONFIG_DATA = config_upload_byNet()
    MODEL_STATUS.update({
        "host": CONFIG_DATA.get("host"),
        "port": CONFIG_DATA.get("port")
    })
    machine_id = CONFIG_DATA.get("machine_id")
    if machine_id:
        # 配置日志
        logger = setup_logger(
            name=f"{CONFIG_DATA.get('machine_id')}_test_{formatted_date}",
            log_dir=os.path.join(WORKING_DIR, "logs", ),
            level=logging.DEBUG if DEBUG_FLAG else logging.INFO
        )
        log_append("success", "日志记录器初始化成功")
    else:
        log_append("error", "配置文件加载失败，未找到machine_id，无法初始化日志记录器")

    # 机器初始化，获取命令状态等
    Configuration_command = CONFIG_DATA.get("Configuration_command", {})
    Configuration_command_status = CONFIG_DATA.get("Configuration_command_status", {})
    
    if Configuration_command and Configuration_command_status:
        if DEBUG_FLAG:
            print("*"*20, Configuration_command, "*"*20)
        Configuration_command = [item.replace("{WORKING_DIR}", WORKING_DIR) if "{WORKING_DIR}" in item else item for item in Configuration_command]
        COMMANDS_STATUS = [{
            "index": i,
            "name": Configuration_command_status[i],
            "cmd": item.replace("{WORKING_DIR}", WORKING_DIR) if "{WORKING_DIR}" in item else item,
            "pid": -1,
            "process": None,
            "alive": False,
            "state": "未启动",
            "last_position": None,
            "listen": True if i in LISTEN_INDICES else False
        } for i, item in enumerate(Configuration_command)]
        if DEBUG_FLAG:
            print("命令状态获取成功:", json.dumps(COMMANDS_STATUS, indent=2, ensure_ascii=False))
        log_append("success", "机器初始化完成！")
    else: 
        print("未获取到命令状态，请查看配置文件是否正确")
        log_append("error", "加载配置失败，未获取到命令状态，请查看配置文件是否正确")

    
if __name__ == "__main__":
    # 初始化配置
    init()
    # 挂载NAS
    log_append(*nas_upload())
    
    # 加载css规则
    css = open(os.path.join(os.path.dirname(__file__), "static", "kai05_deployee.css")).read()
    # js = open(os.path.join(os.path.dirname(__file__), "static", "kai05_deployee.js")).read()
    # 加载指示灯html
    lamp_html = ''.join([f'<div class="lamp-container"><div class="lamp" id="lamp_dot_{i}"></div><div id="lamp_text_{i}">{COMMANDS_STATUS[i]["name"]}</div><div id="lamp_pid_{i}"></div></div>' for i in range(6)])
    if DEBUG_FLAG:
        print('lamp_html:', lamp_html)
    with gr.Blocks(title="模型测试界面", css=css) as demo:
        # 选择模型弹窗
        model_popup = gr.Column(
            visible=False,
            elem_id="model_popup"
        )

        with model_popup:
            # 自动适配
            auto_model_textbox = gr.Textbox(
                placeholder="您可直接将测试列表中的模型（实验）名称复制到此，会自动匹配对应模型",
                label="自动匹配（选填）",
                interactive=True,
                elem_classes="model_dropdown"
            )
            # 一级目录模型选择
            model_dropdown = gr.Dropdown(
                choices=["模型A", "模型B", "模型C"],
                label="选择模型",
                interactive=True,
                elem_classes="model_dropdown"
            )
            # 二级目录模型选择
            model_dropdown2 = gr.Dropdown(
                choices=[],
                label="选择二级目录",
                interactive=True,
                elem_classes="model_dropdown"
            )
            # 步长选择
            steps = gr.Dropdown(
                choices=[],
                label="选择步长",
                interactive=True,
                elem_classes="model_dropdown"
            )
            # 上位机ip
            host = gr.Textbox(
                value=MODEL_STATUS.get("host"),
                label="输入SSH连接的主机地址",
                interactive=True,
                elem_classes="model_dropdown"
            )
            # 上位机端口
            port = gr.Textbox(
                value=MODEL_STATUS.get("port"),
                label="输入上位机启动模型的端口号",
                interactive=True,
                elem_classes="model_dropdown"
            )
            # 模型下载进度  nas -> 本地
            model_slider = gr.Slider(minimum=0, maximum=100, value=0, label="启动模型进度", interactive=False, elem_id="model_progress", visible=False)
            with gr.Row():
                start_model_btn = gr.Button("启动模型", elem_id="start_model_btn")
                model_popup_close_btn = gr.Button("关闭页面", elem_id="model_popup_close_btn")
        
        with gr.Column(elem_id="container"):
            gr.Row(
                gr.HTML("""
                    <div class="header">
                        <h1>Kai05 模型测试页面</h1>
                        <h5>
                            当前机器编号:agilex-001&nbsp;&nbsp;&nbsp;&nbsp;
                            当前上位机IP:未获取&nbsp;&nbsp;&nbsp;&nbsp;
                            当前上位机端口:未获取&nbsp;&nbsp;&nbsp;&nbsp;
                        </h5>
                        <h5>
                            当前测试模型:未启动&nbsp;&nbsp;&nbsp;&nbsp;
                            当前机器状态:未启动&nbsp;&nbsp;&nbsp;&nbsp;
                        </h5>
                    </div>
                """)
            )
            gr.Row(
                gr.HTML("""
                    <div class="camera">
                        <div class="camera-box">左臂画面<img id="cam_l"></div>
                        <div class="camera-box">中间画面<img id="cam_f"></div>
                        <div class="camera-box">右臂画面<img id="cam_r"></div>
                    </div>
                """)
            )
            with gr.Row(elem_id="content"):
                gr.HTML("""
                    <div class="instruct-text">
                        <span>
                        </span>
                    </div>
                """)
                with gr.Column(elem_id="instruct"):
                        gr.HTML(f"""
                            <div class="instruct-lamp">
                                {lamp_html}
                            </div>
                        """),
                        with gr.Row(elem_id="operate"):
                            start_test_btn = gr.Button("开始测试", elem_id="start_test_btn")
                            stop_test_btn = gr.Button("中断测试", elem_id="stop_test_btn")
                            end_machine_btn = gr.Button("结束测试", elem_id="end_machine_btn")
                            reset_btn = gr.Button("从臂复位", elem_id="reset_btn")
                            reset1 = gr.Button("推理点", elem_id="reset1", visible=False)
                            reset0 = gr.Button("零点", elem_id="reset0", visible=False)

        with gr.Row():
                see_log_btn = gr.Button("查看日志", elem_id="see_log_btn")
                chooice_model_btn = gr.Button("选择模型", elem_id="chooice_model_btn")
                start_machine_btn = gr.Button("启动工控机", elem_id="start_machine_btn")

        # 操作提示弹窗
        op_tips = gr.Textbox(visible=True, interactive=False, elem_id="op_tips", label=None, container=False, lines=1, max_lines=1)
        # 选择模型蒙板
        model_mask = gr.Column(visible=False, elem_id="model_mask")
        # 数据状态隐藏组件
        status_data_store = gr.Textbox(visible=False)
        model_start_progress = gr.Textbox(visible=False)
        
        # 自动更新UI并打印状态
        demo.load(
            fn=lambda: update_ui(COMMANDS_STATUS, THIS_LOG, TEST_STATUS, MODEL_STATUS),
            inputs=[],
            outputs=[status_data_store, model_start_progress],
            every=1 / 10  # 每0.1秒更新一次
        )

        # 状态变化时触发前端打印
        status_data_store.change(
            fn=None,
            inputs=[status_data_store],   # ← 注意是 list
            outputs=None,
            js=update_indicator_js(f'{WORKING_DIR}/static/update_indicator.js')
        )

        # 模型数据发生变化时
        model_start_progress.change(
            update_progress,
            inputs=[model_slider],
            outputs=[model_slider, op_tips]
        )

        # 提示弹窗发生变化时
        op_tips.change(
            show_tips_then_hide,
            outputs=op_tips,
            inputs=op_tips
        )

        # 开始测试
        start_test_btn.click(
            start_test,
            inputs=[],
            outputs=[op_tips]
        )

        # 中断测试
        stop_test_btn.click(
            stop_test,
            inputs=[],
            outputs=[op_tips]
        )

        # 结束测试
        end_machine_btn.click(
            end_machine,
            inputs=[],
            outputs=[op_tips]
        )

        # 从臂复位
        reset_btn.click(
            lambda: [gr.update(visible=True), gr.update(visible=True), gr.update(visible=False)],
            inputs=[],
            outputs=[reset1, reset0, reset_btn]
        )

        # 从臂复位 推理点
        reset1.click(
            lambda: reset(1),
            inputs=[],
            outputs=[reset1, reset0, reset_btn, op_tips]
        )

        # 从臂复位 零点
        reset0.click(
            lambda: reset(0),
            inputs=[],
            outputs=[reset1, reset0, reset_btn, op_tips]
        )

        # 查看日志
        see_log_btn.click(
            see_log,
            inputs=[],
            outputs=[]
        )

        # 选择模型
        chooice_model_btn.click(
            chooice_model,
            outputs=[model_dropdown, model_popup, model_mask]
        )
        
        # 启动工控机
        start_machine_btn.click(
            start_machine,
            inputs=[],
            outputs=[op_tips]
        )

        # 模型选择 -> 关闭
        model_popup_close_btn.click(
            lambda: [gr.update(visible=False), gr.update(visible=False)],
            outputs=[model_popup, model_mask]
        )

        # 模型选择 -> 启动
        start_model_btn.click(
            start_model,
            inputs=[model_dropdown, model_dropdown2, steps],
            outputs=[op_tips]
        )
        
        # 自动适配框发生变化时
        auto_model_textbox.change(
            auto_model,
            inputs=[auto_model_textbox],
            outputs=[model_dropdown, model_dropdown2, steps]
        )

        # 一级目录发生变化时
        model_dropdown.change(
            chooice_model_2,
            inputs=[auto_model_textbox, model_dropdown],
            outputs=[auto_model_textbox, model_dropdown2, ]
        )

        # 二级目录发生变化时
        model_dropdown2.change(
            chooice_steps,
            inputs=[auto_model_textbox, model_dropdown, model_dropdown2],
            outputs=[auto_model_textbox, steps, ]
        )

        # 步长发生变化时
        steps.change(
            change_steps,
            inputs=[auto_model_textbox, model_dropdown, model_dropdown2, steps],
            outputs=[auto_model_textbox]
        )

        # 上位机发生变化时
        host.change(
            host_change,
            inputs=[host],
            outputs=[op_tips, host]
        )

        # 端口发生变化时
        port.change(
            port_change,
            inputs=[port],
            outputs=[op_tips, port]
        )
    # 启动Gradio服务
    demo.queue().launch(
        server_port=7860,
        server_name="0.0.0.0",
        inbrowser=True,
        max_threads=4
    )