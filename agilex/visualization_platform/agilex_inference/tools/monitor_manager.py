import datetime
import time
import subprocess
import re

def monitor_process(command=None):
    """
    该函数用于监控指定的命令进程，定期检查进程状态并更新相关信息。可以根据需要修改监控的方式、频率和处理方式。
        - command: 包含进程信息的字典，例如{'name': '测试程序', 'pid': 12345, 'process': <subprocess.Popen对象>, 'alive': True}。
    监控内容:
        - 定期检查进程是否存活，并更新alive状态。
        - 如果是特定的命令（例如Can口），监控其输出以确定是否成功激活。
        - 如果是机械臂相关的命令，监控其输出以判断机械臂状态（零点、浮动、移动）。
    """
    if command is not None:
        process = command.get('process')
        can_flag = False
        index = command.get('index')
        while True:        
            # 如果输出为None，并且说明进程存活
            if process.poll() is None:
                # with status_lock:
                # 获取当前PID存活状态
                is_alive = check_pid_alive(command["pid"])
                # 读取上一次存活状态
                old_alive = command["alive"]
                # print(command.get('name'), is_alive, old_alive)
                # print
                # 如果两次状态不同，说明状态变化并将上一次状态更新
                if is_alive != old_alive:
                    command["alive"] = is_alive
                    print(f"🔄 脚本状态变化: index={index} {command.get('name')} PID={command.get('pid')} 状态 {old_alive} → {is_alive}")
                
                if index == 0:
                    for line in process.stdout:
                        # print("can口激活", line)
                        if "所有 CAN 接口已成功重命名并激活。" in line:
                            can_flag = True

                if index in [4, 5]:  # 机械臂状态监控
                    for line in process.stdout:
                        if line:
                            line = line.strip()
                            pos_str = extract_position_array(line)
                            if not pos_str:
                                continue
                            curr_pos = parse_position(pos_str)
                            if not curr_pos:
                                continue
                            
                            last_pos = command.get("last_position")
                            if all(abs(v) < 1e-6 for v in curr_pos):
                                command["state"] = "zero"
                            elif last_pos and is_small_floating(last_pos, curr_pos, 0.02):
                                command["state"] = "floating"
                            else:
                                command["state"] = "moving"

                            # 更新last_position
                            command["last_position"] = curr_pos
                # time.sleep(1/10)
            else:
                print(f"⚠️ 监控到进程退出: index={index} {command.get('name')} PID={command.get('pid')}")
                # 进程已结束
                if command.get("name") == 'Can口' and can_flag:
                    command.update({
                        "pid": -1,
                        "process": None,
                        "alive": True
                    })
                else:
                    command.update({
                        "pid": -1,
                        "process": None,
                        "alive": False
                    })
                break
            time.sleep(1/5)


# 检测从臂复位状态函数
def monitor_resets(TEST_STATUS, process, point):
    """
    监控机械臂复位进程的状态，并根据进程状态更新TEST_STATUS中的arm_status字段。可以根据需要修改监控的方式、频率和处理方式。
        - TEST_STATUS: 包含测试状态的字典，包含测试的相关信息，例如{"arm_status": "未知", ...}。
        - process: 包含复位进程信息的subprocess.Popen对象，用于监控进程状态。
        - point: 表示当前测试点的整数，例如0表示零点，1表示推理点等。
    监控内容:
        - 定期检查复位进程是否存活。
        - 如果复位进程退出，根据point参数更新TEST_STATUS中的arm_status字段，例如"零点"或"推理点"。
    """
    while True:
        if process.poll() is None:
            pass
        else:
            TEST_STATUS.update({
                "arm_status": "零点" if point == 0 else "推理点",
            })
            print("⚠️ 监控到复位进程退出")
            break
        # 读取复位进程输出
        time.sleep(1/5)


# 检测三分钟函数
def monitor_time(process, TEST_STATUS):
    """
    暂停三分钟，并监控指定进程的状态。如果进程在三分钟内退出，打印相关信息；如果三分钟后进程仍然存活，打印超时信息。可以根据需要修改监控的方式、频率和处理方式。
        - process: 包含需要监控的进程信息的subprocess.Popen对象，用于监控进程状态。
        - TEST_STATUS: 包含测试状态的字典，包含测试的相关信息，例如{"time_status": "未知", ...}。
    监控内容:
        - 暂停三分钟，并定期检查进程是否存活。
        - 如果进程在三分钟内退出，打印相关信息并更新TEST_STATUS中的time_status字段为"已退出"。
        - 如果三分钟后进程仍然存活，打印超时信息并更新TEST_STATUS中的time_status字段为"超时未退出
    """
    """ ！！！暂未启用！！！  """
    time_num = 0
    while True:
        if process.poll() is None:
            for line in process.stdout:
                print(line.strip())
                time_num = line.strip()
        else:
            print(f"测试程序退出，时间为{time_num}")
            break
        time.sleep(1)


# ========= rostopic echo 输出解析函数 ==========
def extract_position_array(line: str):
    """
    从一行 rostopic echo 输出中提取 position: [...] 内容
    :param line: rostopic echo 输出行
    :return: position 数组字符串，或 None 如果未找到
    """
    line = line.strip()
    if "position" not in line:
        return None

    # 匹配 position: [ ... ]
    match = re.search(r"position:\s*\[([^\]]+)\]", line)
    if not match:
        return None

    return match.group(1)

# ========= 机械臂浮动判断函数 ==========
def parse_position(pos_str: str):
    """
    将 position 字符串解析为浮点数列表
    :param pos_str: 形如 "0.12, -0.34,
    """
    try:
        return [float(x.strip()) for x in pos_str.split(",")]
    except Exception:
        return None


# ======== 机械臂浮动判断函数，微小浮动 ========== 
def is_small_floating(last, curr, eps):
    """
    判断机械臂是否处于微小浮动状态
    :param last: 上一次位置列表
    :param curr: 当前位位置列表
    :param eps: 浮动阈值
    """
    if not last or not curr or len(last) != len(curr):
        return False

    max_diff = max(abs(c - l) for c, l in zip(curr, last))
    return max_diff < eps


# ========= 检测 PID 是否存活函数 ==========
def check_pid_alive(pid: int) -> bool:
    """
    检测指定 PID 是否存活
    :param pid: 进程 PID
    :return: True 如果存活，False 否则
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