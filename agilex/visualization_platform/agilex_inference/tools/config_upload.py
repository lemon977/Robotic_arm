import os
import json
import sys
import socket
# 获取当前文件的目录（pyUtil/）
current_dir = os.path.dirname(os.path.abspath(__file__))
# 获取上级目录（即包含pyUtil的目录）
parent_dir = os.path.dirname(current_dir)
# 将上级目录加入Python的模块搜索路径
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)

from tools.host_data_manager import (
    get_id,
    get_command
)

def config_upload():
    """
    该函数用于加载本地配置文件config.json，并返回配置数据的字典。可以根据需要修改配置文件的路径和处理方式。
    返回值:
        - 包含配置数据的字典，如果加载成功；否则返回一个空字典
    """
    config_data = {}
    # 加载配置文件，获取模型列表等初始化操作
    try:
        with open('./config/config.json', 'r') as f:
            config_data = json.load(f)
            print("配置文件加载成功")
            # 这里可以根据需要对config进行处理，例如提取模型列表等
    except Exception as e:
        print(str(e))
        print("加载配置文件失败，请检查config.json是否存在且格式正确")
    return config_data

def config_upload_byNet():
    """
    该函数通过网络获取配置数据，首先获取本机内网IP地址，然后通过与服务器交互获取机器ID和命令数据，并将这些数据保存到本地配置文件config.json中。可以根据需要修改服务器交互的方式、参数和数据处理方式。
    返回值:
        - 包含配置数据的字典，如果获取成功；否则返回一个空字典
    """
    config_data = {}
    try:
        # 通过远程获取任务
        machine_ip = get_local_ip()
        last_one_ip = machine_ip.split('.')[-1]
        # last_one_ip = int(219)
        res_data = get_id(last_one_ip)
        if res_data != None:
            config_data.update({
                data_key: res_data.get(data_key, "") for data_key in res_data
            })
        
        machine_id = config_data.get('machine_id')
        res_data2 = get_command(machine_id)
        if res_data2 != None:
            config_data.update({
                data_key: res_data2.get(data_key, "") for data_key in res_data2
            })
            with open('./config/config.json', 'w') as f:
                f.write(json.dumps(config_data, indent=2, ensure_ascii=False))
    except Exception as e:
        # 发生错误时, 使用离线配置文件
        config_data = config_upload()

    return config_data


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


if __name__ == "__main__":
    config = config_upload_byNet()
    print(json.dumps(config, indent=4, ensure_ascii=False))