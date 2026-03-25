import json
import requests

url = 'http://192.168.9.50:5001'
task_type = "collect"
brand = "agilex"
universal = f"brand={brand}&task_type={task_type}"

# 和服务器交互
def speak(method, api="management_logger", parameter=None, data=None):
    """
    该函数用于与服务器进行交互，支持GET和POST两种请求方式。根据传入的参数构建请求URL或请求体，并发送请求到服务器的指定API接口。函数会处理服务器的响应，如果响应状态码为200，则返回响应中的数据或消息；如果响应状态码不为200，则打印错误信息并返回None。如果在与服务器交互过程中发生异常，则捕获异常并打印错误信息，同时返回None。
        :param method: 请求方法，支持'get'和'post'
        :param api: API接口名称，默认为'management_logger'
        :param parameter: GET请求的查询参数，格式为'key=value'
        :param data: POST请求的JSON数据，格式为字典
    """
    response = None
    try:
        url_path = f'{url}/api/{api}'
        if method == 'get':
            if parameter != None:
                url_path = f'{url_path}?{parameter}'
            print(url_path)
            response = requests.get(url_path).json()    
        if method == 'post':
            response = requests.post(url_path, json=data).json()
        if response['code'] == 200:
            return response['data'] if response.get('data', None) != None else response['message']
        else:
            print(f"交互失败，原因是{response['message']}")
            return None
    except Exception as e:
        print(f'与服务器交互时发生以下报错 {str(e)}')
        return None


# 重播完毕
def replay_finished(machine_info, cmd):
    """
    该函数用于处理重播完成的事件，首先从机器信息中获取数据集目录，并从命令中提取出重播的HDF5文件路径。然后检查重播的HDF5文件是否属于当前机器的数据集目录，如果是，则构建一个包含数据集目录和重播索引的字典，并通过POST请求将该数据发送给服务器的'add_replay_data' API接口。如果重播的HDF5文件不属于当前机器的数据集目录，则返回一个提示信息。函数还包含异常处理，如果在处理过程中发生异常，则返回一个错误信息。
        :param machine_info: 包含机器信息的字典，必须包含'dataset_dir'键
        :param cmd: 包含重播命令的字符串，必须包含HDF5文件路径
    """
    try:
        dataset_dir = machine_info['dataset_dir']
        hdf5_data = cmd.split(" ")[-1]

        # print(hdf5_data, dataset_dir)

        # 如果机器文件夹为当前重播的文件目录
        if dataset_dir in hdf5_data:
            data = {
                "operation": "add_replay_data",
                "dataset_dir": dataset_dir,
                "replay_idx": hdf5_data.split("/")[-1]
            }
            return speak(method='post', data=data)
        else:
            return '请勿播放其他文件夹数据'
    except Exception as e:
        return f'机器信息有误 str{e}'


# 采集满指定条数后自动查询重播记录
def get_replay_data(machine_info):
    """
    该函数用于获取重播记录，首先从机器信息中获取数据集目录，并构建一个包含数据集目录的字典。然后通过POST请求将该数据发送给服务器的'get_replay_data' API接口，并返回服务器的响应结果。如果在处理过程中发生异常，则返回一个错误信息。
        :param machine_info: 包含机器信息的字典，必须包含'dataset_dir'键
    """
    try:
        dataset_dir = machine_info['dataset_dir']
        data = {
            "operation": "get_replay_data",
            "dataset_dir": dataset_dir
        }
        return speak(method='post', data=data)
    except Exception as e:
        return f'机器信息有误 str{e}'
    

# 新增采集记录
def add_collect_data(machine_info, ep_idx):
    """
    该函数用于添加采集记录，首先从机器信息中获取数据集目录，并构建一个包含数据集目录和episode索引的字典。然后通过POST请求将该数据发送给服务器的'add_collect_data' API接口，并返回服务器的响应结果。如果在处理过程中发生异常，则返回一个错误信息。
        :param machine_info: 包含机器信息的字典，必须包含'dataset_dir'键
        :param ep_idx: episode索引，表示当前采集的episode编号
    """
    try:
        dataset_dir = machine_info['dataset_dir']
        data = {
            "operation": "add_collect_data",
            "dataset_dir": dataset_dir,
            "ep_idx": ep_idx
        }
        return speak(method='post', data=data)
    except Exception as e:
        return f'机器信息有误 str{e}'


# 获取采集记录
def get_collect_data(machine_info):
    """
    该函数用于获取采集记录，首先从机器信息中获取数据集目录，并构建一个包含数据集目录的字典。然后通过POST请求将该数据发送给服务器的'get_collect_data' API接口，并返回服务器的响应结果。如果在处理过程中发生异常，则返回一个错误信息。
        :param machine_info: 包含机器信息的字典，必须包含'dataset_dir'键
    """
    try:
        print("获取采集记录", machine_info)
        dataset_dir = machine_info['dataset_dir']
        data = {
            "operation": "get_collect_data",
            "dataset_dir": dataset_dir
        }
        return speak(method='post', data=data)
    except Exception as e:
        return f'机器信息有误 str{e}'


# 通过ip获取id
def get_id(ip):
    """
    该函数用于通过IP地址获取机器的ID，首先构建一个包含IP地址和通用参数的查询字符串，然后通过GET请求将该查询字符串发送给服务器的'get_id' API接口，并返回服务器的响应结果。如果在处理过程中发生异常，则返回一个错误信息。
        :param ip: 机器的IP地址，用于查询对应的机器ID
    """
    parameter = f'ip={ip}&{universal}'
    return speak(method='get', api='get_id', parameter=parameter)


# 获取具体任务
def get_tasks(id):
    """
    该函数用于通过机器ID获取机器的具体任务，首先构建一个包含机器ID和通用参数的查询字符串，然后通过GET请求将该查询字符串发送给服务器的'get_tasks' API接口，并返回服务器的响应结果。如果在处理过程中发生异常，则返回一个错误信息。
        :param id: 机器的ID，用于查询对应的任务信息
    """
    parameter = f'id={id}&{universal}'
    return speak(method='get', api='get_tasks', parameter=parameter)


# 删除数据记录
def delete_host_data(machine_info, idx):
    """
    该函数用于删除数据记录，首先从机器信息中获取数据集目录，并构建一个包含操作类型、数据集目录和索引的字典。然后通过POST请求将该数据发送给服务器的'delete_host_data' API接口，并返回服务器的响应结果。如果在处理过程中发生异常，则返回一个错误信息。
        :param machine_info: 包含机器信息的字典，必须包含'dataset_dir'键
        :param idx: 要删除的数据记录的索引
    """
    try:
        dataset_dir = machine_info['dataset_dir']
        data = {
            "operation": "delete_host_data",
            "dataset_dir": dataset_dir,
            "idx": idx
        }
        return speak(method='post', data=data)
    except Exception as e:
        return f'机器信息有误 str{e}'


MACHINE_INFO = {
  "machine_id": "001",
  "machine_name": "agilex",
  "dataset_dir": "/home/agilex/kai05_data/agilex/flatten_fold/short_sleeve/flatten_fold_weitiao/v1_260226_001_day_shift_250_10000_hdf5",
  "task": "flatten_fold_weitiao",
  "max_timesteps": "10000",
  "plan_count": 250,
  "collect_count": 2,
  "scheduling": "day_shift",
  "version": "v1",
  "date": "260226",
  "machine_tasks": [
    "flatten_fold/short_sleeve/flatten_fold_weitiao"
  ],
  "format_tasks": [
    "flatten_fold_weitiao"
  ],
  "config_file_path": "/home/agilex/kai05_collect/config/config.json",
  "all_tasks": "/home/agilex/kai05_collect/config/all_tasks.json"
}
    

# if __name__ == "__main__":
    # 请求示例
    # 通过IP地址获取ID
    # id = get_id('219')
    # print(f"获取到的ID: {id}")
    # # 通过ID获取命令
    # command = get_tasks(id.get("machine_id"))
    # print(f"获取到的命令: {command}")

    # 添加采集记录
    # b = add_collect_data(MACHINE_INFO, 3)
    # print(b)

    # 获取采集记录
    # c = get_collect_data(MACHINE_INFO)
    # print(c)

    # 添加重播数据
    # d = replay_finished(MACHINE_INFO, "python3 replay.py /home/agilex/kai05_data/agilex/flatten_fold/short_sleeve/flatten_fold_weitiao/v1_260226_001_day_shift_250_10000_hdf5/episode_3.hdf5")
    # print(d)

    # 获取重播数据
    # e = get_replay_data(MACHINE_INFO)
    # print(e)

    # 删除数据记录
    # f = delete_host_data(MACHINE_INFO, 2)
    # print(f)