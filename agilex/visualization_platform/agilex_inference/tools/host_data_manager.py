import json
import requests

url = 'http://192.168.9.50:5001'
task_type = "inference"
brand = "agilex"
universal = f"brand={brand}&task_type={task_type}"

# 和服务器交互
def speak(method, api, parameter=None, data=None):
    """
    该函数用于与服务器进行交互，发送HTTP请求并处理响应。可以根据需要修改请求的方式、参数和数据。
        - method: HTTP请求方法，例如'get'或'post'。
        - api: 服务器API的路径，例如'get_id'或'get_tasks'。
        - parameter: 可选参数，用于构建查询字符串，例如'ip=219&brand=agilex&task_type=inference'。
        - data: 可选参数，用于POST请求的JSON数据。
    返回值:
        - 服务器响应中的数据部分，如果响应代码为200；否则返回None。
    """
    response = None
    try:
        url_path = f"{url}/api/{api}"
        if method == 'get':
            if parameter != None:
                url_path = f"{url_path}?{parameter}"
            # print(url_path)
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


# 通过IP地址获取id
def get_id(ip):
    """
    该函数用于获取机器的ID，通过与服务器交互发送GET请求。可以根据需要修改请求的参数和处理方式。
        - ip: 机器的IP地址，用于构建查询字符串，例如'ip=219&brand=agilex&task_type=inference'。
    返回值:
        - 服务器响应中的数据部分，如果响应代码为200；否则返回None。
    """
    parameter = f'ip={ip}&{universal}'
    return speak(method='get', api='get_id', parameter=parameter)


# 通过id获取指令
def get_command(id):
    """
    该函数用于获取机器的命令，通过与服务器交互发送GET请求。可以根据需要修改请求的参数和处理方式。
        - id: 机器的ID，用于构建查询字符串，例如'id=001&brand=agilex&task_type=inference'。
    返回值:
        - 服务器响应中的数据部分，如果响应代码为200；否则返回None。
    """
    parameter = f'id={id}&{universal}'
    return speak(method='get', api='get_tasks', parameter=parameter)

if __name__ == "__main__":
    # 请求示例
    # 通过IP地址获取ID
    id = get_id('22')
    print(f"获取到的ID: {id}")
    # 通过ID获取命令
    command = get_command(id.get("machine_id"))
    print(f"获取到的命令: {command}")