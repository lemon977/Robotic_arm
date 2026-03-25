import socket

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
        print(s.getsockname())
        local_ip = s.getsockname()[0]
    except Exception as e:
        # 异常时返回回环地址
        local_ip = "127.0.0.1"
        print(f"获取内网IP失败：{e}")
    finally:
        s.close()
    return local_ip

# 调用函数并打印结果
if __name__ == "__main__":
    ip = get_local_ip()
    print(f"你的本机内网IP是：{ip}")
