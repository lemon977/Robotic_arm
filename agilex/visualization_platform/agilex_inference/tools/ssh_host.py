import paramiko
import re
import time
from typing import Optional

def ssh_execute_shell_with_progress(
    host: str,
    username: str,
    password: str,
    model_status,
) -> dict:
    """
    该函数通过SSH连接远程主机，并执行指定的Shell命令，同时实时监听命令输出以捕获进度信息和关键日志。可以根据需要修改SSH连接的参数、命令执行方式和输出处理方式。
    参数:
    - host: 远程主机的IP地址或域名。
    - username: SSH登录的用户名。
    - password: SSH登录的密码。
    - model_status: 包含模型状态信息的字典，用于更新模型运行状态和进度信息。
    返回值:
    - 包含执行结果的字典，包括stdout、stderr、return_code、success和error等字段，以便调用者了解命令执行的结果和状态。
    """
    
    result = {
        "stdout": "",       # 所有输出内容
        "stderr": "",       # 所有错误内容
        "return_code": -1,  # 脚本返回码
        "success": False,   # 执行是否成功
        "error": ""         # 连接/执行错误
    }

    ssh_client = paramiko.SSHClient()
    model_status.update({
        "ssh_client": ssh_client
    })
    try:
        # 初始化SSH连接
        ssh_client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        ssh_client.connect(
            hostname=host,
            username=username,
            password=password,
            port=22,
            timeout=30,
            allow_agent=False,
            look_for_keys=False
        )

        # 创建交互式通道（关键：支持流式输出）
        channel = ssh_client.invoke_shell()
        channel.settimeout(None)  # 取消超时，适配长耗时脚本
        channel.set_combine_stderr(True)  # 合并stdout/stderr（方便统一监听）
        # 发送执行脚本的命令
        channel.send(f"{model_status['cmd']}\n")
        model_status.update({
            "channel": channel,
        })

        # 实时读取输出流（核心逻辑）
        output_buffer = ""
        progress_pattern = re.compile(r'(\d+)%|\d+(\.\d+)?[GMK]B/\d+(\.\d+)?[GMK]B')  # 匹配rsync进度条特征
        while not channel.closed:
            if channel.recv_ready():
                # 读取最新输出（按字节流读取）
                chunk = channel.recv(4096).decode('utf-8', errors='ignore')
                if not chunk:
                    break
                output_buffer += chunk
                result["stdout"] += chunk

                # 按行分割处理（兼容进度条的换行/回车）
                lines = output_buffer.split('\r')  # rsync进度条用\r刷新，而非\n
                for line in lines[:-1]:  # 最后一行可能不完整，暂存
                    line = line.strip()
                    if not line:
                        continue
                    
                    # 判断是否是进度条行
                    is_progress = bool(progress_pattern.search(line))
                    
                    # 调用回调函数处理（如打印进度、更新UI等）
                    # if progress_callback:
                    #     progress_callback(line, is_progress)
                    
                    # 打印到控制台（可选，方便调试）
                    if is_progress:
                        #7.79G  37%   65.32MB/s    0:03:16 (xfr#2, to-chk=3/6)
                        model_status['progress'] = f"\r进度: {line}"
                        print(f"\r进度: {line}", end="", flush=True)
                    else:
                        #8001
                        model_status['output'] = f"输出: {line}"
                        # print(f"输出: {line}")
                        if "listening on 0.0.0.0:" in line:
                            # if 'channel' in locals() and channel.is_open():
                            #     channel.close()
                            # ssh_client.close()
                            model_status.update({"run_status": "finished, 模型启动成功"})
                            # return result
                        if "本地 ckpt 不存在，从 NAS 复制" in line:
                            model_status.update({"run_status": "copying, 从 NAS 复制中..."})

                        if "本地 ckpt 已存在" in line:
                            model_status.update({"run_status": "loading, 加载本地 ckpt 中..."})

                        if "FileNotFoundError" in line:
                            model_status.update({"run_status": "error, 模型拷贝不完整"})

                        if "address already in use" in line:
                            model_status.update({"run_status": "error, 端口号已被占用"})                
                # 保留最后一行不完整数据
                output_buffer = lines[-1]

            # 检查脚本是否执行完成
            if channel.exit_status_ready():
                # 读取剩余输出
                remaining = channel.recv(4096).decode('utf-8', errors='ignore')
                if remaining:
                    result["stdout"] += remaining
                    # if progress_callback:
                    #     progress_callback(remaining, False)
                    print(remaining)
                # 获取返回码
                result["return_code"] = channel.recv_exit_status()
                result["success"] = (result["return_code"] == 0)
                break

            time.sleep(0.1)  # 降低轮询频率，减少资源占用

    except paramiko.AuthenticationException:
        result["error"] = "认证失败：用户名/密码错误"
        model_status.update({"run_status": "error, 认证失败：用户名/密码错误"})
    except paramiko.NoValidConnectionsError:
        result["error"] = f"连接失败：无法访问 {host}:22"
        model_status.update({"run_status": "error, 连接失败：无法访问 {host}:22"})
    except Exception as e:
        result["error"] = f"执行异常：{str(e)}"
        model_status.update({"run_status": "error, 执行异常：" + str(e)})
    finally:
        # 关闭通道和SSH连接
        if 'channel' in locals() and channel.is_open():
            channel.close()
        ssh_client.close()
        # model_status.update({"run_status": "finished" if result["success"] else "error"})

    return result

# ---------------------- 测试调用 ----------------------
if __name__ == "__main__":
    # 定义进度回调函数（处理rsync进度条）
    def handle_progress(line: str, is_progress: bool):
        if is_progress:
            # 提取进度百分比（示例：从进度行中解析数字）
            progress_match = re.search(r'(\d+)%', line)
            if progress_match:
                progress = progress_match.group(1)
                # 这里可以更新UI、记录日志等
                print(f"\r当前拷贝进度：{progress}%", end="", flush=True)

    # 执行Shell脚本（替换为你的实际参数）
    script_cmd = 'bash /home/lirui/Kai05-VLA/serve_policy_yaml.sh /mnt/nas/Kai05-VLA/checkpoints//vla_torch_flatten_fold_standard_2012_baseline/0226_vla_torch_flatten_fold_standard_2012_baseline/25000 "Flatten and fold the short sleeve" 8002'

    # 调用SSH执行函数
    exec_result = ssh_execute_shell_with_progress(
        host="192.168.9.50",
        username="lirui",
        password="1217",
        shell_script_cmd=script_cmd,
        # progress_callback=handle_progress
    )

    # 打印最终结果
    print("\n" + "="*50)
    print(f"执行结果：{'成功' if exec_result['success'] else '失败'}")
    print(f"返回码：{exec_result['return_code']}")
    print(f"错误信息：{exec_result['error']}")
    print(f"完整输出：\n{exec_result['stdout']}")
