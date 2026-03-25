#!/usr/bin/env python3
import os
import subprocess
import time

print("="*40)
print("     方舟进程清理工具")
print("="*40)

keys = [
    "arx_openpi_inference_rtc.py",
    "arx_x5_controller",
    "arx_can"
]

print("\n[1/2] 正在清理业务进程...")
for k in keys:
    subprocess.run(f"pkill -9 -f '{k}'", shell=True, 
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    print(f"  - 已发送终止信号: {k}")

print("\n[2/2] 进程清理完毕。")
print("提示：所有终端窗口将在 3 秒后自动关闭...")

for i in range(3, 0, -1):
    print(f"{i}...", end='', flush=True)
    time.sleep(1)

print("\n\n执行关闭，再见")
time.sleep(0.2)

# 杀掉所有 gnome-terminal (包括当前这个)
subprocess.run("pkill -9 gnome-terminal", shell=True)
