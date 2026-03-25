#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
一键启动多终端演示脚本（含自动输密码 agx）
依赖：tmux
用法:
  ./start_demo.py          # 启动
  ./start_demo.py -k       # 仅清场
"""
import argparse
import subprocess
import time
from pathlib import Path

SESSION   = "piper_demo"
PKG_ROOT  = Path.home() / "cobot_magic/Piper_ros_private-ros-noetic"
DEMO_ROOT = "/home/agilex/Manipulation_Demo_smooth"
LOG_DIR   = Path.home() / "piper_logs"

# 关键：第二条命令里自动喂密码 agx 给 sudo
JOBS = [
    ("00_roscore",     "roscore"),
    ("01_can_config",  f"cd /home/agilex/agilex_deploy/agilex_pipeline && echo agx | sudo -S ./can_config.sh"),   # ← 这里
    ("02_realsense",   f"roslaunch realsense2_camera multi_camera.launch"),
    ("03_piper",       f"cd /home/agilex/agilex_deploy/agilex_pipeline && python3 slave_ros8.py"),
    ("04_inference",   f"exec bash -c 'source ~/miniconda3/etc/profile.d/conda.sh && "
f"conda activate aloha_pi0_py310 && "
f"export PYTHONPATH={DEMO_ROOT}/packages/openpi-client/src:$PYTHONPATH && "
f"python /home/agilex/agilex_deploy/agilex_pipeline/agilex_inference_openpi_smooth.py --host 172.10.1.50 --port 8001 "
f"--ctrl_type joint --use_temporal_smoothing --chunk_size 50'"),
]
# f"exec bash -c 'source ~/miniconda3/etc/profile.d/conda.sh && "
# f"conda activate aloha_pi0_py310 && "
# f"export PYTHONPATH={DEMO_ROOT}/packages/openpi-client/src:$PYTHONPATH && "
# f"python agilex_inference_openpi_smooth_auto_reset.py --host 172.10.1.12 --port 8000 "
# f"--ctrl_type joint --use_temporal_smoothing --chunk_size 50'"

run = lambda cmd, chk=True: subprocess.run(cmd, shell=True, check=chk)

def kill_session():
    run(f"tmux kill-session -t {SESSION} 2>/dev/null", chk=False)

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("-k", "--kill-only", action="store_true", help="仅清场")
    args = parser.parse_args()

    if args.kill_only:
        kill_session()
        print("✅ 已清理 tmux session")
        return

    LOG_DIR.mkdir(exist_ok=True)
    kill_session()

    # 创建会话 & 分屏（兼容小窗口）
    run(f"tmux new-session -d -s {SESSION}")
    run(f"tmux split-window -h -t {SESSION}:0")   # 先横切
    for _ in range(3):
        run(f"tmux split-window -v -t {SESSION}:0")
    run(f"tmux select-layout -t {SESSION}:0 tiled")

    # 发命令
    for idx, (name, cmd) in enumerate(JOBS):
        log = LOG_DIR / f"{name}.log"
        run(f'tmux send-keys -t {SESSION}:0.{idx} "{cmd} 2>&1 | tee {log}" C-m')
        time.sleep(1.5)
        if idx == 1:
            time.sleep(10)

    print("📌 启动完成！日志目录：", LOG_DIR)
    print("   ./start_demo.py -k  可彻底关闭")
    try:
        run(f"tmux attach-session -t {SESSION}")
    except KeyboardInterrupt:
        print("\n🛑 收到 Ctrl-C，自动清理 session …")
        kill_session()

if __name__ == "__main__":
    main()
