#!/usr/bin/env bash
#set -e
SUDO_PWD="agx"
SESSION_NAME="songling_infer"

echo "========================================"
echo "     松灵进程清理工具（严格匹配指定进程）"
echo "========================================\n"

# 1. 终止tmux会话
echo "[1/3] 终止松灵tmux会话..."
tmux kill-session -t $SESSION_NAME 2>/dev/null
echo "  - 已终止tmux会话: $SESSION_NAME"
sleep 1

# 2. 杀死指定核心进程（严格匹配用户命令中的进程）
echo -e "\n[2/3] 清理核心进程..."
PROCESS_LIST=(
    "roscore"
    "rosmaster"
    "roslaunch realsense2_camera multi_camera.launch"
    #"gripper_set.py --change_slave --change_master --slave_dis 70 --master_dis 70"
    "python3 ~/kai05_collect/pyUtil/slave_ros8.py"
    "agilex_inference_openpi_smooth_auto_reset.py --host 192.168.1.10 --port 8000 --ctrl_type joint"
    #"~/kai05_collect/config/can_config.sh"
)
for proc in "${PROCESS_LIST[@]}"; do
    echo $SUDO_PWD | sudo -S pkill -9 -f "$proc" 2>/dev/null
    echo "  - 已终止进程: $proc"
done

# 清理残余松灵相关进程
echo "  - 清理残余松灵进程..."
echo $SUDO_PWD | sudo -S ps aux | grep -E "(kai05_collect|Manipulation_Demo_smooth|aloha_pi0_py310)" | grep -v grep | awk '{print $2}' | xargs -r kill -9 2>/dev/null

# 3. 友好提示+延时
echo -e "\n[3/3] 进程清理完毕！"
echo "提示：终端将在 3 秒后就绪（按任意键退出）..."
for i in {3..1}; do
    echo -n "${i}... "
    sleep 1
done

echo -e "\n✅ 松灵所有进程已终止\n"
sleep 0.5