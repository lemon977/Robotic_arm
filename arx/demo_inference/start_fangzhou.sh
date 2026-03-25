#!/bin/bash

SESSION_NAME="fangzhou_infer"
WORK_DIR="$HOME/arx_inference"

# 杀死已存在的方舟会话
tmux kill-session -t $SESSION_NAME 2>/dev/null

# 创建新会话，启用鼠标支持
tmux new-session -d -s $SESSION_NAME -n "fangzhou"
tmux set-option -t $SESSION_NAME mouse on

# 上半部分左右分割：CAN检测 | 从臂
tmux send-keys -t $SESSION_NAME:0.0 "can_start && echo 'CAN就绪'" C-m

tmux split-window -h -t $SESSION_NAME:0
tmux send-keys -t $SESSION_NAME:0.1 "sleep 3 && ros2 launch arx_x5_controller open_remote_slave.launch.py" C-m

# 水平分割，下半部分放推理（更大）
tmux split-window -v -t $SESSION_NAME:0
tmux resize-pane -t $SESSION_NAME:0.2 -y 20

tmux send-keys -t $SESSION_NAME:0.2 "sleep 6 && cd $WORK_DIR && python3 arx_x5_python/arx_openpi_inference_rtc.py --host 172.10.1.50 --port 8001" C-m

tmux attach -t $SESSION_NAME