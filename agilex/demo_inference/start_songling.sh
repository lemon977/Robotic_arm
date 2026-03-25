SESSION_NAME="songling_infer"
SUDO_PWD="agx"

tmux kill-session -t $SESSION_NAME 2>/dev/null
sleep 1

tmux new-session -d -s $SESSION_NAME -n "base"
tmux set-option -t $SESSION_NAME mouse on

# 窗口0：基础服务
tmux send-keys -t $SESSION_NAME:0 '
roscore &
sleep 2
python3 gripper_set.py --change_slave --change_master --slave_dis 70 --master_dis 70 && echo "行程设置完成"
sleep 2
echo "'$SUDO_PWD'" | sudo -S ~/kai05_collect/config/can_config.sh
sleep 2
roslaunch realsense2_camera multi_camera.launch
' C-m

# 窗口1：从臂
tmux new-window -t $SESSION_NAME -n "slave"
tmux send-keys -t $SESSION_NAME:1 '
sleep 3
echo "'$SUDO_PWD'" | sudo -S bash -c "source /opt/ros/noetic/setup.bash && python3 ~/kai05_collect/pyUtil/slave_ros8.py"
' C-m

# 窗口2：推理
tmux new-window -t $SESSION_NAME -n "inference"
tmux send-keys -t $SESSION_NAME:2 '
sleep 6
cd ~/Manipulation_Demo_smooth
eval "$(conda shell.bash hook)"
conda activate aloha_pi0_py310
python agilex_inference_openpi_smooth_auto_reset.py --host 192.168.1.10 --port 8000 --ctrl_type joint --use_temporal_smoothing --chunk_size 50
' C-m

tmux select-window -t $SESSION_NAME:0