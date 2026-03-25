#!/bin/bash

# 0 = 都没跑，1 = 至少一个正在跑
RUNNING=0

# ---- 工具函数：根据正则拿 PID ----
_find_pids(){
    ps -ef | awk -v pat="$1" '$0~pat{print $2}' | tr '\n' ' '
}

# ---- 真正检测 ----
master_pids=$(_find_pids 'open_remote_master\.launch\.py')
slave_pids=$(_find_pids  'open_remote_slave\.launch\.py')

[[ -n $master_pids ]] && { echo "❌ master 进程已在运行，PID: $master_pids"; RUNNING=1; }
[[ -n $slave_pids  ]] && { echo "❌ slave  进程已在运行，PID: $slave_pids";  RUNNING=1; }

# 根据检查结果决定是否启动
if [[ $RUNNING -eq 1 ]]; then
  echo "======================================="
  echo "❌ 请先关闭已有进程再试！"
  echo "可使用 kill -9 pid 关闭节点"
  echo "节点关闭缓慢，请耐心等待"
  echo "======================================="
  exit 1
else
  echo "✅ 全部节点关闭状态，开始启动..."
  # ros2 launch arx_x5_controller open_remote_master.launch.py
  # ros2 launch arx_x5_controller open_remote_slave.launch.py
  terminator -l arx
  echo "🚀 遥控节点启动完成！"
fi



# #!/bin/bash
# # gnome-terminal -t "can" -- bash -c "cd ~/ARX_X5/ARX_CAN && ./can.sh;exec bash;"

# cd /home/kai/arx_collect_data/script
# conda deactivate
# # python3 collect_data_ros2_noimg.py --max_timesteps 1500 --save_video --dataset_dir ~/data/hang_1020_53_100_v6-1_1500_hdf5 --episode_idx 0


# if [ "$1" == "can" ]; then
#     gnome-terminal --tab -t "can0" -- bash -c "cd /home/kai/ARX_X5/ARX_CAN/arx_can; echo kai0717 | sudo -S ./arx_can0.sh; exec bash;"
#     sleep 0.1
#     gnome-terminal --tab -t "can1" -- bash -c "cd /home/kai/ARX_X5/ARX_CAN/arx_can; echo kai0717 | sudo -S ./arx_can1.sh; exec bash;"
#     sleep 0.1
#     gnome-terminal --tab -t "can2" -- bash -c "cd /home/kai/ARX_X5/ARX_CAN/arx_can; echo kai0717 | sudo -S ./arx_can2.sh; exec bash;"
#     sleep 0.1
#     gnome-terminal --tab -t "can3" -- bash -c "cd /home/kai/ARX_X5/ARX_CAN/arx_can; echo kai0717 | sudo -S ./arx_can3.sh; exec bash;"
# fi

# sleep 2
# gnome-terminal --tab -t "master" -- bash -c "ros2 launch arx_x5_controller open_remote_master.launch.py;exec bash;"
# sleep 2
# gnome-terminal --tab -t "slave" -- bash -c "ros2 launch arx_x5_controller open_remote_slave.launch.py;exec bash;"
