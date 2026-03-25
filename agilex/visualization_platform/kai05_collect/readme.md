* kai05数采网页

1. 配置环境
    1. python3.8 + 以下额外引入库
        - yaml
        - pandas
        - gradio
        - numpy
        - h5py
        - rospy
        - sensor_msgs
        - nav_msgs
        - cv_bridge
        - piper_sdk
        - multiprocessing
        - dm_env
        
    2. camera_web.launch multi_camera2.launch 两个launch文件放入到启动摄像头的ros包下
        - 松灵（agilex）目录为 cobot_magic/camera_ws/src/realsense-ros/realsense2_camera/launch/

2. 代码使用说明
    1. 若想启动整个网页，终端在kai05_collect目录下执行 
        - python3 main.py
    2. 若需要单独发送nas结尾指示，终端在kai05_collect/pyUtil目录下执行 
        - python3 file_manager.py

3. 网页使用说明
    1. 网页启动后等待指示灯全部启动（最后两个指示灯为紫色即可，若不为紫色需要拔插一次从臂can口的usb线）
    2. 开始采集按钮（快捷键为 回车 enter） 可进行数据的采集
    3. 结束采集按钮（快捷键为 空格 space） 可停止本次数据采集并保存（本地+nas）
    4. 放弃保存按钮（快捷键为 退格 backspace） 可放弃本次数据保存，不会生成数据文件
    5. 重播按钮 可以选择采集的数据重播（默认最后一条）
    6. 配置参数 选择自己的任务以及班次（早晚班）
    7. 采集完毕 发送nas结尾指示，效果同代码使用说明第2小点

4. 注意事项
    1. 如果采集后出现未采集到数据，说明主从臂的进程未启动成功就进行了采集。
        1. 若最后两个指示灯为紫色，等待主从臂进程都启动后重新采集即可（不需要重启网页））
        2. 若最后两个指示灯为黄色，拔插从臂usb线后重启网页进行采集

5. 目录解析
    - config/
        - all_tasks.json          本地离线具体任务
        - can_config.sh           can口激活  快速
        - can_port.sh             can口激活
        - config.json             本地机器配置文件  机器编号
        - data_collect_info.yaml  nas复制配置文件
        - find_all_can_port.sh    检测can口编号，使用此sh脚本进行can_port.sh脚本对应的can口更新编号

    - pyUtil/
        - arm_hdf5.py             通过读取指定目录下的hdf5_position文件并使得机械臂到达指定位置   需要重播的hdf5数据的第一帧位置/0点
        - arm_home.py             通过读取指定目录下的master_position文件并使得机械臂到达指定位置   主臂重播前的位置/0点
        - capture_ros.py          负责监听七条机械臂启动脚本的输出
            - capture_command_output() 监听工控机脚本内容
            - capture_collect_output() 监听采集脚本输出内容
            - parse_position() 机械臂浮动判断
            - is_small_floating() 机械臂微小浮动判断
            - extract_position_array() 解析rostopic echo机械臂数据中的position值
            - check_pid_alive() 检测系统对应pid进程是否存活
            - monitor_command_background() 多线程依次启动工控机数采所需依赖
            - start_target_command() 依赖指令实际启动函数
            - monitor_collect_background() 采集监控函数
        - collect_machine2nas.py  nas复制脚本+发送socket消息
            - setup_logging() 配置日志
            - notice_nas_service() 发送数据给nas服务
            - find_similar_folder() 找寻相近的数据集文件目录
            - rename_folder() 目录重命名
            - copy2nas() 将采集的数据上传一份到nas上
        - file_manager.py         文件操作模块脚本
            - update_indicator_js() 更新js
            - read_html_file() 读取HTML文件内容
            - command_indicator_html() 生成ROS命令行状态指示器HTML片段
            - collect_indicator_html() 生成采集命令行状态指示器HTML片段
            - save_config_file() 保存配置文件到config.json
            - read_config_file() 读取配置文件内容
            - get_dir_idx() 获取当前工作的索引
            - scan_episode_indices() 扫描数据文件夹获取可用的 episode 索引列表
            - load_hdf5_data() 读取HDF5文件并拆分左右手数据
            - open_collect_dir() 打开采集目录
            - get_children() 获取子进程PID列表
            - delete_data() 确认删除指定 episode 的数据文件
            - commit_collect() 提交采集结束标识
        - get_local_ip.py         获取本机ip最后一位脚本，当没有config.json配置文件时会通过此脚本获取最后一位ip向本地服务器获取本机ip
        - gripper_set100.py       检测最大夹爪行程是否为100，不为100则进行修改
        - host_data_manager.py    向服务器发送请求的管理脚本
            - speak() 向服务器交互主函数
            - replay_finished() 重播完毕时向服务器发送重播数据，服务器存储记录（强制重播需要）
            - get_replay_data() 采集满指定条数后查询一次重播记录（强制重播需要）
            - add_collect_data() 采集成功后调用函数向本地服务器发送采集数据并保存在服务器日志中
            - get_collect_data() 获取服务器中的采集记录，渲染到当前机器网页中的历史采集记录部分
            - get_id() 通过ip向服务器请求获取当前主机所对应的编号
            - get_tasks() 通过编号向服务器请求获取当前机器所对应的任务
            - delete_host_data() 删除数据记录，本地删除采集数据时向服务器发送数据，修改服务器中所记录的重播/采集/删除记录日志
        - master_arm_position.py  记录主臂当前位置脚本  运行后将会生成arm_home.py运行所需要的master_position文件
        - pipper_ros.py           机械臂主臂使能脚本
        - read_hdf5.py	          读取hdf5第一帧脚本  运行后将会生成arm_hdf5.py运行所需要的hdf5_position文件
        - record2.py              采集脚本    --dataset_dir 文件目录  --episode_idx 文件索引
        - replay.py               重播脚本   文件目录/episode_{idx}.hdf5
        - slave_ros8.py           机械臂从臂使能脚本

    - static/
        - indicator_update.js     更新监听状态的js
        - styles.css              html元素的规则

    - /
        - main.py                 主启动脚本
            - update_ui() 更新UI数据
            - open_config_panel() 打开配置面板
            - delete_data_controller() 删除采集数据
            - save_config() 保存配置文件函数
            - replay_btn_click() 打开重播页面
            - on_delete_episode() 打开删除确认弹窗
            - restart_collect_piper() 重启 主臂会话
            - replay_click() 重播按钮
            - start_replay() 开始重播
            - stop_replay() 停止重播
            - start_collect() 开始采集
            - end_collect() 结束采集
            - stop_collect() 放弃保存
6. 监听进程代码
    - alias jt="watch -n 1 'ps -ef | grep -E \"roscore|roslaunch|rostopic|pipper_ros.py|slave_ros8.py\" | grep -v grep'"
