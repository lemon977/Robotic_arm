* terminator推理窗口

1. 配置环境
    1. python3.8 + 以下额外库
        - cv2
        - numpy
        - rospy
        - torch
        - cv_bridge
        - geometry_msgs
        - nav_msgs
        - piper_msgs
        - sensor_msgs
        - std_msgs
        - matplotlib
        - piper_sdk

    2. 配置alias
        - alias agx_tl='/home/agilex/.config/terminator/start_tl.sh'
        - alias tl='/home/agilex/agilex_deploy/agilex_pipeline/start_demo_kai05.sh'
        - alias ktl='/home/agilex/agilex_deploy/agilex_pipeline/start_demo_kai05.sh -k'
        - alias jt='watch -n 1 "ps -ef | grep -E '\''roscore|roslaunch|rostopic|pipper_ros.py|slave_ros8.py'\'' | grep -v grep"'
    
    3. 配置terminator布局的config文件（将目录中的config文件和start_tl.sh复制到terminator配置的根目录（若文件名相同则建议做好备份））
        - terminator根目录 松灵（agilex） 为 .config/terminator/
    
    4. 给shell脚本添加可执行权限
        - chmod +x ${文件所在目录}/start_demo_kai05.sh
        - chmod +x ${文件所在目录}/start_tl.sh
        
2. 代码使用说明
    1. 若配置了alias
        - 可通过agx_tl一键启动整个terminator窗口
        - 之后可通过ktl/tl实现terminator窗口中的部分关闭/启动操作

    2. 若未配置alias可通过如下步骤以次执行
        1. 激活can口
            - echo agx | ~/agilex_deploy/agilex_pipeline/can_config.sh
        2. roscore
            - roscore
        3. 启动相机
            - roslaunch realsense2_camera multi_camera.launch 
        4. 启动从臂
            - python3 ~/agilex_deploy/agilex_pipeline/slave_ros8.py
        5. 启动测试脚本
            - python3 ~/agilex_deploy/agilex_pipeline/agilex_inference_openpi_smooth.py --host 172.10.1.50 --port 8001 --ctrl_type joint --use_temporal_smoothing --chunk_size 50


3. 目录解析
    - agilex_pipeline/
        - agilex_inference_openpi_smooth.py     测试脚本
        - arm_reset.py                          复位脚本
        - can_config.sh                         can口激活
        - slave_ros8.py                         从臂使能
        - start_demo_kai05.sh                   推理窗口


