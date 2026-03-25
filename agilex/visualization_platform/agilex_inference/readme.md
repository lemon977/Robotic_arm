* 推理网页

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
        - paramiko
        - gradio
        - requests
    2. camera_web.launch multi_camera2.launch 两个launch文件放入到启动摄像头的ros包下
        - 松灵（agilex）目录为 cobot_magic/camera_ws/src/realsense-ros/realsense2_camera/launch/

2. 代码使用
    1. 若想启动整个网页，终端在agilex_inference目录下执行 
        - python3 main.py
    
3. 网页使用说明
    1. 网页启动后需要先选择模型启动+启动工控机
    2. 开始测试按钮 完成第一步+复位到推理点后即可开始测试
    3. 中止测试按钮 中止本条测试进度，以下三点进行使用
        - 衣服折叠完毕
        - 超过单次测试最大时长（例如3min）
        - 机械臂发生故障/位置怪异
    4. 结束测试按钮 结束本次测试并终止当前启动的模型（会释放当前网页上启动成功的上位机模型的端口）[如果启动失败，那么压根没占用端口]
    5. 复位按钮
        - 推理点 需要在推理点方可开始测试
        - 零点   如果需要休息调整，可将从臂回退到零点
    6. 自动适配输入框
        - 将飞书上测试文档的 “实验名” 复制并直接粘贴到本输入框内，会自动选择到当前模型，这时直接启动即可

4. 注意事项
    1. 如果默认分配的上位机ip或端口有误，可进行手动修改
    2. 若启动上位机模型时出现 "error,端口已被占用"，此时有两种解决方式
        - 去上位机停止当前需要启动的端口，如果被占用，那么上位机一定有进程运行于此端口
        - 选择模型 弹窗中修改端口，这时会以一个新的端口启动模型
    3. 如果启动工控机时最后两个状态灯（第二排最后两个）为黄色时，需要重新拔插从臂usb线后重启网页（代码使用第1点）

5. 目录解析（所有传参类型均写到具体的py脚本函数下）
    - config/
        - can_config.sh                         can口激活
            - 单独使用方法  echo agx | agilex_inference/config/can_config.sh

    - static/
        - kai05_deployee.css                    html规则
        - update_indicator.js                   动态渲染页面js

    - tools/
        - agilex_inference_openpi_smooth.py     主推理脚本，如需换别的推理脚本，可直接替换。若需更改其他名字推理脚本需要进行本地服务器修改
        - arm_reset.py                          复位脚本  参数  0 零点   1 推理点
        - config_upload.py                      配置读取脚本
            - config_upload() 通过本地文件配置推理参数
            - config_upload_byNet() 通过本地服务器请求获取本机配置参数，若失败会自动调用本地文件配置参数
            - get_local_ip() 获取本机内网ip
        - host_data_manager.py                  本地服务器交互脚本（获取机器配置）
            - speak() 服务器交互通用函数
            - get_id() 通过本机ip最后一位获取具体机器配置内容
            - get_command() 通过本机编号获取具体机器推理所需要执行的指令
        - html_manager.py                       html页面数据管理脚本
            - read_html_file() 读取文件作为html的内容
            - update_ui() 更新ui时序列化数据
            - update_indicator_js() 加载页面的js规则
        - log_setup.py                          logger配置脚本
            - setup_logger() 配置并返回一个日志器
        - monitor_manager.py                    进程监听管理脚本
            - monitor_process() 监听工程机状态进程
            - monitor_resets() 监听从臂复位进程
            - monitor_time() 监听测试时间3min，暂未启用
            - extract_position_array() 从监听的rostopic echo中读取到机械臂当前的position位置，从而判断机械臂当前和上一次是否进行了运动
            - parse_position() 机械臂浮动判断函数
            - is_small_floating() 机械臂微小浮动判定 关节偏移<0.002
            - check_pid_alive() 检测进程的pid在系统进程中是否存活
        - nas_upload.py                         nas管理脚本
            - nas_upload() 挂载nas
            - nas_select() 查询nas挂载情况
        - slave_ros8.py                         从臂使能脚本
        - ssh_host.py                           上位机启动模型脚本
            - ssh_execute_shell_with_progress() 通过传入的用户名和密码进行ssh连接并执行shell脚本启动需要测试的模型

    - /
        - main.py                               程序启动主脚本
            - log_append() 日志写入函数
            - show_tips_then_hide() 分段显示操作弹窗
            - start_model() 启动模型
            - chooice_model() 从本地nas获取模型列表并渲染到网页中的下拉框中
            - chooice_model_2() 一级下拉框修改选中内容后，实时渲染二级下拉框内容
            - chooice_steps() 二级下拉框选中内容后，根据一级二级下拉框的目录路径获取当前模型有哪些步长并渲染
            - change_steps() 步长发生改变时判断是否需要更新二级目录
            - auto_model() 自动适配实验，用户直接将测试任务表中的实验名复制粘贴到适配框后会自动选中对应模型
            - number_detect() 判断是否为整数
            - host_change() 用户本次运行修改上位机地址
            - port_change() 用户本次运行修改上位机端口
            - see_log() 查看日志
            - start_machine() 启动工控机
            - update_progress() 更新模型启动进度
            - status_check() 判断函数，按需要的频率集成判断启动某个按钮所对应操作所需要的条件
            - start_test() 开始测试
            - stop_test() 中断测试
            - end_machine() 结束测试
            - reset() 从臂复位
            - init() 初始化函数，配置logger(日志)/config(本机配置)/commands_status(进程字典)等