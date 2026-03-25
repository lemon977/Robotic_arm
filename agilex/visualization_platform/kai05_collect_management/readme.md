* kai05数采网页后端管理

1. 配置环境
    - 代码为后端管理用的api接口，需要在本地服务器上配置如下
    1. python3.8 + 以下额外库
        - flask
        - flask_cors
    2. 建议配置守护进程，不然本地服务器断网或者重启无法自动启动api以提供服务
    3. 启动本地服务

2. 代码使用说明
    1. 配置守护进程后启动代码为
        - sudo systemctl start xxx.service
        - sudo systemctl enable xxx.service   设置开机自启
        - sudo systemctl status xxx.service   服务启动状态
        - sudo systemctl stop xxx.service     停止服务
    2. 未配置守护进程启动代码为
        - python3 ${文件存在路径}/main.py

2. 目录解析
    - /
        - get_id.json               记录松灵机器固定ip所对应的松灵机器编号以及最大步长
        - machine_info.json         记录每台松灵机器的具体任务集、版本以及目标任务数
        - main.py                   python服务端主要任务脚本
            - read_json()           总体读取json函数（必要）
            - get_id()              通过ip获取id（必要）
            - get_tasks()           通过id获取任务信息（必要）
            - get_all_tasks()       获取所有机器的任务信息（非必要）
            - receive_data()        重播成功时写入日志文件（需要强制重播时必要）
            - get_replay_data()     获取当前请求机器的重播记录（需要强制重播时必要）
            - add_collect_data()    新增采集记录到本地服务器（非必要）
            - get_collect_data()    查询采集记录，显示到数采网站的历史记录中（需要展示采集记录时必要）
            - delete_host_data()    删除采集记录，用户本地删除后会发送请求删除本地服务器的记录，从而保持数采网站中的采集记录一致性（需要展示采集记录时必要）

3. 建议
    - 本文件夹为后端管理的服务器进程，需配置守护进程确保服务器断网或者异常重启导致的服务器关闭