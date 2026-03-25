#!/usr/bin/env python3
# -*-coding:utf8-*-
# 注意demo无法直接运行，需要pip安装sdk后才能运行
# V2版本sdk
# 夹爪/示教器参数设置指令
# 第一次使用夹爪或者示教器时，需要设定一下这两个末端执行器参数，否则会出现数据没有反馈并且执行器无法控制的情况
# 一般情况下 GripperTeachingPendantParamConfig 函数第二个参数 max_range_config 是70
import time
from piper_sdk import *

def set_gripper(name, max_range_config:int=100):
    """
    该函数用于设置夹爪的最大行程参数，确保夹爪能够正常工作
        :param name: 夹爪所在的机械臂名称，如 'can_l_slave'、'can_r_slave'、'can_l_master'、'can_r_master'
        :param max_range_config: 夹爪最大行程参数，单位与夹爪配置相关，通常为70或100，具体数值请参考夹爪规格和需求
    处理流程：
        1. 创建 C_PiperInterface_V2 对象并连接到指定机械臂
        2. 调用 GripperTeachingPendantParamConfig 方法设置夹爪参数，其中 max_range_config 参数设置为指定值
        3. 调用 ArmParamEnquiryAndConfig 方法查询当前夹爪参数配置，并打印输出以确认设置成功
        4. 断开连接
    """
    print(f'修改{name} 夹爪最大行程到100')
    piper = C_PiperInterface_V2(name)
    piper.ConnectPort()
    piper.GripperTeachingPendantParamConfig(100, max_range_config, 1)
    piper.ArmParamEnquiryAndConfig(4)
    count = 3
    while count := count - 1 > 0:
        time.sleep(0.5)
        print(f"{name} 夹爪查询行程参数：", piper.GetGripperTeachingPendantParamFeedback())
    piper.DisconnectPort()

if __name__ == "__main__":
    max_range_config = 100
    bad = []
    for name in ['can_l_slave', 'can_r_slave', 'can_l_master', 'can_r_master']:
        print(f'查询{name} 夹爪最大行程')
        piper = C_PiperInterface_V2(name)
        piper.ConnectPort()
        piper.ArmParamEnquiryAndConfig(4)
        count = 3
        while count := count - 1 > 0:
            time.sleep(0.5)
            tmp = piper.GetGripperTeachingPendantParamFeedback()
        if tmp.arm_gripper_teaching_param_feedback.max_range_config != max_range_config:
            print(f"{name} 夹爪最大行程为{tmp.arm_gripper_teaching_param_feedback.max_range_config},非{max_range_config}")
            print(f"{name} 夹爪配置参数： {tmp}")
            bad.append(name)
        piper.DisconnectPort()

    for name in bad:
        set_gripper(name, max_range_config=max_range_config)
        
    