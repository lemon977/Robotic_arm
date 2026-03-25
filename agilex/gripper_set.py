#!/usr/bin/env python3
# -*-coding:utf8-*-
# 注意demo无法直接运行，需要pip安装sdk后才能运行
# V2版本sdk
# 夹爪/示教器参数设置指令
# 第一次使用夹爪或者示教器时，需要设定一下这两个末端执行器参数，否则会出现数据没有反馈并且执行器无法控制的情况
import time
import argparse
from piper_sdk import *


def set_gripper(name, max_range_config: int = 100):
    """设置夹爪行程"""
    print(f'修改 {name} 夹爪最大行程到 {max_range_config}')
    piper = C_PiperInterface_V2(name)
    try:
        piper.ConnectPort()
        piper.GripperTeachingPendantParamConfig(100, max_range_config, 1)
        time.sleep(0.5)
        piper.ArmParamEnquiryAndConfig(4)
        
        # 查询确认（3次尝试）
        for i in range(3):
            time.sleep(0.5)
            feedback = piper.GetGripperTeachingPendantParamFeedback()
            if feedback:
                actual = feedback.arm_gripper_teaching_param_feedback.max_range_config
                print(f"  查询{i+1}/3: 当前行程={actual}")
                if actual == max_range_config:
                    print(f"  ✓ 设置成功")
                    return True
        print(f"  ✗ 设置后验证失败")
        return False
    except Exception as e:
        print(f"  错误: {e}")
        return False
    finally:
        piper.DisconnectPort()


def query_gripper(name):
    """查询夹爪当前行程"""
    print(f'查询 {name} 夹爪参数')
    piper = C_PiperInterface_V2(name)
    current_range = None
    try:
        piper.ConnectPort()
        piper.ArmParamEnquiryAndConfig(4)
        
        for i in range(4):
            time.sleep(0.5)
            feedback = piper.GetGripperTeachingPendantParamFeedback()
            if feedback and hasattr(feedback, 'arm_gripper_teaching_param_feedback'):
                current_range = feedback.arm_gripper_teaching_param_feedback.max_range_config
                print(f"  查询{i+1}/4: max_range_config={current_range}")
                return current_range
    except Exception as e:
        print(f"  错误: {e}")
    finally:
        piper.DisconnectPort()
    
    return current_range


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='夹爪行程设置工具')
    parser.add_argument("--change_slave", action="store_true", default=False, 
                       help="是否修改从臂夹爪行程，默认为False")
    parser.add_argument("--change_master", action="store_true", default=False, 
                       help="是否修改主臂夹爪行程，默认为False")
    parser.add_argument('--slave_dis', type=int, default=100, choices=[70, 100],
                       help='从臂夹爪行程，默认为100，可选行程为70、100')
    parser.add_argument('--master_dis', type=int, default=100, choices=[70, 100],
                       help='主臂夹爪行程，默认为100，可选行程为70、100')
    args = parser.parse_args()

    # 定义臂列表
    slave_arms = ['can_l_slave', 'can_r_slave']
    master_arms = ['can_l_master', 'can_r_master']
    all_arms = slave_arms + master_arms
    
    # 先查询所有当前状态
    print("=" * 50)
    print("当前状态查询")
    print("=" * 50)
    
    current_states = {}
    for name in all_arms:
        current_states[name] = query_gripper(name)
        print()
    
    # 根据参数执行修改
    print("=" * 50)
    print("执行修改")
    print("=" * 50)
    
    modified = []
    failed = []
    
    if args.change_slave:
        for name in slave_arms:
            current = current_states.get(name)
            if current != args.slave_dis:
                print(f"{name}: 当前={current}, 目标={args.slave_dis}")
                if set_gripper(name, args.slave_dis):
                    modified.append(name)
                else:
                    failed.append(name)
                print()
            else:
                print(f"{name}: 无需修改（已是{args.slave_dis}）\n")
    
    if args.change_master:
        for name in master_arms:
            current = current_states.get(name)
            if current != args.master_dis:
                print(f"{name}: 当前={current}, 目标={args.master_dis}")
                if set_gripper(name, args.master_dis):
                    modified.append(name)
                else:
                    failed.append(name)
                print()
            else:
                print(f"{name}: 无需修改（已是{args.master_dis}）\n")
    
    # 最终汇总
    print("=" * 50)
    print("执行结果汇总")
    print("=" * 50)
    
    if not args.change_slave and not args.change_master:
        print("未指定 --change_slave 或 --change_master，仅执行查询")
    elif not modified and not failed:
        print("所有夹爪已在目标行程，无需修改")
    else:
        if modified:
            print(f"成功修改 ({len(modified)}个): {', '.join(modified)}")
        if failed:
            print(f"修改失败 ({len(failed)}个): {', '.join(failed)}")
    
    print("=" * 50)