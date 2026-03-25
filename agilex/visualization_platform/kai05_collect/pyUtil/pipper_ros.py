#!/usr/bin/env python3
"""
Piper 双臂关节状态发布器 (ROS 1)
- can_l_master -> /master/joint_left  (左臂)
- can_r_master -> /master/joint_right (右臂)
- 消息包含 7 个自由度: 6 个关节 + 1 个夹爪
- 所有角度单位转换为 ROS 标准弧度 (rad)
"""

import rospy
import math
import time
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from piper_sdk import C_PiperInterface_V2


class PiperDualArmPublisher:
    def __init__(self):
        rospy.init_node('piper_dual_arm_publisher', anonymous=True)
        rospy.loginfo("Initializing Piper Dual Arm Publisher...")

        # 关节名称定义 (6关节 + 1夹爪)
        self.joint_names = [
            'joint0', 'joint1', 'joint2',
            'joint3', 'joint4', 'joint5',
            'joint6'
        ]

        # 初始化 CAN 接口 (带重试机制)
        self.piper_left = self._init_piper_with_retry("can_l_master", "left arm (can_l_master)")
        self.piper_right = self._init_piper_with_retry("can_r_master", "right arm (can_r_master)")

        # 创建 Publishers
        self.pub_left = rospy.Publisher('/master/joint_left', JointState, queue_size=10)
        self.pub_right = rospy.Publisher('/master/joint_right', JointState, queue_size=10)

        # 发布频率 (30 Hz)
        self.rate = rospy.Rate(200)
        rospy.loginfo("Piper Dual Arm Publisher ready. Publishing to:")
        rospy.loginfo("  - /master/joint_left  (can_l_master)")
        rospy.loginfo("  - /master/joint_right (can_r_master)")

    def _init_piper_with_retry(self, can_interface, description, max_retries=3):
        """带重试机制的 Piper 接口初始化"""
        for attempt in range(max_retries):
            try:
                piper = C_PiperInterface_V2(can_interface)
                # 使用 False 避免启动时自动搜索参数导致失败
                if piper:
                    piper.ConnectPort(True)
                    rospy.loginfo(f"✓ Connected to {description}")
                    # 短暂等待总线稳定后手动触发参数搜索
                    time.sleep(0.5)
                    piper.SearchMotorMaxAngleSpdAccLimit()
                    return piper
                else:
                    rospy.logwarn(f"Attempt {attempt+1}/{max_retries}: Failed to connect to {description}")
            except Exception as e:
                rospy.logwarn(f"Attempt {attempt+1}/{max_retries}: Exception on {description}: {str(e)}")
            
            time.sleep(1.0)  # 重试间隔
        
        rospy.logerr(f"✗ Failed to connect to {description} after {max_retries} attempts")
        raise RuntimeError(f"Connection failed for {description}")


    def _extract_gripper_angle_deg(self, gripper_ctrl):
        """
        从 ArmGripperCtrl 对象提取夹爪角度（度）
        注意：gripper_ctrl 是第一层对象，实际数据在 gripper_ctrl.gripper_ctrl 中
        """
        try:
            # 两层嵌套访问
            raw_angle = gripper_ctrl.gripper_ctrl.grippers_angle  # 如 6300
            # 转换为度：原始值 / 1000 （根据日志 105200 -> 105.200 推断）
            # print(raw_angle)
            angle_deg = float(raw_angle) / 1000000
            return angle_deg
        except Exception as e:
            rospy.logwarn(f"Failed to extract gripper angle: {e}. Using 0.0 as fallback.")
            return 0.0

    def _build_joint_state(self, piper, arm_name):
        """从 Piper 接口构建 JointState 消息"""
        try:
            # 获取关节数据
            joint_ctrl = piper.GetArmJointCtrl()
            gripper_ctrl = piper.GetArmGripperCtrl()
            
            # print("joint1:",joint_ctrl.joint_ctrl.joint_1)

            if joint_ctrl is None or gripper_ctrl is None:
                rospy.logwarn(f"Received None data from {arm_name}")
                return None

            # 提取6个关节角度 (度 -> 弧度)
            positions_deg = [
                joint_ctrl.joint_ctrl.joint_1,
                joint_ctrl.joint_ctrl.joint_2,
                joint_ctrl.joint_ctrl.joint_3,
                joint_ctrl.joint_ctrl.joint_4,
                joint_ctrl.joint_ctrl.joint_5,
                joint_ctrl.joint_ctrl.joint_6
            ]
            # print(positions_deg)
            positions_rad = [math.radians(pos)/1000 for pos in positions_deg]
            # positions_rad = [pos/1000 for pos in positions_rad]

            # 提取夹爪角度 (度 -> 弧度)
            gripper_angle_deg = self._extract_gripper_angle_deg(gripper_ctrl)
            positions_rad.append(gripper_angle_deg)
            # print(positions_rad)
            # 构建 JointState 消息
            js = JointState()
            js.header = Header()
            js.header.stamp = rospy.Time.now()
            js.name = self.joint_names
            js.position = positions_rad
            js.velocity = [0.0] * 7  # 无速度数据
            js.effort = [0.0] * 7    # 无力矩数据

            return js

        except Exception as e:
            rospy.logerr(f"Error building JointState for {arm_name}: {str(e)}")
            return None

    def run(self):
        """主循环：持续发布双臂关节状态"""
        rospy.loginfo("Starting publishing loop...")
        while not rospy.is_shutdown():
            # 发布左臂 (can_l_master)
            js_left = self._build_joint_state(self.piper_left, "left arm (can_l_master)")
            if js_left:
                self.pub_left.publish(js_left)

            # 发布右臂 (can_r_master)
            js_right = self._build_joint_state(self.piper_right, "right arm (can_r_master)")
            if js_right:
                self.pub_right.publish(js_right)

            self.rate.sleep()

        # 清理资源
        rospy.loginfo("Shutting down, disconnecting CAN interfaces...")
        try:
            self.piper_left.DisconnectPort()
        except:
            pass
        try:
            self.piper_right.DisconnectPort()
        except:
            pass


if __name__ == '__main__':
    try:
        node = PiperDualArmPublisher()
        node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS shutdown requested")
    except Exception as e:
        rospy.logfatal(f"Unhandled exception: {str(e)}")
        raise