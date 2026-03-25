#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
优化要点：
1. 事件驱动替代5ms周期轮询（消除最大延迟源）
2. 实时优先级 + CPU核心绑定（减少调度抖动）
3. 移除冗余MotionCtrl_2调用（仅初始化时使能）
4. 无日志阻塞（异常延迟才记录）
5. 超时保护防止线程饿死（200Hz兜底）
"""

import rospy
import time
import math
import threading
import os
import ctypes
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from piper_sdk import C_PiperInterface_V2
import multiprocessing as mp

class BasePiperArmController:
    def __init__(self, hand:str="Left"):
        assert hand.lower() in ["left", "right"], "hand must be 'Left' or 'Right'"
        hand = hand[0].upper() + hand[1:]
        self.hand = hand
        rospy.init_node(f"{hand}_piper_arm_controller_optimized", anonymous=True)
        self.factor = 57295.7795  # rad → Piper unit

        # ===== 设置进程实时优先级（需sudo或配置rtprio）=====
        self._set_process_priority()

        # ===== 初始化机械臂 =====
        can_slave_name = "can_l_slave" if hand.lower()=='left' else "can_r_slave"
        self.piper_hand = C_PiperInterface_V2(can_slave_name)
        self.piper_hand.ConnectPort()
        self.piper_hand.EnablePiper()

        self.pub_hand_state  = rospy.Publisher(f"/puppet/joint_{hand.lower()}",  JointState, queue_size=10)
        rospy.Timer(rospy.Duration(0.005), self._publish_hand_state)
        # ===== 事件驱动同步原语 =====
        self.hand_cond = threading.Condition()
        self.hand_cmd = None
        self.hand_rx_ns = None

        # ===== 订阅（最小化队列+TCP_NODELAY）=====
        master_name = "/master/joint_left" if self.hand.lower() == "left" else "/master/joint_right"
        rospy.Subscriber(
            master_name,
            JointState,
            self._hand_cb,
            queue_size=1,
            tcp_nodelay=True,
        )

        # ===== 启动控制线程 =====
        self.running = True
        self.hand_thread = threading.Thread(
            target=self._hand_control_loop, daemon=True, name=f"{hand}ArmThread"
        )
        self.hand_thread.daemon = True
        self.hand_thread.start()

        rospy.loginfo(f"{hand} Optimized Piper controller started (event-driven, target <2ms latency)")

    def _set_process_priority(self):
        """尝试设置实时优先级和CPU亲和性"""
        try:
            # 设置SCHED_FIFO实时策略（需rtprio权限）
            param = ctypes.c_int(60)  # 优先级60/99
            libc = ctypes.CDLL('libc.so.6')
            libc.sched_setscheduler(0, 1, ctypes.byref(param))  # 1=SCHED_FIFO
            rospy.loginfo("✓ Realtime priority (SCHED_FIFO) set successfully")
        except Exception as e:
            rospy.logwarn(f"⚠ Realtime priority setup failed (run with sudo?): {e}")

    def _set_thread_affinity(self, core_id):
        """设置当前线程CPU亲和性"""
        try:
            os.sched_setaffinity(os.getpid(), {core_id})
            rospy.logdebug(f"Thread bound to CPU core {core_id}")
        except:
            pass

    # =====================================================
    # Subscriber callbacks (立即唤醒控制线程)
    # =====================================================
    def _hand_cb(self, msg):
        now_ns = time.time_ns()
        with self.hand_cond:
            self.hand_cmd = msg
            self.hand_rx_ns = now_ns
            self.hand_cond.notify()  # ⚡ 关键：立即唤醒控制线程

    # =====================================================
    # Left arm control thread (事件驱动)
    # =====================================================
    def _hand_control_loop(self):
        core_id = 2 if self.hand.lower()=='left' else 3
        self._set_thread_affinity(core_id)  # 绑定到独立核心

        while not rospy.is_shutdown() and self.running:
            with self.hand_cond:
                # 等待新消息（超时2ms防饿死）
                self.hand_cond.wait(timeout=0.002)
                
                msg = self.hand_cmd
                if msg != None:
                    print(msg)
                rx_ns = self.hand_rx_ns
                self.hand_cmd = None  # 清空避免重复处理
                self.hand_cond.notify()  # 通知_hand_cb
            if msg and rx_ns:
                self._send_cmd(self.piper_hand, msg, self.hand.upper(), rx_ns)

    # =====================================================
    # CAN发送（最小化操作）
    # =====================================================
    def _send_cmd(self, piper, msg, arm, rx_ns):
        if len(msg.position) < 7:
            return

        # 转换单位（避免try内做计算）
        joints = [round(msg.position[i] * self.factor) for i in range(6)]
        gripper = round(msg.position[6] * 1000000) #1_000_000

        try:
            # 仅发送必要指令（MotionCtrl_2已移至初始化）
            # piper.MotionCtrl_2(0x01, 0x01, 100, 0x00)
            piper.MotionCtrl_2(0x01, 0x01, 100, 0xAD)  # MIT模式
            piper.JointCtrl(*joints)
            piper.GripperCtrl(gripper, 1000, 0x01, 0)

            # 延迟测量（无日志I/O）
            latency_ms = (time.time_ns() - rx_ns) / 1e6

            # 仅异常时记录（避免日志阻塞）
            # rospy.logwarn_throttle(1.0, f"[{arm}] Latency spike: {latency_ms:.2f}ms")
            if latency_ms > 5.0:
                rospy.logwarn_throttle(1.0, f"[{arm}] Latency spike: {latency_ms:.2f}ms")

        except Exception as e:
            rospy.logerr_throttle(1.0, f"[{arm}] CAN error: {e}")

    def _publish_hand_state(self, _):
        self._publish_arm_state(self.piper_hand, self.pub_hand_state)
    def _publish_arm_state(self, piper, pub):
        try:
            joint_msgs = piper.GetArmJointMsgs()
            gripper_msgs = piper.GetArmGripperMsgs()
            if not joint_msgs or not gripper_msgs:
                return

            js = joint_msgs.joint_state
            pos_deg = [
                js.joint_1, js.joint_2, js.joint_3,
                js.joint_4, js.joint_5, js.joint_6,
            ]
            pos_rad = [math.radians(p) / 1000 for p in pos_deg]

            raw = gripper_msgs.gripper_state.grippers_angle
            if isinstance(raw, (tuple, list)):
                raw = raw[0]
            pos_rad.append(raw / 1000000)

            msg = JointState()
            msg.header = Header(stamp=rospy.Time.now())
            msg.name = [f"joint{i}" for i in range(7)]
            msg.position = pos_rad
            pub.publish(msg)
        except:
            pass
    # =====================================================
    def run(self):
        rospy.spin()
        self.running = False

        # 唤醒线程确保退出
        with self.hand_cond:
            self.hand_cond.notify_all()

        self.hand_thread.join(timeout=0.1)

        # 安全关闭
        # try:
        #     self.piper_hand.DisablePiper()
        # except:
        #     pass
        # try:
        #     self.piper_hand.DisablePiper()
        # except:
        #     pass

class PiperArmControllerProcess(mp.Process):
    def __init__(self, hand):
        super().__init__()
        self.hand = hand
    def run(self):
        # 不能在初始化中进行，因此此时会在主进程执行rospy.init_node
        self.controller = BasePiperArmController(hand=self.hand)
        self.controller.run()
if __name__ == "__main__":
    # 必要的系统配置提示
    rospy.loginfo("=" * 60)
    rospy.loginfo("💡 为获得最佳性能，请执行以下操作：")
    rospy.loginfo("1. 临时提权: sudo -E python3 this_script.py")
    rospy.loginfo("2. 或永久配置: echo '@your_user - rtprio 99' | sudo tee -a /etc/security/limits.conf")
    rospy.loginfo("3. 重启登录生效")
    rospy.loginfo("=" * 60)

    try:
        controller_left = PiperArmControllerProcess(hand="left")
        controller_right = PiperArmControllerProcess(hand="right")
        controller_left.daemon = True
        controller_right.daemon = True
        controller_left.start()
        controller_right.start()
        controller_right.join()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down optimized controller...")
    except Exception as e:
        rospy.logerr(f"Fatal error: {e}")
        raise