# -- coding: UTF-8 --
"""
独立脚本：机器人手臂回归0点（初始位）
功能：将机器人左右手臂平滑、分步回归到预设的初始0点位置，保留原代码的步长限制和平滑逻辑
"""
import rospy
import threading
from collections import deque
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from sensor_msgs.msg import JointState

# ------------- 配置项（可根据你的机器人修改）-------------
# 0点（初始位）关节位置定义（与原代码保持一致）
# LEFT0 = [0, 0.32, -0.36, 0, 0.24, 0, 0.07]
# RIGHT0 = [0, 0.32, -0.36, 0, 0.24, 0, 0.07]
LEFT0 = [0, 0.0, 0.0, 0.02, 0.43, 0.0, 0.07]
RIGHT0 = [0, 0.0, 0.0, 0.02, 0.43, 0.0, 0.07]
# 每个关节每步最大移动量（与原代码保持一致）
ARM_STEPS_LENGTH = [0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.2]

# ROS话题配置（与原代码保持一致，可根据你的机器人话题修改）
ROS_CONFIG = {
    "puppet_arm_left_topic": "/puppet/joint_left",
    "puppet_arm_right_topic": "/puppet/joint_right",
    "puppet_arm_left_cmd_topic": "/master/joint_left",
    "puppet_arm_right_cmd_topic": "/master/joint_right",
    "publish_rate": 30  # 发布频率（Hz）
}
# ---------------------------------------------------------

class ArmHomeOperator:
    """
    简化版ROS操作类：仅负责机器人手臂回归0点相关的订阅与发布
    """
    def __init__(self, ros_config, arm_steps_length):
        self.ros_config = ros_config
        self.arm_steps_length = arm_steps_length
        
        # 关节状态缓存队列
        self.puppet_arm_left_deque = deque(maxlen=2000)
        self.puppet_arm_right_deque = deque(maxlen=2000)
        
        # ROS发布者与订阅者
        self.puppet_arm_left_publisher = None
        self.puppet_arm_right_publisher = None
        
        # 初始化ROS
        self._init_ros()

    def _init_ros(self):
        """初始化ROS节点、订阅者和发布者"""
        rospy.init_node("arm_home_node", anonymous=True)
        
        # 订阅当前关节状态
        rospy.Subscriber(
            self.ros_config["puppet_arm_left_topic"],
            JointState,
            self._puppet_arm_left_callback,
            queue_size=1000,
            tcp_nodelay=True
        )
        rospy.Subscriber(
            self.ros_config["puppet_arm_right_topic"],
            JointState,
            self._puppet_arm_right_callback,
            queue_size=1000,
            tcp_nodelay=True
        )
        
        # 发布关节控制指令
        self.puppet_arm_left_publisher = rospy.Publisher(
            self.ros_config["puppet_arm_left_cmd_topic"],
            JointState,
            queue_size=10
        )
        self.puppet_arm_right_publisher = rospy.Publisher(
            self.ros_config["puppet_arm_right_cmd_topic"],
            JointState,
            queue_size=10
        )
        
        rospy.loginfo("ROS节点初始化完成，等待关节状态数据...")

    def _puppet_arm_left_callback(self, msg):
        """左手臂关节状态回调"""
        self.puppet_arm_left_deque.append(msg)

    def _puppet_arm_right_callback(self, msg):
        """右手臂关节状态回调"""
        self.puppet_arm_right_deque.append(msg)

    def _publish_joint_state(self, left_joints, right_joints):
        """发布单个关节状态指令"""
        joint_msg = JointState()
        joint_msg.header = Header()
        joint_msg.header.stamp = rospy.Time.now()
        joint_msg.name = ["joint0", "joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
        
        # 发布左手臂
        joint_msg.position = left_joints
        self.puppet_arm_left_publisher.publish(joint_msg)
        
        # 发布右手臂
        joint_msg.position = right_joints
        self.puppet_arm_right_publisher.publish(joint_msg)

    def arm_home(self, left_home=LEFT0, right_home=RIGHT0):
        """
        核心方法：将手臂平滑回归到0点（目标位置）
        :param left_home: 左手臂目标关节位置（列表，长度7）
        :param right_home: 右手臂目标关节位置（列表，长度7）
        """
        rate = rospy.Rate(self.ros_config["publish_rate"])
        
        # 1. 等待获取当前关节状态
        current_left = None
        current_right = None
        while not rospy.is_shutdown():
            if len(self.puppet_arm_left_deque) > 0 and len(self.puppet_arm_right_deque) > 0:
                current_left = list(self.puppet_arm_left_deque[-1].position)
                current_right = list(self.puppet_arm_right_deque[-1].position)
                break
            rospy.loginfo("等待获取当前关节状态...")
            rate.sleep()
        
        rospy.loginfo(f"当前左手臂关节状态：{current_left}")
        rospy.loginfo(f"当前右手臂关节状态：{current_right}")
        rospy.loginfo(f"开始回归0点，目标位置：左={left_home}，右={right_home}")
        
        # 2. 计算每个关节的移动方向（符号）
        left_symbol = [1 if (left_home[i] - current_left[i]) > 0 else -1 for i in range(7)]
        right_symbol = [1 if (right_home[i] - current_right[i]) > 0 else -1 for i in range(7)]
        
        step_count = 0
        while not rospy.is_shutdown():
            step_count += 1
            is_finished = True  # 标记是否已到达目标位置
            
            # 3. 分步更新左手臂关节位置
            for i in range(7):
                diff = abs(left_home[i] - current_left[i])
                if diff < self.arm_steps_length[i]:
                    current_left[i] = left_home[i]
                else:
                    current_left[i] += left_symbol[i] * self.arm_steps_length[i]
                    is_finished = False
            
            # 4. 分步更新右手臂关节位置
            for i in range(7):
                diff = abs(right_home[i] - current_right[i])
                if diff < self.arm_steps_length[i]:
                    current_right[i] = right_home[i]
                else:
                    current_right[i] += right_symbol[i] * self.arm_steps_length[i]
                    is_finished = False
            
            # 5. 发布更新后的关节状态
            self._publish_joint_state(current_left, current_right)
            
            # 6. 检查是否完成回归
            if is_finished:
                rospy.loginfo(f"手臂回归0点完成，总步数：{step_count}")
                break
            
            rospy.loginfo(f"回归中，步数：{step_count}")
            rate.sleep()

def main():
    """脚本主入口"""
    try:
        # 初始化手臂回归操作器
        arm_operator = ArmHomeOperator(ROS_CONFIG, ARM_STEPS_LENGTH)
        # 从日志文件获取最后的主臂位置（如果有）
        left_position = None
        right_position = None
        try:
            with open("/home/agilex/kai05_collect/log/hdf5_position.txt", "r") as f:
                lines = f.readlines()
                if len(lines) >= 2:  # 读取最后两行
                    left_position = [float(x) for x in lines[-2].split(",")]
                    right_position = [float(x) for x in lines[-1].split(",")]
            print("arm_home", left_position, right_position)
        except Exception as e:
            rospy.logwarn(f"读取主臂位置日志文件失败：{e}")
        if left_position is not None and right_position is not None:
            print("执行指定数据")
            arm_operator.arm_home(left_position, right_position)
        else:
            # 执行回归0点
            print("执行默认0点数据")
            arm_operator.arm_home(LEFT0, RIGHT0)
        
    except rospy.ROSInterruptException:
        rospy.logerr("ROS节点被中断")
    except Exception as e:
        rospy.logerr(f"运行出错：{str(e)}")

if __name__ == "__main__":
    main()
