# -- coding: UTF-8 --
"""
脚本功能：运行后捕捉当前时间点主臂（左右手臂）的position关节位置数据
核心：订阅主臂ROS话题，获取最新一次关节状态并打印/返回
"""
import rospy
from sensor_msgs.msg import JointState
from datetime import datetime

# ------------- 配置项（与你的主臂话题保持一致，可修改）-------------
MASTER_CONFIG = {
    "master_arm_left_topic": "/master/joint_left",   # 左主臂关节状态话题
    "master_arm_right_topic": "/master/joint_right", # 右主臂关节状态话题
    "wait_timeout": 10.0  # 最大等待时间（秒），超时未获取数据则退出
}
# ---------------------------------------------------------

class MasterArmPositionCapture:
    """主臂当前位置捕捉类：简化订阅，仅获取最新一次关节数据"""
    def __init__(self, master_config):
        self.master_config = master_config
        
        # 存储当前时间点的主臂位置（初始化为None）
        self.current_left_position = None
        self.current_right_position = None
        
        # 标记是否已获取到有效数据
        self.got_left_data = False
        self.got_right_data = False
        
        # 初始化ROS节点和订阅者
        self._init_ros_subscriber()

    def _init_ros_subscriber(self):
        """初始化ROS节点，订阅主臂左右关节状态话题"""
        try:
            # 初始化ROS节点（匿名=True，避免节点名冲突）
            rospy.init_node("master_arm_position_capture_node", anonymous=True)
            
            # 订阅左主臂话题，收到消息后触发回调（仅缓存最新数据）
            rospy.Subscriber(
                self.master_config["master_arm_left_topic"],
                JointState,
                self._master_left_callback,
                queue_size=10
            )
            
            # 订阅右主臂话题，逻辑与左手一致
            rospy.Subscriber(
                self.master_config["master_arm_right_topic"],
                JointState,
                self._master_right_callback,
                queue_size=10
            )
            
            rospy.loginfo("✅ ROS节点初始化完成，开始监听主臂关节状态...")
        except Exception as e:
            rospy.logerr(f"❌ ROS初始化失败：{str(e)}")
            raise

    def _master_left_callback(self, msg):
        """左主臂回调函数：缓存最新的关节position数据"""
        if msg.position and len(msg.position) == 7:  # 校验数据有效性（7个关节）
            self.current_left_position = list(msg.position)  # 转换为Python标准列表
            self.got_left_data = True

    def _master_right_callback(self, msg):
        """右主臂回调函数：缓存最新的关节position数据"""
        if msg.position and len(msg.position) == 7:  # 校验数据有效性（7个关节）
            self.current_right_position = list(msg.position)  # 转换为Python标准列表
            self.got_right_data = True

    def capture_current_position(self):
        """
        核心方法：捕捉当前时间点的主臂位置数据
        :return: 左主臂位置列表、右主臂位置列表、捕捉时间
        """
        capture_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]  # 精确到毫秒
        timeout_start = rospy.Time.now()
        timeout = self.master_config["wait_timeout"]

        # 等待获取有效数据（带超时保护）
        while not rospy.is_shutdown():
            # 检查是否已获取左右手臂数据
            if self.got_left_data and self.got_right_data:
                rospy.loginfo("✅ 已成功获取当前时间点主臂位置数据！")
                break
            
            # 检查是否超时
            elapsed_time = (rospy.Time.now() - timeout_start).to_sec()
            if elapsed_time > timeout:
                rospy.logerr(f"❌ 超时错误：{timeout}秒内未获取到主臂数据，请检查话题是否发布")
                return None, None, capture_time
            
            rospy.loginfo(f"⌛ 等待获取主臂数据中（已等待{elapsed_time:.1f}秒，超时{timeout}秒）...")
            rospy.sleep(0.5)  # 每0.5秒检查一次

        # 打印清晰的结果
        # self._print_capture_result(capture_time)
        # 确保数据不为None（避免报错）
        if self.current_left_position and self.current_right_position:
            with open('/home/agilex/kai05_collect/log/master_position.txt', 'w', encoding='utf-8') as f:
                # 1. 左手臂：将列表中每个浮点数转为字符串，再用逗号拼接后写入
                left_str = ",".join([str(num) for num in self.current_left_position])
                f.write(left_str)
                f.write("\n")  # 换行分隔左右手臂
                
                # 2. 右手臂：同理，注意引用正确的列表（current_right_position）
                right_str = ",".join([str(num) for num in self.current_right_position])
                f.write(right_str)
            print("主臂数据记录成功，文件目录：master_position.txt")
        # 返回捕捉到的数据
        return self.current_left_position, self.current_right_position, capture_time

    def _print_capture_result(self, capture_time):
        """格式化打印捕捉结果"""
        print("\n" + "=" * 80)
        print(f"📸 主臂位置捕捉结果（当前时间：{capture_time}）")
        print("=" * 80)
        print(f"🔴 左主臂position（7个关节）：")
        print(f"   {self.current_left_position}")
        print(f"\n🟢 右主臂position（7个关节）：")
        print(f"   {self.current_right_position}")
        print("=" * 80)

def main():
    """脚本主入口：运行后直接捕捉当前主臂位置"""
    try:
        # 初始化主臂位置捕捉器
        arm_capture = MasterArmPositionCapture(MASTER_CONFIG)
        
        # 执行捕捉（获取当前时间点数据）
        left_pos, right_pos, cap_time = arm_capture.capture_current_position()
        return left_pos, right_pos, cap_time
        # 后续可直接使用left_pos和right_pos（Python列表格式，与LEFT0/RIGHT0一致）
        if left_pos and right_pos:
            rospy.loginfo("📌 数据已就绪，可用于后续处理")
        else:
            rospy.logwarn("📌 未获取到有效主臂数据")
    
    except rospy.ROSInterruptException:
        rospy.logerr("❌ ROS节点被手动中断")
    except Exception as e:
        rospy.logerr(f"❌ 脚本运行出错：{str(e)}")

if __name__ == "__main__":
    main()
