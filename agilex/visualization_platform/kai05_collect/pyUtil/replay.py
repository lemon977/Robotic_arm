#!/usr/bin/env python
# -- coding: UTF-8 --
"""
replay.py —— ALOHA 动作回放脚本（简化参数版）
用法：python replay.py ./data/aloha_mobile_dummy/episode_0.hdf5
     python replay.py ./data/episode_5.hdf5 --rate 50
"""
import os
import sys
import time
import argparse
import rospy
import numpy as np
from sensor_msgs.msg import JointState
import h5py
import logging
from pathlib import Path
import json
import datetime

# formatted_date = ""
# try:
#     with open('/home/agilex/kai05_collect/config/config.json', 'r') as f:
#         data = json.load(f)
#         formatted_date = data['date']
#         print('collect_machine2nas ', formatted_date)
# except Exception as e:
#     # 既然没有文件那么说明是第一次运行，当前时间和当前日期保持一致
# 获取当前本地时间
current_time = datetime.datetime.now()
# 格式化为 两位年份+两位月份+两位日期（如260213）
formatted_date = current_time.strftime("%y%m%d")

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',
    handlers=[
        logging.StreamHandler(sys.stdout),
        logging.FileHandler(f'/home/agilex/kai05_collect/log/replay_{formatted_date}.log', mode='a', encoding='utf-8')
    ]
)
logger = logging.getLogger(__name__)

def main():
    parser = argparse.ArgumentParser(
        description="ALOHA动作回放（仅需指定HDF5文件路径）",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="示例:\n"
               "  python replay.py ./data/episode_0.hdf5\n"
               "  python replay.py ./data/episode_0.hdf5 --rate 50\n"
               "  python replay.py ./data/episode_0.hdf5 -r 60 --left /custom/left --right /custom/right"
    )
    parser.add_argument('hdf5_path', type=str, 
                        help="HDF5文件的完整路径（必需）")
    parser.add_argument('-r', '--rate', type=int, default=30,
                        help="回放频率（Hz），默认30")
    parser.add_argument('--left', type=str, default='/master/joint_left',
                        help="左臂控制话题，默认: /master/joint_left")
    parser.add_argument('--right', type=str, default='/master/joint_right',
                        help="右臂控制话题，默认: /master/joint_right")
    args = parser.parse_args()

    # 验证文件路径
    hdf5_path = Path(args.hdf5_path).expanduser().resolve()
    if not hdf5_path.exists():
        logger.error(f"❌ HDF5文件不存在: {hdf5_path}")
        sys.exit(1)
    if hdf5_path.suffix != '.hdf5':
        logger.error(f"❌ 文件扩展名必须为 .hdf5: {hdf5_path}")
        sys.exit(1)

    # 加载HDF5数据
    try:
        with h5py.File(hdf5_path, 'r') as f:
            actions = np.array(f['/action'])  # shape: (T, 14)
    except Exception as e:
        logger.error(f"❌ 无法加载HDF5文件: {e}")
        sys.exit(1)

    if len(actions) == 0:
        logger.error("❌ 动作序列为空")
        sys.exit(1)

    logger.info(f"✅ 成功加载动作序列: {actions.shape[0]} 帧 | 路径: {hdf5_path}")

    # 初始化ROS
    rospy.init_node('replay_node', anonymous=True)
    pub_left = rospy.Publisher(args.left, JointState, queue_size=10)
    pub_right = rospy.Publisher(args.right, JointState, queue_size=10)

    rate = rospy.Rate(args.rate)
    logger.info(f"🎬 开始回放 | 帧数: {len(actions)} | 频率: {args.rate}Hz | 左臂话题: {args.left} | 右臂话题: {args.right}")

    try:
        for i, action in enumerate(actions):
            if rospy.is_shutdown():
                logger.warning("ROS已关闭，提前终止回放")
                break
            
            # 分离左右臂动作（前7维左，后7维右）
            left_action = action[:7]
            right_action = action[7:]

            # 构造JointState消息
            js_left = JointState()
            js_left.header.stamp = rospy.Time.now()
            js_left.position = left_action.tolist()

            js_right = JointState()
            js_right.header.stamp = rospy.Time.now()
            js_right.position = right_action.tolist()

            pub_left.publish(js_left)
            pub_right.publish(js_right)

            if (i + 1) % 50 == 0 or i == len(actions) - 1:
                logger.info(f"  回放进度: {i+1}/{len(actions)} ({(i+1)/len(actions)*100:.1f}%)")
            
            rate.sleep()

        logger.info("✅ 回放完成")
        print("\n✅ 动作回放完成！")

    except KeyboardInterrupt:
        logger.warning("⚠️ 用户中断（Ctrl+C）")
        print("用户中断回放")
    except Exception as e:
        logger.error(f"❌ 回放过程中出错: {str(e)}", exc_info=True)
        sys.exit(1)
    # finally:
    #     # 确保机器人回到初始位置（可选安全措施）
    #     if not rospy.is_shutdown():
    #         logger.info("🛑 发送归零指令（安全停止）...")
    #         zero_action = np.zeros(7)
    #         js_zero = JointState()
    #         js_zero.header.stamp = rospy.Time.now()
    #         js_zero.position = zero_action.tolist()
    #         for _ in range(5):  # 发送5次确保接收
    #             pub_left.publish(js_zero)
    #             pub_right.publish(js_zero)
    #             rospy.sleep(0.1)
    #         logger.info("✅ 安全停止完成")

if __name__ == '__main__':
    main()