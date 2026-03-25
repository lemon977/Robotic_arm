#coding=utf-8
import os
import numpy as np
import h5py
from pathlib import Path

def load_hdf5_split_left_right(dataset_dir, task_name, episode_idx, return_first_row_only=True):
    """
    沿用原代码逻辑读取HDF5文件，拆分qpos为左右手数据，转换为指定列表格式
    :param dataset_dir: 数据集根目录
    :param task_name: 任务名称
    :param episode_idx: episode索引
    :param return_first_row_only: 是否仅返回第一行数据（默认True，符合你的需求）
    :return: 左手数据、右手数据（格式与LEFT0/RIGHT0一致，Python列表）
    """
    # 1. 拼接HDF5文件路径（与原代码逻辑一致）
    dataset_name = f'episode_{episode_idx}'
    full_dataset_dir = os.path.join(dataset_dir, task_name)
    dataset_path = os.path.join(full_dataset_dir, dataset_name + '.hdf5')
    dataset_path = Path(dataset_path).expanduser()
    
    # 2. 校验文件是否存在
    if not os.path.isfile(dataset_path):
        print(f'错误：数据集文件不存在！\n路径：{dataset_path}\n')
        return None, None
    
    # 3. 沿用原代码逻辑读取HDF5文件（只读模式）
    with h5py.File(dataset_path, 'r') as root:
        # 读取核心数据集qpos（关节位置，14个值：前7左、后7右）
        qpos = root['/observations/qpos'][()]
    
    # 4. 核心修改：拆分左右手数据，并转换为指定格式
    def convert_to_target_format(data):
        """将numpy数组转换为与LEFT0/RIGHT0一致的Python列表格式"""
        # 步骤1：拆分前7（左）、后7（右）
        left_data_np = data[:7]  # 前7个：左手
        right_data_np = data[7:] # 后7个：右手
        
        # 步骤2：numpy数组 → Python列表（匹配目标格式）
        # 保留小数精度，与示例[0, 0.0, 0.0, 0.02, 0.43, 0.0, 0.07]格式对齐
        left_data_list = left_data_np.tolist()
        right_data_list = right_data_np.tolist()
        # main(left_data_list, right_data_list)
        return left_data_list, right_data_list
    
    # 5. 提取数据（默认仅返回第一行，如需全量数据可设置return_first_row_only=False）
    if return_first_row_only:
        if len(qpos) == 0:
            print("错误：qpos中无有效数据！")
            return None, None
        # 提取第一行数据（索引0），转换为目标格式
        LEFT_DATA, RIGHT_DATA = convert_to_target_format(qpos[0])
    else:
        # 如需全量数据，返回左右手数据列表（每个元素是对应帧的列表格式数据）
        LEFT_DATA = []
        RIGHT_DATA = []
        for frame_data in qpos:
            left_frame, right_frame = convert_to_target_format(frame_data)
            LEFT_DATA.append(left_frame)
            RIGHT_DATA.append(right_frame)
    
    # 6. 打印结果（验证格式是否匹配）
    # print("=" * 80)
    # print(f"成功读取并拆分qpos数据（格式匹配LEFT0/RIGHT0）")
    # print("=" * 80)
    # print(f"示例格式参考：LEFT0 = [0, 0.0, 0.0, 0.02, 0.43, 0.0, 0.07]")
    # print("-" * 80)
    # print(f"左手数据（前7个值，Python列表格式）：")
    # print(f"LEFT = {LEFT_DATA if return_first_row_only else LEFT_DATA[0]}")
    # print(f"右手数据（后7个值，Python列表格式）：")
    # print(f"RIGHT = {RIGHT_DATA if return_first_row_only else RIGHT_DATA[0]}")
    # print("=" * 80)
    
    with open("/home/agilex/kai05_collect/log/hdf5_position.txt", "w") as f:
        f.write(",".join([str(x) for x in LEFT_DATA]) + "\n")
        f.write(",".join([str(x) for x in RIGHT_DATA]))
        print("hdf5第一帧记录成功，文件目录：hdf5_position.txt")
    # 7. 返回拆分后的左右手数据（匹配目标格式）
    return LEFT_DATA, RIGHT_DATA

if __name__ == '__main__':
    # ====================== 请根据你的实际情况修改以下3个参数 ======================
    DATASET_DIR = "/home/agilex/data/0130"  # 你的数据集根目录（与原代码--dataset_dir对应）
    TASK_NAME = "aloha_mobile_dummy"     # 你的任务名称（与原代码--task_name对应）
    EPISODE_IDX = 0                      # 你的episode索引（与原代码--episode_idx对应）
    # ==============================================================================
    
    # 调用函数，获取拆分后的左右手数据（格式与LEFT0/RIGHT0一致）
    left_qpos, right_qpos = load_hdf5_split_left_right(
        dataset_dir=DATASET_DIR,
        task_name=TASK_NAME,
        episode_idx=EPISODE_IDX,
        return_first_row_only=True  # 保持默认True，仅返回第一行数据
    )
    
    # 后续可直接使用left_qpos和right_qpos，格式与你要求的LEFT0/RIGHT0完全一致
    if left_qpos and right_qpos:
        print(f"\n✅ 数据返回成功，可直接用于后续处理：")
        print(f"   左手数据类型：{type(left_qpos)}，长度：{len(left_qpos)}")
        print(f"   右手数据类型：{type(right_qpos)}，长度：{len(right_qpos)}")
