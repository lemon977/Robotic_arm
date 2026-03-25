import yaml
import pandas as pd
import shutil
import socket
import json
import logging
import sys
import os
from pathlib import Path
import datetime

# 获取当前文件的目录（pyUtil/）
current_dir = os.path.dirname(os.path.abspath(__file__))
# 获取上级目录（即包含pyUtil的目录）
parent_dir = os.path.dirname(current_dir)
# 将上级目录加入Python的模块搜索路径
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)

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

# 为这个模块创建独立的logger
logger = logging.getLogger(__name__)


def setup_logging():
    """设置日志配置，避免与其他模块冲突"""
    if not logger.handlers:  # 避免重复添加handler
        logger.setLevel(logging.INFO)

        formatter = logging.Formatter(
            '%(asctime)s - [%(module)s:%(funcName)s] - %(levelname)s - %(message)s'
        )

        # 控制台输出
        console_handler = logging.StreamHandler(sys.stdout)
        console_handler.setFormatter(formatter)

        # 文件输出
        log_path = f'/home/agilex/kai05_collect/log/collect_machine2nas_{formatted_date}.log'
        file_handler = logging.FileHandler(log_path, mode='a', encoding='utf-8')
        file_handler.setFormatter(formatter)

        logger.addHandler(console_handler)
        logger.addHandler(file_handler)

    return logger


# 初始化logger
setup_logging()

def notice_nas_service(data_dir, pre_data_dir, episode_idx, prompt='', is_last_one:bool=False):
    # print(f"发送的消息 - data_dir:{data_dir}\npre_data_dir:{pre_data_dir}\nepisode_idx:{episode_idx}\nprompt:{prompt}")
    with open('/home/agilex/kai05_collect/config/data_collect_info.yaml', 'r') as f:
        info = yaml.safe_load(f)
    host = info['NAS_Service']['host']
    port = info['NAS_Service']['port']
    with open('/home/agilex/kai05_collect/config/config.json', 'r') as f:
        config = json.load(f)
    machine_id = config['machine_id']
    machine_name = config['machine_name']
    msg = {
        "collect_machine_id": machine_id,  # 采集机器的编号
        "machine_name": machine_name,  # 采集机械臂的名称
        "data_dir": str(data_dir),  # 数据地址，与config.json中一致即可
        "pre_data_dir": str(pre_data_dir),
        "episode_idx": episode_idx,  # 视频片段编号
        "prompt": prompt,
        "last_one": is_last_one  # 是否为最后一条视频
    }
    print("发送的消息：\n", json.dumps(msg, indent=2, ensure_ascii=False))
    msg = json.dumps(msg, ensure_ascii=False) + '\n\r\n\r'
    msg = msg.encode('utf8')
    try:
        client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        client_socket.connect((host, port))
        client_socket.send(msg)
    except Exception as e:
        logger.error(f'数据发送失败，发送信息为：{msg},报错为：{e}')
        return False

def find_similar_folder(target_folder):
    target_folder = Path(target_folder)
    if target_folder.exists(): # 如果存在，则自身是最相近的数据集
        return target_folder
    # 判断是否存在仅有倒数第三位不同的文件夹
    dataset_name = target_folder.parent.name # prefix/dataset_name/repo_id
    parent_folder = target_folder.parent.parent
    hope_dataset_name = dataset_name.split('_')
    hope_dataset_name = '_'.join(hope_dataset_name[:-3]+['*']+hope_dataset_name[-2:])
    similar_folders = list(parent_folder.glob(hope_dataset_name))
    if len(similar_folders) == 0: # 如果没有仅在数据量上存在差异的数据集，则创建路径
        target_folder.mkdir(parents=True, exist_ok=True)
        return target_folder
    # 检索到仅在数据位置上存在差异的数据集
    similar_folders.sort()
    similar_folder = similar_folders[-1]
    similar_folder.rename(target_folder.parent) # 找到最相近的数据集，重命名为给定数据集
    return target_folder

def rename_folder(target_folder):
    # 根据数据量，重新命名数据集
    target_folder = Path(target_folder)
    count = len(list(target_folder.glob('*.hdf5')))
    dataset_name = target_folder.parent.name  # prefix/dataset_name/repo_id
    hope_dataset_name = dataset_name.split('_')
    hope_dataset_name = '_'.join(hope_dataset_name[:-3] + [f'{count}'] + hope_dataset_name[-2:])
    parent_folder = target_folder.parent.parent
    if hope_dataset_name == dataset_name:
        return
    try:
        target_folder.parent.rename(parent_folder/hope_dataset_name)
    except Exception as e:
        logger.error(f"{target_folder}重命名失败，错误为：{e}")


def copy2nas(data_dir, episode_idx, cameras, is_last_one:bool=False, do_notice_nas_service:bool=False, do_rename_folder:bool=False):
    with open('/home/agilex/kai05_collect/config/config.json', 'r') as f:
        config = json.load(f)
    machine_name = config['machine_name']
    with open('/home/agilex/kai05_collect/config/data_collect_info.yaml', 'r') as f:
        info = yaml.safe_load(f)
    folder = data_dir.split(machine_name)[-1]
    data_dir = Path(data_dir).expanduser()

    nas_data_dir = info['nas_data_dir']
    target_dir = Path(nas_data_dir)/machine_name/folder.lstrip('/')
    if do_rename_folder:
        target_dir = find_similar_folder(target_dir)
    target_hdf5_dir = target_dir/f'episode_{episode_idx}.hdf5'
    src_hdf5_path = data_dir/f'episode_{episode_idx}.hdf5'
    logger.info(f"复制数据{str(src_hdf5_path)} to {str(target_hdf5_dir)}")
    try:
        if target_hdf5_dir.exists():
            logger.info(f"{str(target_hdf5_dir)}已经存在，删除已有数据")
            os.remove(target_hdf5_dir)
        if not target_hdf5_dir.parent.exists():# 创建父目录
            target_hdf5_dir.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(src_hdf5_path, target_hdf5_dir)
        logger.info(f"复制数据{str(src_hdf5_path)} to {str(target_hdf5_dir)}成功")
    except Exception as e:
        logger.error(f"复制数据{str(src_hdf5_path)} to {str(target_hdf5_dir)}失败，错误为{str(e)}")
    src_video_dir = data_dir / 'video'
    target_video_dir = target_dir / 'video'
    for cam in cameras:
        cam_video_path = src_video_dir / cam / f"episode_{episode_idx}.mp4"
        target_cam_video_path = target_video_dir / cam / f"episode_{episode_idx}.mp4"
        logger.info(f"复制数据{str(cam_video_path)} to {str(target_cam_video_path)}")
        try:
            if target_cam_video_path.exists():
                logger.info(f"删除已经存在的{str(target_cam_video_path)}")
                os.remove(target_cam_video_path)
            if not target_cam_video_path.parent.exists(): # 创建视频父目录
                target_cam_video_path.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(cam_video_path, target_cam_video_path)
        except Exception as e:
            logger.error(f"复制数据{str(cam_video_path)} to {str(target_cam_video_path)}失败，错误为{str(e)}")
        else:
            logger.info(f'复制{str(cam_video_path)}到{str(target_cam_video_path)}成功')
    if do_rename_folder:
        rename_folder(target_dir) # 数据集重命名
    if do_notice_nas_service:
        try:
            notice_nas_service(data_dir, episode_idx, prompt='', is_last_one=is_last_one)
        except Exception as e:
            logger.error(f'{data_dir}复制到nas后，通知nas服务失败，错误为{e}')


