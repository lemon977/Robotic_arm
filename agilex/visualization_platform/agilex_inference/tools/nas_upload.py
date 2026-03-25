import os
import subprocess

# 挂载nas目录
def nas_upload():
    """
    该函数用于挂载NAS目录，如果NAS目录不存在或未挂载成功，则执行挂载操作。可以根据需要修改挂载的方式、参数和处理方式。
    返回值:
        - "success"和相关信息，如果NAS挂载成功；"error"和相关信息，如果NAS挂载失败；"warning"和相关信息，如果NAS已挂载无需重复挂载。
    """
    # 若nas目录不存在，则挂载nas
    try:
        if not nas_select():
            subprocess.run(
                'cd /home/agilex/nas_uploader && echo agx | sudo -S bash nas_mount.sh',
                shell=True
            )
            if nas_select():
                return "success", "NAS挂载成功"
            else:
                return "error", "NAS挂载失败，请检查挂载脚本和NAS状态"
        else:
            return "warning", "NAS已挂载，无需重复挂载"
    except Exception as e:
        return "error", "挂载nas失败，可能是nas_mount.sh脚本执行权限或未存在此脚本"

# 查询nas目录是否存在且是否挂载成功
def nas_select():
    """
    该函数用于查询NAS目录是否存在且是否挂载成功。可以根据需要修改查询的方式和处理方式。
    返回值:
        - True，如果NAS目录存在且挂载成功；否则False。
    """
    if os.path.exists('/mnt/nas'):
        if len(os.listdir('/mnt/nas')) > 0:
            return True
    return False

if __name__ == "__main__":
    nas_upload()