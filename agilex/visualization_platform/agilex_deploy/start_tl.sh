#!/bin/bash
# alias agx_start='/home/agilex/.config/terminator/agx_start.sh'
# 如果没有参数
if [ $# -eq 0 ]; then
    FILE=/home/agilex/.config/terminator/config
    FILE_SIZE=` ls -l $FILE | awk '{ print $5 }' ` #获取文件本身大小
    # echo $FILE_SIZE
    if [ $FILE_SIZE -le 4000 ]
    then
        cp -p ${FILE}_bp $FILE
    fi

    echo "一键启动 推理"
    terminator -l start_tl
fi