#!/bin/bash
# reset_lidar.sh - 清除 YDLIDAR 残留数据

PORT=${1:-/dev/lidar}

echo "[reset_lidar] 清除雷达残留数据: $PORT"

# 1. 设置波特率并清空串口缓冲区
stty -F $PORT 115200 raw 2>/dev/null || { echo "无法访问 $PORT"; exit 1; }
dd if=$PORT of=/dev/null bs=1024 count=20 iflag=nonblock 2>/dev/null

# 2. 发送 YDLIDAR 软复位指令 (0xA5 0x40)
printf '\xA5\x40' > $PORT 2>/dev/null
sleep 0.3

# 3. 再次清空缓冲区
dd if=$PORT of=/dev/null bs=1024 count=20 iflag=nonblock 2>/dev/null

# 4. 延时等雷达稳定
sleep 1

echo "[reset_lidar] 清除完成"