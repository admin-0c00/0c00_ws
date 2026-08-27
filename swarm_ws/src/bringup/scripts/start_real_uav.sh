#!/bin/bash
# SPDX-License-Identifier: GPL-3.0-only
# Copyright (C) 2026 零创无穷（成都）科技有限责任公司
# 本文件是 SwarmCore-Sim 的一部分，
# 依据 GNU GPL v3 发布（协议全文见仓库根目录 LICENSE）。
# 本软件按"现状"提供，不附带任何明示或默示担保。

# SwarmCore 真机接入一键启动
# 链路: 飞控 TELEM2 -> CH9121 端口2 (UDP) -> PTY -> MicroXRCEAgent(serial) -> ROS 2 /uav_N/fmu/*
# 前提: ① CH9121 端口2 已配为 UDP Server / 921600 8N1（tools/comm_module_config）
#       ② 飞控 UXRCE_DDS_CFG 指向接 CH9121 的串口，波特率一致，且 extras.txt 带 -n uav_N
# 用法: start_real_uav.sh [编号N=1] [模块IP=192.168.10.100] [模块UDP端口=8888]
# 多真机时每架一组参数（N 必须等于该机 MAV_SYS_ID 与 UXRCE_DDS_KEY）
set -u

N=${1:-1}
MODULE_IP=${2:-192.168.10.100}
MODULE_PORT=${3:-8888}
BAUD=921600
PTY=/tmp/ttyUAV_$N
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

LOG_BASE="$HOME/0c00_ws/swarm_ws/logs"
LOG_DIR="$LOG_BASE/real_$(date +%Y%m%d_%H%M%S)"
PID_FILE="$LOG_DIR/pids"
mkdir -p "$LOG_DIR"

command -v MicroXRCEAgent >/dev/null || { echo "[real] 错误: MicroXRCEAgent 不在 PATH"; exit 1; }

# 清理本编号可能残留的旧进程
pkill -f "udp_serial_bridge.py .*$PTY" 2>/dev/null
pkill -f "MicroXRCEAgent serial --dev $PTY" 2>/dev/null
sleep 1

# UDP -> PTY 桥
nohup python3 "$DIR/udp_serial_bridge.py" "$MODULE_IP" "$MODULE_PORT" "$PTY" \
    > "$LOG_DIR/bridge.log" 2>&1 &
echo $! >> "$PID_FILE"
sleep 1
[ -e "$PTY" ] || { echo "[real] 错误: PTY 未建立，看 $LOG_DIR/bridge.log"; exit 1; }

# uXRCE-DDS Agent（串口传输：PX4 走 UART 的是 serial 帧，不是 UDP 帧，不能用 udp4）
nohup MicroXRCEAgent serial --dev "$PTY" -b "$BAUD" > "$LOG_DIR/agent.log" 2>&1 &
echo $! >> "$PID_FILE"

echo "[real] uav_$N: $MODULE_IP:$MODULE_PORT -> $PTY -> Agent(serial,$BAUD)"
echo "[real] 日志: $LOG_DIR (agent.log / bridge.log)"
echo "[real] 停止: kill \$(cat $PID_FILE)"
echo "[real] 地面站照常: start_ground_station.sh ，话题前缀 /uav_$N/fmu/"
