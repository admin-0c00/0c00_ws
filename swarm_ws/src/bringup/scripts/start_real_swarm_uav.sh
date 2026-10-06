#!/bin/bash
# SPDX-License-Identifier: GPL-3.0-only
# Copyright (C) 2026 零创无穷（成都）科技有限责任公司
# 本文件是 SwarmCore-Sim 的一部分，
# 依据 GNU GPL v3 发布（协议全文见仓库根目录 LICENSE）。
# 本软件按"现状"提供，不附带任何明示或默示担保。

# SwarmCore 真机接入一键启动（多机版）
# 链路(每架一套): 飞控 TELEM2 -> CH9121 端口2 (UDP) -> PTY -> MicroXRCEAgent(serial) -> ROS 2 /uav_N/fmu/*
# 前提(每架): ① CH9121 端口2 已配为 UDP Server / 921600 8N1（tools/comm_module_config）
#             ② 飞控 UXRCE_DDS_CFG 指向接 CH9121 的串口，波特率一致，且 extras.txt 带 -n uav_N
#             ③ 该机编号 N == MAV_SYS_ID == UXRCE_DDS_KEY
# 用法: start_real_fleet.sh            # 启动 UAV_LIST 里的全部
#       start_real_fleet.sh 1 2        # 只启动指定编号
set -u

BAUD=921600
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# ---- 机队配置: 编号:模块IP:模块UDP端口 ----
UAV_LIST=(
    "1:192.168.10.100:8888"
    "2:192.168.10.101:8888"
)

LOG_BASE="$HOME/0c00_ws/swarm_ws/logs"
LOG_DIR="$LOG_BASE/real_$(date +%Y%m%d_%H%M%S)"
PID_FILE="$LOG_DIR/pids"
mkdir -p "$LOG_DIR"

command -v MicroXRCEAgent >/dev/null || { echo "[real] 错误: MicroXRCEAgent 不在 PATH"; exit 1; }
[ -f "$DIR/udp_serial_bridge.py" ] || { echo "[real] 错误: 找不到 $DIR/udp_serial_bridge.py"; exit 1; }

# 可选过滤: 只启动命令行指定的编号
WANT=("$@")

start_one() {
    local N=$1 MODULE_IP=$2 MODULE_PORT=$3
    local PTY=/tmp/ttyUAV_$N

    # 清理本编号可能残留的旧进程（[0-9] 边界防止 ttyUAV_1 误杀 ttyUAV_10）
    pkill -f "udp_serial_bridge.py .*${PTY}([^0-9]|$)" 2>/dev/null
    pkill -f "MicroXRCEAgent serial --dev ${PTY}([^0-9]|$)" 2>/dev/null
    rm -f "$PTY"
    sleep 1

    # UDP -> PTY 桥
    nohup python3 "$DIR/udp_serial_bridge.py" "$MODULE_IP" "$MODULE_PORT" "$PTY" \
        > "$LOG_DIR/bridge_uav$N.log" 2>&1 &
    echo $! >> "$PID_FILE"
    sleep 1
    if [ ! -e "$PTY" ]; then
        echo "[real] 警告: uav_$N PTY 未建立，看 $LOG_DIR/bridge_uav$N.log，跳过该机"
        return 1
    fi

    # uXRCE-DDS Agent（串口传输：PX4 走 UART 的是 serial 帧，不是 UDP 帧，不能用 udp4）
    nohup MicroXRCEAgent serial --dev "$PTY" -b "$BAUD" > "$LOG_DIR/agent_uav$N.log" 2>&1 &
    echo $! >> "$PID_FILE"

    echo "[real] uav_$N: $MODULE_IP:$MODULE_PORT -> $PTY -> Agent(serial,$BAUD)"
}

FAIL=0
for entry in "${UAV_LIST[@]}"; do
    IFS=':' read -r N IP PORT <<< "$entry"
    if [ ${#WANT[@]} -gt 0 ]; then
        skip=1
        for w in "${WANT[@]}"; do [ "$w" = "$N" ] && skip=0; done
        [ $skip -eq 1 ] && continue
    fi
    start_one "$N" "$IP" "$PORT" || FAIL=1
done

echo "[real] 日志: $LOG_DIR (agent_uavN.log / bridge_uavN.log)"
echo "[real] 停止全部: kill \$(cat $PID_FILE)"
echo "[real] 地面站照常: start_ground_station.sh ，话题前缀 /uav_N/fmu/"
exit $FAIL
