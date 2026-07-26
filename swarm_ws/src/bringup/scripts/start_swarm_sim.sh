#!/bin/bash
# SwarmCore 多机 SITL 一键启动（产品定义书 M2 仿真包）
# 用法: start_swarm_sim.sh [无人机数量=3] [HEADLESS=1]
# 每架无人机: 独立 PX4 实例, ROS 2 命名空间 /uav_<N>/fmu/..., 出生点 y 轴间隔 2m
set -u

N=${1:-3}
# 第 2 参数: 1=无头(不启 Gazebo GUI), 0=带 GUI。rcS 以 HEADLESS 是否为空判断, 传 0 也算"已设置", 必须真正 unset
if [ "${2:-1}" = "0" ]; then
    unset HEADLESS 2>/dev/null || true
else
    export HEADLESS=1
fi

PX4_DIR="$HOME/0c00_ws/PX4-Autopilot"
PX4_BIN="$PX4_DIR/build/px4_sitl_default/bin/px4"
ROOTFS="$PX4_DIR/build/px4_sitl_default/rootfs"
LOG_DIR="$HOME/0c00_ws/swarm_ws/logs/sim_$(date +%Y%m%d_%H%M%S)"
PID_FILE="$LOG_DIR/pids"

mkdir -p "$LOG_DIR"
echo "[swarm] 日志目录: $LOG_DIR"

# 清理可能残留的旧进程
pkill -f "px4_sitl_default/bin/px4" 2>/dev/null
pkill -f "MicroXRCEAgent" 2>/dev/null
pkill -f "gz sim" 2>/dev/null
sleep 2

# 启动 uXRCE-DDS Agent（所有实例共用 8888 端口，靠 session key 区分）
MicroXRCEAgent udp4 -p 8888 > "$LOG_DIR/agent.log" 2>&1 &
echo $! >> "$PID_FILE"
echo "[swarm] MicroXRCEAgent 已启动 (udp4:8888)"

# 逐架启动 PX4 实例
for i in $(seq 0 $((N - 1))); do
    sysid=$((i + 1))
    y=$((i * 2))
    inst_dir="$ROOTFS/instance_$i"
    mkdir -p "$inst_dir"
    # 实例工作目录需要能找到 ROMFS 与 gz 环境
    [ -e "$inst_dir/etc" ] || ln -s "$ROOTFS/etc" "$inst_dir/etc"
    # 每架独立的 uXRCE session key
    echo "param set UXRCE_DDS_KEY $sysid" > "$inst_dir/px4-rc.params"

    cd "$inst_dir"
    PX4_SIMULATOR="gz" \
    PX4_SIM_MODEL="gz_x500" \
    PX4_GZ_WORLD="default" \
    PX4_UXRCE_DDS_NS="uav_$sysid" \
    PX4_GZ_MODEL_POSE="0,$y,0,0,0,0" \
    nohup "$PX4_BIN" -i "$i" > "$LOG_DIR/px4_uav_$sysid.log" 2>&1 < /dev/null &
    echo $! >> "$PID_FILE"
    echo "[swarm] uav_$sysid 启动中 (实例 $i, 出生点 0,$y) ..."
    # 第一个实例要拉起 Gazebo server，多等一会；后续实例等待 spawn
    if [ "$i" -eq 0 ]; then sleep 12; else sleep 6; fi
done

echo "[swarm] $N 架仿真无人机启动完成"
echo "[swarm] 停止: stop_swarm_sim.sh   或   kill \$(cat $PID_FILE)"
