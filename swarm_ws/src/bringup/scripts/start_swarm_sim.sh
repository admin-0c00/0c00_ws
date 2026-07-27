#!/bin/bash
# SwarmCore 多机 SITL 一键启动（产品定义书 M2 仿真包）
# 用法: start_swarm_sim.sh [无人机数量=3] [HEADLESS=1] [机型=gz_x500]
# 每架无人机: 独立 PX4 实例, ROS 2 命名空间 /uav_<N>/fmu/..., 出生点 y 轴间隔 2m
# 机型: gz_x500(默认) / gz_x500_depth(深度相机) / gz_x500_vision(双下视+前视) 等，
#       需存在对应 ROMFS airframe 文件（ROMFS/px4fmu_common/init.d-posix/airframes/40xx_<机型>）。
#       自有机型接入时同样放一个 airframe + gz/models/<模型目录> 即可在此直接用。
set -u

N=${1:-3}
# 第 2 参数: 1=无头(不启 Gazebo GUI), 0=带 GUI。rcS 以 HEADLESS 是否为空判断, 传 0 也算"已设置", 必须真正 unset
if [ "${2:-1}" = "0" ]; then
    unset HEADLESS 2>/dev/null || true
else
    export HEADLESS=1
fi
# 第 3 参数: 机型（PX4_SIM_MODEL），默认 gz_x500
MODEL=${3:-gz_x500}
# 机型合法性预检：airframe 文件不存在时 PX4 会起不来，提前报错比翻日志友好
PX4_DIR_CHECK="$HOME/0c00_ws/PX4-Autopilot"
AIRFRAME_DIR="$PX4_DIR_CHECK/ROMFS/px4fmu_common/init.d-posix/airframes"
MODEL_DIR="$PX4_DIR_CHECK/Tools/simulation/gz/models/${MODEL#gz_}"
if ! ls "$AIRFRAME_DIR"/*_"$MODEL" >/dev/null 2>&1; then
    echo "[swarm] 错误: 机型 '$MODEL' 没有对应 airframe（$AIRFRAME_DIR/*_$MODEL 不存在）"
    echo "[swarm] 可用机型: $(ls "$AIRFRAME_DIR" | grep -oP '(?<=_gz_).*' | sed 's/^/gz_/' | sort -u | tr '\n' ' ')"
    exit 1
fi
if [ ! -d "$MODEL_DIR" ]; then
    echo "[swarm] 错误: 机型 '$MODEL' 没有对应 Gazebo 模型目录（$MODEL_DIR 不存在）"
    exit 1
fi

PX4_DIR="$HOME/0c00_ws/PX4-Autopilot"
PX4_BIN="$PX4_DIR/build/px4_sitl_default/bin/px4"
ROOTFS="$PX4_DIR/build/px4_sitl_default/rootfs"
LOG_BASE="$HOME/0c00_ws/swarm_ws/logs"
LOG_DIR="$LOG_BASE/sim_$(date +%Y%m%d_%H%M%S)"
PID_FILE="$LOG_DIR/pids"
# 围栏动作（写进各实例 px4-rc.params）: 0=None 1=Warning 2=Hold 3=Return 5=Land
GF_ACT=${GF_ACT:-3}
# 返航爬升高度 RTL_RETURN_ALT (m): 0=按当前高度返航，不爬升
RTL_ALT=${RTL_ALT:-0}
# EKF 高度参考 EKF2_HGT_REF: 0=气压计 1=GPS(默认) 2=测距 3=视觉
# SensorGpsSim 的高度噪声已从 0.5m 降到 0.02m（仿真 GPS 高度≈真值），
# 因此直接用 GPS 作高度参考即可，气压计路径不再使用
HGT_REF=${HGT_REF:-1}
# 日志轮转: 只保留最近 5 次仿真的日志（pxh> 刷屏极占磁盘，旧日志及时清）
KEEP_LOGS=5

mkdir -p "$LOG_DIR"
ls -dt "$LOG_BASE"/sim_* 2>/dev/null | tail -n +$((KEEP_LOGS + 1)) | xargs -r rm -rf
echo "[swarm] 日志目录: $LOG_DIR (保留最近 $KEEP_LOGS 次)"

# 清理可能残留的旧进程
pkill -f "px4_sitl_default/bin/px4" 2>/dev/null
pkill -f "MicroXRCEAgent" 2>/dev/null
pkill -f "gz sim" 2>/dev/null
pkill -f "tail -f /dev/null" 2>/dev/null
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
    # 每架独立的 uXRCE session key；围栏(GF_ACT)；返航高度(RTL_ALT)；EKF 高度参考(HGT_REF，默认 GPS)
    # 注意: 参数是持久化的，所有改过的项必须显式写出，否则沿用上次运行的值
    printf 'param set UXRCE_DDS_KEY %s\nparam set GF_ACTION %s\nparam set GF_MAX_HOR_DIST 10\nparam set GF_MAX_VER_DIST 6\nparam set RTL_RETURN_ALT %s\nparam set EKF2_HGT_REF %s\nparam set EKF2_GPS_CTRL 7\n' "$sysid" "$GF_ACT" "$RTL_ALT" "$HGT_REF" > "$inst_dir/px4-rc.params"

    cd "$inst_dir"
    # stdin 必须是"永不 EOF"的输入: 若给 /dev/null，nsh 会读 EOF 死循环刷 pxh> 提示符（几小时能写几十 GB 日志）
    PX4_SIMULATOR="gz" \
    PX4_SIM_MODEL="$MODEL" \
    PX4_GZ_WORLD="default" \
    PX4_UXRCE_DDS_NS="uav_$sysid" \
    PX4_GZ_MODEL_POSE="0,$y,0,0,0,0" \
    nohup "$PX4_BIN" -i "$i" > "$LOG_DIR/px4_uav_$sysid.log" 2>&1 < <(tail -f /dev/null) &
    echo $! >> "$PID_FILE"
    echo "[swarm] uav_$sysid 启动中 (实例 $i, 出生点 0,$y, 机型 $MODEL) ..."
    # 第一个实例要拉起 Gazebo server，多等一会；后续实例等待 spawn
    if [ "$i" -eq 0 ]; then sleep 12; else sleep 6; fi
done

echo "[swarm] $N 架仿真无人机启动完成"
echo "[swarm] 停止: stop_swarm_sim.sh   或   kill \$(cat $PID_FILE)"
