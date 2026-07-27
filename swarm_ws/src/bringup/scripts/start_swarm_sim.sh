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
# EKF 高度参考 EKF2_HGT_REF: 0=气压计(默认) 1=GPS 2=测距 3=视觉
# 气压计是"能在启动瞬间就融合"的高度源：GPS 检查通过前 EKF 靠 IMU 推算，
# z 会漂 1m+，漂移值会被 GPS 高度原点锁存成永久偏差（gps_checks.cpp 的
# _gps_alt_ref = gps.alt + pos(2)）。气压计从启动就压住 z，原点才能锁对。
# 真机光流定高行为最接近干净气压计；SENS_EN_BAROSIM 必须显式写（参数持久化，
# airframe 默认值覆盖不了历史保存值）
HGT_REF=${HGT_REF:-0}
# 日志轮转: 只保留最近 5 次仿真的日志（pxh> 刷屏极占磁盘，旧日志及时清）
KEEP_LOGS=5

mkdir -p "$LOG_DIR"
ls -dt "$LOG_BASE"/sim_* 2>/dev/null | tail -n +$((KEEP_LOGS + 1)) | xargs -r rm -rf
echo "[swarm] 日志目录: $LOG_DIR (保留最近 $KEEP_LOGS 次)"

# 清理可能残留的旧进程
pkill -f "px4_sitl_default/bin/px4" 2>/dev/null
pkill -f "MicroXRCEAgent" 2>/dev/null
pkill -f "gz sim" 2>/dev/null
pkill -f "nsh_pipe" 2>/dev/null
sleep 2

# 启动 uXRCE-DDS Agent（所有实例共用 8888 端口，靠 session key 区分）
MicroXRCEAgent udp4 -p 8888 > "$LOG_DIR/agent.log" 2>&1 &
echo $! >> "$PID_FILE"
echo "[swarm] MicroXRCEAgent 已启动 (udp4:8888)"

# 逐架启动 PX4 实例
for i in $(seq 0 $((N - 1))); do
    sysid=$((i + 1))
    y=$((i * 2))
    inst_dir="$ROOTFS/$i"   # 注意：PX4 的工作目录是 rootfs/<实例号>（实例自带的 0/1/2...），
                            # 不是我们以前以为的 rootfs/instance_<i>——参数文件曾长期写错目录未生效
    mkdir -p "$inst_dir"
    # 每架独立的 uXRCE session key；围栏(GF_ACT)；返航高度(RTL_ALT)；EKF 高度参考(HGT_REF，默认 GPS)
    # 注意: 参数是持久化的，所有改过的项必须显式写出，否则沿用上次运行的值
    printf 'param set UXRCE_DDS_KEY %s\nparam set GF_ACTION %s\nparam set GF_MAX_HOR_DIST 10\nparam set GF_MAX_VER_DIST 6\nparam set RTL_RETURN_ALT %s\nparam set EKF2_HGT_REF %s\nparam set EKF2_GPS_CTRL 5\nparam set SENS_EN_BAROSIM 1\n' "$sysid" "$GF_ACT" "$RTL_ALT" "$HGT_REF" > "$inst_dir/px4-rc.params"

    cd "$inst_dir"
    # stdin 用"tail -f 管道文件"：既不 EOF（否则 nsh 死循环刷 pxh> 写爆磁盘），
    # 又能事后追加命令注入 nsh（见脚本尾部 EKF2 重启）
    : > "$inst_dir/nsh_pipe"
    # PATH 加入实例目录：rcS 用 ". px4-rc.params" 按 PATH 搜索，目录不在 PATH 里
    # 就会命中 ROMFS 默认文件，实例参数永远不会生效（踩过的坑）
    PATH="$inst_dir:$PATH" \
    PX4_SIMULATOR="gz" \
    PX4_SIM_MODEL="$MODEL" \
    PX4_GZ_WORLD="default" \
    PX4_UXRCE_DDS_NS="uav_$sysid" \
    PX4_GZ_MODEL_POSE="0,$y,0,0,0,0" \
    nohup "$PX4_BIN" -i "$i" > "$LOG_DIR/px4_uav_$sysid.log" 2>&1 < <(tail -f "$inst_dir/nsh_pipe") &
    echo $! >> "$PID_FILE"
    echo "[swarm] uav_$sysid 启动中 (实例 $i, 出生点 0,$y, 机型 $MODEL) ..."
    # 第一个实例要拉起 Gazebo server，多等一会；后续实例等待 spawn
    if [ "$i" -eq 0 ]; then sleep 12; else sleep 6; fi
done

# EKF2 热重启：GPS 检查通过的时刻 EKF 会把当时的 z 估计锁为高度原点
# （gps_checks.cpp: _gps_alt_ref = gps.alt + pos(2)），而启动初期 z 还没被
# 气压计拉住会漂 1m+，漂移值被锁成永久偏差（地面站看到飞机"出生就在 1 米"）。
# 等传感器收敛后重启 EKF2 重新估计，原点就能锁在真实地面。
if [ "${EKF_RESTART:-1}" = "1" ]; then
    # 启动后几秒传感器已稳定，等太久只是浪费时间（EKF_RESTART_DELAY 可调）
    sleep "${EKF_RESTART_DELAY:-4}"
    for i in $(seq 0 $((N - 1))); do
        pipe="$ROOTFS/$i/nsh_pipe"
        [ -p "$pipe" ] || [ -e "$pipe" ] || continue
        printf 'ekf2 stop\n' >> "$pipe"; sleep 1
        printf 'ekf2 start\n' >> "$pipe"
    done
    echo "[swarm] EKF2 已重启，高度原点重新锁定中（起飞前留 ~10s 收敛）"
fi

echo "[swarm] $N 架仿真无人机启动完成"
echo "[swarm] 停止: stop_swarm_sim.sh   或   kill \$(cat $PID_FILE)"
