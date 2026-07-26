#!/bin/bash
# SwarmCore-Sim 一键安装脚本
# 目标环境: Ubuntu 22.04 (Jammy) x86_64
# 用法: git clone <本仓库> && cd SwarmCore-Sim && ./install.sh
set -euo pipefail

RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'; NC='\033[0m'
info()  { echo -e "${GREEN}[install]${NC} $*"; }
warn()  { echo -e "${YELLOW}[install]${NC} $*"; }
die()   { echo -e "${RED}[install] 错误:${NC} $*" >&2; exit 1; }

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TUNA_APT="https://mirrors.tuna.tsinghua.edu.cn"
TUNA_PIP="https://pypi.tuna.tsinghua.edu.cn/simple"

# ---------- 0. 环境检查 ----------
[ -f /etc/os-release ] && . /etc/os-release
[ "${VERSION_ID:-}" = "22.04" ] || warn "本脚本针对 Ubuntu 22.04 验证，当前系统: ${PRETTY_NAME:-未知}，继续执行可能有风险"
[ -d "$ROOT/PX4-Autopilot" ] || die "未找到 PX4-Autopilot 目录，请在仓库根目录运行本脚本"

# ---------- 1. 基础工具 ----------
info "步骤 1/7: 安装基础工具 ..."
sudo apt-get update -qq
sudo apt-get install -y -qq git curl wget gnupg lsb-release ca-certificates \
    build-essential cmake ninja-build python3-pip python3-venv python3-colcon-common-extensions

# ---------- 2. ROS 2 Humble（清华镜像） ----------
if [ -d /opt/ros/humble ]; then
    info "步骤 2/7: ROS 2 Humble 已安装，跳过"
else
    info "步骤 2/7: 安装 ROS 2 Humble（清华 apt 镜像）..."
    sudo apt-get install -y -qq software-properties-common
    sudo add-apt-repository -y universe
    # ROS 构建农场签名公钥（走 Ubuntu keyserver，无需访问 GitHub）
    sudo mkdir -p /usr/share/keyrings
    gpg --keyserver keyserver.ubuntu.com --recv-keys F42ED6FBAB17C654
    gpg --export F42ED6FBAB17C654 | sudo tee /usr/share/keyrings/ros-archive-keyring.gpg > /dev/null
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] $TUNA_APT/ros2/ubuntu jammy main" \
        | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    sudo apt-get update -qq
    sudo apt-get install -y -qq ros-humble-desktop ros-humble-rosbridge-suite python3-rosdep
fi
# 已装 ROS 但没装 rosbridge 的情况（地面站需要）
if ! dpkg -l ros-humble-rosbridge-suite 2>/dev/null | grep -q '^ii'; then
    info "补装 ros-humble-rosbridge-suite（Web 地面站依赖）..."
    sudo apt-get update -qq && sudo apt-get install -y -qq ros-humble-rosbridge-suite
fi

# ---------- 3. Gazebo Garden ----------
if command -v gz &>/dev/null && gz sim --versions 2>/dev/null | grep -q '^7\.'; then
    info "步骤 3/7: Gazebo Garden 已安装，跳过"
else
    info "步骤 3/7: 安装 Gazebo Garden ..."
    sudo curl -sSL https://packages.osrfoundation.org/gazebo.gpg \
        -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable jammy main" \
        | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
    sudo apt-get update -qq
    sudo apt-get install -y -qq gz-garden
fi

# ---------- 4. Python 依赖（清华 pip 镜像） ----------
info "步骤 4/7: 安装 PX4 Python 依赖（清华 pip 镜像）..."
pip3 install --user -i "$TUNA_PIP" -r "$ROOT/PX4-Autopilot/Tools/setup/requirements.txt"
pip3 install --user -i "$TUNA_PIP" empy==3.3.4 catkin_pkg rospkg jsonschema

# ---------- 5. MicroXRCEAgent ----------
if command -v MicroXRCEAgent &>/dev/null; then
    info "步骤 5/7: MicroXRCEAgent 已安装，跳过"
else
    info "步骤 5/7: 编译安装 MicroXRCEAgent ..."
    cmake -S "$ROOT/Micro-XRCE-DDS-Agent" -B "$ROOT/Micro-XRCE-DDS-Agent/build" \
        -DCMAKE_BUILD_TYPE=Release -DUAGENT_BUILD_TESTS=OFF
    cmake --build "$ROOT/Micro-XRCE-DDS-Agent/build" -j"$(nproc)"
    sudo cmake --install "$ROOT/Micro-XRCE-DDS-Agent/build"
    sudo ldconfig
fi

# ---------- 6. 编译 PX4 SITL ----------
info "步骤 6/7: 编译 PX4 SITL (px4_sitl_default，仅编译不启动仿真) ..."
# 注意: 不能用 `make px4_sitl gz_x500`，那个目标编译完会直接启动仿真进程，脚本将永远无法退出
make -C "$ROOT/PX4-Autopilot" px4_sitl_default -j"$(nproc)"

# ---------- 7. 编译 swarm_ws ----------
info "步骤 7/7: 编译 ROS 2 工作空间 swarm_ws ..."
# ROS 的 setup.bash 引用了未定义变量，source 前必须临时关掉 set -u
set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
set -u
cd "$ROOT/swarm_ws"
colcon build --symlink-install

# ---------- 自检 ----------
info "自检 ..."
FAIL=0
[ -x "$ROOT/PX4-Autopilot/build/px4_sitl_default/bin/px4" ] && info "  ✓ PX4 SITL 可执行文件" || { warn "  ✗ PX4 SITL 可执行文件缺失"; FAIL=1; }
command -v MicroXRCEAgent &>/dev/null && info "  ✓ MicroXRCEAgent" || { warn "  ✗ MicroXRCEAgent 未找到"; FAIL=1; }
[ -f "$ROOT/swarm_ws/install/setup.bash" ] && info "  ✓ swarm_ws 编译产物" || { warn "  ✗ swarm_ws 编译失败"; FAIL=1; }
[ "$FAIL" = 0 ] || die "自检未全部通过，请检查上方日志"

cat <<EOF

$(echo -e "${GREEN}======== 安装完成 ========${NC}")
每次使用前请先加载环境:
    source /opt/ros/humble/setup.bash
    source $ROOT/swarm_ws/install/setup.bash

启动 3 机仿真（无头）:
    $ROOT/swarm_ws/src/bringup/scripts/start_swarm_sim.sh 3 0

启动 Web 地面站:
    $ROOT/swarm_ws/src/ground_station/scripts/start_ground_station.sh
    然后浏览器打开 http://localhost:8080
EOF
