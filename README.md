# SwarmCore-Sim

零创无穷 SwarmCore 无人机蜂群仿真环境 —— 一体化发行版。

包含完整可离线安装的仿真栈，**无需翻墙、无需拉取任何 git 子模块**：

- **PX4-Autopilot v1.15.4** —— 全部 36 个子模块已拍平为普通文件
- **Micro-XRCE-DDS-Agent v2.4.3** —— PX4 与 ROS 2 之间的 DDS 桥
- **swarm_ws** —— ROS 2 Humble 工作空间（蜂群功能包 + px4_msgs + px4_ros_com + Web 地面站）

## 系统要求

- Ubuntu 22.04 (Jammy) x86_64
- 建议 4 核 CPU / 8GB 内存以上（3 机仿真）

## 一键安装

```bash
git clone https://gitee.com/admin_0c00/0c00_ws.git
cd 0c00_ws
./install.sh
```

脚本自动完成：基础工具 → ROS 2 Humble（清华镜像）→ Gazebo Garden →
Python 依赖（清华 pip 镜像）→ MicroXRCEAgent → PX4 SITL 编译 → swarm_ws 编译，最后自检。

全程约 30~60 分钟（视机器性能），中途仅 Gazebo 的 apt 源在境外（通常可直连，如失败请参考文末说明）。

## 使用

每次新开终端先加载环境：

```bash
source /opt/ros/humble/setup.bash
source ~/0c00_ws/swarm_ws/install/setup.bash   # 按实际 clone 路径调整
```

启动 3 机仿真（第 2 个参数 0 = 带 Gazebo 界面，1 = 无头模式）：

```bash
~/0c00_ws/swarm_ws/src/bringup/scripts/start_swarm_sim.sh 3 0
```

起飞：

```bash
ros2 run bringup swarm_takeoff.py     # 或按 bringup 包内说明
```

单机飞行演示（客户 Demo：起飞 → 向前 2m → 顺时针 2m 正方形 → 回原点 → 降落）：

```bash
# 先启动单机仿真: ~/0c00_ws/swarm_ws/src/bringup/scripts/start_swarm_sim.sh 1 1
python3 ~/0c00_ws/swarm_ws/src/bringup/scripts/demo_square.py      # NED 坐标版
python3 ~/0c00_ws/swarm_ws/src/bringup/scripts/demo_square_enu.py  # ENU 坐标版（ROS 习惯，推荐新手）
# 可调: --ros-args -p takeoff_alt:=2.0 -p side:=3.0
```

集群控制框架 swarm_api（多机并行控制，仿真/真机同构）：

```python
from swarm_api import Swarm

swarm = Swarm(num_drones=3)     # 自动发现在线飞机，真机零改动接入
swarm.takeoff(1.5)              # 全群同时起飞
swarm.goto_all([(0,2,1.5), (2,2,1.5), (4,2,1.5)])  # 各机飞各自目标，同时执行
swarm.goto_formation("triangle", spacing=2.0, z=1.5)  # 编队：line/column/triangle/grid
swarm.land()
```

```bash
# 框架版示例（先启动对应机数的仿真，如 start_swarm_sim.sh 3 1）：
python3 ~/0c00_ws/swarm_ws/src/bringup/scripts/demo_single_drone.py  # 单机（Drone 类）
python3 ~/0c00_ws/swarm_ws/src/bringup/scripts/demo_swarm_square.py  # 三机（Swarm 类）
# 完整教程见 wiki《SwarmCore 集群控制框架（swarm_api）使用说明与教程》
```

Web 地面站（状态卡片 + 3D 地图）：

```bash
~/0c00_ws/swarm_ws/src/ground_station/scripts/start_ground_station.sh
# 浏览器打开 http://localhost:8080
```

停止：

```bash
~/0c00_ws/swarm_ws/src/bringup/scripts/stop_swarm_sim.sh
~/0c00_ws/swarm_ws/src/ground_station/scripts/stop_ground_station.sh
```

## 目录结构

```
0c00_ws/
├── install.sh                # 一键安装脚本
├── PX4-Autopilot/            # PX4 v1.15.4（子模块已拍平，tag v1.15.4 保留供版本检测）
├── Micro-XRCE-DDS-Agent/     # DDS 桥源码
└── swarm_ws/src/             # ROS 2 功能包
    ├── bringup/              # 仿真启停脚本、起飞脚本、demo 示例
    ├── swarm_api/            # 集群控制框架（Drone/Swarm/Strategy，Python）
    ├── ground_station/       # Web 地面站（rosbridge + Three.js）
    ├── swarm_msgs/           # 自定义消息（TargetMap / TaskAssignment）
    └── px4_msgs/ px4_ros_com/# PX4-ROS2 桥接

# 规划中的功能包（感知 perception_*、融合 swarm_fusion、任务 swarm_task、
# 安全 safety_guard、无人车 ugv_bridge、UWB uwb_driver、评估 evaluation 等）
# 按产品定义书 7.1 的结构，在对应子系统开发时创建，不预先放空壳。
```

## 开源协议

本仓库自研代码以 **Apache License 2.0** 开源（见 `LICENSE`），第三方组件（PX4、Micro-XRCE-DDS-Agent、px4_msgs 等）保留其各自许可证（见 `NOTICE`）。

"灵簇"、"SwarmCore" 为我司商标（申请注册中），Apache-2.0 不授予商标使用权——可自由使用代码，但衍生作品不得以 "SwarmCore"/"灵簇" 命名。

中文导读见 `docs/OPEN_SOURCE_LICENSE.md`。

## 注意事项

- 本仓库是**快照发行版**：PX4 子模块已拍平，不能再执行 `git submodule update`；
  如需同步上游 PX4，需重新从上游仓库拍平。
- 仓库根部的 annotated tag `v1.15.4` 请勿删除，PX4 的 CMake 版本检测依赖它。
- 若 Gazebo apt 源（packages.osrfoundation.org）访问失败：手动安装 `gz-garden` 后
  重新运行 `install.sh`（脚本幂等，已完成步骤会自动跳过）。
