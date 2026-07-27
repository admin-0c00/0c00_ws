<p align="center">
  <img src="docs/logo.png" alt="零创无穷 0c00" width="360">
</p>

<h1 align="center">SwarmCore-Sim</h1>

<p align="center">
  零创无穷 SwarmCore 无人机蜂群仿真环境 · 一体化离线发行版<br>
  <a href="https://0c00.com">公司官网</a> ·
  <a href="https://0c00.com/ground-station/">在线体验</a> ·
  <a href="https://0c00.com/docs/SwarmCore/ground-station/">使用文档</a> ·
  <a href="https://gitee.com/admin_0c00/0c00_ws">Gitee 仓库</a>
</p>

---

## 这是什么

一套**开箱即用**的无人机蜂群仿真栈：PX4 SITL + Gazebo + ROS 2 Humble + 自研集群控制框架 + Web 地面站。
clone 后一条脚本装完，**无需翻墙、无需拉取任何 git 子模块**：

- **PX4-Autopilot v1.15.4** —— 全部 36 个子模块已拍平为普通文件
- **Micro-XRCE-DDS-Agent v2.4.3** —— PX4 与 ROS 2 之间的 DDS 桥
- **swarm_ws** —— ROS 2 Humble 工作空间（swarm_api 集群框架、Web 地面站、px4_msgs / px4_ros_com 桥接、demo 示例）

## 不想装环境？先在线体验

我们在官网复刻了一套**在线版地面站**，打开浏览器就能看效果：

- 在线体验：https://0c00.com/ground-station/
- 使用说明：https://0c00.com/docs/SwarmCore/ground-station/

觉得合适，再回来装本地完整仿真环境。

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

全程约 30~60 分钟（视机器性能）。脚本幂等——中断后重跑会自动跳过已完成步骤。

## 快速上手

每次新开终端先加载环境：

```bash
source /opt/ros/humble/setup.bash
source ~/0c00_ws/swarm_ws/install/setup.bash   # 按实际 clone 路径调整
```

**1. 启动仿真**（第 2 个参数 0 = 带 Gazebo 界面，1 = 无头模式；第 3 个参数可选机型，默认 gz_x500）：

```bash
~/0c00_ws/swarm_ws/src/bringup/scripts/start_swarm_sim.sh 3 0
# 带深度相机的机型：start_swarm_sim.sh 1 1 gz_x500_depth
# 相机/点云桥接到 ROS 2：start_sensor_bridge.sh（需 ros-humble-ros-gzgarden-bridge）
```

**2. 起飞**：

```bash
ros2 run bringup swarm_takeoff.py
```

**3. 飞一个 demo**（起飞 → 向前 2m → 顺时针 2m 正方形 → 回原点 → 降落）：

```bash
python3 ~/0c00_ws/swarm_ws/src/bringup/scripts/demo_square_enu.py  # ENU 坐标（推荐新手）
python3 ~/0c00_ws/swarm_ws/src/bringup/scripts/demo_square.py      # NED 坐标版
# 可调: --ros-args -p takeoff_alt:=2.0 -p side:=3.0
```

**4. 集群控制框架 swarm_api**（多机并行，仿真/真机同构——真机接入零改动）：

```python
from swarm_api import Swarm

swarm = Swarm(num_drones=3)     # 自动发现在线飞机
swarm.takeoff(1.5)              # 全群同时起飞
swarm.goto_all([(0,2,1.5), (2,2,1.5), (4,2,1.5)])  # 各机飞各自目标
swarm.goto_formation("triangle", spacing=2.0, z=1.5)  # 编队：line/column/triangle/grid
swarm.land()
```

```bash
# 框架版示例（先启动对应机数的仿真，如 start_swarm_sim.sh 3 1）：
python3 ~/0c00_ws/swarm_ws/src/bringup/scripts/demo_single_drone.py  # 单机（Drone 类）
python3 ~/0c00_ws/swarm_ws/src/bringup/scripts/demo_swarm_square.py  # 三机（Swarm 类）
```

**5. Web 地面站**（状态卡片 + 3D 地图 + 指点飞行 + 电子围栏 + 数据录制回放）：

```bash
~/0c00_ws/swarm_ws/src/ground_station/scripts/start_ground_station.sh
# 浏览器打开 http://localhost:8080
```

**停止**：

```bash
~/0c00_ws/swarm_ws/src/bringup/scripts/stop_swarm_sim.sh
~/0c00_ws/swarm_ws/src/ground_station/scripts/stop_ground_station.sh
```

## 文档

| 内容 | 地址 |
| --- | --- |
| Web 地面站使用说明 | https://0c00.com/docs/SwarmCore/ground-station/ |
| swarm_api 框架教程 / API 参考 / demo 教程 | 见官网文档中心 https://0c00.com |
| 更新维护记录 | `docs/MAINTENANCE.md` |

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

---

<p align="center">
  <a href="https://0c00.com">零创无穷 0c00.com</a> · 无人机蜂群系统
</p>
