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
git clone <本仓库地址>
cd SwarmCore-Sim
./install.sh
```

脚本自动完成：基础工具 → ROS 2 Humble（清华镜像）→ Gazebo Garden →
Python 依赖（清华 pip 镜像）→ MicroXRCEAgent → PX4 SITL 编译 → swarm_ws 编译，最后自检。

全程约 30~60 分钟（视机器性能），中途仅 Gazebo 的 apt 源在境外（通常可直连，如失败请参考文末说明）。

## 使用

每次新开终端先加载环境：

```bash
source /opt/ros/humble/setup.bash
source ~/SwarmCore-Sim/swarm_ws/install/setup.bash   # 按实际 clone 路径调整
```

启动 3 机仿真（第 2 个参数 0 = 无头模式，1 = 带 Gazebo 界面）：

```bash
~/SwarmCore-Sim/swarm_ws/src/bringup/scripts/start_swarm_sim.sh 3 0
```

起飞：

```bash
ros2 run bringup swarm_takeoff.py     # 或按 bringup 包内说明
```

Web 地面站（状态卡片 + 3D 地图）：

```bash
~/SwarmCore-Sim/swarm_ws/src/ground_station/scripts/start_ground_station.sh
# 浏览器打开 http://localhost:8080
```

停止：

```bash
~/SwarmCore-Sim/swarm_ws/src/bringup/scripts/stop_swarm_sim.sh
~/SwarmCore-Sim/swarm_ws/src/ground_station/scripts/stop_ground_station.sh
```

## 目录结构

```
SwarmCore-Sim/
├── install.sh                # 一键安装脚本
├── PX4-Autopilot/            # PX4 v1.15.4（子模块已拍平，tag v1.15.4 保留供版本检测）
├── Micro-XRCE-DDS-Agent/     # DDS 桥源码
└── swarm_ws/src/             # ROS 2 功能包
    ├── bringup/              # 仿真启停脚本、起飞脚本
    ├── ground_station/       # Web 地面站（rosbridge + Three.js）
    ├── swarm_msgs/           # 自定义消息（TargetMap / TaskAssignment）
    ├── px4_msgs/ px4_ros_com/# PX4-ROS2 桥接
    └── ...                   # 其余蜂群骨架包
```

## 注意事项

- 本仓库是**快照发行版**：PX4 子模块已拍平，不能再执行 `git submodule update`；
  如需同步上游 PX4，需重新从上游仓库拍平。
- 仓库根部的 annotated tag `v1.15.4` 请勿删除，PX4 的 CMake 版本检测依赖它。
- 若 Gazebo apt 源（packages.osrfoundation.org）访问失败：手动安装 `gz-garden` 后
  重新运行 `install.sh`（脚本幂等，已完成步骤会自动跳过）。
