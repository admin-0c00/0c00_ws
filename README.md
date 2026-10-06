<p align="center">
  <img src="docs/logo.png" alt="零创无穷 0c00" width="360">
</p>

<h1 align="center">SwarmCore-Sim</h1>

<p align="center">
  零创无穷 SwarmCore 无人机蜂群仿真/真机一体化验证环境 · 一体化离线发行版<br>
  <a href="https://0c00.com">公司官网</a> ·
  <a href="https://0c00.com/ground-station/">在线体验</a> ·
  <a href="https://0c00.com/docs/SwarmCore/ground-station/">使用文档</a> ·
  <a href="https://gitee.com/admin_0c00/0c00_ws">Gitee 仓库</a>
</p>

---

## 这是什么

一套**开箱即用**的无人机蜂群验证栈：PX4 SITL + Gazebo Garden + ROS 2 Humble + 自研集群控制框架 + Web 地面站。
clone 后一条脚本装完，**无需翻墙、无需拉取任何 git 子模块**：

- **PX4-Autopilot v1.15.4** —— 全部 36 个子模块已拍平为普通文件，默认机型为自研"雀"（gz_que）
- **Micro-XRCE-DDS-Agent v2.4.3** —— PX4 与 ROS 2 之间的 DDS 桥
- **swarm_ws** —— ROS 2 Humble 工作空间（swarm_api 集群框架、Web 地面站、px4_msgs / px4_ros_com 桥接、16 个 demo 示例）

核心特性是**仿真/真机同构**：控制代码只写一遍，仿真里验证通过后，真机零改动直接跑。
这不是口号——单机 demo 已全部在仿真与真机（MicoAir H743 + UWB 室内定位）两侧跑通，
真机实测：goto 到位精度 0.09~0.18m，画圆半径误差 <4%。

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

**1. 启动仿真**（第 2 个参数 0 = 带 Gazebo 界面，1 = 无头模式；第 3 个参数可选机型，默认 gz_que 雀）：

```bash
~/0c00_ws/swarm_ws/src/bringup/scripts/start_swarm_sim.sh 3 0
# 相机/点云桥接到 ROS 2（视觉机型接入后）：start_sensor_bridge.sh（需 ros-humble-ros-gzgarden-bridge）
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
swarm.takeoff(1.5)              # 全群同时起飞（阻塞，返回即悬停住）
swarm.goto_all([(0,2,1.5), (2,2,1.5), (4,2,1.5)])       # 各机飞各自目标（本地 ENU）
swarm.goto_formation("triangle", spacing=2.0, z=1.5)    # 编队：line/column/triangle/grid
swarm.move_body_all((1.0, 0.0, 0.0))   # 各机沿各自机头方向前进 1m
swarm.circle_smooth_all((0, 2, 1.0, 0.6, 0.3, 2))       # 全群平滑画圆（真机推荐模式）
swarm.land()
swarm.shutdown()
```

```bash
# 框架版示例（先启动对应机数的仿真，如 start_swarm_sim.sh 3 1）：
python3 ~/0c00_ws/swarm_ws/src/bringup/scripts/demo_single_drone.py  # 单机（Drone 类）
python3 ~/0c00_ws/swarm_ws/src/bringup/scripts/demo_swarm_square.py  # 三机（Swarm 类）
```

**5. Web 地面站**（状态卡片 + 3D 地图 + 指点飞行（支持转机头/机体相对位移）+ 电子围栏 + 数据录制回放）：

```bash
~/0c00_ws/swarm_ws/src/ground_station/scripts/start_ground_station.sh
# 浏览器打开 http://localhost:8080
```

**停止**：

```bash
~/0c00_ws/swarm_ws/src/bringup/scripts/stop_swarm_sim.sh
~/0c00_ws/swarm_ws/src/ground_station/scripts/stop_ground_station.sh
```

## 演示脚本一览

所有 demo 均带 `DEMO_RESULT` 机器可解析结果输出（见下文"面向 AI Agent / 自动化"），仿真与真机通用（真机改命名空间即可）。

| 类别 | 脚本 | 内容 |
| --- | --- | --- |
| 入门 | `demo_takeoff_hover_land.py` | 起飞 → 悬停 → 降落（最小闭环） |
| 入门 | `demo_square_goto.py` / `demo_square_enu.py` / `demo_square.py` | 正方形航线（goto / 教学版 ENU / NED） |
| 入门 | `demo_body_square.py` | 机体系（FLU）相对位移正方形，无需关心绝对坐标 |
| 画圆 | `demo_circle_position.py` | 逐点 goto 画圆（对照组）：有位置闭环，但走走停停、轨迹呈多边形 |
| 画圆 | `demo_circle_trajectory.py` | 位置轨迹流（带速度/加速度前馈）：匀速平滑、有位置闭环 |
| 画圆 | `demo_circle_velocity.py` / `demo_circle.py` | 纯切向速度流：半径准，但无位置闭环会整体漂移 |
| 画圆 | `demo_circle_smooth.py` | 速度前馈 + 径向 PI 纠正：平滑且兜住漂移，带噪定位（UWB/视觉）下真机推荐 |
| 多机 | `demo_swarm_square.py` | 三机并行正方形（Swarm 类） |
| 多机 | `demo_swarm_takeoff_hover_land.py` | 多机起飞悬停降落 |
| 双机 | `demo_concentric_circle.py` / `demo_dual_circle.py` / `demo_dual_concentric_circle.py` | 双机同心圆/双圆编队，自动处理两机坐标原点偏移 |

## 真机接入

仿真里跑通的 demo，真机**零改动**直接运行。当前支持的硬件栈：

- 飞控：MicoAir H743（本仓库已含板型移植与定制固件补丁）
- 链路：CH9121 串口转以太网模块（串口→UDP→MicroXRCEAgent）
- 室内定位：LinkTrack UWB（水平位置）+ 激光测距（高度）+ 陀螺积分航向，无 GPS/罗盘依赖

```bash
~/0c00_ws/swarm_ws/src/bringup/scripts/start_real_uav.sh        # 单机
~/0c00_ws/swarm_ws/src/bringup/scripts/start_real_swarm_uav.sh  # 多机
```

已验证：真机单机起飞/悬停/降落、goto、正方形、画圆（轨迹/平滑两种模式）全流程。
多机编队：仿真已验证，真机各机链路已打通，编队飞行验证进行中。

## 面向 AI Agent / 自动化

本仓库设计为"人和 AI Agent 都能直接驱动"：

- 控制入口是 Python 库 `swarm_api`（非 GUI），Agent 可直接生成代码驱动整个蜂群；
- 每个 demo 最后一行输出机器可解析结果：`DEMO_RESULT {"result":"PASS",...}`（退出码 0）或 `DEMO_RESULT {"result":"FAIL","stage":...}`（退出码 1）；
- 仿真未启动时 demo 会在 30 秒内 FAIL 退出，不会挂死——可作为 CI/Agent 的环境自检探针；
- `swarm_api` 附带单元测试：`cd swarm_ws/src/swarm_api && python3 -m pytest test/`。

验证环境是否正常：起仿真 → 跑任一 demo → 看 DEMO_RESULT 与退出码。

## 文档

| 内容 | 地址 |
| --- | --- |
| Web 地面站使用说明 | https://0c00.com/docs/SwarmCore/ground-station/ |
| swarm_api 框架教程 / API 参考 / demo 教程 | 见官网文档中心 https://0c00.com |
| 开源协议说明 | `docs/OPEN_SOURCE_LICENSE.md` |
| 贡献者协议（CLA） | `docs/CLA.md` |

## 目录结构

```
0c00_ws/
├── install.sh                # 一键安装脚本
├── PX4-Autopilot/            # PX4 v1.15.4（子模块已拍平，tag v1.15.4 保留供版本检测）
│   └── src/drivers/uwb/linktrack/   # LinkTrack UWB 驱动（毛刺拒绝/中值/低通滤波）
├── Micro-XRCE-DDS-Agent/     # DDS 桥源码
├── swarm_ws/src/             # ROS 2 功能包
│   ├── bringup/              # 仿真/真机启停脚本、起飞脚本、16 个 demo 示例
│   ├── swarm_api/            # 集群控制框架（Drone/Swarm/Strategy，Python）+ 单元测试
│   ├── ground_station/       # Web 地面站（rosbridge + Three.js）
│   ├── swarm_msgs/           # 自定义消息（TargetMap / TaskAssignment）
│   └── px4_msgs/ px4_ros_com/# PX4-ROS2 桥接
└── tools/
    ├── drone_model/          # CAD→仿真/地面站模型转换管线脚本
    ├── comm_module_config/   # 通信模块（CH9121 串口转以太网）Web 配置工具
    └── fc_configurator/      # 飞控配置器（参数/校准/电机测试/固件烧录/机载文件管理，支持网络连接）

# 规划中的功能包（感知 perception_*、融合 swarm_fusion、任务 swarm_task、
# 安全 safety_guard、无人车 ugv_bridge、评估 evaluation 等）
# 按产品定义书 7.1 的结构，在对应子系统开发时创建，不预先放空壳。
```

## 开源协议

本仓库采用**分层开源**策略（详见 `NOTICE` 与中文导读 `docs/OPEN_SOURCE_LICENSE.md`）：

- **核心代码**（bringup、ground_station、swarm_msgs、install.sh 等）：**GPL v3**（见 `LICENSE`）——衍生作品分发时必须开源，防止闭源白嫖；
- **SDK/接口层**（swarm_api 集群控制框架）：**LGPL v3**（见 `swarm_ws/src/swarm_api/LICENSE`）——你的科研代码 import swarm_api 不受传染，只有改动 swarm_api 本身才需开源；
- **文档**（`docs/` 及官网课程）：**CC BY-NC 4.0**——教学自由转载，商业盗用必究；
- **第三方组件**（PX4、Micro-XRCE-DDS-Agent、px4_msgs 等）：保留其各自许可证（见 `NOTICE`）。

"灵簇"、"SwarmCore" 为我司商标（申请注册中），上述协议均不授予商标使用权——可依法使用代码，但衍生作品不得以 "SwarmCore"/"灵簇" 命名。

## 注意事项

- 本仓库是**快照发行版**：PX4 子模块已拍平，不能再执行 `git submodule update`；
  如需同步上游 PX4，需重新从上游仓库拍平。
- 仓库根部的 annotated tag `v1.15.4` 请勿删除，PX4 的 CMake 版本检测依赖它。
- 修改 `swarm_ws/src/swarm_api/` 后必须重新构建（`cd swarm_ws && colcon build --packages-select swarm_api`），
  安装是拷贝而非符号链接，不构建则运行时仍是旧代码。
- 若 Gazebo apt 源（packages.osrfoundation.org）访问失败：手动安装 `gz-garden` 后
  重新运行 `install.sh`（脚本幂等，已完成步骤会自动跳过）。

---

<p align="center">
  <a href="https://0c00.com">零创无穷 0c00.com</a> · 无人机蜂群系统
</p>
