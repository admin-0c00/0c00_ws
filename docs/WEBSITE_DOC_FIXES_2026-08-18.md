# 官网 SwarmCore 文档修改稿（2026-08-18）

> 适用范围：官网文档中心 SwarmCore 专栏 5 篇（overview / quickstart-demo / swarm-api / api-reference / ground-station）。
> 背景：2026-08-18 仿真默认机型改为 gz_que（雀）、x500 系列及其他多旋翼移除；另补 2026-07-27 以来实现已变但文档未同步的内容。
> 用法：每条给出【位置 → 原文 → 改为】，可直接照改。截图/视频类见文末"需重录素材"。

---

## 1. 总览与安装（/docs/SwarmCore/overview）

### 1.1 目录结构过时（骨架包已删、漏 swarm_api）

**原文：**
```
└── swarm_ws/src/             # ROS 2 功能包
    ├── bringup/              # 仿真启停脚本、起飞脚本、演示程序
    ├── ground_station/       # Web 地面站（rosbridge + Three.js）
    ├── swarm_msgs/           # 自定义消息（TargetMap / TaskAssignment）
    ├── px4_msgs/ px4_ros_com/# PX4-ROS2 桥接
    └── ...                   # 其余集群骨架包
```

**改为：**
```
└── swarm_ws/src/             # ROS 2 功能包
    ├── bringup/              # 仿真启停脚本、起飞脚本、demo 示例
    ├── swarm_api/            # 集群控制框架（Drone/Swarm/Strategy，Python）
    ├── ground_station/       # Web 地面站（rosbridge + Three.js）
    ├── swarm_msgs/           # 自定义消息（TargetMap / TaskAssignment）
    └── px4_msgs/ px4_ros_com/# PX4-ROS2 桥接
```
（规划中的感知/融合/任务/安全等子系统包按"代码即现状，文档即规划"原则，开发时才创建，不预建空壳。）

---

## 2. 快速入门（/docs/SwarmCore/quickstart-demo）

### 2.1 §2 补回虚拟机配置要点（维护记录中曾有专题，线上版丢失）

**在"3. 一分钟跑通演示"之前插入：**

> **虚拟机用户注意**：最低 4 vCPU / 8GB 内存；性能不足时 EKF 无法收敛，表现为永远"解锁超时"。虚拟机建议用无头模式（第 2 个参数 1）运行仿真；VMware 用户请关闭 VMware Tools 的时间同步（时间跳变会导致仿真异常）。

### 2.2 §3 预期输出样例是 EKF 高度 bug 时代的数值

**原文：**
```
等待飞控数据…（请先启动仿真）
地面实测高度 1.32 m，目标高度 2.82 m
```

**改为：**
```
等待飞控数据…（请先启动仿真）
地面实测高度 0.03 m，目标高度 1.53 m
```
（2026-07-28 起启动脚本会在传感器收敛后热重启 EKF，高度原点锁在真实地面，实测值应接近 0；若仍长期偏出 ±0.5m 属异常。）

### 2.3 §6 参数表：HGT_REF 默认值错误 + 缺 EKF 热重启参数

**原文（错误行）：**
```
HGT_REF | 启动脚本环境变量 | 1（GPS）| EKF 高度参考 EKF2_HGT_REF
```

**改为：**
```
HGT_REF            | 启动脚本环境变量 | 0（气压计） | EKF 高度参考 EKF2_HGT_REF（气压计从启动即压住 z，是高度原点锁对的前提）
EKF_RESTART        | 启动脚本环境变量 | 1（开）     | 启动后自动热重启 EKF2 重新锁定高度原点；0=关闭
EKF_RESTART_DELAY  | 启动脚本环境变量 | 4（秒）     | 启动后等待多少秒再重启 EKF2
```

### 2.4 §9 脚本速查：缺机型参数与新脚本

**原文：**
```
│   ├── start_swarm_sim.sh     # 仿真启动 [机数] [0=GUI/1=无头]
```

**改为：**
```
│   ├── start_swarm_sim.sh     # 仿真启动 [机数] [0=GUI/1=无头] [机型=gz_que]
│   ├── start_sensor_bridge.sh # 相机/点云桥接到 ROS 2（视觉机型接入后使用）
│   ├── demo_single_drone.py   # 单机演示（swarm_api 框架版）
│   ├── demo_swarm_square.py   # 三机演示（swarm_api 框架版）
```
另在 §9 目录树上方加一句：**当前默认机型为自研"雀"（gz_que），不传机型参数即起飞雀。**

### 2.5 Q3 "高度漂移 ±0.5~1m 正常"已过时

**原文要点：** "仿真 EKF 各实例的海拔有 ±0.5~1m 散布（GPS 仿真噪声特性）……属正常"。

**改为：**
> 2026-07-28 起仿真 GPS 噪声已按真机传感器水平配置、并在启动时热重启 EKF 锁定高度原点，各机高度散布已收敛到厘米级。悬停时位置/高度读数的残留小幅漂移源于定位精度与传感器噪声，属正常现象；真机的漂移幅度取决于定位源质量（动捕/RTK > UWB > 光流 > GPS）。地面站显示的高度是"相对各机起飞点"的语义，以卡片读数为准。

---

## 3. swarm_api 教程（/docs/SwarmCore/swarm-api）

### 3.1 §10 波浪号丢失（两处）

- "仿真 GPS 约 0.10.3m" → "仿真 GPS 约 0.1~0.3m"
- "UWB 典型 0.10.3m" → "UWB 典型 0.1~0.3m"

### 3.2 Q8 行数不实

**原文：** "swarm_api/drone.py 全文约 250 行，注释完整"
**改为：** "swarm_api/drone.py 注释完整，通读它是最好的学习材料"（去掉注定腐化的行数）。

---

## 4. API 接口说明（/docs/SwarmCore/api-reference）——补全 0.1.0 已有接口

### 4.1 Drone 状态属性表加一行

```
nav_state | int | PX4 导航状态原始值（VehicleStatus.nav_state，如 14=Offboard、5=AUTO_RTL）
```

### 4.2 Drone 类补三个方法（插在 hover 之后、land 之前均可）

```
arm(timeout=10.0)
解锁电机（不切换飞行模式）。命令每 0.5s 重发直到确认已解锁；超时抛 DroneError。

disarm(timeout=10.0)
上锁（锁桨）。前一半时间普通上锁；若 PX4 因"判定在空中"持续拒绝，后一半时间
自动升级为强制上锁（kill，VEHICLE_CMD_COMPONENT_ARM_DISARM param2=21196）。
⚠ 警告：飞行中调用必然停桨坠落。地面正常上锁约 0.6s，空中强制停桨约 5.6s。

rtl(timeout=15.0)
返航。命令重发直到飞控确认进入 AUTO_RTL；返航与降落过程由 PX4 执行，模式切换
成功后即返回（不等待落地）。内部先停 Offboard 设定点流（不停流会干扰返航-
降落衔接，飞机悬在返航点上方）；再次 takeoff 会自动重启流。
```

### 4.3 PX4 命令使用表补两行

```
VEHICLE_CMD_NAV_RETURN_TO_LAUNCH (20) | — | — | 返航 | rtl
VEHICLE_CMD_COMPONENT_ARM_DISARM (400) | 0.0 | 21196.0 | 强制上锁（kill，空中立即停桨） | disarm 后半程升级
```

### 4.4 版本号

文档版本改为：**v1.1（2026-08-18），对应 swarm_api 0.1.0（补全 arm/disarm/rtl/nav_state 文档）**。

---

## 5. 地面站手册（/docs/SwarmCore/ground-station）

### 5.1 Q9 同 2.5，漂移表述过时

**改为：**
> 不是 Bug，且已大幅改善：2026-07-28 起仿真 GPS 噪声按真机水平配置并热重启 EKF 锁定高度原点，悬停读数的残留小幅漂移源于定位精度与传感器噪声（厘米级），不影响功能验证。真机想减小漂移：升级定位源（动捕/RTK/UWB）、保持传感器校准。

### 5.2 版本号

文档版本改为：**v1.2（2026-08-18）**。

---

## 需重录素材（文字改不了的部分）

- **地面站手册 §2 的操作实录视频、§9 的两张轨迹截图**：画面里是 x500 机型，默认机型改雀后需用 `start_swarm_sim.sh 3 1`（现为雀）重新录制。
- 快速入门 §3 若有配图同理。
- 在线体验版（浏览器模拟内核）不受机型变更影响。

## 本次不需要改的

- 快速入门、swarm_api 教程中所有命令均不带机型参数，默认机型改雀后**原样可用**。
- overview 的安装流程、系统要求、快照发行版注意事项均无变化。
