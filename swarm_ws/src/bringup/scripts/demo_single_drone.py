#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
# Copyright (C) 2026 零创无穷（成都）科技有限责任公司
# 本文件是 SwarmCore-Sim 的一部分，
# 依据 GNU GPL v3 发布（协议全文见仓库根目录 LICENSE）。
# 本软件按"现状"提供，不附带任何明示或默示担保。

"""SwarmCore 单机飞行演示（swarm_api 框架版，新手入门示例）

一架无人机自动完成：起飞 → 向前飞 → 顺时针画正方形 → 回到原点 → 降落

本脚本是 demo_square_enu.py 的"框架版"，用来展示 swarm_api 对单机的价值：
- 不用关心 QoS、坐标转换、Offboard 流式发送，全部封装在 Drone 类里
- 所有坐标都是 ENU（x=东，y=北，z=上），转换由框架内部完成
- 想升级成多机？把 Drone 换成 Swarm 即可（见 demo_swarm_square.py）

航线俯视图（ENU 坐标：x=东，y=北）：

        北 y
        ↑
   (0,2) ←────── (2,2)     ① 起飞到 1.5m，向前（北）飞 2m
     │              │
     ▼              ▼      ② 右转，向东飞 2m
   (0,0) ──────→ (2,0)     ③ 右转，向南飞 2m
        东 x               ④ 右转，向西飞 2m 回到原点 → 降落

运行方法（和框架集群版完全相同）：
    ~/0c00_ws/swarm_ws/src/bringup/scripts/start_swarm_sim.sh 1 1   # 终端 1：单机仿真
    source /opt/ros/humble/setup.bash
    source ~/0c00_ws/swarm_ws/install/setup.bash
    python3 demo_single_drone.py                                      # 终端 2：运行

Ctrl+C 中断后飞机会自动降落（PX4 失联保护）。
"""

import math

from swarm_api import Drone

NS = "uav_1"        # 飞机命名空间（单机仿真固定是 uav_1）
TAKEOFF_ALT = 1.5   # 起飞高度 (m)
SIDE = 2.0          # 正方形边长 (m)


def main():
    # ---------- 1. 连接飞机 ----------
    # Drone 内部会自动：配置话题 QoS、开启后台 20Hz 设定点流线程、
    # 持续接收飞机状态（位置/解锁/模式），你拿到的 pos 永远是 ENU 坐标
    drone = Drone(NS)
    print(f"已连接 {NS}")

    # ---------- 2. 起飞 ----------
    # 高度基准是"当前实测高度 + 1.5m"，自动规避 EKF 高度原点偏差；
    # 解锁、切 Offboard、命令重发这些细节都在 takeoff 内部完成
    print(">>> 起飞")
    drone.takeoff(TAKEOFF_ALT)

    # ---------- 3. 按航点表飞完整个正方形（坐标全是 ENU） ----------
    # ENU 下"顺时针（俯视）"：北 -> 东 -> 南 -> 西
    legs = [(0, SIDE, "向前飞（北）"),
            (SIDE, SIDE, "右转，向东"),
            (SIDE, 0, "右转，向南"),
            (0, 0, "右转，向西，回到原点")]
    prev = (0, 0)
    for x, y, desc in legs:
        print(f">>> {desc}")
        yaw = math.atan2(y - prev[1], x - prev[0])  # ENU 航向：0=东，逆时针为正
        drone.goto(x, y, TAKEOFF_ALT, yaw=yaw)      # 阻塞式：飞到才返回
        prev = (x, y)

    # ---------- 4. 降落 ----------
    print(">>> 降落")
    drone.land()      # 阻塞式：落地自动上锁后才返回
    drone.shutdown()  # 释放 ROS 资源
    print("演示完成 ✔")


if __name__ == "__main__":
    main()
