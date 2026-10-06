#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
# Copyright (C) 2026 零创无穷（成都）科技有限责任公司
# 本文件是 SwarmCore-Sim 的一部分，
# 依据 GNU GPL v3 发布（协议全文见仓库根目录 LICENSE）。
# 本软件按"现状"提供，不附带任何明示或默示担保。

"""单机画圆演示（速度模式轨迹流）：以起飞点为圆心，直径 1m，绕 2 圈后降落

流程：起飞 → 机头向前 0.5m 到圆周起点 → 绕圆 2 圈（半径 0.5m，圆心=起飞点）
      → 降落。

使用 Drone.circle_velocity() 速度模式轨迹流：发切向速度设定点，PX4 直接执行
速度矢量，不受"位置设定点低速率追圆导致径向内切"影响，半径更贴 0.5m。
代价是速度模式无位置闭环、长时间会漂移（2 圈内可控）。

航线示意（俯视，机头初始朝北，世界系 ENU）：

        北 y
        │
     ●──┼──●    ●=圆周，圆心=起飞点(原点)，半径 0.5m
        │       起点=起飞点正北 0.5m，逆时针 2 圈
        ●

运行方法：
    ~/0c00_ws/swarm_ws/src/bringup/scripts/start_swarm_sim.sh 1 1   # 单机仿真
    source /opt/ros/humble/setup.bash
    source ~/0c00_ws/swarm_ws/install/setup.bash
    python3 demo_circle.py

真机运行时把 NS 改成对应命名空间（如 uav_1 / uav_2），其余零改动。
"""

import json
import sys
import time

from swarm_api import Drone

NS = "uav_1"              # 飞机命名空间（单机仿真固定 uav_1；真机按实际改）
TAKEOFF_ALT = 1.0         # 起飞高度 (m)，圆形轨迹有横向速度，高度留足余量
DIAMETER = 1.0            # 圆直径 (m)
RADIUS = DIAMETER / 2.0   # 半径 0.5m
SPEED = 0.5               # 圆周线速度 (m/s)：0.5~1.0 平滑；太小会显得"爬"
LAPS = 2                  # 绕圈数


def emit_result(result, **fields):
    """打印机器可解析的运行结果（AI Agent / CI 判定用）。
    约定：进程最后一行输出 DEMO_RESULT <json>，退出码 0=PASS / 1=FAIL。"""
    print("DEMO_RESULT " + json.dumps({"result": result, **fields}, ensure_ascii=False))


def main():
    t0 = time.time()
    stage = "connect"   # 当前阶段，FAIL 时随结果输出，便于定位
    drone = None
    try:
        # ---------- 1. 连接飞机 ----------
        drone = Drone(NS)
        print(f"已连接 {NS}")

        # ---------- 2. 起飞，记录圆心（起飞点 = 原点） ----------
        stage = "takeoff"
        print(f">>> 起飞到 {TAKEOFF_ALT}m（起飞点作为圆心）")
        drone.takeoff(TAKEOFF_ALT)
        cx, cy = drone.pos[0], drone.pos[1]   # 圆心 = 起飞点
        z = drone.pos[2]                       # 圆周飞行高度 = 起飞到位高度

        # ---------- 3. 机头向前 0.5m 到圆周起点 ----------
        stage = "to_circle"
        print(f">>> 向前飞 {RADIUS}m 到圆周起点")
        drone.move_body(RADIUS, 0.0, 0.0)

        # ---------- 4. 速度模式轨迹流绕圆 2 圈（匀速、半径更准） ----------
        stage = "circle"
        print(f">>> 绕圆 {LAPS} 圈（半径 {RADIUS}m，线速度 {SPEED} m/s，速度模式）")
        # circle_velocity() 阻塞直到转完：以起飞点为圆心，从当前位置方向开始，
        # 切向速度画圆，回到起点后转 follow 悬停。备选：circle() 位置模式。
        drone.circle_velocity(cx, cy, z, RADIUS, SPEED, LAPS)

        # ---------- 5. 降落（回到圆周起点，原地降落） ----------
        stage = "land"
        print(">>> 降落")
        drone.land()
        print("演示完成 ✔")
        emit_result("PASS", drone=NS, diameter=DIAMETER, laps=LAPS,
                    speed=SPEED, duration_s=round(time.time() - t0, 1))
        return 0
    except KeyboardInterrupt:
        emit_result("FAIL", stage=stage, error="用户中断(Ctrl+C)")
        return 1
    except Exception as e:      # DroneError 等（连接超时/解锁被拒/circle 超时）
        emit_result("FAIL", stage=stage, error=str(e))
        return 1
    finally:
        if drone is not None:
            drone.shutdown()  # 释放 ROS 资源


if __name__ == "__main__":
    sys.exit(main())
