#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
# Copyright (C) 2026 零创无穷（成都）科技有限责任公司
# 本文件是 SwarmCore-Sim 的一部分，
# 依据 GNU GPL v3 发布（协议全文见仓库根目录 LICENSE）。
# 本软件按"现状"提供，不附带任何明示或默示担保。

"""双机对置同步旋转演示：1号/2号绕同一物理圆心、同一圆（直径 1.5m）旋转

流程：
  1. 1号、2号同时起飞 1m；
  2. 以 1 号起飞点为**物理圆心** C（半径 0.75m = 直径 1.5m / 2）；
  3. 1 号机头向前 0.75m 到圆上起点（起始角 = 其机头航向 θ1）；
  4. 2 号机飞到与 1 号**相位差 180°** 的对置点（C + R·(cos(θ1+π), sin(θ1+π))）；
  5. 两架用 `circle_velocity`（纯速度流）同步旋转 2 圈：
     两架线速度相同 → 角速度相同（ω=v/R）→ 相位差恒为 180°，不碰撞；
  6. 降落。

【坐标系关键】每架 PX4 的 `vehicle_local_position` 是**各自本地 ENU**（原点=各自
EKF 起点/出生点），两架默认**不共享坐标系**，不能把 1 号本地坐标直接给 2 号当圆心。
本 demo 用 `UAV2_ORIGIN_OFFSET`（2 号本地原点相对 1 号本地原点的世界偏移，ENU）
把圆心换算进 2 号本地系：

  仿真：两机出生点 y 轴间隔 2m -> 偏移 (0.0, 2.0)（start_swarm_sim.sh）
  真机：纯 UWB 定位（EKF2_EV_CTRL=1）两架本地系与锚点系对齐 -> 偏移 (0.0, 0.0)

运行（仿真 2 机）：
    ~/0c00_ws/swarm_ws/src/bringup/scripts/start_swarm_sim.sh 2 1
    source /opt/ros/humble/setup.bash
    source ~/0c00_ws/swarm_ws/install/setup.bash
    python3 demo_dual_circle.py

真机：改 `UAV2_ORIGIN_OFFSET = (0.0, 0.0)`，两架在线即可。
"""

import json
import math
import sys
import time

from swarm_api import Swarm

NS1 = "uav_1"          # 1号机命名空间
NS2 = "uav_2"          # 2号机命名空间
TAKEOFF_ALT = 2      # 起飞高度 (m)
DIAMETER = 1.5         # 圆直径 (m)
RADIUS = DIAMETER / 2.0
SPEED = 0.5            # 圆周线速度 (m/s)，两架相同 → 角速度相同
LAPS = 2               # 绕圈数

# 2号本地ENU原点 相对 1号本地ENU原点 的世界偏移 (ENU: x=东, y=北)。
# 仿真默认 (0, 2)：start_swarm_sim.sh 出生点 y 间隔 2m（uav_1=(0,0), uav_2=(0,2)）。
# 真机（纯UWB锚点共享系）应改为 (0, 0)。
UAV2_ORIGIN_OFFSET = (0.0, 2.0)


def emit_result(result, **fields):
    print("DEMO_RESULT " + json.dumps({"result": result, **fields}, ensure_ascii=False))


def main():
    t0 = time.time()
    stage = "connect"
    swarm = None
    try:
        swarm = Swarm(namespaces=[NS1, NS2])
        d1, d2 = swarm[NS1], swarm[NS2]
        print(f"已连接 {NS1} / {NS2}")

        # ---------- 1. 双机同时起飞；物理圆心 = 1号起飞点 ----------
        stage = "takeoff"
        print(f">>> 双机起飞到 {TAKEOFF_ALT}m")
        swarm.takeoff(TAKEOFF_ALT)
        # 1号本地系里的圆心 = 1号起飞点；2号本地系里的圆心 = 世界圆心 - 偏移
        cx1, cy1 = d1.pos[0], d1.pos[1]
        cx2, cy2 = cx1 - UAV2_ORIGIN_OFFSET[0], cy1 - UAV2_ORIGIN_OFFSET[1]
        z = d1.pos[2]
        print(f"    圆心(世界/1号本地)=({cx1:.2f},{cy1:.2f})  "
              f"2号本地=({cx2:.2f},{cy2:.2f})")

        # ---------- 2. 1号到圆上起点；2号飞到对置点（相位差 180°） ----------
        stage = "to_circle"
        print(f">>> 1号机头向前 {RADIUS}m 到圆上起点")
        d1.move_body(RADIUS, 0.0, 0.0)
        theta1 = d1.yaw                 # 1号起始角 = 机头航向
        opp = (cx2 + RADIUS * math.cos(theta1 + math.pi),
               cy2 + RADIUS * math.sin(theta1 + math.pi))
        print(f">>> 2号飞到对置点 ({opp[0]:.2f}, {opp[1]:.2f})（2号本地系）")
        d2.goto(opp[0], opp[1], z)

        # ---------- 3. 双机同步旋转（各自本地系里的同一物理圆心，相位差恒定）----------
        stage = "rotate"
        print(f">>> 双机同步旋转 {LAPS} 圈（直径 {DIAMETER}m，线速度 {SPEED} m/s，速度模式）")
        swarm.circle_velocity_all([
            (cx1, cy1, z, RADIUS, SPEED, LAPS),   # 1号圆心（1号本地系）
            (cx2, cy2, z, RADIUS, SPEED, LAPS),   # 2号圆心（2号本地系）
        ])

        # ---------- 4. 双机降落 ----------
        stage = "land"
        print(">>> 双机降落")
        swarm.land()
        print("演示完成 ✔")
        emit_result("PASS", drones=[NS1, NS2], diameter=DIAMETER, laps=LAPS,
                    duration_s=round(time.time() - t0, 1))
        return 0
    except KeyboardInterrupt:
        emit_result("FAIL", stage=stage, error="用户中断(Ctrl+C)")
        return 1
    except Exception as e:
        emit_result("FAIL", stage=stage, error=str(e))
        return 1
    finally:
        if swarm is not None:
            swarm.shutdown()


if __name__ == "__main__":
    sys.exit(main())
