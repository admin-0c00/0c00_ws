#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
# Copyright (C) 2026 零创无穷（成都）科技有限责任公司
# 本文件是 SwarmCore-Sim 的一部分，
# 依据 GNU GPL v3 发布（协议全文见仓库根目录 LICENSE）。
# 本软件按"现状"提供，不附带任何明示或默示担保。

"""画圆演示 1/4：**位置控制**（goto 离散航点）

流程：起飞 1m → 机头向前 0.5m 到圆周起点 → 按 24 点/圈离散航点逐点 goto
      → 绕 2 圈 → 降落。

控制方式：每一时刻发一个**固定位置设定点**，`goto()` 阻塞等到位（tol=0.2）再发
下一点。这是最朴素的画圆方式：实现简单、位置有闭环不会漂，但每点到点之间要
"减速-停顿-再加速"，轨迹是 24 边形、速度慢（实测中位 ~0.15 m/s）、到位判定
(tol) 会让圆收缩（实测半径 ~0.38m 而非 0.5m）。

适用：对轨迹平滑度不敏感、重视实现简单的场合。

运行：~/0c00_ws/swarm_ws/src/bringup/scripts/start_swarm_sim.sh 1 1
      python3 demo_circle_position.py
真机改 NS。
"""

import json
import math
import sys
import time

from swarm_api import Drone

NS = "uav_1"           # 飞机命名空间（单机仿真固定 uav_1；真机按实际改）
TAKEOFF_ALT = 1.0      # 起飞高度 (m)
DIAMETER = 1.0         # 圆直径 (m)
RADIUS = DIAMETER / 2.0
LAPS = 2               # 绕圈数
POINTS_PER_LAP = 24    # 每圈离散航点数（24 边形逼近圆）
TOL = 0.2              # goto 到位判定 (m)


def emit_result(result, **fields):
    print("DEMO_RESULT " + json.dumps({"result": result, **fields}, ensure_ascii=False))


def main():
    t0 = time.time()
    stage = "connect"
    drone = None
    try:
        drone = Drone(NS)
        print(f"已连接 {NS}")

        stage = "takeoff"
        print(f">>> 起飞到 {TAKEOFF_ALT}m（起飞点作为圆心）")
        drone.takeoff(TAKEOFF_ALT)
        cx, cy = drone.pos[0], drone.pos[1]
        z = drone.pos[2]

        stage = "to_circle"
        print(f">>> 向前飞 {RADIUS}m 到圆周起点")
        drone.move_body(RADIUS, 0.0, 0.0)

        stage = "circle"
        print(f">>> 位置控制：24 点/圈离散 goto，绕 {LAPS} 圈")
        yaw0 = drone.yaw   # 起始角度 = 机头航向
        step = 2.0 * math.pi / POINTS_PER_LAP
        total = int(LAPS * POINTS_PER_LAP)
        for i in range(1, total + 1):
            theta = yaw0 + i * step
            drone.goto(cx + RADIUS * math.cos(theta),
                       cy + RADIUS * math.sin(theta), z, tol=TOL)
            if i % POINTS_PER_LAP == 0:
                print(f"  --- 第 {i // POINTS_PER_LAP} 圈完成")

        stage = "land"
        print(">>> 降落")
        drone.land()
        print("演示完成 ✔")
        emit_result("PASS", drone=NS, mode="position", laps=LAPS,
                    duration_s=round(time.time() - t0, 1))
        return 0
    except KeyboardInterrupt:
        emit_result("FAIL", stage=stage, error="用户中断(Ctrl+C)")
        return 1
    except Exception as e:
        emit_result("FAIL", stage=stage, error=str(e))
        return 1
    finally:
        if drone is not None:
            drone.shutdown()


if __name__ == "__main__":
    sys.exit(main())
