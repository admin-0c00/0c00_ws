#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
# Copyright (C) 2026 零创无穷（成都）科技有限责任公司
# 本文件是 SwarmCore-Sim 的一部分，
# 依据 GNU GPL v3 发布（协议全文见仓库根目录 LICENSE）。
# 本软件按"现状"提供，不附带任何明示或默示担保。

"""画圆演示 3/4：**速度控制**（circle_velocity 纯速度流）

流程：起飞 1m → 机头向前 0.5m 到圆周起点 → `circle_velocity()` 纯速度流绕 2 圈
      → 降落。

控制方式：发**切向速度设定点**（速度矢量恒沿圆周切线、大小恒为 speed），PX4
直接执行速度矢量。无"位置设定点追圆"的径向内切，**半径很准**（实测 ~0.47m）。
但速度模式**没有位置闭环，整圆会漂移**（实测 2 圈漂 0.23~0.31m）。

适用：半径精度优先、单次圈数少、可接受整体漂移的场合。

运行：~/0c00_ws/swarm_ws/src/bringup/scripts/start_swarm_sim.sh 1 1
      python3 demo_circle_velocity.py
真机改 NS。
"""

import json
import sys
import time

from swarm_api import Drone

NS = "uav_1"           # 飞机命名空间
TAKEOFF_ALT = 1.0      # 起飞高度 (m)
DIAMETER = 1.0         # 圆直径 (m)
RADIUS = DIAMETER / 2.0
SPEED = 0.4            # 圆周线速度 (m/s)
LAPS = 2               # 绕圈数


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
        print(f">>> 速度控制：纯切向速度流，半径 {RADIUS}m，绕 {LAPS} 圈")
        drone.circle_velocity(cx, cy, z, RADIUS, SPEED, LAPS)

        stage = "land"
        print(">>> 降落")
        drone.land()
        print("演示完成 ✔")
        emit_result("PASS", drone=NS, mode="velocity", laps=LAPS,
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
