#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
# Copyright (C) 2026 零创无穷（成都）科技有限责任公司
# 本文件是 SwarmCore-Sim 的一部分，
# 依据 GNU GPL v3 发布（协议全文见仓库根目录 LICENSE）。
# 本软件按"现状"提供，不附带任何明示或默示担保。

"""最简飞行演示：起飞到 1m → 悬停 3s → 降落

验证环境/链路/起飞悬停的最短闭环：takeoff（含水平速度收敛判定，返回即
"悬停住"）→ 保持 3s → land。无航点、无编队，作为一切复杂 demo 的基线。

运行方法：
    ~/0c00_ws/swarm_ws/src/bringup/scripts/start_swarm_sim.sh 1 1   # 单机仿真
    source /opt/ros/humble/setup.bash
    source ~/0c00_ws/swarm_ws/install/setup.bash
    python3 demo_takeoff_hover_land.py

真机运行时把 NS 改成对应命名空间（如 uav_1 / uav_2），其余零改动。
"""

import json
import sys
import time

from swarm_api import Drone

NS = "uav_1"           # 飞机命名空间（单机仿真固定 uav_1；真机按实际改）
TAKEOFF_ALT = 1.0      # 起飞高度 (m)
HOVER_TIME = 3.0       # 悬停时长 (s)


def emit_result(result, **fields):
    """打印机器可解析的运行结果（AI Agent / CI 判定用）。
    约定：进程最后一行输出 DEMO_RESULT <json>，退出码 0=PASS / 1=FAIL。"""
    print("DEMO_RESULT " + json.dumps({"result": result, **fields}, ensure_ascii=False))


def main():
    t0 = time.time()
    stage = "connect"
    drone = None
    try:
        # ---------- 1. 连接飞机 ----------
        drone = Drone(NS)
        print(f"已连接 {NS}")

        # ---------- 2. 起飞到 1m（takeoff 内部会等水平速度收敛，返回即悬停住）----------
        stage = "takeoff"
        print(f">>> 起飞到 {TAKEOFF_ALT}m")
        drone.takeoff(TAKEOFF_ALT)

        # ---------- 3. 悬停 3 秒 ----------
        stage = "hover"
        print(f">>> 悬停 {HOVER_TIME}s")
        drone.hover()
        time.sleep(HOVER_TIME)

        # ---------- 4. 降落 ----------
        stage = "land"
        print(">>> 降落")
        drone.land()
        print("演示完成 ✔")
        emit_result("PASS", drone=NS, alt=TAKEOFF_ALT, hover_s=HOVER_TIME,
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
