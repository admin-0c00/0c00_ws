#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
# Copyright (C) 2026 零创无穷（成都）科技有限责任公司
# 本文件是 SwarmCore-Sim 的一部分，
# 依据 GNU GPL v3 发布（协议全文见仓库根目录 LICENSE）。
# 本软件按"现状"提供，不附带任何明示或默示担保。

"""多机最简飞行演示：N 机起飞到 1m → 悬停 3s → 全群降落（默认 2 机）

验证多机链路/起飞悬停的最短闭环：Swarm.takeoff（含水平速度收敛判定，返回
即各机悬停住）→ 保持 3s → Swarm.land。无航点、无编队，作为一切多机
demo 的基线。

运行方法：
    ~/0c00_ws/swarm_ws/src/bringup/scripts/start_swarm_sim.sh 2 1   # 2 机仿真
    source /opt/ros/humble/setup.bash
    source ~/0c00_ws/swarm_ws/install/setup.bash
    python3 demo_swarm_takeoff_hover_land.py [机数=2]

真机零改动：Swarm(num_drones=N) 自动发现在线飞机（uav_1..uav_N）。
"""

import json
import sys
import time

from swarm_api import Swarm

TAKEOFF_ALT = 1.0      # 起飞高度 (m)
HOVER_TIME = 3.0       # 悬停时长 (s)


def emit_result(result, **fields):
    """打印机器可解析的运行结果（AI Agent / CI 判定用）。
    约定：进程最后一行输出 DEMO_RESULT <json>，退出码 0=PASS / 1=FAIL。"""
    print("DEMO_RESULT " + json.dumps({"result": result, **fields}, ensure_ascii=False))


def main():
    n = int(sys.argv[1]) if len(sys.argv) > 1 else 2
    t0 = time.time()
    stage = "connect"
    swarm = None
    try:
        # ---------- 1. 连接机群 ----------
        swarm = Swarm(num_drones=n)
        ns_list = [d.ns for d in swarm.drones]
        print(f"已连接 {' / '.join(ns_list)}")

        # ---------- 2. 全群起飞到 1m（takeoff 内部会等各机水平速度收敛）----------
        stage = "takeoff"
        print(f">>> {n} 机起飞到 {TAKEOFF_ALT}m")
        swarm.takeoff(TAKEOFF_ALT)

        # ---------- 3. 悬停 3 秒 ----------
        stage = "hover"
        print(f">>> 悬停 {HOVER_TIME}s")
        swarm.hover()
        time.sleep(HOVER_TIME)

        # ---------- 4. 全群降落 ----------
        stage = "land"
        print(">>> 全群降落")
        swarm.land()
        print("演示完成 ✔")
        emit_result("PASS", drones=ns_list, alt=TAKEOFF_ALT, hover_s=HOVER_TIME,
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
