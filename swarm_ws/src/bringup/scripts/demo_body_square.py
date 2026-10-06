#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
# Copyright (C) 2026 零创无穷（成都）科技有限责任公司
# 本文件是 SwarmCore-Sim 的一部分，
# 依据 GNU GPL v3 发布（协议全文见仓库根目录 LICENSE）。
# 本软件按"现状"提供，不附带任何明示或默示担保。

"""单机机体系（FLU）1 米正方形飞行演示

流程：起飞到 0.8m → 向前 1m → 向左 1m → 向后 1m → 向右 1m → 悬停 1s → 降落。

用机体系相对位移（move_body），不需要知道绝对坐标：
  - forward：机头前方，left：机头左侧，up：向上（ENU z）
  - 每次位移以"调用瞬间"的机头航向为基准；全程不转机头，航向保持不变，
    因此 前→左→后→右 各 1m 正好画一个回到起点的 1m 正方形。

航线示意（机头初始朝北，俯视；虚线回到起点）：

        前 (forward)
        ↑
    起点 ──→ 前1m ──→ 左1m
      ↑                 │
    右1m ←────────── 后1m

运行方法：
    ~/0c00_ws/swarm_ws/src/bringup/scripts/start_swarm_sim.sh 1 1   # 单机仿真
    source /opt/ros/humble/setup.bash
    source ~/0c00_ws/swarm_ws/install/setup.bash
    python3 demo_body_square.py

真机运行时把 NS 改成对应命名空间（如 uav_1 / uav_2），其余零改动。
"""

import json
import sys
import time

from swarm_api import Drone

NS = "uav_1"        # 飞机命名空间（单机仿真固定 uav_1；真机按实际改）
TAKEOFF_ALT = 0.8   # 起飞高度 (m)
SIDE = 1.0          # 正方形边长 (m)
HOVER_TIME = 1.0    # 到位后悬停时长 (s)


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

        # ---------- 2. 起飞到 0.8m ----------
        stage = "takeoff"
        print(f">>> 起飞到 {TAKEOFF_ALT}m")
        drone.takeoff(TAKEOFF_ALT)

        # ---------- 3. 机体系画 1m 正方形（每步以当前机头为基准，不转航向）----------
        stage = "square"
        legs = [
            (SIDE, 0.0, "向前飞 1m"),
            (0.0, SIDE, "向左飞 1m"),
            (-SIDE, 0.0, "向后飞 1m"),
            (0.0, -SIDE, "向右飞 1m"),
        ]
        for forward, left, desc in legs:
            print(f">>> {desc}")
            drone.move_body(forward, left)   # 阻塞式：到位才返回

        # ---------- 4. 悬停 1 秒 ----------
        stage = "hover"
        print(f">>> 悬停 {HOVER_TIME}s")
        drone.hover()
        time.sleep(HOVER_TIME)

        # ---------- 5. 降落 ----------
        stage = "land"
        print(">>> 降落")
        drone.land()
        print("演示完成 ✔")
        emit_result("PASS", drone=NS, side=SIDE, legs=len(legs),
                    duration_s=round(time.time() - t0, 1))
        return 0
    except KeyboardInterrupt:
        emit_result("FAIL", stage=stage, error="用户中断(Ctrl+C)")
        return 1
    except Exception as e:      # DroneError 等（连接超时/解锁被拒/move_body 超时）
        emit_result("FAIL", stage=stage, error=str(e))
        return 1
    finally:
        if drone is not None:
            drone.shutdown()  # 释放 ROS 资源


if __name__ == "__main__":
    sys.exit(main())
