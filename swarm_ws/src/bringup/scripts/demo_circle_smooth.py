#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
# Copyright (C) 2026 零创无穷（成都）科技有限责任公司
# 本文件是 SwarmCore-Sim 的一部分，
# 依据 GNU GPL v3 发布（协议全文见仓库根目录 LICENSE）。
# 本软件按"现状"提供，不附带任何明示或默示担保。

"""画圆演示 4/4：**平滑圆控制**（circle_smooth 速度前馈 + 小增益纠正）

流程：起飞 1m → 飞回起飞点 → 机头向前 0.6m 到圆周起点 → `Drone.circle_smooth()`
      绕 2 圈 → 降落。

控制方式：切向**速度前馈**为主（轨迹本身平滑），只用增益 k≈0.3（位置模式
P≈0.95 的 1/3）做径向偏差与相位纠正。针对"位置模式追估计噪声导致物理轨迹
抖动"（真机 log_6：EKF 估计绕圆摆 ±0.11m，物理跟着摆 ±0.13m）——噪声增益
降 ~3 倍，物理轨迹肉眼压线；同时小增益纠正把纯速度模式（circle_velocity）
的开环漂移（2 圈 ~0.3m）和圈数不准兜住。

适用：UWB/视觉等带噪定位源下的平滑轨迹演示；需要严格位置闭环的场合仍用
circle()（轨迹控制）。

运行：~/0c00_ws/swarm_ws/src/bringup/scripts/start_swarm_sim.sh 1 1
      python3 demo_circle_smooth.py
真机改 NS。
"""

import json
import math
import sys
import time

from swarm_api import Drone

NS = "uav_1"           # 飞机命名空间
TAKEOFF_ALT = 1.0      # 起飞高度 (m)
DIAMETER = 1.2         # 圆直径 (m)（与其他画圆 demo 一致）
RADIUS = DIAMETER / 2.0
SPEED = float(sys.argv[1]) if len(sys.argv) > 1 else 0.5  # 圆周线速度 (m/s)
LAPS = 2               # 绕圈数
K = 0.3                # 径向/相位纠正增益 (1/s)，越小越平滑但纠偏越慢


def emit_result(result, **fields):
    print("DEMO_RESULT " + json.dumps({"result": result, **fields}, ensure_ascii=False))


def main():
    t0 = time.time()
    stage = "connect"
    drone = None
    try:
        drone = Drone(NS)
        print(f"已连接 {NS}")

        # Drone() 构造函数不阻塞，首个位置包到达前 pos 是 None，先等数据就绪
        t_wait = time.time()
        while drone.pos is None:
            if time.time() - t_wait > 10.0:
                raise TimeoutError(f"{NS}: 等待飞控位置数据超时")
            time.sleep(0.1)

        stage = "takeoff"
        pad = drone.pos          # 起飞前记录起飞点（pos 已就绪）
        print(f">>> 起飞到 {TAKEOFF_ALT}m（起飞点作为圆心）")
        drone.takeoff(TAKEOFF_ALT)
        z = drone.pos[2]

        # 爬升段零水平速度命令下飞机仍会随速度估计偏差物理滑动
        # （真机 log_283 实测 4s 滑了 0.65m），先飞回起飞点正上方再画圆。
        stage = "return_pad"
        print(">>> 飞回起飞点正上方")
        drone.goto(pad[0], pad[1], z, tol=0.15)
        cx, cy = drone.pos[0], drone.pos[1]

        stage = "to_circle"
        print(f">>> 向前飞 {RADIUS}m 到圆周起点")
        # tol 必须远小于 RADIUS：默认 0.3 对 0.6m 移动等于容忍 50% 欠程，
        # 移动没走完就返回会让下面反算的相位角被噪声主导（log_283 教训）
        drone.move_body(RADIUS, 0.0, 0.0, tol=0.1)

        # 旋转前重锚圆心：捕获的 (cx,cy) 可能在 move_body 期间因 EKF
        # xy reset 过期（真机 log_273 教训）。相位角不受 frame 平移影响，
        # 用"当前位置 - R·相位方向"反算圆心（与双机同心圆 demo 同一逻辑）。
        theta = math.atan2(drone.pos[1] - cy, drone.pos[0] - cx)
        cx = drone.pos[0] - RADIUS * math.cos(theta)
        cy = drone.pos[1] - RADIUS * math.sin(theta)
        print(f"    重锚圆心=({cx:.2f},{cy:.2f})")

        stage = "circle"
        print(f">>> 平滑圆：速度前馈+小增益纠正(k={K})，半径 {RADIUS}m，"
              f"线速度 {SPEED} m/s，绕 {LAPS} 圈")
        drone.circle_smooth(cx, cy, z, RADIUS, SPEED, LAPS, k=K)

        stage = "land"
        print(">>> 降落")
        drone.land()
        print("演示完成 ✔")
        emit_result("PASS", drone=NS, mode="smooth", k=K, laps=LAPS,
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
