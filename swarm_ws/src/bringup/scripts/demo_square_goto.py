#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
# Copyright (C) 2026 零创无穷（成都）科技有限责任公司
# 本文件是 SwarmCore-Sim 的一部分，
# 依据 GNU GPL v3 发布（协议全文见仓库根目录 LICENSE）。
# 本软件按"现状"提供，不附带任何明示或默示担保。

"""画正方形演示：**位置控制**（goto 逐航点）

流程：起飞 1m → 飞回起飞点正上方 → 以起飞点为中心、起飞航向为"前"：
      向前 0.5 边长 → 向左 0.5 → 向后 1 → 向右 1 → 向前 1 → 向左 0.5
      → 向后 0.5 回中心 → 降落。即沿四条正方向边绕完整个正方形，
      每条边都是机体正前/正左/正后/正右的直线，不走对角线。

航线俯视图（机体航向为"前"，边长 SIDE，h=SIDE/2，逆时针绕中心）：

        前
        ↑
   前左 ────── 前右
     ↓           │
     前中 ←── 中心（起飞点）
     ↑           │
   后左 ────── 后右

   中心 →(前h)→ 前中 →(左h)→ 前左 →(后2h)→ 后左 →(右2h)→ 后右
        →(前2h)→ 前右 →(左h)→ 前中 →(后h)→ 中心

控制方式：`Drone.goto()` 逐航点位置控制，每个航点停稳（速度收敛）后再走下一段。
适合检验定位链的绝对精度——角点处能直观看到超调和抖动。

运行：~/0c00_ws/swarm_ws/src/bringup/scripts/start_swarm_sim.sh 1 1
      python3 demo_square_goto.py [边长=1.0]
真机改 NS。
"""

import json
import math
import sys
import time

from swarm_api import Drone

NS = "uav_1"           # 飞机命名空间
TAKEOFF_ALT = 1.0      # 起飞高度 (m)
SIDE = float(sys.argv[1]) if len(sys.argv) > 1 else 1.0  # 正方形边长 (m)
TOL = 0.12             # 角点到达容差 (m)
CORNER_HOLD_S = 0.5    # 每个角点悬停时间 (s)


def emit_result(result, **fields):
    print("DEMO_RESULT " + json.dumps({"result": result, **fields}, ensure_ascii=False))


def main():
    t0 = time.time()
    stage = "connect"
    drone = None
    try:
        drone = Drone(NS)
        print(f"已连接 {NS}")

        t_wait = time.time()
        while drone.pos is None:
            if time.time() - t_wait > 10.0:
                raise TimeoutError(f"{NS}: 等待飞控位置数据超时")
            time.sleep(0.1)

        stage = "takeoff"
        pad = drone.pos          # 起飞前记录起飞点（pos 已就绪）
        print(f">>> 起飞到 {TAKEOFF_ALT}m（起飞点作为正方形中心）")
        drone.takeoff(TAKEOFF_ALT)
        z = drone.pos[2]

        # 爬升段零水平速度命令下飞机仍会物理滑动（真机 log_283 实测 4s 滑
        # 0.65m），先飞回起飞点正上方，保证正方形以起飞点为中心
        stage = "return_pad"
        print(">>> 飞回起飞点正上方")
        drone.goto(pad[0], pad[1], z, tol=0.15)
        cx, cy = drone.pos[0], drone.pos[1]

        # 以当前航向为"前"：forward=(cosψ,sinψ)，right=(sinψ,-cosψ)（ENU）
        yaw = drone.yaw if math.isfinite(drone.yaw) else 0.0
        fx, fy = math.cos(yaw), math.sin(yaw)
        rx, ry = math.sin(yaw), -math.cos(yaw)
        h = SIDE / 2.0

        def corner(fwd, right):
            return (cx + fx * fwd + rx * right, cy + fy * fwd + ry * right)

        # 前中 → 前左 → 后左 → 后右 → 前右 → 前中：逆时针沿四条正方向边绕一整圈
        # （向前 h → 向左 h → 向后 2h → 向右 2h → 向前 2h → 向左 h），
        # 随后由"回中心"一段完成最后向后 h
        legs = [(h, 0.0), (h, -h), (-h, -h), (-h, h), (h, h), (h, 0.0)]  # (前, 右) 偏移
        names = ["前边中点（向前）", "前左角（向左）", "后左角（向后）",
                 "后右角（向右）", "前右角（向前）", "前边中点（向左，闭合）"]

        stage = "square"
        for (f_off, r_off), desc in zip(legs, names):
            x, y = corner(f_off, r_off)
            # 前/左偏移与 WEB 地面站起飞系显示一致（前=X 左=Y），ENU 为 goto 实发值
            print(f">>> {desc} 前{f_off:+.2f} 左{-r_off:+.2f}（ENU {x:.2f},{y:.2f}）")
            drone.goto(x, y, z, tol=TOL)
            time.sleep(CORNER_HOLD_S)

        stage = "return_center"
        print(">>> 回中心")
        drone.goto(cx, cy, z, tol=TOL)
        time.sleep(1.0)

        stage = "land"
        print(">>> 降落")
        drone.land()
        print("演示完成 ✔")
        emit_result("PASS", drone=NS, mode="goto_square", side=SIDE,
                    waypoints=len(legs), duration_s=round(time.time() - t0, 1))
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
