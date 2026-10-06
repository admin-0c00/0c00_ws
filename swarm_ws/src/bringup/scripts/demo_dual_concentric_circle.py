#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
# Copyright (C) 2026 零创无穷（成都）科技有限责任公司
# 本文件是 SwarmCore-Sim 的一部分，
# 依据 GNU GPL v3 发布（协议全文见仓库根目录 LICENSE）。
# 本软件按"现状"提供，不附带任何明示或默示担保。

"""双机同心圆演示：起飞 1m，绕同一物理圆心转 2 圈后降落（仿真/真机无缝切换）

流程：
  1. 1号、2号同时起飞到 1m；
  2. 自动测定两机本地 ENU 原点的世界偏移（仿真/真机切换的关键，见下）；
  3. 以 1 号起飞点为物理圆心 C（直径 1.5m，半径 0.75m）；
  4. 1 号机头向前 0.75m 到圆上起点；
  5. 2 号飞到与 1 号相位差 180° 的对置点；
  6. 两架用纯速度流同步旋转 2 圈（线速度相同 → 角速度相同 → 相位差恒定，不碰撞）；
  7. 双机降落。

【仿真/真机无缝切换原理】每架 PX4 的 vehicle_local_position 是各自本地 ENU 系
（原点=各自 EKF 起点），两机默认不共享坐标系。本脚本用 vehicle_global_position
自动测定偏移：

    offset = (g2 - g1) - (l2 - l1)
    g_i: 全局经纬度换算的米制相对位置（两机同一参考点）
    l_i: 各自本地 ENU 位置

- 仿真（GPS 正常）：自动算出出生点间隔（约 (0, 2m)），无需任何手改；
- 真机室内纯 UWB（global 无效）：自动回退 (0, 0)，即两机共享 UWB 锚点系。
同一脚本零改动跑两种环境。

运行：
    # 仿真
    ~/0c00_ws/swarm_ws/src/bringup/scripts/start_swarm_sim.sh 2 0
    # 真机
    ~/0c00_ws/swarm_ws/src/bringup/scripts/start_real_uav.sh
    # 然后（两者相同）
    source /opt/ros/humble/setup.bash
    source ~/0c00_ws/swarm_ws/install/setup.bash
    python3 demo_dual_concentric_circle.py
"""

import json
import math
import os
import sys
import time

import rclpy
from px4_msgs.msg import VehicleGlobalPosition
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from swarm_api import Swarm

NS1 = "uav_1"
NS2 = "uav_2"
TAKEOFF_ALT = 1.0        # 起飞高度 (m)
DIAMETER = 1.2           # 圆直径 (m)
RADIUS = DIAMETER / 2.0
SPEED = 0.5              # 圆周线速度 (m/s)
LAPS = 2                 # 绕圈数

# 全局位置无效时（室内纯 UWB，两机共享锚点系）的回退偏移
FALLBACK_OFFSET = (0.0, 0.0)
# 手动固定偏移：设置环境变量 DUAL_OFFSET="x,y" 则跳过自动测量（真机排障用）
# 例：DUAL_OFFSET="0,0" python3 demo_dual_concentric_circle.py
MANUAL_OFFSET = os.environ.get("DUAL_OFFSET")
# 经纬度判定无效阈值：|lat|或|lon|小于该值视为未定位（室内/无GPS）
INVALID_LATLON = 1e-6

M_PER_DEG_LAT = 111320.0


def emit_result(result, **fields):
    print("DEMO_RESULT " + json.dumps({"result": result, **fields}, ensure_ascii=False))


def measure_origin_offset(d1, d2, duration=3.0):
    """自动测定 2号本地原点 相对 1号本地原点 的世界偏移 (ENU, m)。

    全局定位可用时用经纬度换算；不可用（室内UWB共享系）回退 FALLBACK_OFFSET。

    采样方式：单节点同时订阅两机全局位置，duration 内每 0.1s 取一组
    (g1,l1,g2,l2) 准同时样本，逐 tick 算 offset 后取中位数。
    单发顺序测量（先读1号再读2号）会被悬停漂移污染，实测圆心差 ~0.3m。
    """
    if MANUAL_OFFSET is not None:
        ox, oy = (float(v) for v in MANUAL_OFFSET.split(","))
        print(f"    使用手动固定偏移 DUAL_OFFSET = ({ox:.2f}, {oy:.2f})")
        return (ox, oy)
    node = rclpy.create_node("_gpos_probe_dual")
    g = {}

    def cb1(m):
        g["g1"] = (m.lat, m.lon)

    def cb2(m):
        g["g2"] = (m.lat, m.lon)

    # PX4 uXRCE-DDS 输出是 best-effort + volatile，必须匹配，否则收不到消息
    qos = QoSProfile(depth=10,
                     reliability=ReliabilityPolicy.BEST_EFFORT,
                     durability=DurabilityPolicy.VOLATILE)
    node.create_subscription(VehicleGlobalPosition,
                             f"/{NS1}/fmu/out/vehicle_global_position", cb1, qos)
    node.create_subscription(VehicleGlobalPosition,
                             f"/{NS2}/fmu/out/vehicle_global_position", cb2, qos)
    samples = []
    t0 = time.time()
    try:
        while time.time() - t0 < duration:
            rclpy.spin_once(node, timeout_sec=0.05)
            if "g1" in g and "g2" in g:
                g1, g2 = g["g1"], g["g2"]
                if abs(g1[0]) > INVALID_LATLON and abs(g1[1]) > INVALID_LATLON \
                        and abs(g2[0]) > INVALID_LATLON and abs(g2[1]) > INVALID_LATLON:
                    lat0 = math.radians((g1[0] + g2[0]) / 2.0)
                    dg = ((g2[1] - g1[1]) * M_PER_DEG_LAT * math.cos(lat0),
                          (g2[0] - g1[0]) * M_PER_DEG_LAT)
                    dl = (d2.pos[0] - d1.pos[0], d2.pos[1] - d1.pos[1])
                    samples.append((dg[0] - dl[0], dg[1] - dl[1]))
            time.sleep(0.05)
    finally:
        node.destroy_node()
    if not samples:
        print(f"    全局定位无效（室内UWB模式），回退偏移 {FALLBACK_OFFSET}")
        return FALLBACK_OFFSET
    xs = sorted(s[0] for s in samples)
    ys = sorted(s[1] for s in samples)
    offset = (xs[len(xs) // 2], ys[len(ys) // 2])
    print(f"    自动测定原点偏移 = ({offset[0]:.2f}, {offset[1]:.2f})  (中位数, {len(samples)} 组样本)")
    return offset


def main():
    t0 = time.time()
    stage = "connect"
    swarm = None
    try:
        swarm = Swarm(namespaces=[NS1, NS2])
        d1, d2 = swarm[NS1], swarm[NS2]
        print(f"已连接 {NS1} / {NS2}")

        # ---------- 1. 双机起飞到 1m ----------
        stage = "takeoff"
        print(f">>> 双机起飞到 {TAKEOFF_ALT}m")
        swarm.takeoff(TAKEOFF_ALT)
        z = d1.pos[2]

        # ---------- 2. 自动测定两机原点偏移（仿真/真机无缝切换） ----------
        stage = "measure_offset"
        offset = measure_origin_offset(d1, d2)

        # ---------- 3. 物理圆心 = 1号起飞点，换算进两机各自本地系 ----------
        cx1, cy1 = d1.pos[0], d1.pos[1]
        cx2, cy2 = cx1 - offset[0], cy1 - offset[1]
        print(f"    圆心(1号本地)=({cx1:.2f},{cy1:.2f})  2号本地=({cx2:.2f},{cy2:.2f})")

        # ---------- 4. 1号到圆上起点；2号到对置点（相位差 180°） ----------
        stage = "to_circle"
        print(f">>> 1号机头向前 {RADIUS}m 到圆上起点")
        # tol 必须远小于 RADIUS：默认 0.3 等于容忍 50% 欠程，移动没走完
        # 就返回会让 theta1 反算被噪声主导（真机 log_283 教训）
        d1.move_body(RADIUS, 0.0, 0.0, tol=0.1)
        # 1号的实际相位用"到位后的实际位置"反算，不要用航向估计——
        # 航向在飞行中可能变化（实测 4 机运行时 EKF/物理航向中途转过 90°），
        # 用航向算相位会让 1 号实际落在别人的相位点上（空中贴合）。
        theta1 = math.atan2(d1.pos[1] - cy1, d1.pos[0] - cx1)
        opp = (cx2 + RADIUS * math.cos(theta1 + math.pi),
               cy2 + RADIUS * math.sin(theta1 + math.pi))
        print(f">>> 2号飞到对置点 ({opp[0]:.2f}, {opp[1]:.2f})（2号本地系）")
        d2.goto(opp[0], opp[1], z)

        # ---------- 4b. 旋转开始前重锚圆心 ----------
        # 步骤 3 捕获的圆心坐标可能在入圆阶段因 EKF xy reset 过期（真机
        # log_273：捕获到旋转之间 frame 累计漂了 ~1.2m，物理圆心随之跑偏）。
        # 相位角是方向量、不受 frame 平移影响，用各机"当前位置 - R·相位方向"
        # 反算圆心，把圆心锚在旋转开始这一刻的真实几何上。
        cx1 = d1.pos[0] - RADIUS * math.cos(theta1)
        cy1 = d1.pos[1] - RADIUS * math.sin(theta1)
        cx2 = d2.pos[0] - RADIUS * math.cos(theta1 + math.pi)
        cy2 = d2.pos[1] - RADIUS * math.sin(theta1 + math.pi)
        print(f"    重锚圆心(1号本地)=({cx1:.2f},{cy1:.2f})  2号本地=({cx2:.2f},{cy2:.2f})")

        # ---------- 5. 双机同心圆同步旋转 2 圈 ----------
        # 用位置/轨迹模式（circle_all）：纯速度模式（circle_velocity_all）无圆心位置
        # 反馈，速度/航向跟踪误差会把圆心拖走 ~0.25m/机，双机圆心差可达 0.4m
        # （2026-10-06 gz 物理真值实测）；位置模式有反馈，圆心保持厘米级。
        stage = "rotate"
        print(f">>> 双机同心圆旋转 {LAPS} 圈（直径 {DIAMETER}m，线速度 {SPEED} m/s，位置模式）")
        swarm.circle_all([
            (cx1, cy1, z, RADIUS, SPEED, LAPS),
            (cx2, cy2, z, RADIUS, SPEED, LAPS),
        ])

        # ---------- 6. 双机降落 ----------
        stage = "land"
        print(">>> 双机降落")
        swarm.land()
        print("演示完成 ✔")
        emit_result("PASS", drones=[NS1, NS2], alt=TAKEOFF_ALT,
                    diameter=DIAMETER, laps=LAPS,
                    origin_offset=[round(offset[0], 2), round(offset[1], 2)],
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
