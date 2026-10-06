#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
# Copyright (C) 2026 零创无穷（成都）科技有限责任公司
# 本文件是 SwarmCore-Sim 的一部分，
# 依据 GNU GPL v3 发布（协议全文见仓库根目录 LICENSE）。
# 本软件按"现状"提供，不附带任何明示或默示担保。

"""N 机均布同心圆演示：起飞 1m，绕同一物理圆心均匀分布转 2 圈后降落

流程：
  1. N 机同时起飞到 1m；
  2. 自动测定各机本地 ENU 原点相对 1 号机的世界偏移（仿真/真机无缝切换，见下）；
  3. 以 1 号起飞点为物理圆心 C（直径 1.5m，半径 0.75m）；
  4. 1 号机头向前 0.75m 到圆上起点（其相位 = 机头航向 θ1）；
  5. 其余各机飞到各自相位点：phase_i = θ1 + i·(2π/N)，即圆周上均匀分布
     （2 机=对置 180°，3 机=120°，4 机=90°）；
  6. N 机用位置模式（circle_all）同速同步旋转 2 圈——同半径同线速度 →
     角速度相同 → 相位差恒定，保持均布不碰撞；
  7. 全群降落。

【仿真/真机无缝切换】每架 PX4 的 vehicle_local_position 是各自本地 ENU 系。
全局定位可用（仿真 GPS）时自动测定各机相对 1 号的原点偏移；不可用
（室内纯 UWB 共享锚点系）回退 (0,0)。手动固定：DUAL_OFFSET="x,y"（仅 N=2）。

运行：
    # 仿真 N 机（数量与启动一致）
    ~/0c00_ws/swarm_ws/src/bringup/scripts/start_swarm_sim.sh 3 0
    # 真机：各机 DDS 在线即可
    source /opt/ros/humble/setup.bash
    source ~/0c00_ws/swarm_ws/install/setup.bash
    python3 demo_concentric_circle.py [N=2]

注意：入圆阶段各机直接 goto 相位点，路径可能交叉（尤其 N≥3 仿真出生点
排成一列时），真实使用请确保场地空旷、遥控随时接管。
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

TAKEOFF_ALT = 1.0        # 起飞高度 (m)
DIAMETER = 1.5           # 圆直径 (m)
RADIUS = DIAMETER / 2.0
SPEED = 0.5              # 圆周线速度 (m/s)，各机相同 → 角速度相同
LAPS = 2                 # 绕圈数

# 全局位置无效时（室内纯 UWB，各机共享锚点系）的回退偏移
FALLBACK_OFFSET = (0.0, 0.0)
# 手动固定偏移（仅 N=2）：DUAL_OFFSET="x,y"
MANUAL_OFFSET = os.environ.get("DUAL_OFFSET")
# 经纬度判定无效阈值
INVALID_LATLON = 1e-6

M_PER_DEG_LAT = 111320.0


def emit_result(result, **fields):
    print("DEMO_RESULT " + json.dumps({"result": result, **fields}, ensure_ascii=False))


def measure_origin_offsets(drones, duration=3.0):
    """测定各机本地原点相对 1 号机本地原点的世界偏移 (ENU, m)。

    返回 [(0,0), off_2, ..., off_N]（1 号偏移恒为 (0,0)）。
    单节点订阅全机全局位置，duration 内逐 tick 取准同时样本算 offset 取中位数。
    全局定位无效（室内 UWB 共享系）时全部回退 FALLBACK_OFFSET。
    """
    ns_list = [d.ns for d in drones]
    if MANUAL_OFFSET is not None and len(ns_list) == 2:
        ox, oy = (float(v) for v in MANUAL_OFFSET.split(","))
        print(f"    使用手动固定偏移 DUAL_OFFSET = ({ox:.2f}, {oy:.2f})")
        return [(0.0, 0.0), (ox, oy)]

    node = rclpy.create_node("_gpos_probe_swarm")
    g = {}
    qos = QoSProfile(depth=10,
                     reliability=ReliabilityPolicy.BEST_EFFORT,
                     durability=DurabilityPolicy.VOLATILE)

    def make_cb(ns):
        def cb(m):
            g[ns] = (m.lat, m.lon)
        return cb

    for ns in ns_list:
        node.create_subscription(VehicleGlobalPosition,
                                 f"/{ns}/fmu/out/vehicle_global_position",
                                 make_cb(ns), qos)
    samples = []
    t0 = time.time()
    try:
        while time.time() - t0 < duration:
            rclpy.spin_once(node, timeout_sec=0.05)
            if all(ns in g for ns in ns_list):
                vals = [g[ns] for ns in ns_list]
                if all(abs(v[0]) > INVALID_LATLON and abs(v[1]) > INVALID_LATLON
                       for v in vals):
                    lat0 = math.radians(sum(v[0] for v in vals) / len(vals))
                    g1 = vals[0]
                    l1 = drones[0].pos
                    tick = []
                    for i in range(1, len(ns_list)):
                        dg = ((vals[i][1] - g1[1]) * M_PER_DEG_LAT * math.cos(lat0),
                              (vals[i][0] - g1[0]) * M_PER_DEG_LAT)
                        dl = (drones[i].pos[0] - l1[0], drones[i].pos[1] - l1[1])
                        tick.append((dg[0] - dl[0], dg[1] - dl[1]))
                    samples.append(tick)
            time.sleep(0.05)
    finally:
        node.destroy_node()
    if not samples:
        print(f"    全局定位无效（室内UWB模式），全部回退偏移 {FALLBACK_OFFSET}")
        return [(0.0, 0.0)] + [FALLBACK_OFFSET] * (len(ns_list) - 1)
    offsets = [(0.0, 0.0)]
    for i in range(len(ns_list) - 1):
        xs = sorted(s[i][0] for s in samples)
        ys = sorted(s[i][1] for s in samples)
        offsets.append((xs[len(xs) // 2], ys[len(ys) // 2]))
    print(f"    自动测定原点偏移（中位数, {len(samples)} 组样本）: "
          + "  ".join(f"{ns_list[i+1]}=({offsets[i+1][0]:.2f},{offsets[i+1][1]:.2f})"
                      for i in range(len(ns_list) - 1)))
    return offsets


def main():
    n = int(sys.argv[1]) if len(sys.argv) > 1 else 2
    if n < 2:
        print("N 必须 >= 2")
        return 1
    ns_list = [f"uav_{i + 1}" for i in range(n)]
    t0 = time.time()
    stage = "connect"
    swarm = None
    try:
        swarm = Swarm(namespaces=ns_list)
        drones = [swarm[ns] for ns in ns_list]
        d1 = drones[0]
        print(f"已连接 {' / '.join(ns_list)}")

        # ---------- 1. 全群起飞到 1m ----------
        stage = "takeoff"
        print(f">>> {n} 机起飞到 {TAKEOFF_ALT}m")
        swarm.takeoff(TAKEOFF_ALT)
        z = d1.pos[2]

        # ---------- 2. 测定各机原点偏移（仿真/真机无缝切换） ----------
        stage = "measure_offset"
        offsets = measure_origin_offsets(drones)

        # ---------- 3. 物理圆心 = 1号起飞点，换算进各机本地系 ----------
        cx1, cy1 = d1.pos[0], d1.pos[1]
        centers = [(cx1 - off[0], cy1 - off[1]) for off in offsets]
        print(f"    物理圆心 = 1号起飞点；各机本地圆心: "
              + "  ".join(f"{ns_list[i]}=({centers[i][0]:.2f},{centers[i][1]:.2f})"
                          for i in range(n)))

        # ---------- 4. 各机到各自相位点（均布 2π/N） ----------
        stage = "to_circle"
        print(f">>> 1号机头向前 {RADIUS}m 到圆上起点")
        # tol 必须远小于 RADIUS：默认 0.3 等于容忍 50% 欠程，移动没走完
        # 就返回会让 theta1 反算被噪声主导（真机 log_283 教训）
        d1.move_body(RADIUS, 0.0, 0.0, tol=0.1)
        # 1号的实际相位用"到位后的实际位置"反算，不要用航向估计——
        # 航向在飞行中可能变化（实测 4 机运行时 EKF/物理航向中途转过 90°），
        # 用航向算相位会让 1 号实际落在别人的相位点上（空中贴合）。
        theta1 = math.atan2(d1.pos[1] - cy1, d1.pos[0] - cx1)
        print(f"    1号实际相位 {math.degrees(theta1):.0f}°（ENU）")
        for i in range(1, n):
            phase = theta1 + i * 2.0 * math.pi / n
            px = centers[i][0] + RADIUS * math.cos(phase)
            py = centers[i][1] + RADIUS * math.sin(phase)
            # 逐架入圆（阻塞）：避免多机同时穿越圆心附近导致空中交叉
            print(f">>> {ns_list[i]} 飞到相位 {i * 360 // n}° 点 "
                  f"({px:.2f},{py:.2f})（本地系）")
            drones[i].goto(px, py, z)

        # ---------- 4b. 旋转开始前重锚圆心 ----------
        # 步骤 3 捕获的圆心坐标可能在入圆阶段因 EKF xy reset 过期（真机
        # log_273：捕获到旋转之间 frame 累计漂了 ~1.2m，物理圆心随之跑偏）。
        # 相位角是方向量、不受 frame 平移影响，用各机"当前位置 - R·相位方向"
        # 反算圆心，把圆心锚在旋转开始这一刻的真实几何上。
        centers = []
        for i in range(n):
            phase = theta1 + i * 2.0 * math.pi / n
            centers.append((drones[i].pos[0] - RADIUS * math.cos(phase),
                            drones[i].pos[1] - RADIUS * math.sin(phase)))
        print("    重锚圆心: "
              + "  ".join(f"{ns_list[i]}=({centers[i][0]:.2f},{centers[i][1]:.2f})"
                          for i in range(n)))

        # ---------- 5. 全群同心圆同步旋转（位置模式，相位差恒定） ----------
        stage = "rotate"
        print(f">>> {n} 机均布同心圆旋转 {LAPS} 圈"
              f"（直径 {DIAMETER}m，线速度 {SPEED} m/s，位置模式）")
        swarm.circle_all([(centers[i][0], centers[i][1], z, RADIUS, SPEED, LAPS)
                          for i in range(n)])

        # ---------- 6. 全群降落 ----------
        stage = "land"
        print(">>> 全群降落")
        swarm.land()
        print("演示完成 ✔")
        emit_result("PASS", drones=ns_list, alt=TAKEOFF_ALT,
                    diameter=DIAMETER, laps=LAPS,
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
