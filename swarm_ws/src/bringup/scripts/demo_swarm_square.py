#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
# Copyright (C) 2026 零创无穷（成都）科技有限责任公司
# 本文件是 SwarmCore-Sim 的一部分，
# 依据 GNU GPL v3 发布（协议全文见仓库根目录 LICENSE）。
# 本软件按"现状"提供，不附带任何明示或默示担保。

"""SwarmCore 集群飞行演示（swarm_api 框架版，新手入门示例）

多架无人机同时完成：起飞 → 各自向前飞 → 顺时针画正方形 → 回到起点 → 降落

本脚本是 demo_square_enu.py 的"集群版"，用来展示 swarm_api 框架的价值：
- 单机版里近 200 行的 QoS / 坐标转换 / Offboard 流式发送样板代码，全部消失
- 控制 N 架飞机和控制 1 架的代码量完全一样，只改 num_drones

航线俯视图（每架飞机在自己的本地坐标系内飞，ENU：x=东，y=北）：

        北 y
        ↑
   (0,2) ←────── (2,2)
     │              │
     ▼              ▼
   (0,0) ──────→ (2,0)

运行方法：
    ~/0c00_ws/swarm_ws/src/bringup/scripts/start_swarm_sim.sh 3 1   # 终端 1：三机仿真
    source /opt/ros/humble/setup.bash
    source ~/0c00_ws/swarm_ws/install/setup.bash
    python3 demo_swarm_square.py                                      # 终端 2：运行

想飞 5 架？把仿真数量改成 5，再把下面的 NUM_DRONES 改成 5，结束。
Ctrl+C 中断后飞机会自动降落（框架兜底 + PX4 失联保护双保险）。
"""

import json
import sys
import time

from swarm_api import Swarm, SwarmError

NUM_DRONES = 3    # 飞机数量（要和仿真启动的数量一致）
TAKEOFF_ALT = 1.5  # 起飞高度 (m)
SIDE = 2.0         # 正方形边长 (m)


def emit_result(result, **fields):
    """打印机器可解析的运行结果（AI Agent / CI 判定用）。
    约定：进程最后一行输出 DEMO_RESULT <json>，退出码 0=PASS / 1=FAIL。"""
    print("DEMO_RESULT " + json.dumps({"result": result, **fields}, ensure_ascii=False))


def main():
    t0 = time.time()
    stage = "connect"   # 当前阶段，FAIL 时随结果输出，便于定位
    swarm = None
    try:
        # ---------- 1. 连接集群 ----------
        # Swarm 会自动扫描 ROS 话题发现在线飞机（uav_1, uav_2, ...），
        # 真机接入时这段代码一个字都不用改（仿真与真机同构）。
        swarm = Swarm(num_drones=NUM_DRONES)
        print(f"已连接 {len(swarm)} 架飞机: {swarm.namespaces}")

        # ---------- 2. 全群同时起飞 ----------
        # 每架飞机一个线程并行执行，某机失败会自动悬停并汇总报错
        stage = "takeoff"
        print(">>> 全群起飞")
        swarm.takeoff(TAKEOFF_ALT)

        # ---------- 3. 全群同时画正方形 ----------
        # ENU 下"顺时针（俯视）"：北 -> 东 -> 南 -> 西
        stage = "square"
        legs = [(0, SIDE, "向前飞（北）"),
                (SIDE, SIDE, "右转，向东"),
                (SIDE, 0, "右转，向南"),
                (0, 0, "右转，向西，回到起点")]
        for x, y, desc in legs:
            print(f">>> {desc}")
            # 所有飞机飞同一个相对点（每机在各自本地系内，所以互不碰撞）
            swarm.goto_all((x, y, TAKEOFF_ALT))

        # ---------- 4. 全群同时降落 ----------
        stage = "land"
        print(">>> 全群降落")
        swarm.land()
        print("演示完成 ✔")
        emit_result("PASS", drones=swarm.namespaces, waypoints=len(legs),
                    duration_s=round(time.time() - t0, 1))
        return 0
    except KeyboardInterrupt:
        emit_result("FAIL", stage=stage, error="用户中断(Ctrl+C)")
        return 1
    except SwarmError as e:
        # 某机失败已被隔离（自动悬停），errors 是 {命名空间: 异常}
        emit_result("FAIL", stage=stage,
                    errors={ns: str(err) for ns, err in e.errors.items()})
        return 1
    except Exception as e:
        emit_result("FAIL", stage=stage, error=str(e))
        return 1
    finally:
        if swarm is not None:
            swarm.shutdown()


if __name__ == "__main__":
    sys.exit(main())
