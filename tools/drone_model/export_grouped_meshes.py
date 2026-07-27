#!/usr/bin/env python3
"""按颜色分组导出雀的 FLU 网格（Gazebo 单 visual 只能单色，分组才能多彩）。

输出 que/meshes/ 下 5 个分组 STL（FLU, mm）:
  frame.stl      底板+顶板（碳黑）
  bay.stl        电池仓+卡槽（深灰蓝）
  gear.stl       起落架（黑）
  guard.stl      保护圈（淡紫）
  motors.stl     4 电机（银）
  standoffs.stl  4 螺柱（灰）
"""
import os
import sys

import numpy as np
import trimesh

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from export_meshes import load_step, export_stl  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
CENTER_CAD = np.array([-37.48, 40.6, -18.0])
M = np.array([[0, 1, 0], [-1, 0, 0], [0, 0, 1]], dtype=float)   # CAD -> FLU

GROUPS = {
    "frame": {"雀_底板_V3", "雀_顶板_V3_nx"},
    "bay": {"雀_电池仓_V3", "4S电池卡槽"},
    "gear": {"雀_起落架"},
    "guard": {"3.5寸桨保护圈"},
    "motors": {"2004电机"},
    "standoffs": {"15mm螺柱"},
}
OUT = "/home/c00/0c00_ws/PX4-Autopilot/Tools/simulation/gz/models/que/meshes"


def main():
    buckets = {k: [] for k in GROUPS}
    for name, shape in load_step(os.path.join(HERE, "雀_NX.stp")):
        if name in ("DT90", "DT90_ccw"):
            continue
        for g, names in GROUPS.items():
            if name in names:
                export_stl(shape, "/tmp/grp_tmp.stl", 0.35)
                m = trimesh.load("/tmp/grp_tmp.stl")
                m.vertices = ((m.vertices - CENTER_CAD) @ M.T)
                buckets[g].append(m)
    for g, meshes in buckets.items():
        assert meshes, f"分组 {g} 为空"
        merged = trimesh.util.concatenate(meshes)
        merged.export(os.path.join(OUT, f"{g}.stl"))
        print(f"{g}.stl: {len(meshes)} 零件, {len(merged.vertices)} 顶点")


if __name__ == "__main__":
    main()
