#!/usr/bin/env python3
"""把雀的网格烘焙成地面站自解析格式 que_model.json（base64 Float32/Uint32）。

不引入 GLTFLoader 的原因：页面用的是 three r160 非模块版（three.min.js），
官方 UMD GLTFLoader 在 r160 已删除；自烘焙 + 30 行解析代码最省心。

输出: swarm_ws/src/ground_station/web/que_model.json
结构:
  body:        {pos, nrm, col, idx}   全机（不含桨）合并几何，顶点色，FRD 米制
  prop_ccw/cw: {pos, nrm, col, idx}   桨（桨毂在原点），FRD 米制
  rotor_pos:   4 个桨的 FRD 安装位置（米）
  prop_kind:   每个桨位用哪份几何 ["ccw","ccw","cw","cw"]
"""
import base64
import json
import os
import sys

import numpy as np
import trimesh

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from export_meshes import load_step, export_stl  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
CENTER_CAD = np.array([-37.48, 40.6, -18.0])
M = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]], dtype=float)   # CAD -> FRD

# FRD 桨位（米）：FR RL FL RR（与 SDF/PX4 一致，桨在桨毂面下 0.02m）
ROTOR_POS = [[0.0552, 0.0517, 0.02], [-0.0552, -0.0518, 0.02],
             [0.0518, -0.0552, 0.02], [-0.0518, 0.0552, 0.02]]
PROP_KIND = ["ccw", "ccw", "cw", "cw"]

COLORS = {
    # 按真机照片：淡紫机身/保护圈、黑色板材/起落架/电机、烟色桨
    "雀_底板_V3": (16, 16, 16),
    "雀_顶板_V3_nx": (16, 16, 16),
    "雀_电池仓_V3": (140, 128, 210),
    "4S电池卡槽": (140, 128, 210),
    "雀_起落架": (16, 16, 16),
    "3.5寸桨保护圈": (140, 128, 210),
    "2004电机": (22, 22, 22),
    "15mm螺柱": (22, 22, 22),
}
PROP_COLOR = (40, 40, 44)   # 烟色桨

OUT = "/home/c00/0c00_ws/swarm_ws/src/ground_station/web/que_model.json"


def b64f(arr):
    return base64.b64encode(np.asarray(arr, dtype="<f4").tobytes()).decode()


def b64i(arr):
    return base64.b64encode(np.asarray(arr, dtype="<u4").tobytes()).decode()


def to_frd_m(v):
    return ((v - CENTER_CAD) @ M.T) / 1000.0


def mesh_json(m):
    # 非索引平直着色：展开成三角形 soup，法线=面法线。
    # 平滑顶点色法线在硬边 CAD 上会产生"肿块"伪影、小孔细节糊掉（实测反馈），
    # 平直着色棱角分明，安装孔清晰可见。
    tri = m.triangles.reshape(-1, 3)
    nrm = np.repeat(m.face_normals, 3, axis=0)
    col = np.repeat(m.visual.face_colors[:, :3] / 255.0, 3, axis=0)
    return {"pos": b64f(tri), "nrm": b64f(nrm), "col": b64f(col), "idx": ""}


def colorize(m, rgb):
    m.visual = trimesh.visual.ColorVisuals(
        m, vertex_colors=np.tile(np.array(rgb, dtype=np.uint8), (len(m.vertices), 1)))
    return m


def main():
    body_parts, props = [], {}
    for name, shape in load_step(os.path.join(HERE, "雀_NX.stp")):
        export_stl(shape, "/tmp/bake_tmp.stl", 0.15)
        m = trimesh.load("/tmp/bake_tmp.stl")
        m.vertices = to_frd_m(m.vertices)
        if name in ("DT90", "DT90_ccw"):
            if name not in props:
                m.vertices -= m.vertices.mean(axis=0)   # 桨毂归零
                props[name] = colorize(m, PROP_COLOR)
        else:
            body_parts.append(colorize(m, COLORS.get(name, (80, 85, 90))))

    body = trimesh.util.concatenate(body_parts)
    model = {
        "body": mesh_json(body),
        "prop_ccw": mesh_json(props["DT90"]),
        "prop_cw": mesh_json(props["DT90_ccw"]),
        "rotor_pos": ROTOR_POS,
        "prop_kind": PROP_KIND,
    }
    with open(OUT, "w") as f:
        json.dump(model, f)
    print(f"{OUT}: {os.path.getsize(OUT)/1e6:.2f} MB, "
          f"body {len(body.vertices)}v/{len(body.faces)}f, prop {len(props['DT90'].vertices)}v")


if __name__ == "__main__":
    main()
