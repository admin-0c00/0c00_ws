#!/usr/bin/env python3
"""生成 Web 地面站用的 que.glb（FRD，米制，桨为独立节点可旋转）。

零件配色（暗色科技风）：
  机架板材   碳黑 #23272b
  电池仓/顶板 深灰 #3a4046
  起落架/保护圈 半透黑 #16181a
  电机       银 #b8bdc2
  桨         品牌橙 #ff6d3f（prop_0..3 独立节点，绕各自 z 轴旋转）
"""
import os
import sys

import numpy as np
import trimesh

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from export_meshes import load_step, export_stl  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
CENTER_CAD = np.array([-37.48, 40.6, -18.0])
M = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]], dtype=float)

# FRD 旋翼位置（米）：FR RL FL RR
ROTOR_POS = [(0.0552, 0.0517, 0.02), (-0.0552, -0.0518, 0.02),
             (0.0518, -0.0552, 0.02), (-0.0518, 0.0552, 0.02)]

COLORS = {
    "雀_底板_V3": (35, 39, 43, 255),
    "雀_顶板_V3_nx": (58, 64, 70, 255),
    "雀_电池仓_V3": (58, 64, 70, 255),
    "雀_起落架": (22, 24, 26, 255),
    "3.5寸桨保护圈": (22, 24, 26, 255),
    "4S电池卡槽": (90, 96, 102, 255),
    "2004电机": (184, 189, 194, 255),
    "15mm螺柱": (150, 155, 160, 255),
}
PROP_COLOR = (255, 109, 63, 255)


def to_frd_m(vertices):
    return ((vertices - CENTER_CAD) @ M.T) / 1000.0


def mesh_of(shape, tmp):
    export_stl(shape, tmp, 0.3)
    m = trimesh.load(tmp)
    m.vertices = to_frd_m(m.vertices)
    return m


def main():
    scene = trimesh.Scene()
    prop_geoms = {}
    for name, shape in load_step(os.path.join(HERE, "雀_NX.stp")):
        if name in ("DT90", "DT90_ccw"):
            if name not in prop_geoms:
                m = mesh_of(shape, "/tmp/glb_p.stl")
                m.vertices -= m.vertices.mean(axis=0)   # 桨毂归零
                m.visual = trimesh.visual.ColorVisuals(
                    m, face_colors=np.tile(PROP_COLOR, (len(m.faces), 1)))
                prop_geoms[name] = m
            continue
        m = mesh_of(shape, "/tmp/glb_b.stl")
        color = COLORS.get(name, (80, 85, 90, 255))
        m.visual = trimesh.visual.ColorVisuals(m, face_colors=np.tile(color, (len(m.faces), 1)))
        scene.add_geometry(m, node_name=f"body_{name}", geom_name=f"body_{name}")

    # 4 个桨节点：0/2 用 DT90(ccw 文件)，1/3 用 DT90_ccw(cw 文件)——方向与 PX4 一致
    keys = ["DT90", "DT90", "DT90_ccw", "DT90_ccw"]
    for i, (pos, key) in enumerate(zip(ROTOR_POS, keys)):
        node = f"prop_{i}"
        scene.add_geometry(prop_geoms[key].copy(), node_name=node, geom_name=node,
                           transform=trimesh.transformations.translation_matrix(pos))

    out = os.path.join(HERE, "out", "que.glb")
    scene.export(out)
    print(f"que.glb: {os.path.getsize(out)/1e6:.2f} MB, 节点: {list(scene.nodes_geometry)}")


if __name__ == "__main__":
    main()
