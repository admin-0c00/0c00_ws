#!/usr/bin/env python3
"""把雀_NX 装配体转成 Gazebo 仿真用网格（FLU 坐标系：x前 y左 z上，Gazebo 约定）。

注意：PX4 的 CA_ROTOR 参数用 FRD（x前 y右 z下），但 Gazebo 模型必须是 FLU，
否则模型上下颠倒 + 左右镜像（踩过的坑）。

输出 drone_model/out/frd/:
  body.stl      机体（不含桨）视觉网格，FLU，原点=电机中心/桨毂平面
  prop_ccw.stl  CCW 桨（DT90），原点=桨毂中心（用于 rotor_0/1）
  prop_cw.stl   CW 桨（DT90_ccw），原点=桨毂中心（用于 rotor_2/3）
  inertia.txt   质量/惯量/质心（按 0.48kg 缩放，FLU 系）
"""
import os
import sys

import numpy as np
import trimesh

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from export_meshes import load_step, export_stl  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "out", "frd")
os.makedirs(OUT, exist_ok=True)

# CAD -> FLU 变换（CAD: x右 y前 z上 → FLU: x前 y左 z上），先减机架中心
CENTER_CAD = np.array([-37.48, 40.6, -18.0])   # 电机中心 xy + 桨毂平面 z（实测）
M = np.array([[0, 1, 0], [-1, 0, 0], [0, 0, 1]], dtype=float)


def to_flu(vertices):
    return (vertices - CENTER_CAD) @ M.T


def shape_to_mesh(shape, deflection, tmp):
    export_stl(shape, tmp, deflection)
    return trimesh.load(tmp)


def main():
    parts = load_step(os.path.join(HERE, "雀_NX.stp"))
    body_meshes, props = [], {}
    for name, shape in parts:
        if name in ("DT90", "DT90_ccw"):
            m = shape_to_mesh(shape, 0.2, "/tmp/prop_tmp.stl")
            props.setdefault(name, m)          # 同型桨只留一份
        else:
            body_meshes.append(shape_to_mesh(shape, 0.35, "/tmp/body_tmp.stl"))

    # ---- 机体网格（不含桨） ----
    body = trimesh.util.concatenate(body_meshes)
    body.vertices = to_flu(body.vertices)
    body.export(os.path.join(OUT, "body.stl"))

    collision = body.copy()
    collision.vertices = to_flu(trimesh.util.concatenate(body_meshes).vertices)
    collision.export(os.path.join(OUT, "collision.stl"))

    # ---- 桨：原点移到桨毂中心 ----
    for name, m in props.items():
        m.vertices = to_flu(m.vertices)
        hub = m.vertices.mean(axis=0)          # 桨叶质心≈桨毂（三叶对称）
        m.vertices -= hub
        out = "prop_ccw.stl" if name == "DT90" else "prop_cw.stl"
        m.export(os.path.join(OUT, out))
        print(f"{name}: 顶点 {len(m.vertices)}, 半径 {np.abs(m.vertices[:, :2]).max():.1f} m?")

    # ---- 质量属性（含桨，按目标总重缩放） ----
    TARGET_MASS = 0.48   # kg，估算：碳板+打印件 240g + 电机 64g + 4S 850mAh 100g + 电子设备 40g + 杂项
    everything = trimesh.util.concatenate(body_meshes + list(props.values()))
    everything.vertices = to_flu(everything.vertices)
    # mm -> m
    everything.vertices /= 1000.0
    vol_m3 = everything.volume if everything.is_volume else abs(everything.volume)
    density = TARGET_MASS / vol_m3
    everything.density = density
    com = everything.center_mass
    inertia = everything.moment_inertia          # 关于质心，kg·m²
    with open(os.path.join(OUT, "inertia.txt"), "w") as f:
        f.write(f"mass {TARGET_MASS}\n")
        f.write(f"com {com[0]:.6f} {com[1]:.6f} {com[2]:.6f}\n")
        f.write(f"ixx {inertia[0,0]:.3e} iyy {inertia[1,1]:.3e} izz {inertia[2,2]:.3e}\n")
        f.write(f"ixy {inertia[0,1]:.3e} ixz {inertia[0,2]:.3e} iyz {inertia[1,2]:.3e}\n")
    print(open(os.path.join(OUT, "inertia.txt")).read())
    print("输出 ->", OUT)


if __name__ == "__main__":
    main()
