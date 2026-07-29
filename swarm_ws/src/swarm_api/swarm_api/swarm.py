# SPDX-License-Identifier: LGPL-3.0-only
# Copyright (C) 2026 零创无穷（成都）科技有限责任公司
# 本文件是 SwarmCore-Sim 的一部分，
# 依据 GNU LGPL v3 发布（协议全文见 swarm_api/LICENSE）。
# 本软件按"现状"提供，不附带任何明示或默示担保。

"""多机并行层：Swarm

给每架 Drone 开一个工作线程，把阻塞式的单机原语变成并行执行的多机指令：
- takeoff(alt)      全群同时起飞
- goto_all(points)  各机同时飞各自目标点
- land()            全群同时降落
- hover() / emergency_stop()

异常隔离：某一机失败（解锁被拒、goto 超时）时，该机自动悬停并报错，
其余飞机继续执行；全部结束后抛 SwarmError 汇总错误。

典型用法（三机同时画正方形）：
    from swarm_api import Swarm
    swarm = Swarm(num_drones=3)          # 自动发现 uav_1..uav_3
    swarm.takeoff(1.5)
    for x, y in [(0, 2), (2, 2), (2, 0), (0, 0)]:
        swarm.goto_all([(x, y, 1.5)] * 3)
    swarm.land()
    swarm.shutdown()
"""

import math
import re
import threading
import time

import rclpy

from .drone import Drone, DroneError


class SwarmError(Exception):
    """多机并行执行的汇总错误，errors 是 {命名空间: 异常} 字典。"""

    def __init__(self, errors):
        self.errors = errors
        detail = "; ".join(f"{ns}: {e}" for ns, e in errors.items())
        super().__init__(f"{len(errors)} 架无人机执行失败 —— {detail}")


def discover_namespaces(timeout=10.0):
    """扫描 ROS 话题，自动发现在线无人机的命名空间（如 ['uav_1', 'uav_2']）。

    规则：凡是发布 /<ns>/fmu/out/vehicle_local_position 的都算一架。
    仿真与真机同构——真机接入时不需要改任何代码。
    """
    if not rclpy.ok():
        rclpy.init()
    probe = rclpy.create_node("_swarm_discovery")
    pattern = re.compile(r"^/([^/]+)/fmu/out/vehicle_local_position$")
    try:
        deadline = time.time() + timeout
        while True:
            found = {m.group(1)
                     for t, _ in probe.get_topic_names_and_types()
                     for m in [pattern.match(t)] if m}
            if found or time.time() > deadline:
                return sorted(found, key=_natural_key)
            time.sleep(0.5)
    finally:
        probe.destroy_node()


def _natural_key(ns):
    """让 uav_10 排在 uav_9 后面的自然排序。"""
    m = re.match(r"^(.*?)(\d+)$", ns)
    return (m.group(1), int(m.group(2))) if m else (ns, 0)


class Swarm:
    """一个无人机集群。构造函数自动发现在线飞机，也可用 namespaces 显式指定。"""

    def __init__(self, num_drones=None, namespaces=None, discovery_timeout=10.0):
        if not rclpy.ok():
            rclpy.init()
        if namespaces is None:
            found = discover_namespaces(timeout=discovery_timeout)
            if num_drones is not None:
                if len(found) < num_drones:
                    raise SwarmError({"discovery": DroneError(
                        f"只发现 {len(found)} 架飞机 {found}，需要 {num_drones} 架（仿真是否已启动？）")})
                found = found[:num_drones]
            namespaces = found
        if not namespaces:
            raise SwarmError({"discovery": DroneError("没有发现任何在线无人机")})
        self.drones = [Drone(ns) for ns in namespaces]

    # ---------------- 容器式访问 ----------------
    def __len__(self):
        return len(self.drones)

    def __getitem__(self, key):
        """swarm[0] 按下标取；swarm['uav_1'] 按命名空间取。"""
        if isinstance(key, str):
            for d in self.drones:
                if d.ns == key:
                    return d
            raise KeyError(key)
        return self.drones[key]

    @property
    def namespaces(self):
        return [d.ns for d in self.drones]

    # ---------------- 并行执行核心 ----------------
    def _parallel(self, func, args=None):
        """让每架飞机同时在各自线程里执行 func(drone, arg)。

        某机抛异常时让它悬停，等所有线程结束后汇总抛 SwarmError。
        """
        args = args or [None] * len(self.drones)
        errors = {}

        def work(drone, arg):
            try:
                func(drone, arg)
            except Exception as e:  # noqa: BLE001 - 任何失败都要隔离到单机
                errors[drone.ns] = e
                try:
                    drone.hover()
                except Exception:
                    pass

        threads = [threading.Thread(target=work, args=(d, a))
                   for d, a in zip(self.drones, args)]
        for t in threads:
            t.start()
        for t in threads:
            t.join()
        if errors:
            raise SwarmError(errors)

    # ---------------- 多机动作 ----------------
    def takeoff(self, alt=1.5, tol=0.3, timeout=60.0):
        """全群同时起飞到相对高度 alt（各自以本机当前高度为基准）。"""
        self._parallel(lambda d, _: d.takeoff(alt, tol=tol, timeout=timeout))

    def goto_all(self, points, tol=0.3, timeout=60.0):
        """各机同时飞向各自目标点。points 为 [(x,y,z), ...]，与机数等长；
        也可以只传一个 (x,y,z)，全群飞同一点（注意避碰）。"""
        if isinstance(points, tuple) and len(points) == 3:
            points = [points] * len(self.drones)
        if len(points) != len(self.drones):
            raise ValueError(f"points 数量({len(points)})与机数({len(self.drones)})不一致")
        self._parallel(lambda d, p: d.goto(*p, tol=tol, timeout=timeout), list(points))

    def set_velocity_all(self, velocities):
        """各机速度控制（非阻塞）。单个 (vx,vy,vz) 或每机一个。"""
        if isinstance(velocities, tuple) and len(velocities) == 3:
            velocities = [velocities] * len(self.drones)
        if len(velocities) != len(self.drones):
            raise ValueError("velocities 数量与机数不一致")
        for d, v in zip(self.drones, velocities):
            d.set_velocity(*v)

    def hover(self):
        """全群原地悬停。"""
        for d in self.drones:
            d.hover()

    def land(self, timeout=60.0):
        """全群同时降落。"""
        self._parallel(lambda d, _: d.land(timeout=timeout))

    def emergency_stop(self):
        """全群立即悬停（急停）。PX4 失联保护仍然是最后防线。"""
        self.hover()

    # ---------------- 编队辅助 ----------------
    @staticmethod
    def formation_points(shape, n, spacing=2.0):
        """生成 n 机的编队偏移 [(x,y), ...]（以编队中心为原点，ENU）。

        shape: "line" 横排 | "column" 纵队 | "triangle" 三角 | "grid" 方阵
        用法：pts = Swarm.formation_points("line", 3, 2.0)
              swarm.goto_all([(x, y, 1.5) for x, y in pts])
        """
        if shape == "line":
            pts = [(i * spacing, 0.0) for i in range(n)]
        elif shape == "column":
            pts = [(0.0, i * spacing) for i in range(n)]
        elif shape == "triangle":
            pts, row = [], 0
            while len(pts) < n:
                for j in range(row + 1):
                    if len(pts) >= n:
                        break
                    pts.append((j * spacing - row * spacing / 2, row * spacing * math.sqrt(3) / 2))
                row += 1
        elif shape == "grid":
            cols = math.ceil(math.sqrt(n))
            pts = [((i % cols) * spacing, (i // cols) * spacing) for i in range(n)]
        else:
            raise ValueError(f"未知编队形状: {shape}（可选 line/column/triangle/grid）")
        # 平移到以中心为原点
        cx = sum(p[0] for p in pts) / n
        cy = sum(p[1] for p in pts) / n
        return [(x - cx, y - cy) for x, y in pts]

    def goto_formation(self, shape, spacing=2.0, z=1.5, center=(0.0, 0.0),
                       tol=0.3, timeout=60.0):
        """全群组成指定编队（每机在自己的本地坐标系内）。"""
        pts = self.formation_points(shape, len(self.drones), spacing)
        targets = [(center[0] + x, center[1] + y, z) for x, y in pts]
        self.goto_all(targets, tol=tol, timeout=timeout)

    # ---------------- 收尾 ----------------
    def shutdown(self):
        """释放全部 ROS 资源（在 land 之后调用）。"""
        for d in self.drones:
            d.shutdown()
