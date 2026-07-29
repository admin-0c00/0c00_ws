#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
# Copyright (C) 2026 零创无穷（成都）科技有限责任公司
# 本文件是 SwarmCore-Sim 的一部分，
# 依据 GNU GPL v3 发布（协议全文见仓库根目录 LICENSE）。
# 本软件按"现状"提供，不附带任何明示或默示担保。

"""SwarmCore 单机飞行演示（新手入门示例）

一架无人机自动完成：起飞 → 向前飞 → 顺时针画正方形 → 回到原点 → 降落

航线俯视图（飞控使用 NED 坐标系：x=北，y=东，z 向下为负，所以高度 -1.5 表示向上 1.5 米）：

        北 x
        ↑
   (0,0) ──────→ (2,0)     ① 起飞到 1.5m，向前（北）飞 2m
     ↑              │
     │              ▼      ② 右转，向东飞 2m
   (0,2) ←────── (2,2)     ③ 右转，向南飞 2m
                           ④ 右转，向西飞 2m 回到原点 → 原地降落

运行方法：
    # 终端 1：启动单机仿真（第 2 个参数 1=无头，0=带 Gazebo 界面）
    ~/0c00_ws/swarm_ws/src/bringup/scripts/start_swarm_sim.sh 1 1

    # 终端 2：加载环境并运行
    source /opt/ros/humble/setup.bash
    source ~/0c00_ws/swarm_ws/install/setup.bash
    python3 demo_square.py

想改高度/边长：直接改下面两个常量即可。Ctrl+C 中断后飞机会自动降落（PX4 失联保护）。
"""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from px4_msgs.msg import (OffboardControlMode, TrajectorySetpoint, VehicleCommand,
                          VehicleLocalPosition, VehicleStatus)

TAKEOFF_ALT = 1.5   # 起飞高度 (m)
SIDE = 2.0          # 正方形边长 (m)
NS = "uav_1"        # 飞机命名空间（单机仿真固定是 uav_1，MAV_SYS_ID=1）
TOL = 0.3           # 航点到达判定误差 (m)


class Demo(Node):
    """负责和飞控收发消息。飞控数据通过 PX4 的 uXRCE-DDS 桥进出 ROS 话题"""

    def __init__(self):
        super().__init__("demo_square")

        # 注意（PX4 新手最容易踩的坑）：
        # 发给飞控的话题 (fmu/in) 必须用默认 QoS（reliable），否则指令被静默丢弃；
        # 飞控发出的话题 (fmu/out) 是 best_effort，订阅时要对应。
        pub_qos = QoSProfile(depth=10)
        sub_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                             history=HistoryPolicy.KEEP_LAST, depth=5)

        # 三个发布者：Offboard 控制方式、位置设定点、飞控命令
        self.pub_mode = self.create_publisher(OffboardControlMode, f"/{NS}/fmu/in/offboard_control_mode", pub_qos)
        self.pub_sp = self.create_publisher(TrajectorySetpoint, f"/{NS}/fmu/in/trajectory_setpoint", pub_qos)
        self.pub_cmd = self.create_publisher(VehicleCommand, f"/{NS}/fmu/in/vehicle_command", pub_qos)

        # 两个订阅者：本地位置（判断有没有飞到）、飞控状态（判断是否解锁/已进入 Offboard）
        self.pos = None        # (x北, y东, z下)，单位 m
        self.armed = False     # 是否已解锁
        self.offboard = False  # 是否处于 Offboard 模式
        self.create_subscription(VehicleLocalPosition, f"/{NS}/fmu/out/vehicle_local_position",
                                 lambda m: setattr(self, "pos", (m.x, m.y, m.z)), sub_qos)
        self.create_subscription(VehicleStatus, f"/{NS}/fmu/out/vehicle_status", self._on_status, sub_qos)

    def _on_status(self, m):
        self.armed = m.arming_state == VehicleStatus.ARMING_STATE_ARMED      # 2 = 已解锁
        self.offboard = m.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD  # 14 = Offboard

    def setpoint(self, n, e, z, yaw=0.0):
        """发一拍位置设定点。Offboard 模式要求设定点持续发送（>=2Hz），断流飞机会自动降落"""
        ts = int(time.time() * 1e6)  # PX4 时间戳：微秒
        mode = OffboardControlMode()
        mode.position = True         # 用位置环控制
        mode.timestamp = ts
        self.pub_mode.publish(mode)
        sp = TrajectorySetpoint()
        sp.position = [float(n), float(e), float(z)]  # 目标点（北, 东, 下）
        sp.yaw = float(yaw)          # 机头朝向（0=北，π/2=东），让机头跟着航段转
        sp.timestamp = ts
        self.pub_sp.publish(sp)

    def command(self, cmd, p1=0.0, p2=0.0):
        """发一条飞控命令（切模式/解锁/降落等）"""
        m = VehicleCommand()
        m.command, m.param1, m.param2 = cmd, p1, p2
        m.target_system = 1          # 必须等于飞机的 MAV_SYS_ID，否则命令被忽略
        m.target_component = 1
        m.source_system, m.source_component = 1, 1
        m.from_external = True
        m.timestamp = int(time.time() * 1e6)
        self.pub_cmd.publish(m)


def fly_to(node, n, e, z, yaw=0.0):
    """持续发设定点，直到飞机到达 (n, e, z) 附近（水平/垂直误差都小于 TOL）"""
    while rclpy.ok():
        node.setpoint(n, e, z, yaw)
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.pos is None:
            continue
        d_xy = math.hypot(node.pos[0] - n, node.pos[1] - e)
        d_z = abs(node.pos[2] - z)
        if d_xy < TOL and d_z < TOL:
            return


def main():
    rclpy.init()
    node = Demo()
    log = node.get_logger().info

    # ---------- 0. 等飞控数据（确认仿真已启动） ----------
    log("等待飞控数据…（请先启动仿真）")
    while node.pos is None:
        rclpy.spin_once(node, timeout_sec=0.1)

    # ---------- 1. 进入 Offboard 并解锁 ----------
    # PX4 规定：必须先连续收到一段设定点流，才允许切 Offboard，所以先发 1 秒再请求
    # 重要：EKF 的高度原点可能有偏差（尤其仿真/室内），
    # 所以目标高度 = 起飞前实测高度 - 想爬的高度（NED 里下为正），而不是写死的绝对值
    z_target = node.pos[2] - TAKEOFF_ALT
    log(f"地面实测 z {node.pos[2]:.2f} m，目标 z {z_target:.2f} m")

    log("预发设定点，然后请求 Offboard 模式 + 解锁…")
    for _ in range(10):
        node.setpoint(0, 0, z_target)
        rclpy.spin_once(node, timeout_sec=0.1)
    while not (node.offboard and node.armed):
        node.setpoint(0, 0, z_target)   # 请求期间设定点不能断
        node.command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)       # 切 Offboard（主模式 6）
        node.command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)   # 解锁
        rclpy.spin_once(node, timeout_sec=0.5)

    # ---------- 2. 按航点表飞完整个正方形 ----------
    # 从原点出发，顺时针一圈回到原点（边长 SIDE）
    legs = [(0, 0, "起飞到 1.5m"),
            (SIDE, 0, "向前飞（北）"),
            (SIDE, SIDE, "右转，向东"),
            (0, SIDE, "右转，向南"),
            (0, 0, "右转，向西，回到原点")]
    prev = (0, 0)
    for n, e, desc in legs:
        log(f">>> {desc}")
        yaw = math.atan2(e - prev[1], n - prev[0])  # 机头对准航段方向
        fly_to(node, n, e, z_target, yaw)
        prev = (n, e)

    # ---------- 3. 原地降落，直到自动上锁 ----------
    log(">>> 降落")
    while node.armed:
        node.command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        rclpy.spin_once(node, timeout_sec=0.5)

    log("已降落并上锁，演示完成 ✔")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
