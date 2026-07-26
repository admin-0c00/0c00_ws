#!/usr/bin/env python3
"""SwarmCore 单机飞行演示（ENU 坐标系版，新手入门示例）

一架无人机自动完成：起飞 → 向前飞 → 顺时针画正方形 → 回到原点 → 降落

本脚本与 demo_square.py 的唯一区别：【你看到的所有坐标都是 ENU】。
- ENU（ROS 常用，REP-103）：x=东，y=北，z=上，航向角 0=东、逆时针为正
- NED（PX4 飞控内部）：x=北，y=东，z=下，航向角 0=北、顺时针为正
脚本只在一个地方做转换（enu_to_ned / yaw_enu_to_ned 两个函数），
其余逻辑和 NED 版完全一样——学坐标系转换，看这两个函数就够了。

航线俯视图（ENU 坐标：x=东，y=北）：

        北 y
        ↑
   (0,2) ←────── (2,2)     ① 起飞到 1.5m，向前（北）飞 2m
     │              │
     ▼              ▼      ② 右转，向东飞 2m
   (0,0) ──────→ (2,0)     ③ 右转，向南飞 2m
        东 x               ④ 右转，向西飞 2m 回到原点 → 降落

运行方法（和 NED 版完全相同）：
    ~/0c00_ws/swarm_ws/src/bringup/scripts/start_swarm_sim.sh 1 1   # 终端 1：单机仿真
    source /opt/ros/humble/setup.bash
    source ~/0c00_ws/swarm_ws/install/setup.bash
    python3 demo_square_enu.py                                       # 终端 2：运行

Ctrl+C 中断后飞机会自动降落（PX4 失联保护）。
"""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from px4_msgs.msg import (OffboardControlMode, TrajectorySetpoint, VehicleCommand,
                          VehicleLocalPosition, VehicleStatus)

TAKEOFF_ALT = 1.5   # 起飞高度 (m)，ENU 里就是 z 值
SIDE = 2.0          # 正方形边长 (m)
NS = "uav_1"        # 飞机命名空间（单机仿真固定是 uav_1，MAV_SYS_ID=1）
TOL = 0.3           # 航点到达判定误差 (m)


# ---------------- 坐标转换：本脚本唯一的 ENU <-> NED 适配层 ----------------
def enu_to_ned(x, y, z):
    """位置 ENU(东,北,上) -> NED(北,东,下)。直接换轴即可，无需旋转矩阵"""
    return y, x, -z


def yaw_enu_to_ned(yaw):
    """航向 ENU(0=东, 逆时针正) -> NED(0=北, 顺时针正)，两个约定相差 90° 且方向相反"""
    return math.pi / 2 - yaw
# -------------------------------------------------------------------------


class Demo(Node):
    """负责和飞控收发消息。飞控数据通过 PX4 的 uXRCE-DDS 桥进出 ROS 话题"""

    def __init__(self):
        super().__init__("demo_square_enu")

        # 注意（PX4 新手最容易踩的坑）：
        # 发给飞控的话题 (fmu/in) 必须用默认 QoS（reliable），否则指令被静默丢弃；
        # 飞控发出的话题 (fmu/out) 是 best_effort，订阅时要对应。
        pub_qos = QoSProfile(depth=10)
        sub_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                             history=HistoryPolicy.KEEP_LAST, depth=5)

        self.pub_mode = self.create_publisher(OffboardControlMode, f"/{NS}/fmu/in/offboard_control_mode", pub_qos)
        self.pub_sp = self.create_publisher(TrajectorySetpoint, f"/{NS}/fmu/in/trajectory_setpoint", pub_qos)
        self.pub_cmd = self.create_publisher(VehicleCommand, f"/{NS}/fmu/in/vehicle_command", pub_qos)

        # self.pos 存 ENU 坐标 (x东, y北, z上)：收到飞控的 NED 后立刻转换
        self.pos = None
        self.armed = False
        self.offboard = False
        self.create_subscription(VehicleLocalPosition, f"/{NS}/fmu/out/vehicle_local_position",
                                 self._on_pos, sub_qos)
        self.create_subscription(VehicleStatus, f"/{NS}/fmu/out/vehicle_status", self._on_status, sub_qos)

    def _on_pos(self, m):
        # 飞控上报的是 NED (x北, y东, z下)，转成 ENU 存起来，后面全部按 ENU 思考
        self.pos = (m.y, m.x, -m.z)

    def _on_status(self, m):
        self.armed = m.arming_state == VehicleStatus.ARMING_STATE_ARMED         # 2 = 已解锁
        self.offboard = m.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD  # 14 = Offboard

    def setpoint(self, x, y, h, yaw=0.0):
        """发一拍位置设定点（入参是 ENU！h 为目标海拔高度），内部转成 NED 发给飞控。
        Offboard 模式要求设定点持续发送（>=2Hz），断流飞机会自动降落"""
        n_ned, e_ned, z_ned = enu_to_ned(x, y, h)
        ts = int(time.time() * 1e6)
        mode = OffboardControlMode()
        mode.position = True
        mode.timestamp = ts
        self.pub_mode.publish(mode)
        sp = TrajectorySetpoint()
        sp.position = [float(n_ned), float(e_ned), float(z_ned)]
        sp.yaw = float(yaw_enu_to_ned(yaw))   # 机头朝向，航向角也要转换约定
        sp.timestamp = ts
        self.pub_sp.publish(sp)

    def command(self, cmd, p1=0.0, p2=0.0):
        """发一条飞控命令（切模式/解锁/降落等），命令本身与坐标系无关"""
        m = VehicleCommand()
        m.command, m.param1, m.param2 = cmd, p1, p2
        m.target_system = 1          # 必须等于飞机的 MAV_SYS_ID，否则命令被忽略
        m.target_component = 1
        m.source_system, m.source_component = 1, 1
        m.from_external = True
        m.timestamp = int(time.time() * 1e6)
        self.pub_cmd.publish(m)


def fly_to(node, x, y, h, yaw=0.0):
    """持续发设定点，直到飞机到达 ENU 点 (x, y, h) 附近（水平/垂直误差都小于 TOL）"""
    while rclpy.ok():
        node.setpoint(x, y, h, yaw)
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.pos is None:
            continue
        d_xy = math.hypot(node.pos[0] - x, node.pos[1] - y)
        d_z = abs(node.pos[2] - h)
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

    # 重要：EKF 的高度原点可能有偏差（尤其仿真/室内），
    # 所以目标高度 = 起飞前实测高度 + 想爬的高度，而不是写死的绝对值
    h_target = node.pos[2] + TAKEOFF_ALT
    log(f"地面实测高度 {node.pos[2]:.2f} m，目标高度 {h_target:.2f} m")

    # ---------- 1. 进入 Offboard 并解锁 ----------
    # PX4 规定：必须先连续收到一段设定点流，才允许切 Offboard，所以先发 1 秒再请求
    log("预发设定点，然后请求 Offboard 模式 + 解锁…")
    for _ in range(10):
        node.setpoint(0, 0, h_target)
        rclpy.spin_once(node, timeout_sec=0.1)
    while not (node.offboard and node.armed):
        node.setpoint(0, 0, h_target)
        node.command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)       # 切 Offboard（主模式 6）
        node.command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)   # 解锁
        rclpy.spin_once(node, timeout_sec=0.5)

    # ---------- 2. 按航点表飞完整个正方形（坐标全是 ENU） ----------
    # ENU 下"顺时针（俯视）"：北 -> 东 -> 南 -> 西
    legs = [(0, 0, "起飞到 1.5m"),
            (0, SIDE, "向前飞（北）"),
            (SIDE, SIDE, "右转，向东"),
            (SIDE, 0, "右转，向南"),
            (0, 0, "右转，向西，回到原点")]
    prev = (0, 0)
    for x, y, desc in legs:
        log(f">>> {desc}")
        yaw = math.atan2(y - prev[1], x - prev[0])  # ENU 航向：0=东，逆时针为正
        fly_to(node, x, y, h_target, yaw)
        prev = (x, y)

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
