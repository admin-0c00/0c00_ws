#!/usr/bin/env python3
"""SwarmCore 单机飞行演示（客户 Demo）

演示内容（一架无人机，命名空间 /uav_1）：
    1. 起飞到指定高度（默认 1.5 m）
    2. 向前（北）飞 2 m —— 正方形的第一条边
    3. 顺时针再飞三条边，画出一个边长 2 m 的正方形
    4. 回到原点（起点正上方）
    5. 原地降落并自动上锁

航线俯视图（NED 坐标系：x=北，y=东，z=向下为负）：

        北 x
        ↑
   (0,0) ──────→ (2,0)
     ↑              │      ① 起飞后向前飞 2m
     │              ▼      ② 右转，向东 2m
   (0,2) ←────── (2,2)     ③ 右转，向南 2m
        西                  ④ 右转，向西 2m 回到原点 → 降落

使用方法：
    # 1. 先启动单机仿真（第 2 个参数 1=无头，0=带 Gazebo 界面）
    ~/0c00_ws/swarm_ws/src/bringup/scripts/start_swarm_sim.sh 1 1

    # 2. 加载 ROS 环境后运行本脚本
    source /opt/ros/humble/setup.bash
    source ~/0c00_ws/swarm_ws/install/setup.bash
    python3 demo_square.py                      # 默认高度 1.5m、边长 2m
    python3 demo_square.py --ros-args -p takeoff_alt:=2.0 -p side:=3.0

    # 随时可以 Ctrl+C 中断：设定点停发后 PX4 会触发 Offboard 失联保护，自动原地降落

可靠性与安全设计：
    - 标准 PX4 Offboard 流程：先持续发设定点（>=2Hz），再切 Offboard 模式并解锁
    - 每个航点都有到达判定（水平+垂直误差同时小于阈值）和超时保护（超时自动降落）
    - 等待飞控数据有 10s 超时提示，避免在仿真未启动时空等
    - 全过程中文日志，每一步都能在终端看到当前阶段

依赖：px4_msgs（swarm_ws 编译后自带）、rclpy
"""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from px4_msgs.msg import (
    OffboardControlMode,   # 告诉 PX4 我们用哪种 Offboard 控制方式（位置环）
    TrajectorySetpoint,    # 位置设定点（NED 坐标 + 航向）
    VehicleCommand,        # 飞控命令（切模式、解锁、降落等）
    VehicleLocalPosition,  # 飞控上报的本地位置（NED）
    VehicleStatus,         # 飞控状态（解锁状态、飞行模式）
)

# ----------------------------- 可调参数 -----------------------------
REACH_XY = 0.30     # 航点到达判定：水平误差阈值 (m)
REACH_Z = 0.20      # 航点到达判定：垂直误差阈值 (m)
LEG_TIMEOUT = 40.0  # 每段航程超时时间 (s)，超时自动降落
WAIT_FC_TIMEOUT = 10.0  # 等待飞控数据的超时 (s)
# -------------------------------------------------------------------


def now_us():
    """当前时间（微秒），PX4 消息的时间戳用"""
    return int(time.time() * 1e6)


class DemoSquare(Node):
    """单机演示状态机：预检 -> 起飞 -> 逐航点飞行 -> 返航 -> 降落 -> 完成"""

    # 状态枚举（用字符串，日志里直接可读）
    ST_PREFLIGHT = "预检"
    ST_FLY = "飞行"
    ST_LAND = "降落"
    ST_DONE = "完成"
    ST_FAIL = "失败"

    def __init__(self):
        super().__init__("demo_square")

        # ROS 参数（可用 --ros-args -p 名字:=值 覆盖）
        self.declare_parameter("takeoff_alt", 1.5)  # 起飞高度 (m)
        self.declare_parameter("side", 2.0)         # 正方形边长 (m)
        self.declare_parameter("namespace", "uav_1")
        self.declare_parameter("sysid", 1)          # 飞控 MAV_SYS_ID
        alt = self.get_parameter("takeoff_alt").value
        side = self.get_parameter("side").value
        self.ns = self.get_parameter("namespace").value
        self.sysid = self.get_parameter("sysid").value

        # 航点表：(北 n, 东 e, 说明文字)。高度统一为 -alt（NED 中负值=向上）
        # 从原点出发顺时针一圈回到原点；yaw 在运行时按航段方向自动计算
        self.legs = [
            (0.0,  0.0,  "起飞"),
            (side, 0.0,  "向前飞（北边）"),
            (side, side, "右转，向东"),
            (0.0,  side, "右转，向南"),
            (0.0,  0.0,  "右转，向西，回到原点"),
        ]
        self.alt = alt

        # 运行状态
        self.state = self.ST_PREFLIGHT
        self.leg_idx = 0              # 当前目标航点下标
        self.leg_start = time.time()  # 当前状态开始时间
        self.counter = 0              # 设定点已发送拍数
        self.pos = None               # 最新本地位置 (x,y,z)
        self.armed = False            # 是否已解锁
        self.nav_state = None         # 当前飞行模式
        self.got_data_at = None       # 首次收到飞控数据的时间

        # ---- 通信 QoS（重要！）----
        # PX4(uXRCE-DDS) 的 fmu/in 订阅端是 RELIABLE，发布端必须用默认 QoS（也是 reliable），
        # 否则指令会被静默丢弃；fmu/out 发布端是 BEST_EFFORT，订阅端要对应。
        pub_qos = QoSProfile(depth=10)
        sub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        # 发布者：Offboard 控制方式、位置设定点、飞控命令
        self.pub_mode = self.create_publisher(
            OffboardControlMode, f"/{self.ns}/fmu/in/offboard_control_mode", pub_qos)
        self.pub_sp = self.create_publisher(
            TrajectorySetpoint, f"/{self.ns}/fmu/in/trajectory_setpoint", pub_qos)
        self.pub_cmd = self.create_publisher(
            VehicleCommand, f"/{self.ns}/fmu/in/vehicle_command", pub_qos)

        # 订阅者：本地位置、飞控状态
        self.create_subscription(
            VehicleLocalPosition, f"/{self.ns}/fmu/out/vehicle_local_position",
            self._pos_cb, sub_qos)
        self.create_subscription(
            VehicleStatus, f"/{self.ns}/fmu/out/vehicle_status",
            self._status_cb, sub_qos)

        # 10Hz 定时器驱动状态机（Offboard 要求设定点 >= 2Hz 持续发送）
        self.timer = self.create_timer(0.1, self._tick)

        self.get_logger().info(
            f"[DEMO] 演示程序已启动：高度 {alt} m，边长 {side} m，目标飞机 /{self.ns}")

    # ------------------------- 数据回调 -------------------------
    def _pos_cb(self, msg):
        self.pos = (msg.x, msg.y, msg.z)
        self._mark_data()

    def _status_cb(self, msg):
        self.nav_state = msg.nav_state
        self.armed = msg.arming_state == VehicleStatus.ARMING_STATE_ARMED
        self._mark_data()

    def _mark_data(self):
        if self.got_data_at is None:
            self.got_data_at = time.time()
            self.get_logger().info("[DEMO] 已收到飞控数据，链路正常")

    # ------------------------- 指令发送 -------------------------
    def send_command(self, command, p1=0.0, p2=0.0):
        """发送一条飞控命令（MAVLink 命令经 uXRCE-DDS 转发）"""
        m = VehicleCommand()
        m.command = command
        m.param1, m.param2 = p1, p2
        m.target_system = self.sysid      # 必须等于该机的 MAV_SYS_ID，否则被忽略
        m.target_component = 1
        m.source_system = 1
        m.source_component = 1
        m.from_external = True
        m.timestamp = now_us()
        self.pub_cmd.publish(m)

    def publish_setpoint(self, n, e, yaw):
        """发布一拍位置设定点（NED + 航向角）"""
        ts = now_us()
        mode = OffboardControlMode()
        mode.position = True              # 使用位置环（其余速度/姿态环关闭）
        mode.timestamp = ts
        self.pub_mode.publish(mode)

        sp = TrajectorySetpoint()
        sp.position = [float(n), float(e), float(-self.alt)]
        sp.yaw = float(yaw)               # 机头朝航段方向，演示更直观
        sp.timestamp = ts
        self.pub_sp.publish(sp)

    @staticmethod
    def leg_yaw(n0, e0, n1, e1):
        """航段方向对应的 NED 偏航角：0=北，+90°=东，±180°=南，-90°=西"""
        return math.atan2(e1 - e0, n1 - n0)

    def dist_to_leg(self, n, e):
        """当前位置到指定航点的水平/垂直误差"""
        if self.pos is None:
            return None, None
        dn = self.pos[0] - n
        de = self.pos[1] - e
        dxy = math.hypot(dn, de)
        dz = abs(self.pos[2] - (-self.alt))
        return dxy, dz

    # ------------------------- 状态机 -------------------------
    def _tick(self):
        # 任何状态下都持续发设定点（Offboard 保活的前提）
        if self.state in (self.ST_PREFLIGHT, self.ST_FLY):
            n, e, _ = self.legs[min(self.leg_idx, len(self.legs) - 1)]
            if self.leg_idx == 0:
                yaw = self.leg_yaw(0, 0, self.legs[1][0], self.legs[1][1])
            else:
                p = self.legs[self.leg_idx - 1]
                yaw = self.leg_yaw(p[0], p[1], n, e)
            self.publish_setpoint(n, e, yaw)

        if self.state == self.ST_PREFLIGHT:
            self._step_preflight()
        elif self.state == self.ST_FLY:
            self._step_fly()
        elif self.state == self.ST_LAND:
            self._step_land()

    def _goto(self, state, log):
        self.state = state
        self.leg_start = time.time()
        self.get_logger().info(f"[DEMO] === {log} ===")

    def _step_preflight(self):
        # 仿真未启动时给出明确提示，而不是干等
        if self.got_data_at is None:
            if time.time() - self.leg_start > WAIT_FC_TIMEOUT:
                self._goto(self.ST_FAIL,
                           "10 秒未收到飞控数据！请确认仿真和 MicroXRCEAgent 已启动")
            return

        self.counter += 1
        # PX4 要求：先收到连续的设定点流，才允许切 Offboard
        if self.counter < 10:
            return
        in_offboard = self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD
        if not (in_offboard and self.armed):
            if self.counter % 10 == 0:  # 每秒重试一次
                self.get_logger().info("[DEMO] 请求 Offboard 模式并解锁…")
                # 176=DO_SET_MODE: param2=6 即 PX4 的 Offboard 主模式
                self.send_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
                # 400=ARM_DISARM: param1=1 解锁
                self.send_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
            return
        self._goto(self.ST_FLY, f"已解锁并进入 Offboard，开始 {self.legs[0][2]}")

    def _step_fly(self):
        n, e, desc = self.legs[self.leg_idx]
        dxy, dz = self.dist_to_leg(n, e)

        # 到达判定：水平与垂直误差同时小于阈值
        if dxy is not None and dxy < REACH_XY and dz < REACH_Z:
            if self.leg_idx == len(self.legs) - 1:
                self._goto(self.ST_LAND, "已回到原点上方，执行降落")
            else:
                self.leg_idx += 1
                nxt = self.legs[self.leg_idx]
                self._goto(self.ST_FLY, f"到达航点，下一段：{nxt[2]}")
            return

        # 超时保护：一段飞太久说明异常，直接降落保安全
        if time.time() - self.leg_start > LEG_TIMEOUT:
            self._goto(self.ST_LAND, f"航段超时（{LEG_TIMEOUT:.0f}s），为保护安全改为降落")

    def _step_land(self):
        # 持续发降落命令直到上锁；21=NAV_LAND
        if self.armed:
            if int((time.time() - self.leg_start) * 10) % 10 == 0:  # 每秒一次
                self.send_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
            # 降落兜底：超过 LEG_TIMEOUT 仍未上锁也强制重发（上句已覆盖，此处仅防卡死）
            if time.time() - self.leg_start > LEG_TIMEOUT:
                self._goto(self.ST_FAIL, "降落超时，请人工检查")
            return
        self._goto(self.ST_DONE, "已降落并上锁，演示完成 ✔")

    # ------------------------- 收尾 -------------------------
    def print_result(self):
        if self.state == self.ST_DONE:
            self.get_logger().info("[DEMO] 流程全部完成：起飞 → 向前 → 顺时针正方形 → 回原点 → 降落")
        else:
            self.get_logger().warn(f"[DEMO] 演示未正常完成，最终状态: {self.state}")


def main():
    rclpy.init()
    node = DemoSquare()
    try:
        # 跑到完成/失败状态后退出（降落过程中仍需保持事件循环发指令）
        while rclpy.ok() and node.state not in (DemoSquare.ST_DONE, DemoSquare.ST_FAIL):
            rclpy.spin_once(node, timeout_sec=0.1)
        # 再转 1 秒，确保最后的降落/日志消息发完
        t_end = time.time() + 1.0
        while time.time() < t_end:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        node.get_logger().warn("[DEMO] 用户中断（Ctrl+C）。设定点停发，PX4 将自动失联降落")
    finally:
        node.print_result()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
