# SPDX-License-Identifier: LGPL-3.0-only
# Copyright (C) 2026 零创无穷（成都）科技有限责任公司
# 本文件是 SwarmCore-Sim 的一部分，
# 依据 GNU LGPL v3 发布（协议全文见 swarm_api/LICENSE）。
# 本软件按"现状"提供，不附带任何明示或默示担保。

"""单机控制层：Drone

把和一架 PX4 无人机打交道所需的全部样板代码封装起来：
- ROS 话题的 QoS 配置（fmu/in 要 reliable，fmu/out 是 best_effort）
- ENU <-> NED 坐标转换（用户只接触 ENU：x=东, y=北, z=上）
- Offboard 模式要求的 >=2Hz 设定点流（后台 20Hz 线程自动维持，用户不用管）
- 切模式 / 解锁命令的重发等待

用户只需要调用动作原语：takeoff / goto / set_velocity / hover / land。

典型用法（单机）：
    from swarm_api import Drone
    d = Drone("uav_1")
    d.takeoff(1.5)
    d.goto(0, 2, 1.5)
    d.land()
    d.shutdown()

多机请用 Swarm 类（见 swarm.py），它内部就是给每架 Drone 开一个线程。
"""

import math
import re
import threading
import time
from collections import deque

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from px4_msgs.msg import (OffboardControlMode, TrajectorySetpoint, VehicleCommand,
                          VehicleLocalPosition, VehicleStatus)

NAN = float("nan")


class DroneError(Exception):
    """单机操作失败（超时、无定位、解锁被拒等）。Swarm 层捕获后会让该机悬停。"""


# ---------------- 坐标转换：用户侧统一 ENU，飞控内部是 NED ----------------
def enu_to_ned(x, y, z):
    """位置 ENU(东,北,上) -> NED(北,东,下)。直接换轴即可。"""
    return y, x, -z


def yaw_enu_to_ned(yaw):
    """航向 ENU(0=东, 逆时针正) -> NED(0=北, 顺时针正)。"""
    return math.pi / 2 - yaw


def _sys_id_from_ns(namespace):
    """从命名空间尾部数字推断 MAV_SYS_ID：uav_2 -> 2。推不出来就用 1。"""
    m = re.search(r"(\d+)$", namespace)
    return int(m.group(1)) if m else 1


class Drone:
    """一架无人机的控制句柄。所有位置/速度入参都是 ENU 坐标系。"""

    def __init__(self, namespace, sys_id=None):
        if not rclpy.ok():
            rclpy.init()
        self.ns = namespace
        self.sys_id = sys_id or _sys_id_from_ns(namespace)

        # ---- 状态（由订阅回调更新，属性只读） ----
        self.pos = None        # 本地 ENU (x东, y北, z上)
        self.yaw = 0.0         # ENU 航向
        self.armed = False
        self.offboard = False
        self.failsafe = False
        self.nav_state = 0     # PX4 导航状态原始值（VehicleStatus.nav_state）
        self.xy_valid = False
        self.v_xy_valid = False
        self.dead_reckoning = False
        self.vz = NAN

        # ---- 当前设定点（由动作原语修改，流线程负责持续发出） ----
        self._mode = "position"           # "position" | "velocity"
        self._pos_mode = "local"          # position: "local" | "follow" | "circle"
        self._target = [0.0, 0.0, 0.0]    # 本地 ENU 位置目标
        self._target_z = 0.0              # ENU 高度目标
        self._sp_yaw = 0.0                # ENU 航向目标
        self._vel = [0.0, 0.0, 0.0]       # ENU 速度目标
        self._yaw_rate = 0.0
        self._streaming = False
        self._last_xy_reset = None
        self._action_phase = None

        # ---- 圆形轨迹流状态（由 circle() 设置，_publish_setpoint 逐周期推进） ----
        self._circle_cx = 0.0
        self._circle_cy = 0.0
        self._circle_radius = 0.0
        self._circle_omega = 0.0       # 角速度 rad/s，符号定方向（ENU 逆时针为正）
        self._circle_angle = 0.0       # 当前角度，随流推进
        self._circle_start_angle = 0.0
        self._circle_target_rad = 0.0  # 需要扫过的总弧度 = 2π·laps
        self._circle_last_t = 0.0      # 上次推进时刻（monotonic）
        self._circle_velocity_mode = False  # True=circle_velocity 速度流（mode=velocity）
        self._circle_smooth_mode = False    # True=circle_smooth 平滑圆速度流（mode=velocity）
        self._circle_k = 0.3           # circle_smooth 径向/相位纠正增益 (1/s)
        self._circle_ki = 0.1          # circle_smooth 径向积分增益 (m/s 每 m·s)
        self._circle_radial_int = 0.0  # circle_smooth 径向积分累积 (m/s)，限幅 ±0.25
        self._circle_t0 = 0.0          # circle_smooth 起步时刻（速度缓升用）
        self._lock = threading.Lock()

        self.node = Node(f"swarm_api_{namespace.replace('/', '_')}")

        pub_qos = QoSProfile(depth=10)
        sub_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                             history=HistoryPolicy.KEEP_LAST, depth=5)
        self._pub_mode = self.node.create_publisher(
            OffboardControlMode, f"/{namespace}/fmu/in/offboard_control_mode", pub_qos)
        self._pub_sp = self.node.create_publisher(
            TrajectorySetpoint, f"/{namespace}/fmu/in/trajectory_setpoint", pub_qos)
        self._pub_cmd = self.node.create_publisher(
            VehicleCommand, f"/{namespace}/fmu/in/vehicle_command", pub_qos)
        self.node.create_subscription(
            VehicleLocalPosition, f"/{namespace}/fmu/out/vehicle_local_position",
            self._on_pos, sub_qos)
        self.node.create_subscription(
            VehicleStatus, f"/{namespace}/fmu/out/vehicle_status",
            self._on_status, sub_qos)

        # 订阅回调线程
        self._exec = SingleThreadedExecutor()
        self._exec.add_node(self.node)
        self._spin_thread = threading.Thread(target=self._exec.spin, daemon=True)
        self._spin_thread.start()

        # 设定点流线程（Offboard 的生命线：断流飞机会触发失联保护）
        self._stop = threading.Event()
        self._stream_thread = threading.Thread(target=self._stream_loop, daemon=True)
        self._stream_thread.start()

    # ---------------- 订阅回调 ----------------
    def _on_pos(self, m):
        new_pos = (m.y, m.x, -m.z)
        with self._lock:
            self.pos = new_pos
            self.yaw = math.pi / 2 - m.heading
            if self._last_xy_reset is not None and m.xy_reset_counter != self._last_xy_reset:
                delta_enu = (m.delta_xy[1], m.delta_xy[0])
                if self._mode == "position" and self._pos_mode == "local":
                    self._target[0] += delta_enu[0]
                    self._target[1] += delta_enu[1]
                elif self._mode == "position" and self._pos_mode == "circle":
                    # 圆形轨迹的圆心是本地系坐标：EKF 原点 reset 时圆心随之一并平移，
                    # 否则圆设定点会相对新原点跳变（与 goto 的 reset 补偿同理）
                    self._circle_cx += delta_enu[0]
                    self._circle_cy += delta_enu[1]
                    self._target[0] += delta_enu[0]
                    self._target[1] += delta_enu[1]
                elif self._circle_smooth_mode:
                    # circle_smooth 的径向/相位纠正实时引用圆心（本地系坐标），
                    # reset 时圆心随 frame 平移；pos 同步平移，pos-center 不变
                    self._circle_cx += delta_enu[0]
                    self._circle_cy += delta_enu[1]
            self._last_xy_reset = m.xy_reset_counter
            self.xy_valid = bool(getattr(m, "xy_valid", True))
            self.v_xy_valid = bool(getattr(m, "v_xy_valid", True))
            self.dead_reckoning = bool(getattr(m, "dead_reckoning", False))
            self.vz = -float(getattr(m, "vz", 0.0))
            # ENU 水平速度：NED x=北, y=东 -> ENU x=东=m.vy, y=北=m.vx
            self.vx = float(getattr(m, "vy", 0.0))
            self.vy = float(getattr(m, "vx", 0.0))

    def _on_status(self, m):
        with self._lock:
            self.armed = m.arming_state == VehicleStatus.ARMING_STATE_ARMED
            self.offboard = m.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD
            self.failsafe = bool(getattr(m, "failsafe", False))
            self.nav_state = m.nav_state

    # ---------------- 设定点流（后台 20Hz） ----------------
    def _stream_loop(self):
        while not self._stop.is_set():
            if self._streaming:
                self._publish_setpoint()
            time.sleep(0.05)

    def _publish_setpoint(self):
        ts = int(time.time() * 1e6)
        vel_ff = None
        acc_ff = None
        with self._lock:
            mode_name = self._mode
            sp_yaw = self._sp_yaw
            vel = list(self._vel)
            yaw_rate = self._yaw_rate
            if mode_name == "position":
                if self._pos_mode == "follow" and self.pos is not None:
                    target = (self.pos[0], self.pos[1], self._target_z)
                elif self._pos_mode == "circle":
                    # 圆形轨迹流：每个流周期按角速度推进设定点，让 PX4 位置控制器
                    # 跟踪连续移动的目标 -> 匀速平滑画圆（比逐点 goto 快且圆）。
                    # dt 限幅防线程卡顿后设定点大跳。
                    now = time.monotonic()
                    dt = min(now - self._circle_last_t, 0.25)
                    self._circle_last_t = now
                    self._circle_angle += self._circle_omega * dt
                    self._target[0] = (self._circle_cx
                                       + self._circle_radius * math.cos(self._circle_angle))
                    self._target[1] = (self._circle_cy
                                       + self._circle_radius * math.sin(self._circle_angle))
                    target = tuple(self._target)
                    # 切向速度前馈（ENU）：v = R*w*(-sinθ, cosθ)，与位置设定点同步下发
                    vel_ff = (-self._circle_radius * self._circle_omega
                              * math.sin(self._circle_angle),
                              self._circle_radius * self._circle_omega
                              * math.cos(self._circle_angle),
                              0.0)
                    # 向心加速度前馈：a = -w^2*R*(cosθ, sinθ)。只给速度前馈时，
                    # 向心加速度仍需靠速度误差生成 -> 残余相位滞后 + 半径外扩
                    # （仿真实测 R +22%）；补齐加速度前馈后轨迹跟踪才完整
                    w2r = self._circle_omega * self._circle_omega * self._circle_radius
                    acc_ff = (-w2r * math.cos(self._circle_angle),
                              -w2r * math.sin(self._circle_angle),
                              0.0)
                else:
                    target = tuple(self._target)
            else:
                # 速度模式：circle_velocity / circle_smooth 在此逐周期推进速度设定点
                if self._circle_velocity_mode:
                    now = time.monotonic()
                    dt = min(now - self._circle_last_t, 0.25)
                    self._circle_last_t = now
                    self._circle_angle += self._circle_omega * dt
                    self._vel[0] = (-self._circle_radius * self._circle_omega
                                    * math.sin(self._circle_angle))
                    self._vel[1] = (self._circle_radius * self._circle_omega
                                    * math.cos(self._circle_angle))
                    vel = list(self._vel)
                elif self._circle_smooth_mode:
                    now = time.monotonic()
                    dt = min(now - self._circle_last_t, 0.25)
                    self._circle_last_t = now
                    self._circle_angle += self._circle_omega * dt
                    # 起步 1s 速度缓升，避免 0 -> speed 阶跃造成入圆甩动
                    ramp = min((now - self._circle_t0) / 1.0, 1.0)
                    vx = (-self._circle_radius * self._circle_omega
                          * math.sin(self._circle_angle)) * ramp
                    vy = (self._circle_radius * self._circle_omega
                          * math.cos(self._circle_angle)) * ramp
                    if self.pos is not None:
                        dx = self.pos[0] - self._circle_cx
                        dy = self.pos[1] - self._circle_cy
                        r = math.hypot(dx, dy)
                        if r > 0.05:
                            # 小增益径向纠正（k≈0.3，位置模式 P≈0.95 的 1/3）：
                            # 把"追估计噪声"变成"只纠偏"，物理轨迹不被估计抖动带走
                            err = self._circle_radius - r
                            # 径向积分：真机 log_8 实测速度执行存在近似恒定的旋转
                            # 偏差（指令径向 -0.13 向内、实际 +0.09 向外，纯 P 卡在
                            # r≈0.9 不收口），积分慢速吃掉这类恒定偏差，不增加噪声
                            # 带宽；限幅防 windup
                            self._circle_radial_int += self._circle_ki * err * dt
                            self._circle_radial_int = max(
                                -0.25, min(0.25, self._circle_radial_int))
                            rad_corr = (self._circle_k * err
                                        + self._circle_radial_int)
                            vx += rad_corr * dx / r
                            vy += rad_corr * dy / r
                            # 小增益相位纠正：锁指令相位，保证圈数准确/多机相位保持。
                            # v = k*(θ_cmd-θ_d)*R 沿 (-sinθ_d, cosθ_d)，转向符号自动抵消
                            ang_d = math.atan2(dy, dx)
                            d_ang = ((self._circle_angle - ang_d + math.pi)
                                     % (2.0 * math.pi) - math.pi)
                            vt = self._circle_k * d_ang * self._circle_radius
                            vx += vt * (-math.sin(ang_d))
                            vy += vt * math.cos(ang_d)
                    self._vel[0] = vx
                    self._vel[1] = vy
                    # z 通道位置闭环：速度模式下只发 vz=0 会让高度随扰动漂移
                    # （真机 log_24：目标 1.0m 漂到 1.3m+）。小增益拉回目标
                    # 高度，限幅 ±0.5 m/s 保持平滑
                    if self.pos is not None:
                        self._vel[2] = max(-0.5, min(
                            0.5, 1.0 * (self._target_z - self.pos[2])))
                    else:
                        self._vel[2] = 0.0
                    vel = list(self._vel)
                target = None
        mode = OffboardControlMode()
        mode.timestamp = ts
        sp = TrajectorySetpoint()
        sp.timestamp = ts
        if mode_name == "position":
            mode.position = True
            sp.position = [float(v) for v in enu_to_ned(*target)]
            if vel_ff is not None:
                # 圆轨迹切向速度前馈：PX4 位置控制器把随位置设定点一同下发的
                # 速度当作 feedforward，消除追移动点时的相位滞后/半径内切
                # （真机 log_4：无前馈时相位滞后 ~40°，半径内切 ~25%）
                sp.velocity = [float(v) for v in enu_to_ned(*vel_ff)]
                sp.acceleration = [float(v) for v in enu_to_ned(*acc_ff)]
            else:
                sp.velocity = [NAN, NAN, NAN]
            sp.yaw = float(yaw_enu_to_ned(sp_yaw))
        else:  # velocity
            mode.velocity = True
            sp.position = [NAN, NAN, NAN]
            sp.velocity = [float(v) for v in enu_to_ned(*vel)]
            sp.yawspeed = float(-yaw_rate)  # ENU 逆时针正 -> NED 顺时针正
        self._pub_mode.publish(mode)
        self._pub_sp.publish(sp)

    # ---------------- 底层命令 ----------------
    def _command(self, cmd, p1=0.0, p2=0.0):
        m = VehicleCommand()
        m.command, m.param1, m.param2 = cmd, float(p1), float(p2)
        m.target_system = self.sys_id
        m.target_component = 1
        m.source_system, m.source_component = 1, 1
        m.from_external = True
        m.timestamp = int(time.time() * 1e6)
        self._pub_cmd.publish(m)

    def _wait_connected(self, timeout=10.0):
        t0 = time.time()
        while self.pos is None:
            if time.time() - t0 > timeout:
                raise DroneError(f"{self.ns}: 等待飞控数据超时（仿真/真机是否已启动？）")
            time.sleep(0.1)

    # ---------------- 动作原语（全部阻塞，超时抛 DroneError） ----------------
    def takeoff(self, alt=1.5, vz=0.5, tol=0.15, timeout=60.0,
                hover_vxy_tol=0.3, settle_time=0.5):
        """起飞到相对高度 alt（米）。**零水平速度**（物理原地爬升/悬停），返回时已悬停住。

        用 velocity 模式：水平速度恒 0（不追 EKF 位置漂移，物理原地，杜绝
        "起飞后往前跑"），垂直按 vz 爬升、到位后 vz=0 保持高度。
        返回条件：高度到位 **且** 最近 1s 平均水平速度 ≤ hover_vxy_tol 持续
        settle_time。持续不收敛则超时报错，附当前高度/水平速度便于诊断。

        高度目标用"起飞瞬间实测高度 + alt"固定，规避 EKF 高度原点偏差。
        """
        self._wait_connected()
        with self._lock:
            self._action_phase = "takeoff"
            # 航向目标锚定当前航向：_sp_yaw 初始默认 0（ENU 东），不锚定时
            # 起飞会命令机头转向东——真机机头朝北起飞实测向右急转 ~90°
            # （log_274）。锚定后全程保持起飞航向，不旋转朝向。
            if math.isfinite(self.yaw):
                self._sp_yaw = self.yaw
        z0 = self.pos[2]
        target_z = z0 + alt
        # 预发零速度设定点 1 秒：PX4 要求先收到设定点流才允许切 Offboard
        with self._lock:
            self._mode = "velocity"
            self._vel = [0.0, 0.0, 0.0]
            self._yaw_rate = 0.0
        self._streaming = True
        time.sleep(1.0)

        # 请求 Offboard + 解锁，命令要重发直到生效（飞控可能丢掉单条命令）
        t0 = time.time()
        try:
            while not (self.offboard and self.armed):
                if time.time() - t0 > timeout:
                    raise DroneError(f"{self.ns}: 进入 Offboard/解锁超时（检查 preflight 状态）")
                self._command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)  # 主模式 6 = Offboard
                self._command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
                time.sleep(0.5)
        except Exception:
            # 起飞失败必须停流：残留的设定点流会干扰 PX4 后续的返航/降落衔接，
            # 飞机会悬在目标高度落不下来（实测踩过的坑）
            self._streaming = False
            raise

        # 开始爬升：垂直速度 vz（ENU 向上），水平保持 0
        with self._lock:
            self._vel[2] = float(vz)

        # 等待"飞到位并保持"：爬升到位（vz 切 0）→ 高度保持闭环（掉出目标容差
        # 自动爬升/下降修正回 target_z，不允许起飞后掉高）→ 高度稳定 + 水平收敛
        # 持续 settle_time 才返回。中途 Offboard 丢失按 failsafe 处理。
        t0 = time.time()
        lost_since = None
        settled_since = None
        vz_zeroed = False
        vxy_history = deque(maxlen=10)   # 0.1s 采样 x10 = 近 1s
        while True:
            if not self.offboard:
                if lost_since is None:
                    lost_since = time.time()
                elif time.time() - lost_since > 2.0:
                    reason = "failsafe=true" if self.failsafe else "failsafe=false"
                    raise DroneError(f"{self.ns}: Offboard 中途丢失（{reason}），takeoff 中止")
            else:
                lost_since = None
            with self._lock:
                pos = self.pos
                vxy = math.hypot(self.vx, self.vy)
                dz = None
                if pos is not None:
                    dz = pos[2] - target_z
                    if not vz_zeroed and abs(dz) < tol:
                        vz_zeroed = True
                    if not vz_zeroed and dz > 0.5:
                        # 爬升远超目标（高度源异常/估计封顶导致永远不到位），
                        # 强制停爬并报错，防止飞机无限上升
                        self._vel[2] = 0.0
                        raise DroneError(
                            f"{self.ns}: takeoff 高度异常（超出目标 {dz:.2f}m 仍未到位），"
                            f"检查高度源（距离传感器）读数")
                    if vz_zeroed:
                        # 高度保持闭环：掉出目标容差就修正回 target_z
                        if dz < -tol:
                            self._vel[2] = float(vz)     # 低于目标 → 爬升
                        elif dz > tol:
                            self._vel[2] = -float(vz)    # 高于目标 → 下降
                        else:
                            self._vel[2] = 0.0           # 在容差内 → 保持
            vxy_history.append(vxy)
            vxy_avg = sum(vxy_history) / len(vxy_history)
            height_ok = vz_zeroed and dz is not None and abs(dz) <= tol
            if height_ok and vxy_avg <= hover_vxy_tol:
                if settled_since is None:
                    settled_since = time.time()
                elif time.time() - settled_since >= settle_time:
                    with self._lock:
                        self._vel[2] = 0.0
                        self._action_phase = None
                    return
            else:
                settled_since = None
            if time.time() - t0 > timeout:
                raise DroneError(
                    f"{self.ns}: takeoff({alt:.1f}m) 超时"
                    f"（高度{'未到' if not vz_zeroed else '未稳住'}，"
                    f"水平速度 {vxy_avg:.2f} m/s）")
            time.sleep(0.1)

    def goto(self, x, y, z, yaw=None, tol=0.3, timeout=60.0):
        """飞到本地 ENU 点；停稳在目标点上后切回水平 follow 悬停。

        到位判定分两段：先进入 tol 半径，随后**保持目标设定点**直到水平速度
        收敛（≤0.15 m/s 持续 0.3s，最多等 3s）再切 follow——避免带着残余速度
        切 follow 把悬停点冻结在目标点之外（"指哪打哪"）。
        """
        if not self._streaming:
            raise DroneError(f"{self.ns}: 尚未 takeoff，不能 goto")
        with self._lock:
            self._mode = "position"
            self._pos_mode = "local"
            self._target = [float(x), float(y), float(z)]
            if yaw is not None:
                self._sp_yaw = float(yaw)
        t0 = time.time()
        lost_since = None
        arrived_since = None
        while True:
            lost_since = self._check_offboard(lost_since, "goto")
            with self._lock:
                pos = self.pos
                target = tuple(self._target)
                sp_yaw = self._sp_yaw
                vxy = math.hypot(self.vx, self.vy)
            if pos is not None:
                if math.hypot(pos[0] - target[0], pos[1] - target[1]) < tol \
                        and abs(pos[2] - target[2]) < tol:
                    if arrived_since is None:
                        arrived_since = time.time()
                    # 到位后保持目标设定点，等水平速度停稳再切 follow；
                    # 3s 停不稳（持续扰动）也切，不卡死流程
                    settle_elapsed = time.time() - arrived_since
                    if (vxy <= 0.15 and settle_elapsed >= 0.3) \
                            or settle_elapsed >= 3.0:
                        self._set_position_follow(pos[2], sp_yaw)
                        return
                else:
                    arrived_since = None
            if time.time() - t0 > timeout:
                raise DroneError(f"{self.ns}: goto({x:.1f},{y:.1f},{z:.1f}) 超时")
            time.sleep(0.1)

    def move_body(self, forward, left=0.0, up=0.0, yaw=None, tol=0.3,
                  timeout=60.0):
        """从当前 PX4 本地位置按调用瞬间机头 FLU 方向相对移动。"""
        values = (forward, left, up) if yaw is None else (forward, left, up, yaw)
        if not all(math.isfinite(value) for value in values):
            raise ValueError("机体相对位移和航向必须是有限数")
        with self._lock:
            if not self._streaming:
                raise DroneError(f"{self.ns}: 尚未 takeoff，不能 move_body")
            local_pos = self.pos
            actual_yaw = self.yaw
            if local_pos is None:
                raise DroneError(f"{self.ns}: 本地位置不可用，不能 move_body")
            if not math.isfinite(actual_yaw):
                raise DroneError(f"{self.ns}: 当前航向不可用，不能 move_body")
        cos_yaw = math.cos(actual_yaw)
        sin_yaw = math.sin(actual_yaw)
        target = (
            local_pos[0] + forward * cos_yaw - left * sin_yaw,
            local_pos[1] + forward * sin_yaw + left * cos_yaw,
            local_pos[2] + up,
        )
        return self.goto(*target, yaw=yaw, tol=tol, timeout=timeout)

    def circle(self, cx, cy, z, radius, speed, laps=1.0, start_angle=None,
               ccw=True, yaw=None, timeout=120.0):
        """以 (cx, cy, z) 为圆心、radius 为半径画圆（轨迹流，阻塞直到转完）。

        设定点由后台 20Hz 流逐周期沿圆周推进（线速度恒为 speed），PX4 位置控制器
        跟踪连续移动的目标 -> 匀速、平滑的圆，而不是逐点 goto 的"减速-停顿"多边形。
        转 laps 圈后停在起点（=起飞前向方向 +radius 处）并转入水平跟随悬停。

        参数：
            cx, cy, z  圆心 ENU 位置（通常取起飞点作为圆心）
            radius     圆半径 (m)
            speed      圆周线速度 (m/s)，建议 0.5~1.0
            laps       圈数（可为小数）
            start_angle 起始角度（ENU，0=东，逆时针正）；None=从圆心指向当前位置开始
            ccw        True=逆时针（ENU 角度递增），False=顺时针
            yaw        目标航向（None=保持当前）
        抛出：DroneError（未起飞 / Offboard 丢失 / 超时）。超时或失败会先切到
        水平 follow 悬停再抛错，不留失控的圆设定点。
        """
        if not self._streaming:
            raise DroneError(f"{self.ns}: 尚未 takeoff，不能 circle")
        if not all(math.isfinite(float(v)) for v in (cx, cy, z, radius, speed, laps)):
            raise ValueError("圆轨迹参数必须是有限数")
        if radius <= 0 or speed <= 0 or laps <= 0:
            raise ValueError("radius/speed/laps 必须为正数")
        omega = speed / radius
        if not ccw:
            omega = -omega
        with self._lock:
            pos = self.pos
            if start_angle is None:
                if pos is None:
                    raise DroneError(f"{self.ns}: 本地位置不可用，不能 circle")
                start_angle = math.atan2(pos[1] - cy, pos[0] - cx)
            self._mode = "position"
            self._pos_mode = "circle"
            self._circle_cx = float(cx)
            self._circle_cy = float(cy)
            self._circle_radius = float(radius)
            self._circle_omega = float(omega)
            self._circle_angle = float(start_angle)
            self._circle_start_angle = float(start_angle)
            self._circle_target_rad = 2.0 * math.pi * float(laps)
            self._circle_last_t = time.monotonic()
            self._target = [
                self._circle_cx + radius * math.cos(start_angle),
                self._circle_cy + radius * math.sin(start_angle),
                float(z),
            ]
            self._target_z = float(z)
            if yaw is not None:
                self._sp_yaw = float(yaw)
            self._action_phase = "circle"
        t0 = time.time()
        lost_since = None
        while True:
            lost_since = self._check_offboard(lost_since, "circle")
            with self._lock:
                swept = self._circle_angle - self._circle_start_angle
            if abs(swept) >= self._circle_target_rad:
                self._set_position_follow(float(z), self._sp_yaw)
                with self._lock:
                    self._action_phase = None
                return
            if time.time() - t0 > timeout:
                self._set_position_follow(float(z), self._sp_yaw)
                with self._lock:
                    self._action_phase = None
                raise DroneError(
                    f"{self.ns}: circle(圆心=({cx:.1f},{cy:.1f}), r={radius:.1f}m, "
                    f"{laps:.1f}圈) 超时")
            time.sleep(0.1)

    def circle_velocity(self, cx, cy, z, radius, speed, laps=1.0, start_angle=None,
                        ccw=True, yaw_rate=0.0, timeout=120.0):
        """以 (cx, cy, z) 为圆心画圆（**速度模式**轨迹流，阻塞直到转完）。

        与 circle()（位置设定点流）不同：本方法发**切向速度设定点**
        （mode.velocity=True，速度矢量沿圆周切线、大小恒为 speed），PX4 直接
        执行速度矢量，不受"位置设定点低速率追圆导致径向内切"影响，半径更贴近
        设定值。代价是速度模式没有位置闭环，**整圆会漂移**（真机 2 圈漂约 0.3m）。
        转 laps 圈（内部角度判定）后切回位置 follow 悬停。

        参数同 circle()，另：
            yaw_rate  偏航角速度 (rad/s，ENU 逆时针正)；默认 0=保持机头不转
        抛出：DroneError（未起飞 / Offboard 丢失 / 超时）。超时或失败先切 follow 悬停。
        """
        if not self._streaming:
            raise DroneError(f"{self.ns}: 尚未 takeoff，不能 circle_velocity")
        if not all(math.isfinite(float(v)) for v in (cx, cy, z, radius, speed, laps)):
            raise ValueError("圆轨迹参数必须是有限数")
        if radius <= 0 or speed <= 0 or laps <= 0:
            raise ValueError("radius/speed/laps 必须为正数")
        omega = speed / radius
        if not ccw:
            omega = -omega
        with self._lock:
            pos = self.pos
            if start_angle is None:
                if pos is None:
                    raise DroneError(f"{self.ns}: 本地位置不可用，不能 circle_velocity")
                start_angle = math.atan2(pos[1] - cy, pos[0] - cx)
            self._mode = "velocity"
            self._circle_velocity_mode = True
            self._circle_cx = float(cx)
            self._circle_cy = float(cy)
            self._circle_radius = float(radius)
            self._circle_omega = float(omega)
            self._circle_angle = float(start_angle)
            self._circle_start_angle = float(start_angle)
            self._circle_target_rad = 2.0 * math.pi * float(laps)
            self._circle_last_t = time.monotonic()
            # 初始切向速度：ENU 圆上 d/dθ = (-R sinθ, R cosθ)，乘角速度即速度
            self._vel = [
                -radius * omega * math.sin(start_angle),
                 radius * omega * math.cos(start_angle),
                 0.0,
            ]
            self._yaw_rate = float(yaw_rate)
            self._target_z = float(z)
            self._action_phase = "circle_velocity"
        t0 = time.time()
        lost_since = None
        while True:
            lost_since = self._check_offboard(lost_since, "circle_velocity")
            with self._lock:
                swept = self._circle_angle - self._circle_start_angle
            if abs(swept) >= self._circle_target_rad:
                self._set_position_follow(float(z), self._sp_yaw)
                with self._lock:
                    self._action_phase = None
                    self._circle_velocity_mode = False
                return
            if time.time() - t0 > timeout:
                self._set_position_follow(float(z), self._sp_yaw)
                with self._lock:
                    self._action_phase = None
                    self._circle_velocity_mode = False
                raise DroneError(
                    f"{self.ns}: circle_velocity(圆心=({cx:.1f},{cy:.1f}), "
                    f"r={radius:.1f}m, {laps:.1f}圈) 超时")
            time.sleep(0.1)

    def circle_smooth(self, cx, cy, z, radius, speed, laps=1.0, start_angle=None,
                      ccw=True, k=0.3, k_i=0.1, yaw_rate=0.0, timeout=120.0):
        """以 (cx, cy, z) 为圆心画圆（**平滑模式**：速度前馈为主 + 小增益纠正）。

        针对"位置模式追估计噪声导致物理轨迹抖动"（真机 log_6：EKF 估计绕圆摆
        ±0.11m，物理跟着摆 ±0.13m）：主通道是切向速度前馈（速度模式，轨迹本身
        平滑），只用增益 k（≈位置模式 P≈0.95 的 1/3）做径向偏差与相位纠正——
        噪声增益降 ~3 倍，物理轨迹肉眼压线；同时小增益纠正把 circle_velocity
        的开环漂移（真机 2 圈漂 ~0.3m）和圈数不准兜住。
        径向另有积分项 k_i：真机速度执行存在近似恒定的旋转偏差（log_8 实测
        指令径向 -0.13 向内、实际 +0.09 向外），纯 P 会在错误半径上卡住，
        积分慢速收口到指令半径，限幅 ±0.25 m/s 防 windup。
        起步 1s 速度缓升防入圆甩动；z 通道走小增益位置闭环（±0.5 m/s 限幅），
        速度模式下高度不会随扰动漂移（真机 log_24：曾从 1.0m 漂到 1.3m）。
        转 laps 圈（指令角度判定，相位纠正保证实际跟得上）后切回位置 follow 悬停。

        参数同 circle_velocity()，另：
            k    径向/相位纠正增益 (1/s)，越小越平滑但纠偏越慢，建议 0.2~0.4
            k_i  径向积分增益，收口恒定半径偏差的快慢，建议 0.05~0.15；0=关闭
        抛出：DroneError（未起飞 / Offboard 丢失 / 超时）。超时或失败先切 follow 悬停。
        """
        if not self._streaming:
            raise DroneError(f"{self.ns}: 尚未 takeoff，不能 circle_smooth")
        if not all(math.isfinite(float(v))
                   for v in (cx, cy, z, radius, speed, laps, k, k_i)):
            raise ValueError("圆轨迹参数必须是有限数")
        if radius <= 0 or speed <= 0 or laps <= 0 or k <= 0 or k_i < 0:
            raise ValueError("radius/speed/laps/k 必须为正数，k_i 不能为负")
        omega = speed / radius
        if not ccw:
            omega = -omega
        with self._lock:
            pos = self.pos
            if start_angle is None:
                if pos is None:
                    raise DroneError(f"{self.ns}: 本地位置不可用，不能 circle_smooth")
                start_angle = math.atan2(pos[1] - cy, pos[0] - cx)
            self._mode = "velocity"
            self._circle_smooth_mode = True
            self._circle_cx = float(cx)
            self._circle_cy = float(cy)
            self._circle_radius = float(radius)
            self._circle_omega = float(omega)
            self._circle_angle = float(start_angle)
            self._circle_start_angle = float(start_angle)
            self._circle_target_rad = 2.0 * math.pi * float(laps)
            self._circle_last_t = time.monotonic()
            self._circle_t0 = self._circle_last_t
            self._circle_k = float(k)
            self._circle_ki = float(k_i)
            self._circle_radial_int = 0.0
            # 初始切向速度（ramp 从 0 缓升，初值给 0 即可）
            self._vel = [0.0, 0.0, 0.0]
            self._yaw_rate = float(yaw_rate)
            self._target_z = float(z)
            self._action_phase = "circle_smooth"
        t0 = time.time()
        lost_since = None
        while True:
            lost_since = self._check_offboard(lost_since, "circle_smooth")
            with self._lock:
                swept = self._circle_angle - self._circle_start_angle
            if abs(swept) >= self._circle_target_rad:
                self._set_position_follow(float(z), self._sp_yaw)
                with self._lock:
                    self._action_phase = None
                    self._circle_smooth_mode = False
                return
            if time.time() - t0 > timeout:
                self._set_position_follow(float(z), self._sp_yaw)
                with self._lock:
                    self._action_phase = None
                    self._circle_smooth_mode = False
                raise DroneError(
                    f"{self.ns}: circle_smooth(圆心=({cx:.1f},{cy:.1f}), "
                    f"r={radius:.1f}m, {laps:.1f}圈) 超时")
            time.sleep(0.1)

    def _check_offboard(self, lost_since, action):
        if self.offboard:
            return None
        if lost_since is None:
            return time.time()
        if time.time() - lost_since > 2.0:
            reason = "failsafe=true" if self.failsafe else "failsafe=false"
            raise DroneError(
                f"{self.ns}: Offboard 中途丢失（{reason}），{action} 中止")
        return lost_since

    def set_velocity(self, vx, vy, vz, yaw_rate=0.0):
        """速度控制（非阻塞）：以 ENU 速度 (vx, vy, vz) 飞行，yaw_rate 逆时针为正 (rad/s)。

        调用后飞机持续按该速度飞，直到 hover()/goto()/land() 改变状态。
        """
        if not self._streaming:
            raise DroneError(f"{self.ns}: 尚未 takeoff，不能 set_velocity")
        self._mode = "velocity"
        self._vel = [float(vx), float(vy), float(vz)]
        self._yaw_rate = float(yaw_rate)

    def arm(self, timeout=10.0):
        """解锁（命令重发直到确认已解锁）。不切换飞行模式。"""
        self._wait_connected()
        t0 = time.time()
        while not self.armed:
            if time.time() - t0 > timeout:
                raise DroneError(f"{self.ns}: 解锁超时（检查 preflight 状态）")
            self._command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
            time.sleep(0.5)

    # PX4 强制上锁（kill switch）：VEHICLE_CMD_COMPONENT_ARM_DISARM 的 param2 填这个魔数时，
    # 即使判断为在空中也立即停转电机
    _FORCE_DISARM_MAGIC = 21196.0

    def disarm(self, timeout=10.0):
        """上锁（锁桨，命令重发直到确认已上锁）。

        前一半时间用普通上锁；若 PX4 因"判定在空中"持续拒绝，后一半时间自动升级为
        强制上锁（kill，param2=21196）——因为地面站"上锁"按钮的语义就是必须停桨。
        警告：飞行中调用必然停桨坠落，谨慎！
        """
        t0 = time.time()
        force = False
        while self.armed:
            elapsed = time.time() - t0
            if elapsed > timeout:
                raise DroneError(f"{self.ns}: 上锁超时")
            if elapsed > timeout / 2:
                force = True
            self._command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
                          0.0, self._FORCE_DISARM_MAGIC if force else 0.0)
            time.sleep(0.5)

    def rtl(self, timeout=15.0):
        """返航（命令重发直到飞控确认进入返航模式）。返航与降落过程由 PX4 执行，
        本函数在模式切换成功后即返回，不等待落地。

        与 land() 同理必须先停掉 Offboard 设定点流：持续的设定点流会干扰
        PX4 的返航-降落衔接，飞机会悬在返航点上方不落地（实测踩过的坑）。
        再次 takeoff 会自动重启设定点流，无需额外处理。"""
        self._streaming = False
        self._wait_connected()
        t0 = time.time()
        while self.nav_state != VehicleStatus.NAVIGATION_STATE_AUTO_RTL:
            if time.time() - t0 > timeout:
                raise DroneError(f"{self.ns}: 返航模式切换超时")
            self._command(VehicleCommand.VEHICLE_CMD_NAV_RETURN_TO_LAUNCH)
            time.sleep(0.5)

    def _set_position_follow(self, target_z, sp_yaw):
        """切换到 position+follow：水平持续跟随当前位置，垂直目标固定为 target_z。

        这是 Offboard 悬停/起飞悬停的正确语义——不冻结绝对水平点，让水平 setpoint
        每帧重锚到 EKF 当前估计位置，从根上规避慢漂导致的冲飞。
        """
        self._mode = "position"
        self._pos_mode = "follow"
        self._target_z = float(target_z)
        # fallback：self.pos 意外为 None 时退回绝对点；正常由 _publish_setpoint 跟随
        self._target = list(self.pos) if self.pos is not None else [0.0, 0.0, float(target_z)]
        self._sp_yaw = float(sp_yaw)

    def hover(self):
        """原地悬停（零速度模式）：水平/垂直速度恒 0，物理原地不动。

        与位置 follow 悬停的区别：follow 把水平目标锚到 EKF 估计，估计漂移时
        飞机跟着漂（"起飞后往前跑"的根源）；零速度悬停只要求估计速度=0
        （物理静止时速度估计≈0），位置估计怎么漂都不影响，物理保持原地——
        与手动 Position 杆中位的稳悬停同理。
        """
        self._wait_connected()
        with self._lock:
            self._mode = "velocity"
            self._vel = [0.0, 0.0, 0.0]
            self._yaw_rate = 0.0

    def land(self, timeout=60.0):
        """原地降落，返回时已上锁。降落由 PX4 执行。

        必须先停掉 Offboard 设定点流：持续的位置设定点会让 PX4 保持 Offboard
        并拒绝 NAV_LAND，飞机会悬停不动。
        """
        with self._lock:
            self._action_phase = "land"
        self._streaming = False
        t0 = time.time()
        while self.armed:
            if time.time() - t0 > timeout:
                raise DroneError(f"{self.ns}: 降落超时")
            self._command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
            time.sleep(0.5)
        with self._lock:
            self._action_phase = None

    def shutdown(self):
        """释放 ROS 资源。程序退出前应调用（或交给 Swarm.shutdown）。"""
        self._stop.set()
        self._exec.shutdown()
        current = threading.current_thread()
        for thread in (self._spin_thread, self._stream_thread):
            if thread is not current and thread.is_alive():
                thread.join(timeout=2.0)
        self.node.destroy_node()
