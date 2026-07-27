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
        self.pos = None        # ENU (x东, y北, z上)
        self.yaw = 0.0         # ENU 航向
        self.armed = False
        self.offboard = False
        self.nav_state = 0     # PX4 导航状态原始值（VehicleStatus.nav_state）

        # ---- 当前设定点（由动作原语修改，流线程负责持续发出） ----
        self._mode = "position"           # "position" | "velocity"
        self._target = [0.0, 0.0, 0.0]    # ENU 位置目标
        self._sp_yaw = 0.0                # ENU 航向目标
        self._vel = [0.0, 0.0, 0.0]       # ENU 速度目标
        self._yaw_rate = 0.0
        self._streaming = False           # True 后后台线程开始 20Hz 发设定点

        self.node = Node(f"swarm_api_{namespace.replace('/', '_')}")

        pub_qos = QoSProfile(depth=10)  # fmu/in 必须 reliable（默认），否则指令被静默丢弃
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
        # 飞控上报 NED，转成 ENU 存起来；heading 是 NED 航向，也转 ENU
        self.pos = (m.y, m.x, -m.z)
        self.yaw = math.pi / 2 - m.heading

    def _on_status(self, m):
        self.armed = m.arming_state == VehicleStatus.ARMING_STATE_ARMED
        self.offboard = m.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD
        self.nav_state = m.nav_state

    # ---------------- 设定点流（后台 20Hz） ----------------
    def _stream_loop(self):
        while not self._stop.is_set():
            if self._streaming:
                self._publish_setpoint()
            time.sleep(0.05)

    def _publish_setpoint(self):
        ts = int(time.time() * 1e6)
        mode = OffboardControlMode()
        mode.timestamp = ts
        sp = TrajectorySetpoint()
        sp.timestamp = ts
        if self._mode == "position":
            mode.position = True
            sp.position = [float(v) for v in enu_to_ned(*self._target)]
            sp.velocity = [NAN, NAN, NAN]
            sp.yaw = float(yaw_enu_to_ned(self._sp_yaw))
        else:  # velocity
            mode.velocity = True
            sp.position = [NAN, NAN, NAN]
            sp.velocity = [float(v) for v in enu_to_ned(*self._vel)]
            sp.yawspeed = float(-self._yaw_rate)  # ENU 逆时针正 -> NED 顺时针正
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
    def takeoff(self, alt=1.5, tol=0.3, timeout=60.0):
        """起飞到相对高度 alt（米）。返回时飞机已在空中悬停。

        高度用"当前实测高度 + alt"而不是绝对值，规避 EKF 高度原点偏差。
        """
        self._wait_connected()
        x, y, z0 = self.pos
        # 先以当前位置为目标预发设定点 1 秒：PX4 要求先收到设定点流才允许切 Offboard
        self._mode = "position"
        self._target = [x, y, z0 + alt]
        self._sp_yaw = self.yaw
        self._streaming = True
        time.sleep(1.0)

        # 请求 Offboard + 解锁，命令要重发直到生效（飞控可能丢掉单条命令）
        t0 = time.time()
        while not (self.offboard and self.armed):
            if time.time() - t0 > timeout:
                raise DroneError(f"{self.ns}: 进入 Offboard/解锁超时（检查 preflight 状态）")
            self._command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)  # 主模式 6 = Offboard
            self._command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
            time.sleep(0.5)

        self.goto(x, y, z0 + alt, tol=tol, timeout=timeout)

    def goto(self, x, y, z, yaw=None, tol=0.3, timeout=60.0):
        """飞到 ENU 点 (x, y, z)，yaw 为 ENU 航向（None=保持当前航向）。到达后返回。"""
        if not self._streaming:
            raise DroneError(f"{self.ns}: 尚未 takeoff，不能 goto")
        self._mode = "position"
        self._target = [float(x), float(y), float(z)]
        if yaw is not None:
            self._sp_yaw = float(yaw)
        t0 = time.time()
        lost_since = None
        while True:
            # Offboard 中途丢失说明飞控触发了 failsafe（姿态异常/围栏/失联），
            # 飞控已接管，再等下去没有意义，立即报错让上层处理
            if not self.offboard:
                if lost_since is None:
                    lost_since = time.time()
                elif time.time() - lost_since > 2.0:
                    raise DroneError(f"{self.ns}: Offboard 中途丢失（飞控触发 failsafe 接管），goto 中止")
            else:
                lost_since = None
            if self.pos is not None:
                d_xy = math.hypot(self.pos[0] - x, self.pos[1] - y)
                d_z = abs(self.pos[2] - z)
                if d_xy < tol and d_z < tol:
                    return
            if time.time() - t0 > timeout:
                raise DroneError(f"{self.ns}: goto({x:.1f},{y:.1f},{z:.1f}) 超时")
            time.sleep(0.1)

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

    def disarm(self, timeout=10.0):
        """上锁（命令重发直到确认已上锁）。警告：飞行中上锁电机立即停转！"""
        t0 = time.time()
        while self.armed:
            if time.time() - t0 > timeout:
                raise DroneError(f"{self.ns}: 上锁超时")
            self._command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)
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

    def hover(self):
        """原地悬停（把位置目标设为当前位置）。"""
        self._wait_connected()
        self._mode = "position"
        self._target = list(self.pos)
        self._sp_yaw = self.yaw

    def land(self, timeout=60.0):
        """原地降落，返回时已上锁。降落由 PX4 执行。

        必须先停掉 Offboard 设定点流：持续的位置设定点会让 PX4 保持 Offboard
        并拒绝 NAV_LAND，飞机会悬停不动。
        """
        self._streaming = False
        t0 = time.time()
        while self.armed:
            if time.time() - t0 > timeout:
                raise DroneError(f"{self.ns}: 降落超时")
            self._command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
            time.sleep(0.5)

    def shutdown(self):
        """释放 ROS 资源。程序退出前应调用（或交给 Swarm.shutdown）。"""
        self._stop.set()
        self._exec.shutdown()
        self.node.destroy_node()
