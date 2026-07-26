#!/usr/bin/env python3
"""SwarmCore 集群起飞基线（产品定义书 M2：三机 SITL + Offboard 控制）

对 /uav_1../uav_N 命名空间下的每架仿真无人机：
  1. 持续发布 OffboardControlMode + TrajectorySetpoint（位置环，z=-takeoff_alt，NED）
  2. 切入 Offboard 模式并解锁
  3. 全体到达目标高度后报告成功，继续保持悬停

用法: ros2 run 或 python3 swarm_takeoff.py --ros-args -p num_drones:=3
依赖: px4_msgs（需先 source swarm_ws/install/setup.bash）
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus,
)


def now_us():
    return int(rclpy.clock.Clock().now().nanoseconds / 1000)


class UavController:
    """单机 Offboard 起飞状态机"""

    def __init__(self, node: Node, namespace: str, sysid: int, takeoff_alt: float):
        self.node = node
        self.ns = namespace
        self.sysid = sysid
        self.takeoff_alt = takeoff_alt
        self.offboard_setpoint_counter = 0
        self.nav_state = None
        self.armed = False
        self.z = None  # NED，到达 -takeoff_alt 即视为成功
        self.reached = False

        # PX4 uXRCE 的 fmu/in 订阅端是 RELIABLE，发布端必须用默认 QoS（reliable）
        pub_qos = QoSProfile(depth=10)
        sub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        pub = lambda topic, msg_type: node.create_publisher(
            msg_type, f"/{namespace}/fmu/in/{topic}", pub_qos
        )
        self.pub_offboard_mode = pub("offboard_control_mode", OffboardControlMode)
        self.pub_setpoint = pub("trajectory_setpoint", TrajectorySetpoint)
        self.pub_command = pub("vehicle_command", VehicleCommand)

        node.create_subscription(
            VehicleLocalPosition, f"/{namespace}/fmu/out/vehicle_local_position",
            lambda msg: setattr(self, "z", msg.z), sub_qos)
        node.create_subscription(
            VehicleStatus, f"/{namespace}/fmu/out/vehicle_status",
            self._status_cb, sub_qos)

    def _status_cb(self, msg):
        self.nav_state = msg.nav_state
        self.armed = msg.arming_state == VehicleStatus.ARMING_STATE_ARMED

    def publish_command(self, command, **params):
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.target_system = self.sysid
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = now_us()
        self.pub_command.publish(msg)

    def step(self):
        """每个控制周期调用一次（>=2Hz 才能保持 Offboard）"""
        ts = now_us()

        mode = OffboardControlMode()
        mode.position = True
        mode.timestamp = ts
        self.pub_offboard_mode.publish(mode)

        sp = TrajectorySetpoint()
        sp.position = [0.0, 0.0, -self.takeoff_alt]
        sp.yaw = 0.0
        sp.timestamp = ts
        self.pub_setpoint.publish(sp)

        # 先流 10 拍设定点，再切 Offboard + 解锁；未成功前每 1s 重试
        self.offboard_setpoint_counter += 1
        in_offboard = self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD
        if self.offboard_setpoint_counter >= 10 and not (in_offboard and self.armed):
            if self.offboard_setpoint_counter % 10 == 0:
                self.node.get_logger().info(f"[{self.ns}] 请求 Offboard 模式并解锁")
                self.publish_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
                                     param1=1.0, param2=6.0)
                self.publish_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
                                     param1=1.0)

        if not self.reached and self.z is not None and self.z < -(self.takeoff_alt - 0.5):
            self.reached = True
            self.node.get_logger().info(
                f"[{self.ns}] 到达目标高度 {self.takeoff_alt} m (z={self.z:.2f})")


class SwarmTakeoff(Node):
    def __init__(self):
        super().__init__("swarm_takeoff")
        self.declare_parameter("num_drones", 3)
        self.declare_parameter("takeoff_alt", 5.0)
        self.declare_parameter("namespace_prefix", "uav_")
        n = self.get_parameter("num_drones").value
        alt = self.get_parameter("takeoff_alt").value
        prefix = self.get_parameter("namespace_prefix").value

        self.uavs = [UavController(self, f"{prefix}{i}", i, alt) for i in range(1, n + 1)]
        self.timer = self.create_timer(0.1, self._tick)  # 10 Hz
        self.get_logger().info(f"集群起飞基线: {n} 架, 目标高度 {alt} m")

    def _tick(self):
        for uav in self.uavs:
            uav.step()
        if all(u.reached for u in self.uavs):
            if not getattr(self, "_reported", False):
                self._reported = True
                zs = ", ".join(f"{u.ns}: z={u.z:.2f}" for u in self.uavs)
                self.get_logger().info(f"全体到达目标高度! {zs}")


def main():
    rclpy.init()
    node = SwarmTakeoff()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
