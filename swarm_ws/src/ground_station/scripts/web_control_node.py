#!/usr/bin/env python3
"""Web 地面站控制后端（web_control_node）

把网页按钮的"单发 VehicleCommand"升级为 swarm_api 的可靠动作：
- 命令自动重发直到飞控状态确认（解锁/上锁/返航）
- 起飞走完整 Offboard 流程（预发设定点流 → 切模式 → 解锁 → 爬升）
- 每机一把互斥锁，同一架飞机上一个动作没执行完时拒绝新动作
- 每个动作都有结果回执，网页可以明确提示成功/失败

接口（std_msgs/String，JSON）：
  订阅 /web_control/cmd    {"id": 1, "ns": "uav_1", "action": "takeoff", "alt": 1.5}
  发布 /web_control/result {"id": 1, "ns": "uav_1", "action": "takeoff", "ok": true, "msg": "ok"}

action 取值: arm / disarm / takeoff / land / rtl / hover
"""

import json
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from swarm_api import Drone

ACTIONS = {"arm", "disarm", "takeoff", "land", "rtl", "hover"}


class WebControl(Node):
    def __init__(self):
        super().__init__("web_control")
        self.drones = {}   # ns -> Drone（懒创建，复用其状态订阅与设定点流）
        self.locks = {}    # ns -> Lock（每机同时只执行一个动作）
        self.create_subscription(String, "/web_control/cmd", self._on_cmd, 10)
        self.pub = self.create_publisher(String, "/web_control/result", 10)
        self.get_logger().info("web_control 后端已就绪，等待 /web_control/cmd")

    def _drone(self, ns):
        if ns not in self.drones:
            self.drones[ns] = Drone(ns)
            self.locks[ns] = threading.Lock()
        return self.drones[ns]

    def _on_cmd(self, msg):
        try:
            req = json.loads(msg.data)
            assert req["action"] in ACTIONS and isinstance(req["ns"], str)
        except Exception:
            self.get_logger().warn(f"忽略非法指令: {msg.data[:100]}")
            return
        drone = self._drone(req["ns"])
        lock = self.locks[req["ns"]]
        if not lock.acquire(blocking=False):
            self._reply(req, False, "上一个动作还在执行中，请稍候")
            return
        threading.Thread(target=self._run, args=(req, drone, lock), daemon=True).start()

    def _run(self, req, drone, lock):
        try:
            action = req["action"]
            if action == "arm":
                drone.arm()
            elif action == "disarm":
                drone.disarm()
            elif action == "takeoff":
                drone.takeoff(float(req.get("alt", 1.5)))
            elif action == "land":
                drone.land()
            elif action == "rtl":
                drone.rtl()
            elif action == "hover":
                drone.hover()
            self._reply(req, True, "ok")
        except Exception as e:  # DroneError 或其他异常都回执给网页
            self._reply(req, False, str(e))
        finally:
            lock.release()

    def _reply(self, req, ok, msg):
        out = {"id": req.get("id"), "ns": req.get("ns"),
               "action": req.get("action"), "ok": ok, "msg": msg}
        self.pub.publish(String(data=json.dumps(out, ensure_ascii=False)))
        self.get_logger().info(f"{out['ns']} {out['action']}: {'OK' if ok else 'FAIL'} ({msg})")

    def shutdown_drones(self):
        for d in self.drones.values():
            d.shutdown()


def main():
    rclpy.init()
    node = WebControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown_drones()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
