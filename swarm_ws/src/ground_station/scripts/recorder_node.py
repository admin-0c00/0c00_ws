#!/usr/bin/env python3
"""SwarmCore 话题记录节点

订阅 /recorder/config (std_msgs/String, JSON):
  {"action": "start", "topics": ["/uav_1/fmu/out/vehicle_status", ...]}
  {"action": "stop"}
收到 start 时用 `ros2 bag record` 录制勾选的话题（只录所选，控制磁盘占用），
stop 时 SIGINT 优雅停止。状态发布到 /recorder/status (String, JSON)，1Hz。

由 start_ground_station.sh 拉起，依赖: ros2 bag（ros-humble-rosbag2）。
"""
import json
import os
import signal
import subprocess
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

BAG_BASE = os.path.expanduser("~/0c00_ws/swarm_ws/logs/bags")
SETUP_CMD = ("source /opt/ros/humble/setup.bash && "
             "source ~/0c00_ws/swarm_ws/install/setup.bash && ")


def dir_size_mb(path):
    total = 0
    for root, _dirs, files in os.walk(path):
        for f in files:
            try:
                total += os.path.getsize(os.path.join(root, f))
            except OSError:
                pass
    return round(total / 1e6, 1)


class RecorderNode(Node):
    def __init__(self):
        super().__init__("swarm_recorder")
        self.proc = None
        self.bag_dir = None
        self.topics = []
        self.create_subscription(String, "/recorder/config", self._config_cb, 10)
        self.status_pub = self.create_publisher(String, "/recorder/status", 10)
        self.create_timer(1.0, self._publish_status)
        os.makedirs(BAG_BASE, exist_ok=True)
        self.get_logger().info(f"话题记录节点就绪，bag 目录: {BAG_BASE}")

    def _config_cb(self, msg):
        try:
            cfg = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn("无法解析的 config JSON")
            return
        action = cfg.get("action")
        if action == "start":
            self._start(cfg.get("topics") or [])
        elif action == "stop":
            self._stop()

    def _start(self, topics):
        if self.proc is not None:
            self.get_logger().warn("已在录制中，忽略 start")
            return
        # 只接受 / 开头的绝对话题名，防注入
        topics = [t for t in topics if isinstance(t, str) and t.startswith("/")
                  and all(c.isalnum() or c in "/_" for c in t)]
        if not topics:
            self.get_logger().warn("start 但未选择任何话题")
            return
        self.bag_dir = os.path.join(BAG_BASE, time.strftime("bag_%Y%m%d_%H%M%S"))
        cmd = f"{SETUP_CMD}exec ros2 bag record -o '{self.bag_dir}' " + " ".join(topics)
        self.proc = subprocess.Popen(["bash", "-c", cmd],
                                     preexec_fn=os.setsid)  # 进程组，便于整体 SIGINT
        self.topics = topics
        self.get_logger().info(f"开始录制 {len(topics)} 个话题 -> {self.bag_dir}")

    def _stop(self):
        if self.proc is None:
            return
        self.get_logger().info("停止录制")
        try:
            os.killpg(os.getpgid(self.proc.pid), signal.SIGINT)  # ros2 bag 需要 SIGINT 收尾
            self.proc.wait(timeout=10)
        except Exception:
            try:
                os.killpg(os.getpgid(self.proc.pid), signal.SIGKILL)
            except Exception:
                pass
        self.proc = None

    def _publish_status(self):
        if self.proc is not None and self.proc.poll() is not None:
            self.get_logger().warn(f"ros2 bag 进程退出 (code={self.proc.returncode})")
            self.proc = None
        st = {
            "recording": self.proc is not None,
            "dir": self.bag_dir if (self.proc or self.bag_dir) else None,
            "topics": self.topics if self.proc else [],
            "size_mb": dir_size_mb(self.bag_dir) if self.bag_dir and os.path.isdir(self.bag_dir) else 0,
        }
        self.status_pub.publish(String(data=json.dumps(st)))

    def destroy_node(self):
        self._stop()
        super().destroy_node()


def main():
    rclpy.init()
    node = RecorderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
