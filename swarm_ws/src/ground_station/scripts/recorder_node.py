#!/usr/bin/env python3
"""SwarmCore 话题记录节点

订阅 /recorder/config (std_msgs/String, JSON):
  {"action": "start", "topics": ["/uav_1/fmu/out/vehicle_status", ...], "note": "实验备注"}
  {"action": "stop"}
  {"action": "delete", "name": "bag_20260727_120000"}
收到 start 时用 `ros2 bag record` 录制勾选的话题（只录所选，控制磁盘占用），
stop 时 SIGINT 优雅停止并写入 metadata.json（备注/话题/大小）。
状态发布到 /recorder/status (String, JSON)，1Hz，含历史 bag 列表与总占用。

由 start_ground_station.sh 拉起，依赖: ros2 bag（ros-humble-rosbag2）。
"""
import json
import os
import re
import shutil
import signal
import subprocess
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

BAG_BASE = os.path.expanduser("~/0c00_ws/swarm_ws/logs/bags")
SETUP_CMD = ("source /opt/ros/humble/setup.bash && "
             "source ~/0c00_ws/swarm_ws/install/setup.bash && ")
BAG_NAME_RE = re.compile(r"^bag_\d{8}_\d{6}$")


def dir_size_mb(path):
    total = 0
    for root, _dirs, files in os.walk(path):
        for f in files:
            try:
                total += os.path.getsize(os.path.join(root, f))
            except OSError:
                pass
    return round(total / 1e6, 1)


def list_bags(limit=10):
    """最近的 bag 列表（新在前）+ 总占用，随状态一起发给网页。"""
    bags, total = [], 0.0
    try:
        names = sorted((n for n in os.listdir(BAG_BASE) if BAG_NAME_RE.match(n)), reverse=True)
    except OSError:
        names = []
    for n in names:
        p = os.path.join(BAG_BASE, n)
        if not os.path.isdir(p):
            continue
        mb = dir_size_mb(p)
        total += mb
        if len(bags) < limit:
            bags.append({"name": n, "mb": mb})
    return bags, round(total, 1)


class RecorderNode(Node):
    def __init__(self):
        super().__init__("swarm_recorder")
        self.proc = None
        self.bag_dir = None
        self.topics = []
        self.note = ""
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
            self._start(cfg.get("topics") or [], note=str(cfg.get("note") or ""))
        elif action == "stop":
            self._stop()
        elif action == "delete":
            self._delete(str(cfg.get("name") or ""))

    def _delete(self, name):
        # 只允许删除 BAG_BASE 下形如 bag_YYYYMMDD_HHMMSS 的目录，防路径穿越
        if not BAG_NAME_RE.match(name):
            self.get_logger().warn(f"拒绝删除非法名称: {name!r}")
            return
        if self.bag_dir and os.path.basename(self.bag_dir) == name and self.proc is not None:
            self.get_logger().warn("正在录制中的 bag 不能删除")
            return
        path = os.path.join(BAG_BASE, name)
        if os.path.isdir(path):
            shutil.rmtree(path, ignore_errors=True)
            self.get_logger().info(f"已删除 bag: {name}")

    def _write_metadata(self):
        """把实验备注写入 bag 目录（对应产品定义书附录 D 实验记录模板）。"""
        if not self.bag_dir or not os.path.isdir(self.bag_dir):
            return
        meta = {"note": self.note, "topics": self.topics,
                "start_time": os.path.basename(self.bag_dir).replace("bag_", ""),
                "size_mb": dir_size_mb(self.bag_dir)}
        try:
            with open(os.path.join(self.bag_dir, "metadata.json"), "w", encoding="utf-8") as f:
                json.dump(meta, f, ensure_ascii=False, indent=2)
        except OSError as e:
            self.get_logger().warn(f"写入 metadata.json 失败: {e}")

    def _start(self, topics, note=""):
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
        self.note = note[:500]
        cmd = f"{SETUP_CMD}exec ros2 bag record -o '{self.bag_dir}' " + " ".join(topics)
        self.proc = subprocess.Popen(["bash", "-c", cmd],
                                     preexec_fn=os.setsid)  # 进程组，便于整体 SIGINT
        self.topics = topics
        self.get_logger().info(f"开始录制 {len(topics)} 个话题 -> {self.bag_dir}")
        self._write_metadata()

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
        self._write_metadata()   # 停止时补写（含最终大小）

    def _publish_status(self):
        if self.proc is not None and self.proc.poll() is not None:
            self.get_logger().warn(f"ros2 bag 进程退出 (code={self.proc.returncode})")
            self.proc = None
        bags, total_mb = list_bags()
        st = {
            "recording": self.proc is not None,
            "dir": self.bag_dir if (self.proc or self.bag_dir) else None,
            "topics": self.topics if self.proc else [],
            "size_mb": dir_size_mb(self.bag_dir) if self.bag_dir and os.path.isdir(self.bag_dir) else 0,
            "bags": bags,
            "total_mb": total_mb,
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
