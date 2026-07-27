#!/usr/bin/env python3
"""SwarmCore 话题记录节点

订阅 /recorder/config (std_msgs/String, JSON):
  {"action": "start", "topics": ["/uav_1/fmu/out/vehicle_status", ...], "note": "实验备注"}
  {"action": "stop"}
  {"action": "delete", "name": "bag_20260727_120000"}
  {"action": "play", "name": "bag_..."}        回放（ros2 bag play，回放前请先停仿真/真机）
  {"action": "play_stop"}                      停止回放
  {"action": "export_csv", "name": "bag_..."}  导出全部话题为 CSV 并打包 zip（通用，不限消息类型）
收到 start 时用 `ros2 bag record` 录制勾选的话题（只录所选，控制磁盘占用），
stop 时 SIGINT 优雅停止并写入 metadata.json（备注/话题/大小）。
状态发布到 /recorder/status (String, JSON)，1Hz，含历史 bag 列表、时长/消息数、
回放与导出状态。

由 start_ground_station.sh 拉起，依赖: ros2 bag（ros-humble-rosbag2）。
"""
import json
import os
import re
import shutil
import signal
import subprocess
import threading
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


def bag_brief(path):
    """bag 摘要：时长/消息数（metadata.yaml）+ 实验备注（metadata.json）+ 话题清单。"""
    dur, msgs, note, topics = None, None, "", []
    try:
        import yaml
        with open(os.path.join(path, "metadata.yaml"), encoding="utf-8") as f:
            rd = yaml.safe_load(f)["rosbag2_bagfile_information"]
        dur = round(rd["duration"]["nanoseconds"] / 1e9, 1)
        msgs = rd["message_count"]
        topics = [t["topic_metadata"]["name"]
                  for t in rd.get("topics_with_message_count", [])]
    except Exception:
        pass
    try:
        with open(os.path.join(path, "metadata.json"), encoding="utf-8") as f:
            note = json.load(f).get("note", "")
    except Exception:
        pass
    return dur, msgs, note, topics


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
            dur, msgs, note, topics = bag_brief(p)
            bags.append({"name": n, "mb": mb, "dur": dur, "msgs": msgs,
                         "note": note, "topics": topics})
    return bags, round(total, 1)


class RecorderNode(Node):
    def __init__(self):
        super().__init__("swarm_recorder")
        self.proc = None
        self.bag_dir = None
        self.topics = []
        self.note = ""
        self.play_proc = None     # 回放子进程
        self.play_name = None
        self.exporting = None     # 正在导出的 bag 名
        self.last_export = None   # 最近一次导出结果（网页据此显示下载链接）
        self.rec_topics = []      # 可记录话题（有发布者的），10s 刷新
        self._rec_topics_at = 0.0
        self.create_subscription(String, "/recorder/config", self._config_cb, 10)
        self.status_pub = self.create_publisher(String, "/recorder/status", 10)
        self.series_pub = self.create_publisher(String, "/recorder/series", 10)
        self.create_timer(1.0, self._publish_status)
        os.makedirs(BAG_BASE, exist_ok=True)
        self.get_logger().info(f"话题记录节点就绪，bag 目录: {BAG_BASE}")

    def _refresh_rec_topics(self):
        """ROS 图中有发布者的话题清单（网页"记录"页据此渲染勾选项）。

        不能用纯话题枚举：网页会预订阅 uav_1~6，只订阅不发布的话题录了也是空的。
        """
        out = []
        for name, types in self.get_topic_names_and_types():
            try:
                if self.count_publishers(name) > 0:
                    out.append({"name": name, "type": types[0] if types else ""})
            except Exception:
                pass
        self.rec_topics = sorted(out, key=lambda t: t["name"])
        self._rec_topics_at = time.time()

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
        elif action == "play":
            self._play(str(cfg.get("name") or ""))
        elif action == "play_stop":
            self._play_stop()
        elif action == "export_csv":
            self._export_csv(str(cfg.get("name") or ""))
        elif action == "series":
            self._series(str(cfg.get("name") or ""), str(cfg.get("topic") or ""))

    MAX_SERIES_POINTS = 2000   # 曲线降采样上限，控制 JSON 体积

    def _series(self, name, topic):
        """读取 bag 中指定话题的数值字段时序（降采样），发到 /recorder/series 供网页画图。"""
        if not BAG_NAME_RE.match(name) or not topic.startswith("/"):
            self.get_logger().warn(f"拒绝曲线请求: {name!r} {topic!r}")
            return
        threading.Thread(target=self._series_worker, args=(name, topic), daemon=True).start()

    def _series_worker(self, name, topic):
        try:
            import yaml
            from rosbag2_py import (SequentialReader, StorageFilter,
                                    StorageOptions, ConverterOptions)
            from rclpy.serialization import deserialize_message
            from rosidl_runtime_py.convert import message_to_ordereddict
            from rosidl_runtime_py.utilities import get_message

            path = os.path.join(BAG_BASE, name)
            # 预算采样间隔：metadata.yaml 里有各话题消息数，只转换需要的帧
            count = 0
            try:
                with open(os.path.join(path, "metadata.yaml"), encoding="utf-8") as f:
                    rd = yaml.safe_load(f)["rosbag2_bagfile_information"]
                for t in rd.get("topics_with_message_count", []):
                    if t["topic_metadata"]["name"] == topic:
                        count = t["message_count"]
                        break
            except Exception:
                pass
            stride = max(1, count // self.MAX_SERIES_POINTS + 1)

            reader = SequentialReader()
            reader.open(StorageOptions(uri=path, storage_id="sqlite3"),
                        ConverterOptions("", ""))
            types = {t.name: t.type for t in reader.get_all_topics_and_types()}
            if topic not in types:
                self.series_pub.publish(String(data=json.dumps(
                    {"name": name, "topic": topic, "ok": False, "msg": "bag 中无此话题"})))
                return
            msg_cls = get_message(types[topic])
            # 存储层过滤：只读目标话题（不全量扫描整个 bag，大 bag 提速两个数量级）
            reader.set_filter(StorageFilter(topics=[topic]))
            ts, fields = [], {}
            i = 0
            while reader.has_next():
                _tp, data, t = reader.read_next()
                if i % stride == 0:
                    d = message_to_ordereddict(deserialize_message(data, msg_cls))
                    ts.append(t)
                    for k, v in d.items():
                        if isinstance(v, bool) or not isinstance(v, (int, float)):
                            continue
                        if k in ("timestamp", "timestamp_sample"):
                            continue   # 纳秒时间戳(~1e15)会把 Y 轴撑爆，其他字段全压成直线
                        fields.setdefault(k, []).append(float(v))
                i += 1
            if not ts:
                self.series_pub.publish(String(data=json.dumps(
                    {"name": name, "topic": topic, "ok": False, "msg": "该话题没有数据"})))
                return
            # 结果写文件走 HTTP 下载，rosbridge 只发"就绪"通知：
            # 大 JSON 经 DDS best-effort 订阅会静默丢（UDP 缓冲不足无重传，实测 ~600KB 阈值）
            out = {"rows": i, "t": [t / 1e9 for t in ts], "series": fields}
            series_path = os.path.join(path, "series.json")
            with open(series_path, "w", encoding="utf-8") as f:
                json.dump(out, f)
            self.series_pub.publish(String(data=json.dumps(
                {"name": name, "topic": topic, "ok": True, "url": f"/bags/{name}/series.json",
                 "rows": i, "fields": len(fields)})))
            self.get_logger().info(f"曲线数据: {name} {topic} ({i} 点->{len(ts)} 帧, {len(fields)} 字段)")
        except Exception as e:
            self.get_logger().error(f"曲线读取失败: {e}")
            self.series_pub.publish(String(data=json.dumps(
                {"name": name, "topic": topic, "ok": False, "msg": str(e)})))

    def _play(self, name):
        """ros2 bag play 回放（子进程，SIGINT 可停）。"""
        if not BAG_NAME_RE.match(name):
            self.get_logger().warn(f"拒绝回放非法名称: {name!r}")
            return
        path = os.path.join(BAG_BASE, name)
        if not os.path.isdir(path):
            return
        self._play_stop()
        cmd = f"{SETUP_CMD}exec ros2 bag play '{path}'"
        self.play_proc = subprocess.Popen(["bash", "-c", cmd], preexec_fn=os.setsid)
        self.play_name = name
        self.get_logger().info(f"开始回放 {name}（警告：会往同名话题发历史数据）")

    def _play_stop(self):
        if self.play_proc is None:
            return
        try:
            os.killpg(os.getpgid(self.play_proc.pid), signal.SIGINT)
            self.play_proc.wait(timeout=5)
        except Exception:
            try:
                os.killpg(os.getpgid(self.play_proc.pid), signal.SIGKILL)
            except Exception:
                pass
        self.play_proc = None
        self.play_name = None

    def _export_csv(self, name):
        if not BAG_NAME_RE.match(name):
            self.get_logger().warn(f"拒绝导出非法名称: {name!r}")
            return
        if self.exporting is not None:
            self.get_logger().warn("已有导出任务在进行中")
            return
        threading.Thread(target=self._export_worker, args=(name,), daemon=True).start()

    def _export_worker(self, name):
        """把 bag 里所有话题导出为 CSV（每话题一个文件），打包成 csv_export.zip。

        通用实现：rosidl 反序列化后转 OrderedDict，顶层标量成列、复杂字段转字符串，
        不依赖具体消息类型——视觉等新消息类型自动支持。
        """
        self.exporting = name
        try:
            import csv
            from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
            from rclpy.serialization import deserialize_message
            from rosidl_runtime_py.convert import message_to_ordereddict
            from rosidl_runtime_py.utilities import get_message

            path = os.path.join(BAG_BASE, name)
            outdir = os.path.join(path, "csv")
            shutil.rmtree(outdir, ignore_errors=True)
            os.makedirs(outdir, exist_ok=True)

            reader = SequentialReader()
            reader.open(StorageOptions(uri=path, storage_id="sqlite3"),
                        ConverterOptions("", ""))
            types = {t.name: t.type for t in reader.get_all_topics_and_types()}
            writers, files, headers, rows = {}, {}, {}, {}
            while reader.has_next():
                topic, data, t = reader.read_next()
                d = message_to_ordereddict(deserialize_message(data, get_message(types[topic])))
                flat = {"t_ns": t}
                flat.update({k: (v if isinstance(v, (int, float, str)) else str(v))
                             for k, v in d.items()})
                if topic not in writers:
                    safe = topic.strip("/").replace("/", "_") or "root"
                    f = open(os.path.join(outdir, safe + ".csv"), "w", newline="", encoding="utf-8")
                    files[topic] = f
                    headers[topic] = list(flat.keys())
                    writers[topic] = csv.writer(f)
                    writers[topic].writerow(headers[topic])
                    rows[topic] = 0
                writers[topic].writerow([flat.get(k, "") for k in headers[topic]])
                rows[topic] += 1
            for f in files.values():
                f.close()
            shutil.make_archive(os.path.join(path, "csv_export"), "zip", outdir)
            self.last_export = {"name": name, "ok": True, "zip": f"{name}/csv_export.zip",
                                "topics": len(files), "rows": sum(rows.values())}
            self.get_logger().info(f"导出完成: {name} ({len(files)} 话题, {sum(rows.values())} 行)")
        except Exception as e:
            self.last_export = {"name": name, "ok": False, "msg": str(e)}
            self.get_logger().error(f"导出失败: {name}: {e}")
        finally:
            self.exporting = None

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
        if self.play_proc is not None and self.play_proc.poll() is not None:
            self.get_logger().info("回放结束")
            self.play_proc = None
            self.play_name = None
        bags, total_mb = list_bags()
        if time.time() - self._rec_topics_at > 10:
            self._refresh_rec_topics()
        st = {
            "recording": self.proc is not None,
            "dir": self.bag_dir if (self.proc or self.bag_dir) else None,
            "topics": self.topics if self.proc else [],
            "size_mb": dir_size_mb(self.bag_dir) if self.bag_dir and os.path.isdir(self.bag_dir) else 0,
            "bags": bags,
            "total_mb": total_mb,
            "playing": self.play_proc is not None,
            "playing_name": self.play_name,
            "exporting": self.exporting,
            "last_export": self.last_export,
            "rec_topics": self.rec_topics,
        }
        self.status_pub.publish(String(data=json.dumps(st)))

    def destroy_node(self):
        self._stop()
        self._play_stop()
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
