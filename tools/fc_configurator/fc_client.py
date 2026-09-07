# SPDX-License-Identifier: GPL-3.0-only
# Copyright (C) 2026 零创无穷（成都）科技有限责任公司
# 本文件是 SwarmCore-Sim 的一部分，
# 依据 GNU GPL v3 发布（协议全文见仓库根目录 LICENSE）。
# 本软件按"现状"提供，不附带任何明示或默示担保。

"""飞控会话层：连接发现、参数协议、校准流程、电机测试

线程模型：一个接收泵线程（recv → 解帧 → 进 inbox + 状态更新），
操作（拉参数/写参数/校准/电机测试）在工作线程里同步执行，
对外通过 op_status dict 暴露进度，HTTP 层轮询。

同一时刻只允许一个长操作（op_lock）。同一时刻只连一架飞机（server 层保证）。
"""

import struct
import threading
import time

import mavlink as mv

GCS_SYSID = 255
GCS_COMPID = 190  # MAV_COMP_ID_MISSIONPLANNER 段，惯例地面站组件 id

# 校准命令参数映射: kind -> COMMAND_LONG 241 的 7 个参数
CALIB_PARAMS = {
    "gyro":  (1, 0, 0, 0, 0, 0, 0),
    "mag":   (0, 1, 0, 0, 0, 0, 0),
    "baro":  (0, 0, 1, 0, 0, 0, 0),
    "accel": (0, 0, 0, 0, 1, 0, 0),   # 六面校准
    "level": (0, 0, 0, 0, 2, 0, 0),   # 水平校准
}


class FCError(Exception):
    pass


class FCClient:
    def __init__(self, transport):
        self.t = transport
        self.sysid = None          # 飞控 sysid（心跳发现）
        self.compid = 1            # 自动驾驶仪组件 id
        self.armed = False
        self.last_heartbeat = 0.0
        self.info = {}             # 机型/固件等
        self.inbox = []            # [(t, msgid, parsed, sysid, compid)]
        self._lock = threading.Lock()
        self._op_lock = threading.Lock()
        self._seq = 0
        self._closed = False
        self.op_status = {"op": None, "progress": 0, "total": 0, "msg": ""}
        self.statustext_log = []   # [(t, severity, text)] 最近 200 条
        self._rx = threading.Thread(target=self._pump, daemon=True)
        self._hb = threading.Thread(target=self._heartbeat_loop, daemon=True)
        self._rx.start()
        self._hb.start()

    # ---------- 底层 ----------

    def close(self):
        self._closed = True
        self.t.close()

    def _next_seq(self):
        self._seq = (self._seq + 1) & 0xFF
        return self._seq

    def _send(self, frame: bytes):
        self.t.send(frame)

    def _pump(self):
        parser = mv.Parser()
        while not self._closed:
            try:
                data = self.t.recv(0.2)
            except Exception:
                break
            if not data:
                continue
            for msgid, payload, sysid, compid in parser.feed(data):
                parsed = mv.parse(msgid, payload)
                if parsed is None:
                    continue
                now = time.time()
                with self._lock:
                    self.inbox.append((now, msgid, parsed, sysid, compid))
                    if len(self.inbox) > 2000:
                        del self.inbox[:1000]
                if msgid == mv.MSG_HEARTBEAT and parsed["autopilot"] == 12:  # MAV_AUTOPILOT_PX4
                    # 只认 PX4（autopilot=12；GCS 自己是 8=INVALID，ArduPilot 是 3）
                    if self.sysid is None:
                        self.sysid, self.compid = sysid, compid
                    if sysid == self.sysid:
                        self.armed = parsed["armed"]
                        self.last_heartbeat = now
                elif msgid == mv.MSG_STATUSTEXT:
                    self.statustext_log.append((now, parsed["severity"], parsed["text"]))
                    del self.statustext_log[:-200]
                elif msgid == mv.MSG_AUTOPILOT_VERSION and sysid == self.sysid:
                    self.info.update(parsed)

    def _heartbeat_loop(self):
        while not self._closed:
            try:
                self._send(mv.pack_heartbeat_gcs(GCS_SYSID, GCS_COMPID, self._next_seq()))
            except Exception:
                break
            time.sleep(1.0)

    def _scan_inbox(self, since, msgid, pred=None):
        """取 since 之后匹配的收件箱消息（含历史，便于先收后等）。"""
        with self._lock:
            items = [x for x in self.inbox if x[0] >= since and x[1] == msgid]
        for t, _, parsed, sysid, compid in items:
            if sysid == self.sysid and (pred is None or pred(parsed)):
                return parsed
        return None

    # ---------- 连接 ----------

    def wait_connect(self, timeout=8.0):
        """等飞控心跳，并拉 AUTOPILOT_VERSION。返回信息 dict。"""
        deadline = time.time() + timeout
        while time.time() < deadline:
            if self.sysid is not None:
                break
            time.sleep(0.1)
        if self.sysid is None:
            raise FCError(f"未发现飞控心跳（{timeout}s 超时）。检查通道、接线、以及 QGC 是否占用了链路")
        # 请求 AUTOPILOT_VERSION（尽力而为，不阻塞主流程）
        try:
            since = time.time()
            self._send(mv.pack_command_long(
                self.sysid, self.compid, mv.MAV_CMD_REQUEST_MESSAGE,
                (mv.MSG_AUTOPILOT_VERSION, 0, 0, 0, 0, 0, 0),
                sysid=GCS_SYSID, compid=GCS_COMPID, seq=self._next_seq()))
            dl = time.time() + 3
            while time.time() < dl:
                v = self._scan_inbox(since, mv.MSG_AUTOPILOT_VERSION)
                if v:
                    self.info.update(v)
                    break
                time.sleep(0.1)
        except Exception:
            pass
        return {
            "sysid": self.sysid, "compid": self.compid,
            "channel": self.t.describe(),
            "firmware": self.info.get("flight_sw_version_str", "未知"),
            "vendor_id": self.info.get("vendor_id"),
            "product_id": self.info.get("product_id"),
        }

    @property
    def connected(self):
        return self.sysid is not None and (time.time() - self.last_heartbeat) < 5

    # ---------- 参数 ----------

    def fetch_all_params(self, timeout=30.0):
        """全量拉取参数：REQUEST_LIST → 按 index 补缺重传（QGC 同款机制）。

        返回 {name: {"value":v, "type":t, "index":i}}。进度写 op_status。
        """
        with self._op_lock:
            self._require_conn()
            self.op_status = {"op": "params", "progress": 0, "total": 0, "msg": "请求参数表…"}
            params = {}       # index -> dict
            since = time.time()
            deadline = since + timeout
            self._send(mv.pack_param_request_list(
                self.sysid, self.compid, sysid=GCS_SYSID, compid=GCS_COMPID, seq=self._next_seq()))
            last_rx = time.time()
            retries = 0
            while time.time() < deadline:
                pv = self._scan_new_param(since, params)
                if pv is not None:
                    params[pv["index"]] = pv
                    total = pv["count"]
                    self.op_status.update(progress=len(params), total=total,
                                          msg=f"已收 {len(params)}/{total}")
                    last_rx = time.time()
                    continue
                total = self.op_status["total"]
                if total and len(params) >= total:
                    break  # 收齐
                if time.time() - last_rx > 1.5:
                    # 有缺口：按 index 精确重传；表长未知则重发 LIST
                    missing = [i for i in range(total) if i not in params] if total else []
                    if missing:
                        for i in missing[:50]:
                            self._send(mv.pack_param_request_read(
                                self.sysid, self.compid, b"", i,
                                sysid=GCS_SYSID, compid=GCS_COMPID, seq=self._next_seq()))
                    else:
                        self._send(mv.pack_param_request_list(
                            self.sysid, self.compid, sysid=GCS_SYSID, compid=GCS_COMPID,
                            seq=self._next_seq()))
                    retries += 1
                    last_rx = time.time()
                    if retries > 8:
                        raise FCError(f"参数拉取不完整（缺 {total - len(params)} 个），链路质量差")
                time.sleep(0.02)
            total = self.op_status["total"]
            if not total or len(params) < total:
                raise FCError("参数拉取超时/不完整")
            out = {p["name"]: {"value": p["value"], "type": p["type"], "index": i}
                   for i, p in sorted(params.items()) if p["name"]}
            self.op_status = {"op": None, "progress": 0, "total": 0, "msg": ""}
            return out

    def _scan_new_param(self, since, known):
        """从收件箱取一个还没收过的 PARAM_VALUE。"""
        with self._lock:
            items = [x for x in self.inbox if x[0] >= since and x[1] == mv.MSG_PARAM_VALUE]
        for _, _, pv, sysid, _ in items:
            if sysid == self.sysid and pv["index"] not in known:
                return pv
        return None

    def set_param(self, name, value, ptype=None, timeout=5.0):
        """写参数并等 PARAM_VALUE 回显校验。"""
        self._require_conn()
        if ptype is None:
            ptype = mv.PARAM_TYPE_REAL32
        since = time.time()
        self._send(mv.pack_param_set(
            self.sysid, self.compid, name, value, ptype,
            sysid=GCS_SYSID, compid=GCS_COMPID, seq=self._next_seq()))
        deadline = since + timeout
        while time.time() < deadline:
            pv = self._scan_inbox(since, mv.MSG_PARAM_VALUE, lambda d: d["name"] == name)
            if pv:
                if pv["type"] != mv.PARAM_TYPE_REAL32:
                    return int(round(pv["value"])) == int(round(float(value)))
                return abs(pv["value"] - float(value)) < 1e-6
            time.sleep(0.05)
        raise FCError(f"写参数 {name} 无回显（超时）")

    # ---------- 通用命令 ----------

    def send_command(self, command, params=(), timeout=6.0, wait_final=True):
        """发 COMMAND_LONG 并等 ACK。返回 ack dict。

        wait_final=True 时把 IN_PROGRESS 当作中间态继续等最终结果（校准用）。
        """
        self._require_conn()
        since = time.time()
        self._send(mv.pack_command_long(
            self.sysid, self.compid, command, params,
            sysid=GCS_SYSID, compid=GCS_COMPID, seq=self._next_seq()))
        deadline = since + timeout
        last_ack = None
        while time.time() < deadline:
            ack = self._scan_inbox(since, mv.MSG_COMMAND_ACK,
                                   lambda d: d["command"] == command)
            if ack:
                last_ack = ack
                if not (wait_final and ack["result"] == 5):  # 5=IN_PROGRESS
                    return ack
            time.sleep(0.05)
        if last_ack:
            return last_ack
        raise FCError(f"命令 {command} 无应答（超时）")

    # ---------- 校准 ----------

    def calibrate(self, kind, timeout=180.0):
        """校准流程状态机（阻塞，工作线程里跑）。

        进度来源：STATUSTEXT 提示文本 + COMMAND_ACK(progress) + 最终 ACK。
        结果写 op_status 并返回 (ok, msg)。
        """
        if kind not in CALIB_PARAMS:
            raise FCError(f"未知校准类型 {kind}")
        with self._op_lock:
            self._require_conn()
            names = {"gyro": "陀螺仪", "mag": "磁罗盘", "baro": "气压计",
                     "accel": "加速度计（六面）", "level": "水平"}
            self.op_status = {"op": f"calib_{kind}", "progress": 0, "total": 100,
                              "msg": f"{names[kind]}校准：启动…"}
            since = time.time()
            try:
                ack = self.send_command(mv.MAV_CMD_PREFLIGHT_CALIBRATION,
                                        CALIB_PARAMS[kind], timeout=timeout)
            except FCError as e:
                self.op_status.update(op=None, msg=f"校准失败：{e}")
                return False, str(e)
            ok = ack["result"] == 0  # ACCEPTED
            msg = "校准完成" if ok else f"校准失败：{ack['result_name']}"
            self.op_status.update(op=None, progress=100 if ok else 0, msg=msg)
            return ok, msg

    def poll_calib_events(self, since):
        """取 since 之后的 STATUSTEXT/ACK，供校准期间前端轮询展示。"""
        with self._lock:
            sts = [(t, p["severity"], p["text"]) for t, mid, p, s, c in self.inbox
                   if t >= since and mid == mv.MSG_STATUSTEXT and s == self.sysid]
            acks = [p for t, mid, p, s, c in self.inbox
                    if t >= since and mid == mv.MSG_COMMAND_ACK
                    and s == self.sysid and p["command"] == mv.MAV_CMD_PREFLIGHT_CALIBRATION]
        prog = max([a.get("progress", 0) for a in acks], default=None)
        return {"messages": sts, "progress": prog}

    # ---------- 电机测试 / 重启 ----------

    def motor_test(self, motor, value, timeout_s=2.0):
        """MAV_CMD_ACTUATOR_TEST(310): p1=输出值(-1..1) p2=超时(s) p5=输出函数(Motor N = N)。"""
        if self.armed:
            raise FCError("飞机处于解锁状态，禁止电机测试")
        if not (0.0 <= value <= 1.0) or not (1 <= motor <= 12):
            raise FCError("参数越界")
        ack = self.send_command(mv.MAV_CMD_ACTUATOR_TEST,
                                (value, timeout_s, 0, 0, mv.OUTPUT_FUNC_MOTOR_BASE + motor, 0, 0))
        if ack["result"] != 0:
            raise FCError(f"电机测试被拒绝：{ack['result_name']}（检查是否已解锁/安全开关）")
        return ack

    # ---------- MAVLink FTP（SD 卡文件管理） ----------

    FTP_WINDOW = 10        # 窗口化读文件的并发请求数
    FTP_REQ_TIMEOUT = 1.0  # 单个 FTP 请求超时（秒）
    FTP_RETRIES = 4

    def _ftp_seq(self):
        self._ftp_seq_n = (getattr(self, "_ftp_seq_n", 0) + 1) & 0xFFFF
        return self._ftp_seq_n

    def _ftp_request(self, opcode, session=0, offset=0, data=b"", size=None, timeout=None):
        """发一个 FTP 请求并等匹配应答（req_opcode 匹配，目标是我们）。

        返回应答 dict；Nak 抛 FCError（EOF 不抛，置 _eof 由调用方判断）。
        """
        self._require_conn()
        timeout = timeout or self.FTP_REQ_TIMEOUT
        for attempt in range(self.FTP_RETRIES):
            since = time.time()
            self._send(mv.pack_ftp(self.sysid, self.compid, self._ftp_seq(), session, opcode,
                                   offset=offset, data=data, size=size,
                                   sysid=GCS_SYSID, compid=GCS_COMPID,
                                   msg_seq=self._next_seq()))
            deadline = since + timeout
            while time.time() < deadline:
                # FTP 应答的 seq 是飞控自己的计数（不回显我们的），
                # 匹配靠 req_opcode + 目标是本机（串行使用下足够）
                resp = self._scan_inbox(
                    since, mv.MSG_FILE_TRANSFER_PROTOCOL,
                    lambda d: d["req_opcode"] == opcode
                              and d["target_system"] == GCS_SYSID
                              and not d.pop("_consumed", False))
                if resp:
                    resp["_consumed"] = True
                    if resp["opcode"] == mv.FTP_NAK:
                        err = resp["data"][0] if resp["data"] else 1
                        if err == 6:  # EOF：正常结束信号
                            resp["_eof"] = True
                            return resp
                        detail = f"，errno={resp['data'][1]}" if err == 2 and len(resp["data"]) > 1 else ""
                        raise FCError(f"FTP 操作被拒绝：{mv.FTP_ERR.get(err, err)}{detail}")
                    return resp
                time.sleep(0.01)
        raise FCError(f"FTP 操作 {opcode} 无应答（重试 {self.FTP_RETRIES} 次均超时）")

    def ftp_listdir(self, path):
        """列目录，返回 [{name, is_dir}]。条目格式: 'F<name>\\0' / 'D<name>\\0' / 'S' 跳过。"""
        entries = []
        offset = 0
        while True:
            resp = self._ftp_request(mv.FTP_OP["ListDirectory"], offset=offset,
                                     data=path.encode())
            if resp.get("_eof"):
                break
            chunk = resp["data"]
            n = 0
            for ent in chunk.split(b"\0"):
                if not ent:
                    continue
                n += 1  # 分页 offset 按全部条目计（含 'S' 跳过项），否则目录会重复
                kind, name = chr(ent[0]), ent[1:].decode("utf-8", "replace")
                if kind == "S" or not name:
                    continue
                # PX4 扩展：文件名后跟 \t<字节数>
                size = None
                if "\t" in name:
                    name, sz = name.rsplit("\t", 1)
                    try:
                        size = int(sz)
                    except ValueError:
                        size = None
                entries.append({"name": name, "is_dir": kind == "D", "size": size})
            if not n:
                break  # 空块防死循环
            offset += n
        return sorted(entries, key=lambda e: (not e["is_dir"], e["name"]))

    def ftp_open_ro(self, path):
        """OpenFileRO，返回 (session, size)。"""
        resp = self._ftp_request(mv.FTP_OP["OpenFileRO"], data=path.encode())
        size = struct.unpack("<I", resp["data"][:4].ljust(4, b"\0"))[0]
        return resp["session"], size

    def ftp_download(self, path, op_name="download"):
        """停等式逐块下载（ReadFile, size=239）。

        为什么不用 BurstReadFile：PX4 v1.15 的突发流在 SITL 上实测会死循环
        重发（35KB 分块续传语义诡异），而停等 ARQ（应答 seq=请求 seq+1、
        重复请求触发重发）正是 PX4 FTP 的原生设计，921600 链路上约 100+KB/s，
        几 MB 的日志完全够用。
        """
        with self._op_lock:
            self._require_conn()
            session, size = self.ftp_open_ro(path)
            self.op_status = {"op": op_name, "progress": 0, "total": size,
                              "msg": f"下载 {path}（{size} 字节）"}
            if not size:
                self._safe_terminate(session)
                self.op_status = {"op": None, "progress": 0, "total": 0, "msg": "空文件"}
                return b""
            buf = bytearray()
            try:
                while len(buf) < size:
                    resp = self._ftp_request(mv.FTP_OP["ReadFile"], session=session,
                                             offset=len(buf), size=239)
                    if resp.get("_eof"):
                        break
                    if resp["offset"] != len(buf):
                        raise FCError(f"下载错位：期望 offset {len(buf)}，收到 {resp['offset']}")
                    if not resp["data"]:
                        break
                    buf += resp["data"]
                    self.op_status["progress"] = len(buf)
                    if len(buf) % (239 * 100) < 239:
                        self.op_status["msg"] = f"下载 {path}（{len(buf)//1024}/{size//1024} KB）"
            finally:
                self._safe_terminate(session)
            self.op_status = {"op": None, "progress": 0, "total": 0,
                              "msg": f"下载完成：{path}（{len(buf)} 字节）"}
            return bytes(buf)

    def _safe_terminate(self, session):
        try:
            self._ftp_request(mv.FTP_OP["TerminateSession"], session=session)
        except FCError:
            pass

    def ftp_upload(self, path, data: bytes, op_name="upload"):
        """上传文件（CreateFile → 分块 WriteFile → Terminate）。"""
        with self._op_lock:
            self._require_conn()
            resp = self._ftp_request(mv.FTP_OP["CreateFile"], data=path.encode())
            session = resp["session"]
            total = len(data)
            self.op_status = {"op": op_name, "progress": 0, "total": total,
                              "msg": f"上传 {path}（{total} 字节）"}
            try:
                offset = 0
                while offset < total:
                    chunk = data[offset:offset + 239]
                    self._ftp_request(mv.FTP_OP["WriteFile"], session=session,
                                      offset=offset, data=chunk)
                    offset += len(chunk)
                    self.op_status["progress"] = offset
            finally:
                try:
                    self._ftp_request(mv.FTP_OP["TerminateSession"], session=session)
                except FCError:
                    pass
            self.op_status = {"op": None, "progress": 0, "total": 0,
                              "msg": f"上传完成：{path}"}

    def ftp_remove(self, path):
        self._ftp_request(mv.FTP_OP["RemoveFile"], data=path.encode())

    def ftp_mkdir(self, path):
        resp = self._ftp_request(mv.FTP_OP["CreateDirectory"], data=path.encode())
        if resp.get("_eof"):
            return
        # CreateDirectory 的 Nak 已在 _ftp_request 里抛错，Ack 即成功

    def reboot(self, to_bootloader=False):
        """MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN(246): p1=1 重启, p1=3 进 bootloader。"""
        ack = self.send_command(mv.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
                                (3 if to_bootloader else 1, 0, 0, 0, 0, 0, 0))
        if ack["result"] != 0:
            raise FCError(f"重启被拒绝：{ack['result_name']}")
        return ack

    def _require_conn(self):
        if not self.connected:
            raise FCError("飞控连接已断开（心跳超时），请重新连接")
