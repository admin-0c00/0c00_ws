# SPDX-License-Identifier: GPL-3.0-only
# Copyright (C) 2026 零创无穷（成都）科技有限责任公司
# 本文件是 SwarmCore-Sim 的一部分，
# 依据 GNU GPL v3 发布（协议全文见仓库根目录 LICENSE）。
# 本软件按"现状"提供，不附带任何明示或默示担保。

"""飞控配置器 HTTP 服务（纯标准库）

用法: python3 server.py [--port 8082] [--bind 0.0.0.0]
浏览器打开 http://localhost:8082

API（统一响应 {ok:true,...} / {ok:false,error}）:
  GET  /api/ports                     串口列表 + 本机网卡（连接页用）
  POST /api/connect    {channel:"udp",host,port[,bind_port]}
                       {channel:"serial",dev,baud}   连接并发现飞控
  POST /api/disconnect                断开
  GET  /api/status                    连接状态 + 当前长操作进度 + 最近 STATUSTEXT
  POST /api/params/refresh            全量拉取参数（后台跑，status 看进度）
  GET  /api/params                    {params:{name:{value,type}}, meta:{...}}
  POST /api/params/set  {name, value} 写参数并回读校验
  GET  /api/params/export             下载参数 JSON
  POST /api/calibrate   {what}        gyro/mag/baro/accel/level（后台跑）
  POST /api/motor_test  {motor, value, timeout_s}
  POST /api/reboot      {bootloader}  重启/进 bootloader
  POST /api/fs/list     {path}        MAVLink FTP 列目录
  POST /api/fs/download {path}        后台下载到缓存，status 看进度与 fs_dl.key
  GET  /api/fs/get?key=N              取回已下载的文件（attachment）
  POST /api/fs/upload   raw body + X-Path 头   后台上传
  POST /api/fs/delete   {path}        删除机载文件

单会话锁：同一时刻只连一架飞机；再 connect 会先断开旧连接。
"""

import argparse
import json
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import fc_client
import mavlink as mv
import param_meta
import px4_upload
import transport

WEB_DIR = Path(__file__).parent / "web"

# ---- 全局会话状态（单飞控） ----
_lock = threading.Lock()
_client: fc_client.FCClient = None
_params = {}          # 最近一次全量拉取的参数 {name: {value,type,index}}
_meta = param_meta.load()
_conn_info = {}
_flash = {"running": False, "pct": 0, "msg": ""}   # 烧录进度
_fs_dl = {"key": None, "name": "", "error": ""}    # 最近一次后台下载结果
DL_CACHE = Path("/tmp/fc_dl_cache")


def _get_client():
    with _lock:
        return _client


def _set_client(c):
    global _client, _conn_info, _params
    with _lock:
        old, _client = _client, c
        if c is None:
            _conn_info, _params = {}, {}
    if old:
        old.close()


class Handler(BaseHTTPRequestHandler):
    def log_message(self, fmt, *args):
        pass  # 静默访问日志

    # ---- 基础工具 ----
    def _json(self, obj, code=200):
        body = json.dumps(obj, ensure_ascii=False).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _static(self, path, ctype):
        try:
            body = path.read_bytes()
        except OSError:
            self._json({"ok": False, "error": "not found"}, 404)
            return
        self.send_response(200)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    @staticmethod
    def _require(c):
        if c is None:
            raise fc_client.FCError("未连接飞控")
        return c

    # ---- GET ----
    def do_GET(self):
        if self.path in ("/", "/index.html"):
            self._static(WEB_DIR / "index.html", "text/html; charset=utf-8")
        elif self.path == "/favicon.ico":
            self._static(WEB_DIR / "favicon.ico", "image/x-icon")
        elif self.path == "/favicon_32.png":
            self._static(WEB_DIR / "favicon_32.png", "image/png")
        elif self.path == "/logo.png":
            self._static(WEB_DIR / "logo.png", "image/png")
        elif self.path == "/api/ports":
            self._json({"ok": True,
                        "serials": transport.list_serial_ports(),
                        "nics": transport.list_udp_candidates()})
        elif self.path == "/api/status":
            c = _get_client()
            st = {"ok": True, "connected": bool(c and c.connected),
                  "info": _conn_info if c else {}, "flash": dict(_flash),
                  "fs_dl": dict(_fs_dl)}
            if c:
                st["op"] = dict(c.op_status)
                st["armed"] = c.armed
                st["messages"] = [
                    {"t": t, "severity": s, "text": x}
                    for t, s, x in c.statustext_log[-20:]]
            self._json(st)
        elif self.path == "/api/params":
            with _lock:
                self._json({"ok": True, "params": _params, "meta": _meta,
                            "meta_count": len(_meta)})
        elif self.path == "/api/params/export":
            with _lock:
                out = {"_type": "fc_params", "_version": 1,
                       "_exported_at": time.strftime("%Y-%m-%dT%H:%M:%S"),
                       "_firmware": _conn_info.get("firmware"),
                       "params": {n: p["value"] for n, p in _params.items()}}
            body = json.dumps(out, ensure_ascii=False, indent=1).encode()
            self.send_response(200)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Disposition",
                             'attachment; filename="fc_params.json"')
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        elif self.path.startswith("/api/fs/get?"):
            from urllib.parse import parse_qs, urlparse
            q = parse_qs(urlparse(self.path).query)
            self._fs_get(q.get("key", [""])[0])
        else:
            self._json({"ok": False, "error": "not found"}, 404)

    def _fs_get(self, key):
        """把后台下载缓存的文件以 attachment 形式发给浏览器。"""
        try:
            f = DL_CACHE / f"{int(key)}"
            body = f.read_bytes()
            name = _fs_dl["name"] if str(_fs_dl["key"]) == key else "download.bin"
        except (OSError, ValueError):
            self._json({"ok": False, "error": "文件不存在或已过期"}, 404)
            return
        from urllib.parse import quote
        self.send_response(200)
        self.send_header("Content-Type", "application/octet-stream")
        self.send_header("Content-Disposition",
                         f"attachment; filename*=UTF-8''{quote(name)}")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)
    # ---- POST ----
    def do_POST(self):
        # 固件烧录走原始字节流（.px4 是 JSON+base64，直接整个上传）
        if self.path == "/api/firmware/flash":
            length = int(self.headers.get("Content-Length", 0))
            self._flash(self.rfile.read(length),
                        self.headers.get("X-Flash-Dev", ""),
                        self.headers.get("X-Flash-Force") == "1")
            return
        # 机载文件上传也是原始字节流，目标路径放 X-Path 头
        if self.path == "/api/fs/upload":
            length = int(self.headers.get("Content-Length", 0))
            self._fs_upload(self.rfile.read(length),
                            self.headers.get("X-Path", ""))
            return
        try:
            length = int(self.headers.get("Content-Length", 0))
            req = json.loads(self.rfile.read(length) or b"{}")
        except (ValueError, json.JSONDecodeError):
            self._json({"ok": False, "error": "非法 JSON 请求"}, 400)
            return

        try:
            if self.path == "/api/connect":
                self._connect(req)
            elif self.path == "/api/disconnect":
                _set_client(None)
                self._json({"ok": True, "msg": "已断开"})
            elif self.path == "/api/params/refresh":
                self._params_refresh()
            elif self.path == "/api/params/set":
                self._param_set(req)
            elif self.path == "/api/calibrate":
                self._calibrate(req)
            elif self.path == "/api/motor_test":
                self._motor_test(req)
            elif self.path == "/api/reboot":
                self._reboot(req)
            elif self.path == "/api/fs/list":
                self._fs_list(req)
            elif self.path == "/api/fs/download":
                self._fs_download(req)
            elif self.path == "/api/fs/delete":
                self._fs_delete(req)
            else:
                self._json({"ok": False, "error": "not found"}, 404)
        except fc_client.FCError as e:
            self._json({"ok": False, "error": str(e)})
        except transport.TransportError as e:
            self._json({"ok": False, "error": str(e)})
        except (KeyError, ValueError) as e:
            self._json({"ok": False, "error": f"参数错误: {e}"}, 400)

    # ---- 各 API 实现 ----

    def _connect(self, req):
        global _conn_info
        ch = req["channel"]
        if ch == "udp":
            t = transport.UDPTransport(req["host"], int(req["port"]),
                                       int(req.get("bind_port", 0)))
        elif ch == "serial":
            t = transport.SerialTransport(req["dev"], int(req.get("baud", 115200)))
        else:
            raise ValueError(f"未知通道 {ch}")
        c = fc_client.FCClient(t)
        try:
            info = c.wait_connect(timeout=float(req.get("timeout", 8)))
        except Exception:
            c.close()
            raise
        _set_client(c)  # 会顺带断开旧连接
        _conn_info = info
        self._json({"ok": True, "info": info})

    def _params_refresh(self):
        c = self._require(_get_client())
        if c.op_status.get("op"):
            raise fc_client.FCError(f"正在执行 {c.op_status['op']}，等它完成")

        def work():
            global _params
            try:
                result = c.fetch_all_params()
                with _lock:
                    _params = result
            except Exception as e:
                c.op_status = {"op": None, "progress": 0, "total": 0,
                               "msg": f"拉取失败：{e}"}

        threading.Thread(target=work, daemon=True).start()
        self._json({"ok": True, "msg": "参数拉取已开始，轮询 /api/status 看进度"})

    def _param_set(self, req):
        c = self._require(_get_client())
        name = req["name"]
        value = req["value"]
        with _lock:
            cur = _params.get(name)
        ptype = cur["type"] if cur else None
        ok = c.set_param(name, value, ptype)
        if not ok:
            raise fc_client.FCError(f"写参数 {name} 后回读校验不一致")
        with _lock:
            if name in _params:
                _params[name]["value"] = value
        reboot = bool(_meta.get(name, {}).get("reboot_required"))
        self._json({"ok": True, "msg": f"{name} 已写入" + ("（需重启生效）" if reboot else "")})

    def _calibrate(self, req):
        c = self._require(_get_client())
        what = req["what"]
        if what not in fc_client.CALIB_PARAMS:
            raise ValueError(f"未知校准类型 {what}（支持 {sorted(fc_client.CALIB_PARAMS)}）")
        if c.op_status.get("op"):
            raise fc_client.FCError(f"正在执行 {c.op_status['op']}，等它完成")

        def work():
            c.calibrate(what)

        threading.Thread(target=work, daemon=True).start()
        self._json({"ok": True, "msg": f"校准 {what} 已开始"})

    def _motor_test(self, req):
        c = self._require(_get_client())
        c.motor_test(int(req["motor"]), float(req["value"]),
                     float(req.get("timeout_s", 2.0)))
        self._json({"ok": True, "msg": f"电机 {req['motor']} 测试指令已执行"})

    def _reboot(self, req):
        c = self._require(_get_client())
        c.reboot(to_bootloader=bool(req.get("bootloader")))
        _set_client(None)
        self._json({"ok": True, "msg": "重启指令已执行，连接已断开"})

    def _fs_list(self, req):
        c = self._require(_get_client())
        path = req["path"]
        entries = c.ftp_listdir(path)
        self._json({"ok": True, "path": path, "entries": entries})

    def _fs_download(self, req):
        """后台下载机载文件到 DL_CACHE，完成后 fs_dl.key 可取回。"""
        global _fs_dl
        c = self._require(_get_client())
        path = req["path"]
        if c.op_status.get("op"):
            raise fc_client.FCError(f"正在执行 {c.op_status['op']}，等它完成")
        DL_CACHE.mkdir(exist_ok=True)
        key = int(time.time() * 1000)
        _fs_dl = {"key": None, "name": Path(path).name, "error": ""}

        def work():
            global _fs_dl
            try:
                data = c.ftp_download(path)
                (DL_CACHE / str(key)).write_bytes(data)
                _fs_dl = {"key": key, "name": Path(path).name, "error": ""}
            except Exception as e:
                _fs_dl = {"key": None, "name": Path(path).name,
                          "error": str(e)}

        threading.Thread(target=work, daemon=True).start()
        self._json({"ok": True, "msg": f"开始下载 {path}，轮询 /api/status 看进度"})

    def _fs_upload(self, body, path):
        try:
            c = self._require(_get_client())
            if not path:
                raise ValueError("缺少目标路径（X-Path）")
            if c.op_status.get("op"):
                raise fc_client.FCError(
                    f"正在执行 {c.op_status['op']}，等它完成")
        except fc_client.FCError as e:
            self._json({"ok": False, "error": str(e)})
            return
        except ValueError as e:
            self._json({"ok": False, "error": str(e)}, 400)
            return

        def work():
            c.ftp_upload(path, body)

        threading.Thread(target=work, daemon=True).start()
        self._json({"ok": True,
                    "msg": f"开始上传 {len(body)} 字节到 {path}"})

    def _fs_delete(self, req):
        c = self._require(_get_client())
        c.ftp_remove(req["path"])
        self._json({"ok": True, "msg": f"已删除 {req['path']}"})

    def _flash(self, body, dev, force):
        """烧录 .px4（raw body）。先断 MAVLink 会话，再在后台线程烧录。"""
        global _flash
        try:
            if _flash["running"]:
                raise fc_client.FCError("已有烧录任务在进行")
            if not dev:
                raise ValueError("缺少串口设备（X-Flash-Dev）")
            fw = px4_upload.load_px4_file(body)  # 先解析，坏文件直接报错
        except Exception as e:  # 固件解析失败（JSON/zlib/字段缺失）统一报错
            self._json({"ok": False, "error": f"固件文件无效: {e}"}, 400)
            return

        # 烧录占用串口，必须断开当前会话
        _set_client(None)
        _flash = {"running": True, "pct": 0, "msg": "开始烧录…"}

        def work():
            global _flash
            def prog(pct, msg):
                _flash.update(pct=pct, msg=msg)
            try:
                px4_upload.Uploader(dev, prog).flash(fw, force=force)
            except Exception as e:
                _flash.update(msg=f"烧录失败：{e}")
            finally:
                _flash["running"] = False

        threading.Thread(target=work, daemon=True).start()
        self._json({"ok": True, "msg": "烧录已开始，轮询 /api/status 的 flash 字段看进度"})


def main():
    ap = argparse.ArgumentParser(description="飞控配置器")
    ap.add_argument("--port", type=int, default=8082)
    ap.add_argument("--bind", default="0.0.0.0")
    args = ap.parse_args()
    srv = ThreadingHTTPServer((args.bind, args.port), Handler)
    print(f"[飞控配置] 配置器已启动: http://localhost:{args.port}")
    print(f"[飞控配置] 参数元数据: {len(_meta)} 条"
          + ("" if _meta else "（未找到 parameters.xml，参数页无描述信息）"))
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
