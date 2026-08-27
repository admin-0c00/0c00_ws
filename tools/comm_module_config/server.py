# SPDX-License-Identifier: GPL-3.0-only
# Copyright (C) 2026 零创无穷（成都）科技有限责任公司
# 本文件是 SwarmCore-Sim 的一部分，
# 依据 GNU GPL v3 发布（协议全文见仓库根目录 LICENSE）。
# 本软件按"现状"提供，不附带任何明示或默示担保。

"""通信模块配置工具 HTTP 服务（纯标准库）

用法: python3 server.py [--port 8081] [--bind 0.0.0.0]
浏览器打开 http://localhost:8081

API:
  GET  /api/nics                       网卡列表 [{name, ip, mac}]
  POST /api/search        {nic}        搜索模块 -> {devices: [...]}
  POST /api/get_config    {nic, mac}   读配置 -> {config: {...}}
  POST /api/set_config    {nic, mac, config}  写配置（先读后写，reserved 保留）
  POST /api/factory_reset {nic, mac}   恢复出厂
"""

import argparse
import json
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import ch9121

WEB_DIR = Path(__file__).parent / "web"


def list_nics():
    """枚举有 IPv4 地址的网卡（跳过 lo）。"""
    nics = []
    for name in sorted(p.name for p in Path("/sys/class/net").iterdir()):
        if name == "lo":
            continue
        try:
            ip, _ = ch9121.nic_addr_bcast(name)
            mac = ch9121.mac_bytes2str(ch9121.nic_mac(name))
            nics.append({"name": name, "ip": ip, "mac": mac})
        except OSError:
            continue  # 无 IPv4 地址的网卡（down 或未分配）
    return nics


class Handler(BaseHTTPRequestHandler):
    def log_message(self, fmt, *args):
        pass  # 静默访问日志

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

    def do_GET(self):
        if self.path in ("/", "/index.html"):
            self._static(WEB_DIR / "index.html", "text/html; charset=utf-8")
        elif self.path == "/favicon.ico":
            self._static(WEB_DIR / "favicon.ico", "image/x-icon")
        elif self.path == "/favicon_32.png":
            self._static(WEB_DIR / "favicon_32.png", "image/png")
        elif self.path == "/api/nics":
            self._json({"ok": True, "nics": list_nics()})
        else:
            self._json({"ok": False, "error": "not found"}, 404)

    def do_POST(self):
        try:
            length = int(self.headers.get("Content-Length", 0))
            req = json.loads(self.rfile.read(length) or b"{}")
        except (ValueError, json.JSONDecodeError):
            self._json({"ok": False, "error": "非法 JSON 请求"}, 400)
            return

        try:
            if self.path == "/api/search":
                devs = ch9121.search(req["nic"], ch9121.nic_mac(req["nic"]))
                self._json({"ok": True, "devices": devs})

            elif self.path == "/api/get_config":
                cfg = ch9121.get_config(req["nic"], ch9121.nic_mac(req["nic"]), req["mac"])
                cfg.pop("_raw", None)
                self._json({"ok": True, "config": cfg})

            elif self.path == "/api/set_config":
                mac = req["mac"]
                pcmac = ch9121.nic_mac(req["nic"])
                # 先读当前配置作为底版（保留 reserved 字段），再覆写表单值
                cur = ch9121.get_config(req["nic"], pcmac, mac)
                ch9121.set_config(req["nic"], pcmac, mac, req["config"], cur["_raw"])
                self._json({"ok": True, "msg": "配置已写入，模块自动重启中，3 秒后请重新搜索确认"})

            elif self.path == "/api/factory_reset":
                ch9121.factory_reset(req["nic"], ch9121.nic_mac(req["nic"]), req["mac"])
                self._json({"ok": True, "msg": "已恢复出厂设置，模块重启中，3 秒后请重新搜索"})

            else:
                self._json({"ok": False, "error": "not found"}, 404)

        except ch9121.CH9121Error as e:
            self._json({"ok": False, "error": str(e)})
        except (KeyError, ValueError) as e:
            self._json({"ok": False, "error": f"参数错误: {e}"}, 400)


def main():
    ap = argparse.ArgumentParser(description="通信模块配置工具")
    ap.add_argument("--port", type=int, default=8081)
    ap.add_argument("--bind", default="0.0.0.0")
    args = ap.parse_args()
    srv = ThreadingHTTPServer((args.bind, args.port), Handler)
    print(f"[通信模块] 配置工具已启动: http://localhost:{args.port}")
    print("[通信模块] 多网卡机器请在页面右上角选择接模块的那个网卡")
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
