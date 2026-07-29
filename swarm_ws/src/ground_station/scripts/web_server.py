#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
# Copyright (C) 2026 零创无穷（成都）科技有限责任公司
# 本文件是 SwarmCore-Sim 的一部分，
# 依据 GNU GPL v3 发布（协议全文见仓库根目录 LICENSE）。
# 本软件按"现状"提供，不附带任何明示或默示担保。

"""SwarmCore 地面站单端口 Web 服务

8080 同时提供:
  - 静态文件（web/ 目录）
  - /ws 的 WebSocket 中转 -> localhost:9090 (rosbridge)

背景: 部分网络环境（路由器/AP/安全软件）只放行"像网页"的端口，
9090 的 rosbridge 会被中间设备丢弃，浏览器永远连不上。
单端口部署后只暴露 8080，ws 流量混在网页端口里通过。

依赖: pip install websockets (>=13)
"""
import asyncio
import mimetypes
from pathlib import Path

from websockets.asyncio.client import connect
from websockets.asyncio.server import serve
from websockets.datastructures import Headers
from websockets.http11 import Response

WEB_DIR = Path(__file__).resolve().parent.parent / "web"
BAGS_DIR = Path.home() / "0c00_ws/swarm_ws/logs/bags"   # 数据页 CSV 导出下载
ROSBRIDGE = "ws://localhost:9090"
LISTEN_PORT = 8080

mimetypes.add_type("application/javascript", ".js")


async def process_request(connection, request):
    if request.path == "/ws":
        return None  # 交给 WebSocket 握手
    if request.path.startswith("/bags/"):
        # bag 产物下载（CSV 导出 zip 等），限制在 bags 目录内防路径穿越
        rel = request.path[len("/bags/"):]
        target = (BAGS_DIR / rel).resolve()
        if not str(target).startswith(str(BAGS_DIR)) or not target.is_file():
            return Response(404, "Not Found",
                            Headers([("Content-Type", "text/plain")]), b"not found")
        body = target.read_bytes()
        return Response(200, "OK",
                        Headers([("Content-Type", "application/octet-stream"),
                                 ("Content-Disposition", f'attachment; filename="{target.name}"'),
                                 ("Content-Length", str(len(body)))]), body)
    rel = request.path.lstrip("/") or "index.html"
    target = (WEB_DIR / rel).resolve()
    if not str(target).startswith(str(WEB_DIR)) or not target.is_file():
        return Response(404, "Not Found",
                        Headers([("Content-Type", "text/plain")]), b"not found")
    body = target.read_bytes()
    mime = mimetypes.guess_type(target.name)[0] or "application/octet-stream"
    return Response(200, "OK",
                    Headers([("Content-Type", mime),
                             ("Content-Length", str(len(body)))]), body)


async def handler(connection):
    if connection.request.path != "/ws":
        await connection.close(1008, "only /ws")
        return
    try:
        # 本机上游无需 keepalive；rosbridge 高负载时回 ping 慢，默认 20s 超时会误杀连接
        # max_size 必须放大：曲线结果 JSON 可达数 MB，websockets 默认 1MiB 上限会把
        # 大帧直接掐断连接（用户"曲线打不开、连接闪断"的根因）
        async with connect(ROSBRIDGE, ping_interval=None, max_size=32 * 1024 * 1024) as upstream:
            async def down():  # 浏览器 -> rosbridge
                async for msg in connection:
                    await upstream.send(msg)

            async def up():  # rosbridge -> 浏览器
                async for msg in upstream:
                    await connection.send(msg)

            tasks = [asyncio.create_task(down()), asyncio.create_task(up())]
            _done, pending = await asyncio.wait(tasks, return_when=asyncio.FIRST_COMPLETED)
            for t in pending:
                t.cancel()
    except Exception:
        try:
            await connection.close(1011)
        except Exception:
            pass


async def main():
    # 浏览器侧 keepalive 放宽: 页面高负载时回 pong 可能较慢，超时收紧会误杀
    async with serve(handler, "0.0.0.0", LISTEN_PORT,
                     process_request=process_request,
                     ping_interval=30, ping_timeout=120,
                     max_size=32 * 1024 * 1024):   # 同上游，放行大帧
        print(f"[web_server] http://0.0.0.0:{LISTEN_PORT} "
              f"(静态: {WEB_DIR}, /ws -> {ROSBRIDGE})")
        await asyncio.get_running_loop().create_future()  # run forever


if __name__ == "__main__":
    asyncio.run(main())
