#!/usr/bin/env python3
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
ROSBRIDGE = "ws://localhost:9090"
LISTEN_PORT = 8080

mimetypes.add_type("application/javascript", ".js")


async def process_request(connection, request):
    if request.path == "/ws":
        return None  # 交给 WebSocket 握手
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
        async with connect(ROSBRIDGE) as upstream:
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
    async with serve(handler, "0.0.0.0", LISTEN_PORT,
                     process_request=process_request):
        print(f"[web_server] http://0.0.0.0:{LISTEN_PORT} "
              f"(静态: {WEB_DIR}, /ws -> {ROSBRIDGE})")
        await asyncio.get_running_loop().create_future()  # run forever


if __name__ == "__main__":
    asyncio.run(main())
