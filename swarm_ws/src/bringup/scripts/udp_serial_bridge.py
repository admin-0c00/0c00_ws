#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
# Copyright (C) 2026 零创无穷（成都）科技有限责任公司
# 本文件是 SwarmCore-Sim 的一部分，
# 依据 GNU GPL v3 发布（协议全文见仓库根目录 LICENSE）。
# 本软件按"现状"提供，不附带任何明示或默示担保。

"""UDP <-> PTY 字节流桥（纯标准库）

用途：真机飞控的 uXRCE-DDS 串口帧经 CH9121（串口转 UDP）到达地面电脑后，
Agent 只认 serial 传输（PTY/串口设备），本脚本把 UDP 字节流落成一个 PTY：

    CH9121(UDP) <---> udp_serial_bridge.py <---> /tmp/ttyUAV_N(PTY) <---> MicroXRCEAgent serial

用法: udp_serial_bridge.py <模块IP> <模块UDP端口> <PTY链接路径>
CH9121 端口须配为 UDP Server 模式：模块收到本桥发出的首个数据报后锁定对端。
"""

import errno
import os
import pty
import select
import socket
import sys
import time
import tty


def main():
    if len(sys.argv) != 4:
        sys.exit(__doc__)
    ip, port, link = sys.argv[1], int(sys.argv[2]), sys.argv[3]

    master, slave = pty.openpty()
    tty.setraw(slave)  # 原始字节流，关掉回显/换行转换，否则污染 XRCE 帧
    if os.path.lexists(link):
        os.unlink(link)
    os.symlink(os.ttyname(slave), link)

    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setblocking(False)
    peer = (ip, port)
    print(f"[bridge] {link} -> {os.ttyname(slave)}，UDP 对端 {ip}:{port}", flush=True)

    # CH9121 UDP Server 模式要先收到本网来的包才会把串口数据转发过来（锁定对端），
    # 且模块重启/掉电后锁定丢失。Agent 收到 client 首帧前不发数据——互不先开口会死锁。
    # 因此做看门狗敲门：只要超过 2 秒没收到模块的 UDP 包，就恢复 1Hz 敲门，
    # 直到链路再次来包。敲门文本是可见 ASCII，方便在飞控端 cat 串口验证方向。
    last_rx = 0.0
    knock_at = 0.0
    while True:
        r, _, _ = select.select([master, s], [], [], 0.2)
        now = time.monotonic()
        if now - last_rx > 2.0 and now >= knock_at:
            s.sendto(b"~KNOCK\n", peer)
            knock_at = now + 1.0
        if master in r:
            try:
                data = os.read(master, 4096)
            except OSError as e:
                if e.errno == errno.EIO:  # Agent 还没打开 PTY 从端
                    time.sleep(0.05)
                    continue
                raise
            if not data:
                break
            s.sendto(data, peer)
        if s in r:
            data, _ = s.recvfrom(65535)
            last_rx = time.monotonic()
            os.write(master, data)


if __name__ == "__main__":
    main()
