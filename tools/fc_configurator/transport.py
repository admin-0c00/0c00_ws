# SPDX-License-Identifier: GPL-3.0-only
# Copyright (C) 2026 零创无穷（成都）科技有限责任公司
# 本文件是 SwarmCore-Sim 的一部分，
# 依据 GNU GPL v3 发布（协议全文见仓库根目录 LICENSE）。
# 本软件按"现状"提供，不附带任何明示或默示担保。

"""MAVLink 传输通道（纯标准库）

两个实现，统一接口:
  send(bytes)        发一段字节流
  recv(timeout)      在 timeout 秒内收到数据返回 bytes，超时返回 None
  close()            关闭
  describe()         给人看的通道描述

UDPTransport    —— 网络通道。对 CH9121 这类 UDP Server 模块：发给 (host, port)，
                   应答回到本机源端口，同一 socket 收发即可（recv 前 bind ''）。
                   对 PX4 SITL：SITL 在 local 14556 收、向 remote 14550 发（广播），
                   用 bind_port=14550 + send_port=14556 即可（见 README）。
SerialTransport —— USB 串口通道。termios 原始模式直接读写 /dev/ttyACM*，
                   不依赖 pyserial。
"""

import glob
import os
import socket
import struct
import termios

# termios 波特率常量（Linux）
_BAUD = {
    57600: termios.B57600, 115200: termios.B115200, 230400: termios.B230400,
    460800: termios.B460800, 921600: termios.B921600,
}


class TransportError(Exception):
    pass


class UDPTransport:
    def __init__(self, host, port, bind_port=0):
        """host:port 为发送目的；bind_port 为本机监听端口（0=临时端口）。

        CH9121 场景 bind_port=0 即可；SITL 场景用 bind_port=14550 收它的广播。
        """
        self._peer = (host, port)
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # 关键：绑 '' 而不是网卡 IP，否则收不到广播（CH9121 工具同款坑）
        self._sock.bind(("", bind_port))
        self._sock.settimeout(0.2)
        self._desc = f"udp://{host}:{port} (本机:{self._sock.getsockname()[1]})"

    def send(self, data: bytes):
        self._sock.sendto(data, self._peer)

    def recv(self, timeout: float):
        self._sock.settimeout(timeout)
        try:
            data, _ = self._sock.recvfrom(65535)
            return data
        except socket.timeout:
            return None

    def describe(self):
        return self._desc

    def close(self):
        self._sock.close()


class SerialTransport:
    def __init__(self, dev, baud=115200):
        if baud not in _BAUD:
            raise TransportError(f"不支持的波特率 {baud}（支持 {sorted(_BAUD)}）")
        try:
            self._fd = os.open(dev, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        except OSError as e:
            raise TransportError(f"打不开 {dev}: {e}")
        # 原始模式：8N1，无流控，非阻塞读
        attrs = termios.tcgetattr(self._fd)
        attrs[0] = 0                                   # iflag
        attrs[1] = 0                                   # oflag
        attrs[2] = _BAUD[baud] | termios.CS8 | termios.CREAD | termios.CLOCAL  # cflag
        attrs[3] = 0                                   # lflag
        attrs[6][termios.VMIN] = 0                     # 非阻塞
        attrs[6][termios.VTIME] = 0
        termios.tcsetattr(self._fd, termios.TCSANOW, attrs)
        self._dev, self._baud = dev, baud
        self._desc = f"{dev} @ {baud}"

    def send(self, data: bytes):
        os.write(self._fd, data)

    def recv(self, timeout: float):
        import select
        r, _, _ = select.select([self._fd], [], [], timeout)
        if not r:
            return None
        try:
            return os.read(self._fd, 65535)
        except BlockingIOError:
            return None

    def describe(self):
        return self._desc

    def fileno(self):
        return self._fd

    def close(self):
        os.close(self._fd)


def list_serial_ports():
    """枚举可能的飞控串口设备。"""
    ports = []
    for pat in ("/dev/ttyACM*", "/dev/ttyUSB*"):
        ports.extend(sorted(glob.glob(pat)))
    return ports


def list_udp_candidates():
    """枚举有 IPv4 地址的网卡，给出本机 IP 供连接页参考。"""
    import fcntl
    nics = []
    for name in sorted(os.listdir("/sys/class/net")):
        if name == "lo":
            continue
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            ip = socket.inet_ntoa(fcntl.ioctl(s.fileno(), 0x8915,  # SIOCGIFADDR
                                              struct.pack("256s", name.encode()))[20:24])
            s.close()
            nics.append({"name": name, "ip": ip})
        except OSError:
            continue
    return nics
