# SPDX-License-Identifier: GPL-3.0-only
# Copyright (C) 2026 零创无穷（成都）科技有限责任公司
# 本文件是 SwarmCore-Sim 的一部分，
# 依据 GNU GPL v3 发布（协议全文见仓库根目录 LICENSE）。
# 本软件按"现状"提供，不附带任何明示或默示担保。

"""CH9121 网络配置协议层（纯标准库）

协议要点（依据 WCH 串口命令文档与 NetModuleConfig 抓包协议逆向资料）：
- UDP 广播：模块监听 50000，上位机绑定 60000 收应答。
- 报文定长 285 字节：
    flag[16] = "CH9121_CFG_FLAG\\0"
    cmd[1] | 模块MAC[6] | PC MAC[6] | len[1] | data[255]
- 命令/应答：搜索 04->84，读配置 02->82，写配置 01->81，恢复出厂 03->83，
  校验错 NAK C0/C1/C2。
- 配置数据区 len=0xCC=204：DeviceHWConfigS(74) + DevicePortConfigS[2](65*2)。
  PortCfg[0]=端口2（辅助），PortCfg[1]=端口1（主通道）。
- 字节序：多字节参数全部小端（16 位端口、32 位波特率/打包长度）——已按实机读回数据校正。
"""

import socket
import struct

MODULE_PORT = 50000   # CH9121 广播接收端口
HOST_PORT = 60000     # 上位机广播接收端口

CMD_SET = 0x01
CMD_GET = 0x02
CMD_RESET = 0x03
CMD_SEARCH = 0x04

ACK_SET = 0x81
ACK_GET = 0x82
ACK_RESET = 0x83
ACK_SEARCH = 0x84

NAK = {0xC0: "搜索校验错", 0xC1: "配置校验错", 0xC2: "获取校验错"}

FLAG = b"CH9121_CFG_FLAG\x00"   # 16 字节固定通信标识
PKT_LEN = 285                   # 16+1+6+6+1+255
CFG_LEN = 0xCC                  # 配置数据区长度 204

NET_MODES = {0: "TCP Server", 1: "TCP Client", 2: "UDP Server", 3: "UDP Client"}
PARITY = {0: "奇", 1: "偶", 2: "Mark", 3: "Space", 4: "无"}


class CH9121Error(Exception):
    pass


def mac_str2bytes(s):
    return bytes(int(x, 16) for x in s.strip().split(":"))


def mac_bytes2str(b):
    return ":".join(f"{x:02x}" for x in b)


def ip_str2bytes(s):
    parts = s.strip().split(".")
    if len(parts) != 4:
        raise ValueError(f"非法 IP: {s}")
    return bytes(int(x) for x in parts)


def ip_bytes2str(b):
    return ".".join(str(x) for x in b)


def nic_mac(nic):
    """读网卡 MAC（/sys/class/net/<nic>/address）。"""
    with open(f"/sys/class/net/{nic}/address") as f:
        return mac_str2bytes(f.read().strip())


def nic_addr_bcast(nic):
    """读网卡 IPv4 地址与定向广播地址（ioctl，免 ip 命令依赖）。"""
    import fcntl
    SIOCGIFADDR = 0x8915
    SIOCGIFBRDADDR = 0x8919
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        ifreq = struct.pack("256s", nic.encode()[:15])
        ip = socket.inet_ntoa(fcntl.ioctl(s.fileno(), SIOCGIFADDR, ifreq)[20:24])
        bcast = socket.inet_ntoa(fcntl.ioctl(s.fileno(), SIOCGIFBRDADDR, ifreq)[20:24])
        return ip, bcast
    finally:
        s.close()


def _pack(cmd, dev_mac, pc_mac, data=b""):
    pkt = bytearray(PKT_LEN)
    pkt[0:16] = FLAG
    pkt[16] = cmd
    pkt[17:23] = dev_mac
    pkt[23:29] = pc_mac
    pkt[29] = len(data)
    pkt[30:30 + len(data)] = data
    return bytes(pkt)


def _parse(pkt):
    if len(pkt) < 30 or pkt[0:16] != FLAG:
        return None
    cmd = pkt[16]
    if cmd in NAK:
        raise CH9121Error(f"模块应答校验错：{NAK[cmd]} (0x{cmd:02x})")
    return {
        "cmd": cmd,
        "mac": mac_bytes2str(pkt[17:23]),
        "len": pkt[29],
        "data": bytes(pkt[30:30 + pkt[29]]),
        "raw_data": bytes(pkt[30:285]),  # 搜索应答的固件版本在 len 计数之外，用它兜底
    }


def _open_sock():
    """绑定全网卡 60000 端口的 UDP 广播 socket。

    注意必须绑 '' 而不是具体网卡 IP：模块的应答是广播包，
    绑定到具体 IP 的 socket 收不到目的地址为广播地址的包（踩过的坑）。
    发包走哪个网卡由目的地址（定向广播）决定，不依赖 bind。
    """
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        s.bind(("", HOST_PORT))
    except OSError as e:
        s.close()
        raise CH9121Error(
            f"绑定 {HOST_PORT} 端口失败（{e}）。"
            "端口被占用时请关闭 NetModuleConfig 类软件"
        )
    return s


def _send(s, pkt, bcast):
    """定向广播 + 有限广播各发一份，多网卡/禁定向广播场景兜底。"""
    for addr in dict.fromkeys((bcast, "255.255.255.255")):
        s.sendto(pkt, (addr, MODULE_PORT))


def search(nic, pc_mac, timeout=1.5):
    """经指定网卡广播搜索，返回 [{mac, ip, name, version}, ...]。"""
    _, bcast = nic_addr_bcast(nic)
    s = _open_sock()
    try:
        s.settimeout(0.3)
        _send(s, _pack(CMD_SEARCH, b"\x00" * 6, pc_mac), bcast)
        devs = {}
        deadline = timeout
        import time
        end = time.monotonic() + deadline
        while time.monotonic() < end:
            try:
                pkt, _ = s.recvfrom(2048)
            except socket.timeout:
                continue
            r = _parse(pkt)
            if r and r["cmd"] == ACK_SEARCH and r["mac"] not in devs:
                # 搜索应答：IP[4] + 设备名（变长，null 结尾）+ 固件版本[1]
                # 注意 len 只数到设备名 null 为止，版本字节在 len 之外（实机实测）
                d = r["raw_data"]
                nul = d.find(b"\x00", 4)
                devs[r["mac"]] = {
                    "mac": r["mac"],
                    "ip": ip_bytes2str(d[0:4]),
                    "name": d[4:nul].decode("ascii", "replace") if nul > 4 else "?",
                    "version": d[nul + 1] if 0 < nul + 1 < len(d) else 0,
                }
        return list(devs.values())
    finally:
        s.close()


def _transact(nic, pc_mac, cmd, dev_mac, data=b"", ack=0, timeout=2.0, retries=2):
    """发一条命令并等待该 MAC 的对应应答，带重试。"""
    _, bcast = nic_addr_bcast(nic)
    s = _open_sock()
    try:
        for _ in range(retries + 1):
            _send(s, _pack(cmd, dev_mac, pc_mac, data), bcast)
            s.settimeout(timeout)
            import time
            end = time.monotonic() + timeout
            while time.monotonic() < end:
                try:
                    pkt, _ = s.recvfrom(2048)
                except socket.timeout:
                    break
                r = _parse(pkt)
                if r and r["cmd"] == ack and r["mac"] == mac_bytes2str(dev_mac):
                    return r["data"]
        raise CH9121Error("模块无应答（超时）。请确认网卡选择正确、模块在线")
    finally:
        s.close()


# ---------- 配置结构体解析/打包 ----------
# DeviceHWConfigS 偏移（74 字节）
HW_FMT_OFF = {
    "dev_type": 0, "aux_type": 1, "index": 2, "hw_ver": 3, "sw_ver": 4,
    "name": (5, 21), "mac": (26, 6), "ip": (32, 4), "gw": (36, 4),
    "mask": (40, 4), "dhcp": 44,
    # 45-46 WEB 端口（保留回写）, 47-64 保留, 65 串口协商使能, 66-73 保留
    "com_cfg_en": 65,
}
HW_LEN = 74

# DevicePortConfigS 偏移（65 字节）
PORT_LEN = 65


def _parse_port(b):
    return {
        "index": b[0],
        "enabled": b[1],
        "mode": b[2],
        "rand_sport": b[3],
        "local_port": struct.unpack("<H", b[4:6])[0],      # 16 位端口也是小端
        "dest_ip": ip_bytes2str(b[6:10]),
        "dest_port": struct.unpack("<H", b[10:12])[0],
        "baud": struct.unpack("<I", b[12:16])[0],          # 32 位小端
        "data_bits": b[16],
        "stop_bits": b[17],
        "parity": b[18],
        "phy_disconnect": b[19],
        "rx_pack_len": struct.unpack("<I", b[20:24])[0],
        "rx_pack_timeout": struct.unpack("<I", b[24:28])[0],  # 单位 10ms，0=关闭
        "retry": b[28],
        "clear_on_connect": b[29],
        "domain_enable": b[30],
        "domain": b[31:51].split(b"\x00")[0].decode("ascii", "replace"),
    }


def _pack_port(p, raw):
    """在原始 65 字节基础上覆写表单字段，保留 reserved 字节。"""
    b = bytearray(raw)
    b[1] = p["enabled"]
    b[2] = p["mode"]
    b[3] = p["rand_sport"]
    b[4:6] = struct.pack("<H", p["local_port"])
    b[6:10] = ip_str2bytes(p["dest_ip"])
    b[10:12] = struct.pack("<H", p["dest_port"])
    b[12:16] = struct.pack("<I", p["baud"])
    b[16] = p["data_bits"]
    b[17] = p["stop_bits"]
    b[18] = p["parity"]
    b[19] = p["phy_disconnect"]
    b[20:24] = struct.pack("<I", p["rx_pack_len"])
    b[24:28] = struct.pack("<I", p["rx_pack_timeout"])
    b[29] = p["clear_on_connect"]
    b[30] = p["domain_enable"]
    dom = p["domain"].encode("ascii", "replace")[:20]
    b[31:51] = dom.ljust(20, b"\x00")
    return bytes(b)


def parse_config(data):
    """204 字节配置区 -> JSON 表单模型。保留原始字节供写回。"""
    if len(data) < CFG_LEN:
        raise CH9121Error(f"配置数据长度不足：{len(data)} < {CFG_LEN}")
    hw = data[:HW_LEN]
    return {
        "_raw": data,  # 内部保留，set_config 时覆写（不进 JSON 响应）
        "dev_type": hw[0], "hw_ver": hw[3], "sw_ver": hw[4],
        "name": hw[5:26].split(b"\x00")[0].decode("ascii", "replace").strip(),
        "mac": mac_bytes2str(hw[26:32]),
        "ip": ip_bytes2str(hw[32:36]),
        "gateway": ip_bytes2str(hw[36:40]),
        "mask": ip_bytes2str(hw[40:44]),
        "dhcp": hw[44],
        "uart_negotiate": hw[65],
        # 注意：PortCfg[0]=端口2，PortCfg[1]=端口1
        "port2": _parse_port(data[HW_LEN:HW_LEN + PORT_LEN]),
        "port1": _parse_port(data[HW_LEN + PORT_LEN:HW_LEN + 2 * PORT_LEN]),
    }


def pack_config(cfg, raw):
    """在原始 204 字节配置上覆写表单字段（reserved 字节原样保留）。"""
    b = bytearray(raw[:CFG_LEN])
    name = cfg["name"].encode("ascii", "replace")[:21]
    b[5:26] = name.ljust(21, b"\x00")
    # MAC 允许修改（26:32）
    b[26:32] = mac_str2bytes(cfg["mac"])
    b[32:36] = ip_str2bytes(cfg["ip"])
    b[36:40] = ip_str2bytes(cfg["gateway"])
    b[40:44] = ip_str2bytes(cfg["mask"])
    b[44] = cfg["dhcp"]
    b[65] = cfg["uart_negotiate"]
    b[HW_LEN:HW_LEN + PORT_LEN] = _pack_port(cfg["port2"], raw[HW_LEN:HW_LEN + PORT_LEN])
    b[HW_LEN + PORT_LEN:HW_LEN + 2 * PORT_LEN] = _pack_port(
        cfg["port1"], raw[HW_LEN + PORT_LEN:HW_LEN + 2 * PORT_LEN])
    return bytes(b)


def get_config(nic, pc_mac, mac):
    data = _transact(nic, pc_mac, CMD_GET, mac_str2bytes(mac), ack=ACK_GET)
    return parse_config(data)


def set_config(nic, pc_mac, mac, cfg, raw):
    """写配置（基于 raw 原始配置覆写，reserved 字段不动）。返回模块回显配置。"""
    data = pack_config(cfg, raw)
    echo = _transact(nic, pc_mac, CMD_SET, mac_str2bytes(mac), data, ack=ACK_SET)
    # 模块写入后会自动重启；回显包可用于核对
    return parse_config(echo) if len(echo) >= CFG_LEN else None


def factory_reset(nic, pc_mac, mac):
    _transact(nic, pc_mac, CMD_RESET, mac_str2bytes(mac), ack=ACK_RESET)
