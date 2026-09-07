# SPDX-License-Identifier: GPL-3.0-only
# Copyright (C) 2026 零创无穷（成都）科技有限责任公司
# 本文件是 SwarmCore-Sim 的一部分，
# 依据 GNU GPL v3 发布（协议全文见仓库根目录 LICENSE）。
# 本软件按"现状"提供，不附带任何明示或默示担保。

"""MAVLink 最小实现（纯标准库）

只实现飞控配置器用到的消息子集（common.xml）：
  接收: HEARTBEAT(0) / PARAM_VALUE(22) / COMMAND_ACK(77) /
        STATUSTEXT(253) / AUTOPILOT_VERSION(148)
  发送: HEARTBEAT(0) / PARAM_REQUEST_READ(20) / PARAM_REQUEST_LIST(21) /
        PARAM_SET(23) / COMMAND_LONG(76)

帧格式同时兼容 MAVLink v1(0xFE) 和 v2(0xFD)，不签名。
注意 MAVLink 线序：字段按类型大小降序排（与 XML 声明顺序无关），
下面的 pack/unpack 均已按线序硬编码，不要按声明顺序改。

校验: X25 CRC（帧头+载荷 + 每条消息一个 crc_extra 字节）。
"""

import struct

# ---- 消息定义: msgid -> (name, crc_extra) ----
MSG_HEARTBEAT = 0
MSG_PARAM_REQUEST_READ = 20
MSG_PARAM_REQUEST_LIST = 21
MSG_PARAM_VALUE = 22
MSG_PARAM_SET = 23
MSG_COMMAND_LONG = 76
MSG_COMMAND_ACK = 77
MSG_AUTOPILOT_VERSION = 148
MSG_STATUSTEXT = 253
MSG_FILE_TRANSFER_PROTOCOL = 110

CRC_EXTRA = {
    0: 50, 20: 214, 21: 159, 22: 220, 23: 168,
    76: 152, 77: 143, 148: 178, 253: 83, 110: 84,
}

# ---- MAVLink FTP（FILE_TRANSFER_PROTOCOL 内嵌载荷） ----
FTP_OP = {
    "TerminateSession": 1, "ResetSessions": 2, "ListDirectory": 3,
    "OpenFileRO": 4, "ReadFile": 5, "CreateFile": 6, "WriteFile": 7,
    "RemoveFile": 8, "CreateDirectory": 9, "RemoveDirectory": 10,
    "OpenFileWO": 11, "TruncateFile": 12, "Rename": 13,
    "CalcFileCRC32": 14, "BurstReadFile": 15,
}
FTP_ACK = 128
FTP_NAK = 129
FTP_ERR = {
    0: "无", 1: "失败", 2: "系统错误", 3: "数据长度非法", 4: "会话无效",
    5: "会话耗尽", 6: "EOF", 7: "未知命令", 8: "文件已存在",
    9: "文件被保护", 10: "文件不存在",
}

# MAV_PARAM_TYPE: UINT8=1 INT8=2 UINT16=3 INT16=4 UINT32=5 INT32=6
#                 UINT64=7 INT64=8 REAL32=9 REAL64=10
PARAM_TYPE_INT8 = 2
PARAM_TYPE_INT32 = 6
PARAM_TYPE_REAL32 = 9

# MAV_CMD（本工具用到的）
MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN = 246
MAV_CMD_PREFLIGHT_CALIBRATION = 241
MAV_CMD_ACTUATOR_TEST = 310
MAV_CMD_REQUEST_MESSAGE = 512
MAV_CMD_COMPONENT_ARM_DISARM = 400

# ACTUATOR_OUTPUT_FUNCTION: Motor1..N = 1..N（注意不是 101 起）
OUTPUT_FUNC_MOTOR_BASE = 0

# MAV_RESULT
RESULT_NAMES = {
    0: "ACCEPTED", 1: "TEMPORARILY_REJECTED", 2: "DENIED",
    3: "UNSUPPORTED", 4: "FAILED", 5: "IN_PROGRESS",
    6: "CANCELLED", 7: "LONG_ONLY", 8: "INT_ONLY", 9: "UNSUPPORTED_MAV_FRAME",
}


class MAVLinkError(Exception):
    pass


def x25_crc(data: bytes, extra: int = None) -> int:
    crc = 0xFFFF
    for b in data:
        tmp = b ^ (crc & 0xFF)
        tmp = (tmp ^ (tmp << 4)) & 0xFF
        crc = ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF
    if extra is not None:
        tmp = extra ^ (crc & 0xFF)
        tmp = (tmp ^ (tmp << 4)) & 0xFF
        crc = ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF
    return crc


def pack_v2(msgid: int, payload: bytes, sysid: int, compid: int, seq: int) -> bytes:
    """打包 MAVLink v2 帧（v2 帧尾零字节可截断，这里不截断，简单可靠）。"""
    extra = CRC_EXTRA.get(msgid)
    if extra is None:
        raise MAVLinkError(f"未知消息 id {msgid}")
    header = bytes([len(payload), 0, 0, seq & 0xFF, sysid & 0xFF, compid & 0xFF,
                    msgid & 0xFF, (msgid >> 8) & 0xFF, (msgid >> 16) & 0xFF])
    crc = x25_crc(header + payload, extra)
    return b"\xFD" + header + payload + struct.pack("<H", crc)


class Parser:
    """流式解帧器：feed(bytes) -> [(msgid, payload, sysid, compid), ...]

    自动跳过非帧头字节（串口/UDP 桥混入的杂字节），CRC 校验不过的帧丢弃。
    """

    def __init__(self):
        self._buf = bytearray()

    def feed(self, data: bytes):
        self._buf += data
        out = []
        while True:
            msg = self._try_parse()
            if msg is None:
                break
            out.append(msg)
        return out

    def _try_parse(self):
        buf = self._buf
        # 找帧头；CRC 校验失败说明是噪声里的假帧头，只跳过一个字节重新扫描
        # （不能按假帧声明的长度整块丢弃，会吞掉嵌在里面的真帧）
        while buf:
            while buf and buf[0] not in (0xFE, 0xFD):
                del buf[0]
            if len(buf) < 3:
                return None
            if buf[0] == 0xFE:  # v1: magic len seq sys comp msgid payload crc
                plen = buf[1]
                total = 6 + plen + 2
                if len(buf) < total:
                    return None
                frame = bytes(buf[:total])
                msgid = frame[5]
                payload = frame[6:6 + plen]
                hdr = frame[1:6]
                sysid, compid = frame[3], frame[4]
            else:  # v2
                plen = buf[1]
                total = 10 + plen + 2
                if len(buf) < total:
                    return None
                frame = bytes(buf[:total])
                if frame[2] & 0x01:  # 签名帧：不验签，整帧跳过
                    del buf[:total + 13]
                    continue
                msgid = frame[7] | (frame[8] << 8) | (frame[9] << 16)
                payload = frame[10:10 + plen]
                hdr = frame[1:10]
                sysid, compid = frame[5], frame[6]
            extra = CRC_EXTRA.get(msgid)
            if extra is not None:
                want = struct.unpack("<H", frame[-2:])[0]
                if x25_crc(hdr + payload, extra) != want:
                    del buf[0]  # 假帧头：只跳 1 字节重扫
                    continue
            # 未知 msgid（无 crc_extra）无法校验，原样上交，由上层忽略
            del buf[:total]
            return (msgid, payload, sysid, compid)
        return None


# ================= 发送侧 pack（线序：大类型在前） =================

def pack_heartbeat_gcs(sysid=255, compid=190, seq=0):
    """地面站心跳: type=6(MAV_TYPE_GCS), autopilot=8(INVALID)。"""
    payload = struct.pack("<IBBBBB", 0, 6, 8, 0, 4, 3)  # custom_mode, type, autopilot, base_mode, system_status(ACTIVE), mavlink_version
    return pack_v2(MSG_HEARTBEAT, payload, sysid, compid, seq)


def pack_param_request_list(target_sys, target_comp, sysid=255, compid=190, seq=0):
    payload = struct.pack("<BB", target_sys, target_comp)
    return pack_v2(MSG_PARAM_REQUEST_LIST, payload, sysid, compid, seq)


def pack_param_request_read(target_sys, target_comp, param_id=b"", param_index=-1,
                            sysid=255, compid=190, seq=0):
    if isinstance(param_id, str):
        param_id = param_id.encode()
    param_id = param_id[:16].ljust(16, b"\0")
    payload = struct.pack("<hBB", param_index, target_sys, target_comp) + param_id
    return pack_v2(MSG_PARAM_REQUEST_READ, payload, sysid, compid, seq)


def pack_param_set(target_sys, target_comp, name: str, value, param_type,
                   sysid=255, compid=190, seq=0):
    pid = name.encode()[:16].ljust(16, b"\0")
    # 4 字节值字段按类型编码：整型必须位填充 int32（float 5.0 的字节会被读成 1084227584）
    if param_type == PARAM_TYPE_REAL32:
        vb = struct.pack("<f", float(value))
    else:
        vb = struct.pack("<i", int(round(float(value))))
    payload = vb + struct.pack("<BB", target_sys, target_comp) + pid + bytes([param_type])
    return pack_v2(MSG_PARAM_SET, payload, sysid, compid, seq)


def pack_command_long(target_sys, target_comp, command, params=(),
                      confirmation=0, sysid=255, compid=190, seq=0):
    p = list(params) + [0.0] * (7 - len(params))
    payload = struct.pack("<7fHBBB", *p[:7], command, target_sys, target_comp, confirmation)
    return pack_v2(MSG_COMMAND_LONG, payload, sysid, compid, seq)


# ================= 接收侧 unpack（返回 dict） =================

def parse_heartbeat(p: bytes):
    custom_mode, typ, autopilot, base_mode, status = struct.unpack("<IBBBB", p[:8])
    return {
        "custom_mode": custom_mode, "type": typ, "autopilot": autopilot,
        "base_mode": base_mode, "system_status": status,
        "armed": bool(base_mode & 0x80),  # MAV_MODE_FLAG_SAFETY_ARMED
    }


def parse_param_value(p: bytes):
    # param_value 线序是 4 字节 union：REAL32 按 float 解，整型按位重解释为 int32
    # （QGC/PX4 约定；直接 unpack <f 会把 int 5 读成 7e-45 ≈ 0）
    raw, count, index = struct.unpack("<IHH", p[:8])
    pid = p[8:24].split(b"\0")[0].decode("ascii", "replace")
    ptype = p[24] if len(p) > 24 else PARAM_TYPE_REAL32
    if ptype == PARAM_TYPE_REAL32:
        value = struct.unpack("<f", struct.pack("<I", raw))[0]
    else:
        value = struct.unpack("<i", struct.pack("<I", raw))[0]
    return {"name": pid, "value": value, "type": ptype, "count": count, "index": index}


def parse_command_ack(p: bytes):
    p = p.ljust(13, b"\0")  # v2 尾零截断补齐；扩展字段按声明顺序（不重排）
    command, result = struct.unpack("<HB", p[:3])
    ack = {"command": command, "result": result,
           "result_name": RESULT_NAMES.get(result, f"UNKNOWN({result})")}
    if len(p) > 3:  # v2 扩展: progress u8, result_param2 i32, target_sys u8, target_comp u8
        ack["progress"] = p[3]
        ack["result_param2"] = struct.unpack("<i", p[4:8])[0]
    return ack


def parse_statustext(p: bytes):
    severity = p[0]
    text = p[1:51].split(b"\0")[0].decode("utf-8", "replace")
    return {"severity": severity, "text": text}


def parse_autopilot_version(p: bytes):
    # 线序(实测核对): u64 capabilities, u64 uid, 4×u32 (flight/mw/os/board version),
    #       u16 vendor_id, u16 product_id, 3×u8[8] custom versions; v2 尾零截断需补齐
    p = p.ljust(60, b"\0")
    caps, uid, fsw = struct.unpack("<QQI", p[:20])
    vendor, product = struct.unpack("<HH", p[32:36])
    return {
        "capabilities": caps, "uid": uid,
        "flight_sw_version": fsw,
        "flight_sw_version_str": f"{(fsw >> 24) & 0xFF}.{(fsw >> 16) & 0xFF}.{(fsw >> 8) & 0xFF}",
        "vendor_id": vendor, "product_id": product,
    }


def parse(msgid, payload):
    """按 msgid 分发解析；未实现的消息返回 None。"""
    fn = {
        MSG_HEARTBEAT: parse_heartbeat,
        MSG_PARAM_VALUE: parse_param_value,
        MSG_COMMAND_ACK: parse_command_ack,
        MSG_STATUSTEXT: parse_statustext,
        MSG_AUTOPILOT_VERSION: parse_autopilot_version,
        MSG_FILE_TRANSFER_PROTOCOL: parse_ftp,
    }.get(msgid)
    return fn(payload) if fn else None


# ================= MAVLink FTP（消息 110） =================
# FILE_TRANSFER_PROTOCOL 线序（对照 PX4 v1.15 生成的 mavlink C 头核实）：
#   target_network u8 @0, target_system u8 @1, target_component u8 @2,
#   uint8 payload[251] @3   （本方言 target_network 在基座里，不是扩展，总长 254）
# FTP 载荷内部：seq u16, session u8, opcode u8, size u8, req_opcode u8,
#               burst_complete u8, padding u8, offset u32, data[239]

def pack_ftp(target_sys, target_comp, seq, session, opcode, req_opcode=0,
             offset=0, data=b"", size=None, burst_complete=0,
             sysid=255, compid=190, msg_seq=0):
    # size 字段 = 请求读长度（ReadFile 用，PX4 按它决定返回多少字节），
    # 与 data 长度独立；不传则取 len(data)（写文件/列目录等场景）
    data = bytes(data)[:239]
    if size is None:
        size = len(data)
    ftp = struct.pack("<HBBBBBBI", seq & 0xFFFF, session & 0xFF, opcode & 0xFF,
                      size & 0xFF, req_opcode & 0xFF, burst_complete & 0xFF, 0,
                      offset & 0xFFFFFFFF) + data
    ftp = ftp.ljust(251, b"\0")
    payload = bytes([0, target_sys & 0xFF, target_comp & 0xFF]) + ftp
    return pack_v2(MSG_FILE_TRANSFER_PROTOCOL, payload, sysid, compid, msg_seq)


def parse_ftp(p: bytes):
    """解析 FTP 应答。返回 dict（含 target_system/component 供过滤）。"""
    p = p.ljust(254, b"\0")
    seq, session, opcode, size, req_op, burst, _, offset = struct.unpack("<HBBBBBBI", p[3:15])
    size = min(size, 239)
    return {
        "target_system": p[1], "target_component": p[2],
        "seq": seq, "session": session, "opcode": opcode, "req_opcode": req_op,
        "burst_complete": burst, "offset": offset, "data": p[15:15 + size],
    }
