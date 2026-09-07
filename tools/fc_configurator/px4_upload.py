# SPDX-License-Identifier: GPL-3.0-only
# Copyright (C) 2026 零创无穷（成都）科技有限责任公司
# 本文件是 SwarmCore-Sim 的一部分，
# 依据 GNU GPL v3 发布（协议全文见仓库根目录 LICENSE）。
# 本软件按"现状"提供，不附带任何明示或默示担保。

"""PX4 bootloader 串口烧录（纯标准库）

协议参照 PX4-Autopilot/Tools/px_uploader.py（BSD）重写：
  GET_SYNC(0x21) 同步 → GET_DEVICE(0x22) 读板型/flash 上限 → CHIP_ERASE(0x23)
  → PROG_MULTI(0x27) 分块写入（每块 ≤252B，4 对齐）→ GET_CRC(0x29) 校验
  → REBOOT(0x30)
  应答: INSYNC(0x12) + OK(0x10)/FAILED(0x11)/INVALID(0x13)

固件文件为 .px4（JSON：image=base64+zlib，board_id，image_maxsize）。
CRC32 为 zlib 反射表（poly 0xEDB88320），init=0 无尾异或，按 bootloader 口径
对镜像 0xFF 填充到 flash 上限后计算——与 px_uploader firmware.crc() 一致。

只做串口（bootloader 不走网络）。不依赖 pyserial。
"""

import base64
import json
import os
import select
import struct
import termios
import time
import zlib

INSYNC = b"\x12"
EOC = b"\x20"
OK = b"\x10"
FAILED = b"\x11"
INVALID = b"\x13"

GET_SYNC = b"\x21"
GET_DEVICE = b"\x22"
CHIP_ERASE = b"\x23"
PROG_MULTI = b"\x27"
GET_CRC = b"\x29"
REBOOT = b"\x30"

INFO_BL_REV = b"\x01"
INFO_BOARD_ID = b"\x02"
INFO_FLASH_SIZE = b"\x04"

PROG_MULTI_MAX = 252  # 协议上限 255，须为 4 的倍数


class FlashError(Exception):
    pass


def _make_crctab():
    tab = []
    for i in range(256):
        c = i
        for _ in range(8):
            c = (c >> 1) ^ 0xEDB88320 if c & 1 else c >> 1
        tab.append(c)
    return tab


_CRCTAB = _make_crctab()


def _crc32(data, state=0):
    for b in data:
        state = _CRCTAB[(state ^ b) & 0xFF] ^ (state >> 8)
    return state


def load_px4_file(path_or_bytes):
    """解析 .px4 固件，返回 {image, board_id, image_size, image_maxsize, summary}。"""
    if isinstance(path_or_bytes, (bytes, bytearray)):
        desc = json.loads(bytes(path_or_bytes).decode())
    else:
        with open(path_or_bytes, encoding="utf-8") as f:
            desc = json.load(f)
    image = bytearray(zlib.decompress(base64.b64decode(desc["image"])))
    while len(image) % 4:
        image.append(0xFF)
    return {
        "image": image,
        "board_id": desc.get("board_id"),
        "image_size": desc.get("image_size", len(image)),
        "image_maxsize": desc.get("image_maxsize"),
        "summary": desc.get("summary", ""),
        "git_identity": desc.get("git_identity", ""),
    }


def image_crc(image, padlen):
    """bootloader 口径 CRC：镜像 + 0xFF 填充到 padlen-1（步进 4）。"""
    state = _crc32(image)
    pad = b"\xff\xff\xff\xff"
    for _ in range(len(image), padlen - 1, 4):
        state = _crc32(pad, state)
    return state


class _Serial:
    """阻塞式原始串口（bootloader 交互用，115200 8N1）。"""

    def __init__(self, dev, baud=115200):
        self.fd = os.open(dev, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        attrs = termios.tcgetattr(self.fd)
        attrs[0] = attrs[1] = attrs[3] = 0
        attrs[2] = termios.B115200 | termios.CS8 | termios.CREAD | termios.CLOCAL
        attrs[6][termios.VMIN] = 0
        attrs[6][termios.VTIME] = 0
        termios.tcsetattr(self.fd, termios.TCSANOW, attrs)
        # bootloader 固定 115200（CH341/CDC 都支持）
        self.dev = dev

    def read(self, n, timeout=0.5):
        out = bytearray()
        deadline = time.time() + timeout
        while len(out) < n and time.time() < deadline:
            r, _, _ = select.select([self.fd], [], [], max(0, deadline - time.time()))
            if r:
                try:
                    out += os.read(self.fd, n - len(out))
                except BlockingIOError:
                    pass
        if len(out) < n:
            raise FlashError(f"串口读超时（要 {n} 字节，收到 {len(out)}）")
        return bytes(out)

    def write(self, data):
        os.write(self.fd, data)

    def flush_input(self):
        try:
            while select.select([self.fd], [], [], 0)[0]:
                os.read(self.fd, 4096)
        except BlockingIOError:
            pass

    def close(self):
        os.close(self.fd)


class Uploader:
    """一次烧录会话。progress_cb(pct:int, msg:str) 上报进度。"""

    def __init__(self, dev, progress_cb=None):
        self.dev = dev
        self.progress_cb = progress_cb or (lambda pct, msg: None)
        self.ser = None
        self.bl_rev = None
        self.board_id = None
        self.fw_maxsize = None

    def _report(self, pct, msg):
        self.progress_cb(pct, msg)

    # ---- 协议原语 ----
    def _sync_once(self):
        self.ser.flush_input()
        self.ser.write(GET_SYNC + EOC)
        insync = self.ser.read(1, timeout=0.3)
        if insync != INSYNC:
            raise FlashError("bootloader 无响应（不在烧录模式？）")
        return self.ser.read(1, timeout=0.3)

    def _get_sync(self):
        if self.ser.read(1, timeout=1.0) != INSYNC:
            raise FlashError("协议失步（缺 INSYNC）")
        code = self.ser.read(1, timeout=1.0)
        if code != OK:
            raise FlashError(f"bootloader 返回错误 {code.hex()}")
        return code

    def _get_info(self, prop):
        self.ser.write(GET_DEVICE + prop + EOC)
        val = struct.unpack("<I", self.ser.read(4))[0]
        self._get_sync()
        return val

    # ---- 流程 ----
    def sync(self, tries=20):
        """循环尝试同步，兼容"飞控还在跑应用、刚发完重启进 bootloader 命令"的窗口期。"""
        for i in range(tries):
            try:
                code = self._sync_once()
                if code in (OK, FAILED, INVALID):
                    return
            except (FlashError, OSError):
                pass
            self._report(0, f"等待 bootloader 同步…({i + 1}/{tries})")
            time.sleep(0.3)
        raise FlashError("无法与 bootloader 同步。请确认飞控已进入烧录模式"
                         "（可通过本工具的【重启进 Bootloader】，或重新上电飞控）")

    def identify(self):
        self.bl_rev = self._get_info(INFO_BL_REV)
        if not 2 <= self.bl_rev <= 5:
            raise FlashError(f"不支持的 bootloader 协议版本 {self.bl_rev}")
        self.board_id = self._get_info(INFO_BOARD_ID)
        self.fw_maxsize = self._get_info(INFO_FLASH_SIZE)
        self._report(5, f"bootloader rev{self.bl_rev}, 板型 {self.board_id}, "
                        f"flash 上限 {self.fw_maxsize // 1024}KB")

    def erase(self):
        self.ser.write(CHIP_ERASE + EOC)
        deadline = time.time() + 30
        while time.time() < deadline:
            self._report(5 + int(20 * (1 - (deadline - time.time()) / 30)), "擦除 flash…")
            try:
                insync = self.ser.read(1, timeout=0.5)
            except FlashError:
                continue
            if insync == INSYNC:
                code = self.ser.read(1, timeout=1.0)
                if code == OK:
                    self._report(25, "擦除完成")
                    return
                raise FlashError(f"擦除失败 {code.hex()}")
        raise FlashError("擦除超时（30s）")

    def program(self, image):
        groups = [image[i:i + PROG_MULTI_MAX] for i in range(0, len(image), PROG_MULTI_MAX)]
        for i, g in enumerate(groups):
            self.ser.write(PROG_MULTI + bytes([len(g)]) + bytes(g) + EOC)
            self._get_sync()
            if i % 16 == 0 or i == len(groups) - 1:
                self._report(25 + int(60 * (i + 1) / len(groups)),
                             f"写入 {i + 1}/{len(groups)} 块")

    def verify(self, image):
        if self.bl_rev < 3:
            self._report(95, "bootloader 过旧，跳过 CRC 校验")
            return
        expect = image_crc(image, self.fw_maxsize)
        self.ser.write(GET_CRC + EOC)
        time.sleep(0.5)
        got = struct.unpack("<I", self.ser.read(4, timeout=3))[0]
        self._get_sync()
        if got != expect:
            raise FlashError(f"CRC 校验失败：期望 {expect:#x}，读到 {got:#x}。"
                             "固件可能已损坏，请重新烧录")
        self._report(95, "CRC 校验通过")

    def reboot(self):
        self.ser.write(REBOOT + EOC)
        if self.bl_rev >= 3:
            self._get_sync()

    def flash(self, fw, force=False):
        """完整烧录流程。fw 为 load_px4_file() 的返回。"""
        try:
            self.ser = _Serial(self.dev)
            self.sync()
            self.identify()
            if fw["board_id"] is not None and fw["board_id"] != self.board_id and not force:
                raise FlashError(
                    f"固件与板型不匹配：固件 board_id={fw['board_id']}，"
                    f"飞控 board_id={self.board_id}。确认选对了固件；"
                    f"确实要强制烧录再加 force")
            if fw["image_size"] > self.fw_maxsize:
                raise FlashError("固件超出该板 flash 上限")
            self.erase()
            self.program(fw["image"])
            self.verify(fw["image"])
            self.reboot()
            self._report(100, "烧录完成，飞控重启中")
        finally:
            if self.ser:
                try:
                    self.ser.close()
                except OSError:
                    pass
                self.ser = None
