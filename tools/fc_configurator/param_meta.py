# SPDX-License-Identifier: GPL-3.0-only
# Copyright (C) 2026 零创无穷（成都）科技有限责任公司
# 本文件是 SwarmCore-Sim 的一部分，
# 依据 GNU GPL v3 发布（协议全文见仓库根目录 LICENSE）。
# 本软件按"现状"提供，不附带任何明示或默示担保。

"""PX4 参数元数据提取

从本工作区 PX4 构建产物 build/*/parameters.xml(.xz) 提取参数元数据
（描述/分组/范围/单位/枚举值/是否需重启），供 UI 做搜索、分组和输入校验。

找不到元数据时优雅降级（返回空表，UI 显示裸参数表）。
"""

import gzip
import lzma
import xml.etree.ElementTree as ET
from pathlib import Path

PX4_BUILD_DIR = Path(__file__).resolve().parents[2] / "PX4-Autopilot" / "build"


def _find_xml():
    if not PX4_BUILD_DIR.is_dir():
        return None
    for name in ("parameters.xml", "parameters.xml.xz", "parameters.xml.gz"):
        hits = sorted(PX4_BUILD_DIR.glob(f"*/{name}"))
        if hits:
            return hits[0]
    return None


def _open(path):
    if path.suffix == ".xz":
        return lzma.open(path, "rt", encoding="utf-8")
    if path.suffix == ".gz":
        return gzip.open(path, "rt", encoding="utf-8")
    return open(path, encoding="utf-8")


def load():
    """返回 {name: meta dict}；失败返回 {}。"""
    path = _find_xml()
    if not path:
        return {}
    out = {}
    with _open(path) as f:
        root = ET.parse(f).getroot()
    for group in root.iter("group"):
        gname = group.get("name", "")
        for p in group.iter("parameter"):
            def txt(tag):
                el = p.find(tag)
                return (el.text or "").strip() if el is not None else None
            meta = {
                "group": gname,
                "type": p.get("type", "FLOAT"),
                "default": p.get("default"),
                "short_desc": txt("short_desc"),
                "long_desc": txt("long_desc"),
                "min": txt("min"), "max": txt("max"),
                "unit": txt("unit"), "increment": txt("increment"),
                "decimal": txt("decimal"),
                "reboot_required": (txt("reboot_required") == "true"),
                "volatile": p.get("volatile") == "true",
            }
            values = p.find("values")
            if values is not None:
                meta["values"] = [{"code": v.get("code"), "label": (v.text or "").strip()}
                                  for v in values.iter("value")]
            bitmask = p.find("bitmask")
            if bitmask is not None:
                meta["bitmask"] = [{"index": b.get("index"), "label": (b.text or "").strip()}
                                   for b in bitmask.iter("bit")]
            out[p.get("name")] = meta
    return out


if __name__ == "__main__":
    import json, sys
    meta = load()
    print(f"提取 {len(meta)} 条参数元数据", file=sys.stderr)
    name = sys.argv[1] if len(sys.argv) > 1 else "MPC_XY_VEL_MAX"
    print(json.dumps(meta.get(name, "未找到"), ensure_ascii=False, indent=2))
