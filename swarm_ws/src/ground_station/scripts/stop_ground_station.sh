#!/bin/bash
# SPDX-License-Identifier: GPL-3.0-only
# Copyright (C) 2026 零创无穷（成都）科技有限责任公司
# 本文件是 SwarmCore-Sim 的一部分，
# 依据 GNU GPL v3 发布（协议全文见仓库根目录 LICENSE）。
# 本软件按"现状"提供，不附带任何明示或默示担保。

# SwarmCore Web 地面站停止
pkill -f "rosbridge_websocket" 2>/dev/null
pkill -f "web_server.py" 2>/dev/null
pkill -f "http.server 8080" 2>/dev/null
pkill -f "recorder_node.py" 2>/dev/null
pkill -f "web_control_node.py" 2>/dev/null
sleep 1
echo "[ground_station] 已停止"
