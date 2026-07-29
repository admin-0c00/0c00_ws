#!/bin/bash
# SPDX-License-Identifier: GPL-3.0-only
# Copyright (C) 2026 零创无穷（成都）科技有限责任公司
# 本文件是 SwarmCore-Sim 的一部分，
# 依据 GNU GPL v3 发布（协议全文见仓库根目录 LICENSE）。
# 本软件按"现状"提供，不附带任何明示或默示担保。

# SwarmCore 多机 SITL 一键停止
pkill -f "px4_sitl_default/bin/px4" 2>/dev/null
pkill -f "MicroXRCEAgent" 2>/dev/null
pkill -f "gz sim" 2>/dev/null
pkill -f "nsh_pipe" 2>/dev/null          # PX4 的 stdin 保持进程（防 pxh> 刷屏/命令注入管道）
sleep 1
echo "[swarm] 仿真进程已全部停止"
