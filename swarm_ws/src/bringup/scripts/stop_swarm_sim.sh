#!/bin/bash
# SwarmCore 多机 SITL 一键停止
pkill -f "px4_sitl_default/bin/px4" 2>/dev/null
pkill -f "MicroXRCEAgent" 2>/dev/null
pkill -f "gz sim" 2>/dev/null
pkill -f "tail -f /dev/null" 2>/dev/null   # PX4 的 stdin 保持进程（防 pxh> 刷屏用）
sleep 1
echo "[swarm] 仿真进程已全部停止"
