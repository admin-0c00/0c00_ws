#!/bin/bash
# SwarmCore 多机 SITL 一键停止
pkill -f "px4_sitl_default/bin/px4" 2>/dev/null
pkill -f "MicroXRCEAgent" 2>/dev/null
pkill -f "gz sim" 2>/dev/null
sleep 1
echo "[swarm] 仿真进程已全部停止"
