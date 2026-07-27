#!/bin/bash
# SwarmCore Web 地面站停止
pkill -f "rosbridge_websocket" 2>/dev/null
pkill -f "web_server.py" 2>/dev/null
pkill -f "http.server 8080" 2>/dev/null
pkill -f "recorder_node.py" 2>/dev/null
pkill -f "web_control_node.py" 2>/dev/null
sleep 1
echo "[ground_station] 已停止"
