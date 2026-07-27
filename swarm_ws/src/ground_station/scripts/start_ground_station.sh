#!/bin/bash
# SwarmCore Web 地面站一键启动
# 依赖: ros-humble-rosbridge-suite；页面: http://localhost:8080
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WEB_DIR="$DIR/../web"
[ -d "$WEB_DIR" ] || WEB_DIR="$HOME/0c00_ws/swarm_ws/src/ground_station/web"

source /opt/ros/humble/setup.bash
# rosbridge 需要能找到 px4_msgs 等自定义消息类型
[ -f "$HOME/0c00_ws/swarm_ws/install/setup.bash" ] && source "$HOME/0c00_ws/swarm_ws/install/setup.bash"

# 分别检查 rosbridge 与 http 服务，缺哪个起哪个
if pgrep -f "rosbridge_websocket" > /dev/null; then
    echo "[ground_station] rosbridge 已在运行 (ws://localhost:9090)"
else
    nohup ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:=9090 \
        > /tmp/rosbridge.log 2>&1 &
    echo "[ground_station] rosbridge 已启动 (ws://localhost:9090)"
fi

if pgrep -f "web_server.py" > /dev/null; then
    echo "[ground_station] Web 页面已在运行: http://localhost:8080"
else
    nohup python3 "$DIR/web_server.py" > /tmp/gs_http.log 2>&1 &
    echo "[ground_station] Web 页面已启动: http://localhost:8080 (ws 走同端口 /ws 中转)"
fi

# 话题记录节点（web“记录”标签页的后端）
if pgrep -f "recorder_node.py" > /dev/null; then
    echo "[ground_station] 话题记录节点已在运行"
else
    nohup python3 "$DIR/recorder_node.py" > /tmp/gs_recorder.log 2>&1 &
    echo "[ground_station] 话题记录节点已启动"
fi

# 控制后端（web 按钮的可靠动作执行，基于 swarm_api）
if pgrep -f "web_control_node.py" > /dev/null; then
    echo "[ground_station] 控制后端已在运行"
else
    nohup python3 "$DIR/web_control_node.py" > /tmp/gs_control.log 2>&1 &
    echo "[ground_station] 控制后端已启动 (/web_control/cmd)"
fi
