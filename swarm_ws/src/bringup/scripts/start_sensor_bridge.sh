#!/bin/bash
# SwarmCore 传感器桥接：把 Gazebo 相机/雷达话题桥到 ROS 2
# 用法: start_sensor_bridge.sh [话题前缀=]
#   默认桥接 gz_x500_depth（OakD-Lite）的话题：
#     /camera, /camera_info, /depth_camera, /depth_camera/points
#   自定义机型加传感器后，按 gz topic -l 的实际话题名改下面 BRIDGES 即可。
#
# 注意（实测踩过的坑）：
#   - 点云的 Gazebo 类型是 PointCloudPacked，不是 PointCloud，写错桥会静默失败
#     （日志里只有一行 WARN）
#   - 桥接包是 ros-humble-ros-gzgarden-bridge（Garden 专用，不是 ros-gz 默认版）
PREFIX=${1:-}

source /opt/ros/humble/setup.bash
set -u

BRIDGES=(
    "${PREFIX}/camera@sensor_msgs/msg/Image@gz.msgs.Image"
    "${PREFIX}/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo"
    "${PREFIX}/depth_camera@sensor_msgs/msg/Image@gz.msgs.Image"
    "${PREFIX}/depth_camera/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked"
)

echo "[bridge] Gazebo -> ROS 2 话题桥接启动："
printf '  %s\n' "${BRIDGES[@]}"
exec ros2 run ros_gz_bridge parameter_bridge "${BRIDGES[@]}"
