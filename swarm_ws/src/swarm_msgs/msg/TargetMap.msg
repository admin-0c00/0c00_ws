# SPDX-License-Identifier: GPL-3.0-only
# Copyright (C) 2026 零创无穷（成都）科技有限责任公司
# 本文件是 SwarmCore-Sim 的一部分，
# 依据 GNU GPL v3 发布（协议全文见仓库根目录 LICENSE）。
# 本软件按"现状"提供，不附带任何明示或默示担保。

# SwarmCore 目标地图消息（见产品定义书 附录A）
std_msgs/Header header
uint32 target_id
string label
float32 confidence
geometry_msgs/Point position
float64[9] position_covariance
uint8 source_uav_id
float32 detection_age
uint8 tracking_state
