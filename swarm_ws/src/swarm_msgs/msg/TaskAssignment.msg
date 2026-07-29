# SPDX-License-Identifier: GPL-3.0-only
# Copyright (C) 2026 零创无穷（成都）科技有限责任公司
# 本文件是 SwarmCore-Sim 的一部分，
# 依据 GNU GPL v3 发布（协议全文见仓库根目录 LICENSE）。
# 本软件按"现状"提供，不附带任何明示或默示担保。

# SwarmCore 任务分配消息（见产品定义书 附录A）
std_msgs/Header header
uint32 task_id
uint8 assignee_type    # 0 无人机 1 无人车
uint8 assignee_id
uint8 task_type        # 0 确认 1 跟踪 2 接近 3 待命
uint32 target_id
geometry_msgs/Pose goal
