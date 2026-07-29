# SPDX-License-Identifier: LGPL-3.0-only
# Copyright (C) 2026 零创无穷（成都）科技有限责任公司
# 本文件是 SwarmCore-Sim 的一部分，
# 依据 GNU LGPL v3 发布（协议全文见 swarm_api/LICENSE）。
# 本软件按"现状"提供，不附带任何明示或默示担保。

"""SwarmCore 集群控制框架

from swarm_api import Swarm, Drone, Strategy

- Swarm    多机并行控制（推荐入口）
- Drone    单机控制
- Strategy 策略插件基类（编写可复现、可对比的集群算法）
"""

from .drone import Drone, DroneError, enu_to_ned, yaw_enu_to_ned
from .strategy import Strategy
from .swarm import Swarm, SwarmError, discover_namespaces

__all__ = [
    "Drone", "DroneError", "Swarm", "SwarmError", "Strategy",
    "discover_namespaces", "enu_to_ned", "yaw_enu_to_ned",
]
