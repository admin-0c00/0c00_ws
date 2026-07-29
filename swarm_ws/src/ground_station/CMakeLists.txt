# SPDX-License-Identifier: GPL-3.0-only
# Copyright (C) 2026 零创无穷（成都）科技有限责任公司
# 本文件是 SwarmCore-Sim 的一部分，
# 依据 GNU GPL v3 发布（协议全文见仓库根目录 LICENSE）。
# 本软件按"现状"提供，不附带任何明示或默示担保。

cmake_minimum_required(VERSION 3.8)
project(ground_station)

find_package(ament_cmake REQUIRED)

install(DIRECTORY web/
  DESTINATION share/${PROJECT_NAME}/web
)

install(PROGRAMS scripts/start_ground_station.sh scripts/stop_ground_station.sh
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
