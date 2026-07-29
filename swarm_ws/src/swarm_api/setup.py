# SPDX-License-Identifier: LGPL-3.0-only
# Copyright (C) 2026 零创无穷（成都）科技有限责任公司
# 本文件是 SwarmCore-Sim 的一部分，
# 依据 GNU LGPL v3 发布（协议全文见 swarm_api/LICENSE）。
# 本软件按"现状"提供，不附带任何明示或默示担保。

from setuptools import setup

package_name = 'swarm_api'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='SwarmCore',
    maintainer_email='dev@lcw.local',
    description='SwarmCore 集群控制框架：多机并行的位置/速度控制 API',
    license='LGPL-3.0-only',
)
