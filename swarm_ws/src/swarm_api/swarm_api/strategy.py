# SPDX-License-Identifier: LGPL-3.0-only
# Copyright (C) 2026 零创无穷（成都）科技有限责任公司
# 本文件是 SwarmCore-Sim 的一部分，
# 依据 GNU LGPL v3 发布（协议全文见 swarm_api/LICENSE）。
# 本软件按"现状"提供，不附带任何明示或默示担保。

"""策略插件基类（对应产品定义书 4.6：状态机 + 策略插件）

集群算法以统一接口注册，便于：
- 客户的新算法与内置基线在相同环境、相同指标下公平对比
- 仿真回归测试把每个策略当用例跑

写一个自己的策略只需要继承 Strategy 并实现 run()：

    from swarm_api import Strategy

    class MyFormation(Strategy):
        name = "my_formation"

        def run(self, swarm):
            swarm.takeoff(1.5)
            swarm.goto_formation("triangle", spacing=2.0, z=1.5)
            # ... 你的算法 ...

    if __name__ == "__main__":
        MyFormation().main(num_drones=3)   # 起飞/异常处理/降落/收尾全部托管
"""

import traceback
from abc import ABC, abstractmethod


class Strategy(ABC):
    """集群策略插件基类。生命周期：setup -> run -> teardown（异常也会执行 teardown）。"""

    name = "unnamed"

    def setup(self, swarm):
        """run 之前调用，可重写（默认无操作）。例如订阅传感器、加载地图。"""

    @abstractmethod
    def run(self, swarm):
        """策略主体。在这里调用 swarm 的 takeoff/goto_all/... 实现算法。"""

    def teardown(self, swarm):
        """run 结束或异常后调用（此时飞机已降落/悬停），可重写做清理。"""

    def main(self, num_drones=None, namespaces=None):
        """托管执行：发现飞机 -> setup -> run -> 异常则全群悬停 -> 尝试降落 -> teardown。

        直接 python3 my_strategy.py 即可运行，Ctrl+C 会触发降落。
        """
        from .swarm import Swarm  # 延迟导入，避免循环依赖

        swarm = Swarm(num_drones=num_drones, namespaces=namespaces)
        print(f"[{self.name}] 发现 {len(swarm)} 架飞机: {swarm.namespaces}")
        try:
            self.setup(swarm)
            self.run(swarm)
        except KeyboardInterrupt:
            print(f"[{self.name}] 用户中断，全群降落")
        except Exception:
            print(f"[{self.name}] 策略异常，全群悬停后降落：")
            traceback.print_exc()
            try:
                swarm.hover()
            except Exception:
                pass
        finally:
            try:
                swarm.land()
            except Exception as e:
                print(f"[{self.name}] 降落阶段异常（检查飞控状态）: {e}")
            self.teardown(swarm)
            swarm.shutdown()
        print(f"[{self.name}] 结束")
