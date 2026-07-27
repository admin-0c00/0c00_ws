# SwarmCore 维护记录

> 记录每次重要更新的内容与原因，供维护者快速回溯。只记关键变更，细节以 git log 为准。
> 工作流：在 `~/0c00_ws`（开发主工作区）修改验证 → 同步进 `SwarmCore-Sim`（发行仓库）→ 推送 Gitee + 内网 Gitea。

## 2026-07-26 安装与发行链路修复

- **install.sh 两处致命修复**：
  - 步骤 6 误用 `make px4_sitl gz_x500`（编译完会直接启动仿真、脚本永不退出），改为 `make px4_sitl_default` 仅编译；
  - `set -u` 导致 `source /opt/ros/humble/setup.bash` 报未绑定变量退出，source 前后加 `set +u/-u` 保护。（`75d9fc18`）
- **拍平仓库补齐被 .gitignore 吞掉的源码**：`pymavlink/generator/C/`（`3dc0c64b`）、`Micro-XRCE-DDS-Client/src/c/core/log/`（`5eb2579c`）——克隆仓库编译缺文件的根因。
- 依赖补充：`websockets>=13`（单端口 web 服务用）。
- README：路径示例改为实际目录名 `0c00_ws`；clone 地址写为 Gitee 实际地址（`d3b888cc`）。

## 2026-07-26 Web 地面站（ground_station）

- **标记与姿态**：竖直圆锥改为 X 型四旋翼 + 机头指示；订阅 `vehicle_attitude`，NED 四元数正确转换到 three.js 坐标显示真实姿态；桨叶解锁旋转/上锁停转。
- **控制通道**：卡片按钮（解锁/起飞/返航/降落/上锁）+ 顶栏全局按钮；`VehicleCommand` 经 rosbridge 下发（链路实测 ACK）。起飞按钮内置"先解锁 1s 后起飞"（PX4 未解锁时 NAV_TAKEOFF 不执行）；起飞高度可填（默认 1.5m，param7=各机 ref_alt+h）。
- **电子围栏（双层）**：web 侧全息围栏（R10m×H6m，圆心=uav_1 起始位置），越界变红告警并按可选动作（关/悬停/降落/返航）自动下发命令；飞控侧各实例写入 `GF_ACTION=3`、`GF_MAX_HOR_DIST=10`、`GF_MAX_VER_DIST=6`（`GF_ACT` 环境变量可改）兜底。已端到端验证越界自动返航。
- **坐标系**：原点指示改为 ROS 风格右手系（X东/Y北/Z上）；XY 位置支持 `GPS 共享`（经纬度换算共享平面）/ `本机/UWB`（本地坐标即全局）两种模式切换，localStorage 持久化；高度统一用本地 `-z`。
- **稳定性**：
  - 话题订阅加 `throttle_rate` 限流（PX4 话题 ~100Hz，全速转发会把 rosbridge 压到 100% CPU）（`516329b8`）；
  - 修复 rosbridge 重启/断线后 roslib 不恢复订阅导致"已连接但无数据"（`63d313d5`）；
  - 修复 `const h` 重复声明导致页面脚本整体解析失败（`049673ac`）；
  - **单端口化**：`web_server.py` 在 8080 同时提供静态页与 `/ws` 的 rosbridge 中转（部分网络拦截 9090 的根治方案）（`a93c187a`）。
- **"记录"标签页**：勾选话题 → recorder_node 调 `ros2 bag record` 只录所选（`4a30976b`）。

## 2026-07-26 PX4 仿真侧

- **pxh> 刷屏根治**：PX4 的 stdin 由 `/dev/null`（EOF 导致 nsh 死循环刷提示符，几小时几十 GB）改为 `tail -f /dev/null`；仿真日志只保留最近 5 次。
- **GPS 高度噪声修复（治本）**：`SensorGpsSim.cpp` 高度白噪声 0.5m→0.02m，上报精度 eph/epv 同步下调——这是各机 EKF 高度原点各漂、地面站显示"有的飞机在地面以下"的根因（`c188a022`）。
- 参数：`RTL_RETURN_ALT` 默认 0（返航不爬升，`RTL_ALT` 可改）；`EKF2_HGT_REF`/`EKF2_GPS_CTRL` 显式写出（PX4 参数持久化，历史改动必须显式覆盖）。
- 光流定高结论：本版 PX4 的 gz 桥接不支持光流/测距传感器，改模型文件无效；以干净 GPS 高度近似真机 UWB/光流精度。

## 2026-07-27 演示程序与文档

- **demo_square.py（NED）/ demo_square_enu.py（ENU）**：起飞→向前 2m→顺时针 2m 正方形→回原点→降落。新手友好线性结构（~120 行）；高度基准为"起飞前实测高度 ± 爬升量"，免疫 EKF 高度原点偏差（`4f2e5c28`）。
- **官网教程手册** `wiki/swarmcore-demo-tutorial.md`：安装、一分钟跑通、demo 逐段讲解（Offboard 铁律、NED/ENU 对照、QoS 坑）、地面站指南、参数速查、FAQ、真机要点、虚拟机配置专题。
- **虚拟机结论**：最低 4 vCPU / 8GB、无头模式、关闭 VMware Tools 时间同步；性能不足症状已写入手册第 2 节与 FAQ Q7。

## 运维注意事项

- **PX4 参数是持久化的**：QGC 或历史脚本改过的参数会残留，`start_swarm_sim.sh` 必须把每个改动项显式写出。
- **rosbridge 脆弱性**：高负载会饿死（100% CPU）甚至卡死不响应握手；保持页面限流、关闭 QGC 等无关重负载进程。
- **页面异常先 Ctrl+F5**：浏览器缓存是本项目的头号"假故障"来源。
- **真机待办**：UWB/光流定位接入 EKF、机载 bringup、swarm_fusion/swarm_task 填实、safety_guard 策略、HITL 过渡。详见教程手册"从仿真到真机"一节。
## 2026-07-27 swarm_api 集群控制框架

- **新增 `swarm_ws/src/swarm_api` 包**：集群控制框架，把 QoS、ENU↔NED 转换、Offboard 20Hz 设定点流、命令重发全部封装。三层结构：`Drone`（单机原语 takeoff/goto/set_velocity/hover/land）→ `Swarm`（每机一线程并行、自动发现命名空间、异常隔离、编队 line/column/triangle/grid）→ `Strategy`（策略插件基类，对齐产品定义书 4.6）。
- **两个实测中抓到的问题**：land 时设定点流未停导致 PX4 拒绝 NAV_LAND 悬停不落（已修，降落前先断流）；goto 增加 Offboard 丢失 2s 快速报错（仿真 failsafe 场景不再傻等 60s）。
- **示例**：`demo_single_drone.py`（单机，80 行）/ `demo_swarm_square.py`（三机，67 行），3 机仿真位置控制与速度控制均实测通过。
- **wiki 教程** `swarmcore-swarm-api-tutorial.md`：API 参考、常见用法、Strategy 写法、安全机制、真机迁移、FAQ（官网上传由负责人处理）。
- README：新增框架章节；修正仿真第 2 个参数说明写反（实际 0=GUI、1=无头）。
## 2026-07-27 开源协议

- **自研代码采用 Apache License 2.0**：根目录新增 `LICENSE`（协议全文）与 `NOTICE`（版权+商标声明+第三方组件清单）。选型理由：与 PX4(BSD-3)/ROS 2(Apache-2.0) 生态兼容；自带专利授权+反制条款与商标保护条款，符合产品定义书 13.1"宽松协议+商业双轨"策略。
- **`docs/OPEN_SOURCE_LICENSE.md`**：中文协议导读（权利义务速查、商标/专利保护、五层原创保护体系、第三方合规、贡献条款、违规联系）。
- 15 个自研功能包的 `package.xml` license 字段由 BSD-3-Clause 统一改为 Apache-2.0；px4_msgs/px4_ros_com 为 PX4 官方包，保留 BSD 不动。
- README 新增"开源协议"章节。
