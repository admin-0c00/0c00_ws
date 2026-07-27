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
## 2026-07-27 Web 地面站按钮可靠化（基于 swarm_api）

- **新增控制后端 `web_control_node.py`**：按钮指令不再由网页直发单条 VehicleCommand，改为发 JSON 到 `/web_control/cmd`，后端用 swarm_api 执行（命令重发直到状态确认、起飞走完整 Offboard 流程、每机互斥锁防并发、结果回执 `/web_control/result`）。start/stop 脚本已集成。
- **前端**：按钮显示"执行中/成功/失败"提示并防重复点击；围栏自动动作保留原直发通道。
- **框架新增**：`Drone.arm/disarm/rtl` + `nav_state` 属性。
- **两个实测抓到的缺陷**：
  1. 网页围栏 15s 冷却期重复发 NAV_RTL 会让 navigator 不断重启返航，飞机在边界永远落不了地——前端修复：nav_state 已在目标模式时不再重复下发；
  2. RTL 后 Offboard 设定点流不断会干扰返航-降落衔接，飞机悬在返航点上方——`rtl()` 现在与 `land()` 一样先停流（再 takeoff 自动重启）。已端到端验证 takeoff→RTL→落地→再起飞→降落。
## 2026-07-27 指点飞行 + 记录页增强 + 品牌素材

- **指点飞行**：顶栏"指点飞行"开关 → 点击卡片选飞机（高亮边框）→ 单击 3D 地面，射线与等高水平面求交得 ENU 目标点，换算到该机本地系后经 `/web_control/cmd` 的 goto 动作执行 `Drone.goto()`。目标点在围栏外拒绝；金色圆锥标记到达/失败后自动移除；拖拽旋转不触发（位移 >5px 判定）。后端 web_control_node 增加 goto 动作。全链路实测 takeoff→goto(2,2,1.5)→goto(0,0,1.5)→land 通过。
- **记录页**：历史 bag 列表（名称/大小/删除，总占用 >20GB 提醒）；实验备注写入 bag 的 metadata.json（对应产品定义书附录 D）；预设话题组合（评估标准集/全选/清空）；无飞机时的提示补充了启动仿真说明。recorder_node 增加 delete 动作（名称白名单防穿越）与 bags/total_mb 状态字段。
- **品牌**：接入 0C∞ 渐变 logo（暗夜黑版横版）到页头，favicon 补齐（修掉 404），页头加 © 零创无穷。
- **修正**：指点飞行首发版本把目标点按 NED 顺序传给 ENU 接口（点东飞北），已改为显式 localE/localN 映射并加注释。
## 2026-07-27 上锁（锁桨）语义修正

- **问题**：网页"上锁"在飞行中点击无反应——PX4 拒绝空中普通 disarm，后端重试 10s 才超时，且确认框承诺"停桨"与实际行为不符。
- **修复**：`Drone.disarm` 前一半时间普通上锁，被拒后自动升级强制上锁（kill，param2=21196）。实测：地面上锁 0.6s，空中 5.6s 强制停桨。
- **命名消歧**：按钮改为"解锁电机/上锁电机"（含全局按钮与提示文案），确认框明确"空中=强制停桨=坠落"。
## 2026-07-27 仓库结构精简

- 删除 11 个纯空壳骨架包（perception_*、safety_guard、swarm_fusion、swarm_task、ugv_bridge、uwb_driver、evaluation、px4_bridge）——只有 package.xml+CMakeLists、无任何引用，属产品定义书 7.1 的预建规划。原则改为"代码即现状，文档即规划"：子系统开发时才按定义书建包。
- 现有 6 个包：bringup、swarm_api、ground_station、swarm_msgs（TargetMap/TaskAssignment）、px4_msgs、px4_ros_com。px4_bridge 职责已由 swarm_api + web_control_node 实际承担。
- README 目录结构、NOTICE 自研代码清单同步修订。
- **顺带抓到框架级 bug**：takeoff 失败（如 preflight 未过）后设定点流不会停止，残留的 20Hz 流会一直干扰 PX4 后续返航/降落（飞机悬着落不下来）。已修：takeoff 失败先停流再抛错。回归通过。
## 2026-07-27 记录页空白根治 + 清除轨迹按钮

- **记录页从上线起就一直空白**：switchTab 用 `style.display=''` 显示面板，但 #tab-record 样式表规则是 display:none，清内联样式后回落为 none——两张面板同时隐藏。status 页没有 CSS display 规则所以正常，掩盖了 bug。修为显式 'block'/'none'。
- 顶栏新增"清除轨迹"按钮：一键清空所有飞机轨迹线（drawRange 归零），不用刷新页面。
## 2026-07-27 记录页话题列表动态化

- 话题列表不再写死 5 个：通过 rosapi（rosbridge 自带）动态枚举 ROS 图全部话题，按命名空间分组渲染；图像/点云类标注 ⚠大流量。视觉等新话题上线后点"刷新话题"即可出现，无需改代码。
- 勾选结果存 localStorage（刷新不丢）；无历史选择时默认评估标准集；新增"刷新话题"按钮。后端 recorder_node 无需改动（本就接受任意话题名）。
## 2026-07-27 数据页（分析/回放/导出）+ 幽灵话题过滤

- **新增"数据"标签页**：bag 卡片显示时长/消息数/大小/备注；回放（ros2 bag play，含停仿真确认框）、导出 CSV（通用实现：rosidl 反序列化 + message_to_ordereddict，全字段成列，不限消息类型）、zip 下载（web_server 新增 /bags/ 路由，防穿越）、删除。
- **幽灵话题根治**：rosapi 的 /rosapi/publishers 只支持单话题查询（空响应的坑），改为 recorder_node 用节点图 API count_publishers 枚举"有发布者"的话题，随 /recorder/status 10s 下发；页面预订阅的 uav_4~6 不再出现。
- 事故记录：编辑失误把 __init__ 尾部并入 _refresh_rec_topics 导致节点不发布（前台运行+日志定位修复）。
## 2026-07-27 地面站内置数据曲线

- 数据页 bag 卡片新增"查看数据"：选话题后后端 _series_worker 提取全部数值字段（降采样 ≤2000 点）发 /recorder/series，前端用内嵌 uPlot（离线库 50KB）在 3D 视图左下角画时序曲线，字段可勾选组合。不依赖 PlotJuggler/CDN。
- bag_brief 增加话题清单（metadata.yaml 解析），卡片上直接选话题。
- **曲线加载提速**：series 原实现全库扫描+全消息反序列化（大 bag 极慢），改为 metadata 预算 stride + StorageFilter 存储层过滤，只转换所需帧；2018 条话题实测 0.2s。
- **下拉框秒缩修复**：数据页 1Hz 状态刷新会重建卡片 HTML，销毁打开中的 select——改为签名对比，内容（bag 清单/回放/导出状态）变化才重渲染。
- **加载中超时兜底**：曲线结果 15s 未到显示"请 Ctrl+F5"（旧缓存页面收不到 series 订阅结果时的明确指引）。经 ws 中转实测端到端 0.02s，"很慢"实为旧页面。
- **曲线结果丢失根治**：rosbridge 按订阅代际投递，用户连接频繁闪断重连（rosbridge 日志可见每秒级重订阅），一次性结果发到死连接上即丢失。改为超时自动重试 2 次（新连接上重发请求必达）；另加全局 JS 错误 toast 与"已收到 N 点，绘图中"阶段埋点，区分"没收到"与"绘图崩"。
## 2026-07-27 曲线大消息丢失根治（HTTP 下载架构）

- **根因链**：vehicle_status 秒开、vehicle_local_position 必丢——rosbridge 订阅是 best-effort QoS，~600KB 以上大消息在 DDS 层静默丢（UDP 缓冲溢出无重传，可靠订阅会 NACK 重传所以 rclpy 能收到）。现象随时间/负载漂移，极具迷惑性（曾误判为缓存、连接闪断、帧上限）。
- **修复**：series 结果写 bag 目录 series.json，rosbridge 只发 200 字节"就绪"通知（含 url），前端 fetch 下载——彻底绕开 DDS 大消息问题，任意大小都可靠。
- 直线问题：剔除 timestamp/timestamp_sample 字段（1e15 量级压扁 Y 轴）。
- web_server /bags 路由与 32MB max_size 保留（防御性）。
