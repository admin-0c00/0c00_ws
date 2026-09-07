# SPDX-License-Identifier: GPL-3.0-only
# Copyright (C) 2026 零创无穷（成都）科技有限责任公司

# 飞控配置器（Web）

参照 MicoConfigurator 产品形态自研的 PX4 飞控 Web 配置工具。与它的本质区别：
**支持网络连接**——浏览器可通过 CH9121 串口转以太网模块远程配置飞控，无需 USB 线。
纯 Python 标准库 + 单页 Web UI，零依赖，离线可用。

## 启动

```bash
python3 tools/fc_configurator/server.py       # 默认 8082 端口
# 浏览器打开 http://localhost:8082
```

## 连接方式

| 场景 | 通道 | 参数 |
| --- | --- | --- |
| 真机经 CH9121 | 网络 UDP | host=模块 IP，port=14550，bind_port=0 |
| USB 直连 | USB 串口 | 选 /dev/ttyACM*，波特率 115200 |
| SITL（默认链路） | 网络 UDP | host=127.0.0.1，port=18570，bind_port=14550（QGC 占用时见下） |

**注意：用本工具连真机端口1 时请先关闭 QGC。** CH9121 UDP Server 只把串口数据
转发给最近发包的对端，本工具和 QGC 会互抢链路。

### SITL 与 QGC 并存（可选）

SITL 默认把 MAVLink 广播到 14550（QGC 占着）。想并存时给 SITL 加一条专用链路：
复制 `ROMFS/px4fmu_common/init.d-posix/px4-rc.mavlink` 到一个目录，末尾追加：

```sh
mavlink start -x -u 18571 -o 14551 -r 4000000
mavlink stream -r 10 -s ATTITUDE -u 18571
```

启动 px4 时把该目录放在 PATH 最前（rcS 按 PATH 搜 px4-rc.mavlink），
然后本工具用 port=18571、bind_port=14551 连接即可。

## 功能

- **参数**：全量拉取（丢包按 index 补缺重传）、搜索/分组过滤、描述/范围/默认值显示、
  单参数写入回读校验、JSON 导入（diff 预览后逐条写入）/导出、需重启参数标注
- **设置**：定位配置（EKF2）——列出定位源相关参数（GPS/气压计/测距/罗盘/外部定位融合/
  高度参考/无航向初始化）及中文注释（含枚举与位掩码含义），用户自行填值、只写改动项；
  安全机制（电子围栏动作/半径/高度、返航高度、低电动作、降落自动上锁）
- **校准**：陀螺仪、加速度计（六面）、磁罗盘、气压计、水平——进度条 + 飞控
  STATUSTEXT 实时提示（六面校准时跟着提示摆方向）
- **电机测试**：MAV_CMD_ACTUATOR_TEST(310)，逐个电机试转；解锁状态下禁用
- **固件**：查看版本、重启/重启进 Bootloader、上传 .px4 烧录（**仅 USB 串口通道**，
  bootloader 不走网络；烧录前强制校验板型 board_id，CRC 校验通过才算成功）
- **文件管理**：MAVLink FTP 浏览机载文件系统（真机 SD 卡在 `/fs/microsd`，
  飞行日志在 `/fs/microsd/log`），下载（后台跑，完成后浏览器自动保存）、
  上传、删除；传输为停等式逐块读（PX4 v1.15 的 BurstReadFile 实测死循环，不可用）

## HTTP API

统一响应 `{ok:true,...}` / `{ok:false,error}`：

| 方法 | 路径 | 说明 |
| --- | --- | --- |
| GET | `/api/ports` | 串口列表 + 本机网卡 |
| POST | `/api/connect` | `{channel:"udp",host,port,bind_port}` 或 `{channel:"serial",dev,baud}` |
| POST | `/api/disconnect` | 断开 |
| GET | `/api/status` | 连接状态 + 长操作进度(op) + 烧录进度(flash) + 最近 STATUSTEXT |
| POST | `/api/params/refresh` | 后台全量拉取参数 |
| GET | `/api/params` | 参数表 + 元数据（描述/范围/分组，来自 PX4 build 的 parameters.xml） |
| POST | `/api/params/set` | `{name, value}` 写参数并回读校验 |
| GET | `/api/params/export` | 下载参数 JSON |
| POST | `/api/calibrate` | `{what: gyro/mag/baro/accel/level}` |
| POST | `/api/motor_test` | `{motor:1-12, value:0~1, timeout_s}` |
| POST | `/api/reboot` | `{bootloader: bool}` |
| POST | `/api/firmware/flash` | raw body=.px4，头 `X-Flash-Dev`（串口）、`X-Flash-Force`（1=跳过板型检查） |
| POST | `/api/fs/list` | `{path}` 列目录，返回 `{entries:[{name,is_dir,size}]}` |
| POST | `/api/fs/download` | `{path}` 后台下载到 `/tmp/fc_dl_cache/`，完成后 status 的 `fs_dl.key` 可取回 |
| GET | `/api/fs/get?key=N` | 取回已下载文件（attachment） |
| POST | `/api/fs/upload` | raw body=文件内容，头 `X-Path`=机载目标路径，后台上传 |
| POST | `/api/fs/delete` | `{path}` 删除机载文件 |

## 代码结构

| 文件 | 职责 |
| --- | --- |
| `mavlink.py` | MAVLink v1/v2 帧编解码 + 消息子集（已用 pymavlink 逐字节交叉验证） |
| `transport.py` | UDP/串口（termios）统一通道 |
| `fc_client.py` | 会话层：心跳发现、参数状态机、校准流程、电机测试、FTP（列目录/停等读写/删） |
| `param_meta.py` | 从 PX4 构建产物提取参数元数据（找不到时降级为裸参数表） |
| `px4_upload.py` | PX4 bootloader 烧录（参照 px_uploader.py 重写，CRC 已逐字节对齐验证） |
| `server.py` | HTTP API（单会话锁：同一时刻只连一架飞机） |

## 验证状态

- 参数/校准（陀螺、水平）/电机测试：**SITL（gz_que）全链路实测通过**
- 文件管理：列目录/下载（2.2MB 日志字节一致）/上传/删除 **SITL 全链路实测通过**；
  真机 SD 卡路径 `/fs/microsd` 待真机验证
- 六面加速度计/磁罗盘校准：命令路径相同，SITL 无法摆姿态，需真机验证
- 固件烧录：协议与 CRC 与官方 px_uploader.py 交叉验证一致；**真机烧录待验证**
- 真机 CH9121 网络链路：待验证
