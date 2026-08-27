# SPDX-License-Identifier: GPL-3.0-only
# Copyright (C) 2026 零创无穷（成都）科技有限责任公司

# 通信模块配置工具

替代 Windows 版 NetModuleConfig.exe 的局域网通信模块（CH9121 串口转以太网）配置工具（Linux/macOS 可用，纯 Python 标准库，零依赖）。

## 启动

```bash
python3 tools/comm_module_config/server.py       # 默认 8081 端口
# 浏览器打开 http://localhost:8081
```

## 使用

1. 右上角选择**接模块的网卡**（多网卡机器必看，选错搜不到）；
2. 【搜索设备】→ 左侧列表选中模块 → 【读取参数】；
3. 修改基础设置 / 端口 1 / 端口 2 参数 → 【写入配置】；
4. 模块自动重启，约 3 秒后重新【搜索设备】确认生效。

【恢复出厂设置】有确认框，谨慎使用。

## HTTP API

`server.py` 对外提供 JSON API（统一响应 `{ok: true, ...}` / `{ok: false, error}`，参数错误 HTTP 400）：

| 方法 | 路径 | 请求体 | 响应 |
| --- | --- | --- | --- |
| GET | `/api/nics` | — | `{nics: [{name, ip, mac}]}` |
| POST | `/api/search` | `{nic}` | `{devices: [{mac, ip, name, version}]}` |
| POST | `/api/get_config` | `{nic, mac}` | `{config: {...}}`（基础设置 + port1/port2） |
| POST | `/api/set_config` | `{nic, mac, config}` | `{msg}`（先读后写保留 reserved，模块写后自动重启） |
| POST | `/api/factory_reset` | `{nic, mac}` | `{msg}` |

`config` 字段与 `ch9121.py` 的 `parse_config()` 输出一一对应；写配置只需在 `get_config` 读回的基础上改字段后整体提交。

## 协议说明

- UDP 广播配置：模块端口 50000，本机绑定 60000（被占用时会报错，先关闭其他配置软件）。
- 报文格式与结构体定义见 `ch9121.py` 头部注释。
- 若模块被设为"禁止网络配置"（串口命令 0x52），本工具将无法搜索/配置它，需用串口通道恢复（拉低 CFG 脚）。
