# 工业网关（STM32F407 + FreeRTOS + LVGL）

RS485 温湿度采集 → 本地存储 → 云端上报 → OTA 升级的一体化网关。
带 3.2 寸触摸屏，采集、链路、存储状态全部上屏，支持断网补传与固件回滚。

## 功能

- **采集**：Modbus RTU 主站，RS485 挂 HZ8706 温湿度变送器（地址 1 / 9600 8N1）
- **显示**：LVGL 9.5 三页界面（首页 / 设置 / 关于），滑动或 KEY2 翻页；屏上可扫描并输入 WiFi 凭据
- **存储**：W25Q128 上按 4KB 扇区做环形日志区，掉电续号、已发/未发标记落盘
- **上报**：ESP-01S AT 透传 + 手写 MQTT 3.1.1（CONNECT / PUBLISH / PINGREQ），连巴法云
- **补传**：断网期间数据留在 Flash，重连后按 200ms 间隔限速追平，避免灌满 ESP 串口缓冲
- **时间**：LSE + VBAT 的 RTC，联网后 UDP 直发 NTP 校时；未校准时界面显示 `--:--`，不显示假时间
- **OTA**：屏上弹窗显示下载进度，含黄金备份与坏固件自动回滚（bootloader 独立工程）

## 目录

```
gw/
├── boot/        Bootloader：校验暂存区、安装、try 计数、回滚
├── gwbase/      应用工程（Keil MDK AC5）
│   ├── APP/     任务调度 / UI / OTA / 配置 / 参数模型
│   ├── drv/     板级驱动：uart、modbus_rtu、rtc、spiflash、adc
│   ├── port/    移植层：net_if_esp01s（AT）、mqtt_client、lv_port
│   ├── User/    启动文件、时钟、中断
│   └── Public/  屏、字库等公共代码
└── tools/       OTA 服务器、字体生成、坏固件构造等脚本
```

## 数据链路

```
485 采集 → uart_poll_task → sample_task ─┬→ UI（最新值）
                                        ├→ 队列 → Flash 日志区（环形，带 seq/ts）
                                        └→ MQTT 上报（失败留库，重连补传）
```

上报报文：

```json
{"ts":1794288672,"temp":20.1,"humi":41.7,"seq":2163}
```

`ts` 为 UTC Unix 秒（RTC 校准后为真实时间）；`seq` 单调递增，重启不归零。

## Flash 分区与升级

| 区域 | 位置 | 用途 |
|---|---|---|
| Bootloader | 内部 Flash `0x08000000` | 校验、安装、回滚 |
| APP | 内部 Flash `0x08010000` ~ `0x08100000` | 运行区 |
| 暂存区 | 外部 Flash | OTA 下载落点，装完即校验 |
| 黄金备份 | 外部 Flash | 最近一次确认可运行的固件 |

升级流程：APP 从服务器拉固件写入暂存区并校验 CRC32 → 复位 → boot 校验暂存区 → 安装进 APP 区 → 首次安装同时写黄金备份。
若新固件起不来，boot 按 try 计数重试，达到阈值后从黄金备份回滚。

## 编译与烧录

```bash
# 编译（应用）
D:/keil5/UV4/UV4.exe -b gwbase/gwbase.uvprojx -j0 -o build.log

# 生成 bin
D:/keil5/ARM/ARMCC/bin/fromelf.exe --bin --output gwbase/Obj/gwbase.bin gwbase/Obj/gwbase.axf
```

真实 WiFi / MQTT 凭据放 `gwbase/APP/app_config_local.h`（已 gitignore），模板见 `app_config.h`。

## OTA 升级

```bash
python tools/ota_server.py --bin gwbase/Obj/gwbase.bin --version 1.0
```

服务器监听 `0.0.0.0:8888`，板上按 KEY1 触发升级，屏上弹窗显示进度与结果。

## 已知限制

- MQTT 只上行不下行：云端还不能改参数（缺 SUBSCRIBE + 下行报文解析）
- 未校准的时间在补传时仍是编译时刻兜底值，云端可能收到异常时间点
- 无纽扣电池时，每次断电都会回到编译时刻，需联网重新校准
- OTA 传输偶发握手后 0 块断开，板子会自动重连重试，通常第二次成功

## 版本

当前 **V1.0**：Modbus 采集 + Flash 存储 + MQTT 上报/补传 + RTC/NTP 时间戳 + OTA 升级与回滚。
