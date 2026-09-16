# 装甲击打检测模块（Armor_Cmzy26_081）

## 作用与三板边界

本工程运行在每块装甲板 STM32L432 上，负责 ADS131M04 采样、击打判定、板级故障与灯光；不维护机器人 HP，不连接 ESP32，也不直接向比赛服务器发送数据。

```text
装甲板 ×4
  │  有效 HIT / 故障 / 状态
  └── CAN1，500 kbps，标准 11 位 ID ──> L431PM
                                            │ USART2 二进制状态/事件
                                            ▼
                                         ESP32-S3 ── Wi-Fi UDP ──> 服务器
```

## 当前已实现的联调接口

- 四块装甲板使用相同固件，通过 `0x120~0x123` 自动枚举并由 L431PM 分配 NodeID `1~4`。
- 分配完成后，业务 CAN ID 为 `0x130 + (NodeID - 1) × 0x10 + offset`。
- 一次有效击打通过 `offset=0x03`、DLC=8 上报；Byte0=通道、Byte1=强度、Byte2~3=力、Byte4~7=峰值，均以 [Docs/CAN_PROTOCOL.md](Docs/CAN_PROTOCOL.md) 为准。
- 命中包由 L431PM 的 `app_referee_can.c` 统计；L431PM 的 `app_match.c` 在存活时每收到一次 HIT 扣 20 HP，并决定死亡与 5 秒复活。装甲板不得自行扣血、判死或发送 ESP32/UDP 事件。
- 节点状态查询、LED 控制、故障和身份确认均已在 `App/comm/board_comm.c` 中实现。

## 当前联调结论

装甲板和 L431PM 的 CAN ID、DLC、字节序、枚举 ACK 标记 `0xA5` 已在两边代码中一致。联调前提是四块板全部完成枚举；未完成枚举时本工程按设计不处理业务帧。

与 ESP32 没有直接物理或协议接口。当前装甲板只负责把命中事实通过 CAN 交给 L431PM；L431PM 再通过 USART2 交给 ESP32，ESP32 转换为 V2 UDP。装甲板不应为 ESP32 的 UDP 或 USART2 协议修改 CAN 帧。

## 关键文档与操作

- [App/README.md](App/README.md)：应用层目录、构建和中断约束。
- [Docs/CAN_PROTOCOL.md](Docs/CAN_PROTOCOL.md)：唯一 CAN 协议定义。
- [Docs/装甲模块固件设计方案.md](Docs/装甲模块固件设计方案.md)：硬件、采样与检测算法设计。

烧录或改动 CubeMX 前先阅读 `App/README.md`：应用代码位于 `App/`，CubeMX 重生成会影响 `Core/` 与 `Makefile`，不应直接覆盖应用层文件。
