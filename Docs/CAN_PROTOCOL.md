# 装甲模块 CAN 通信协议

版本：v1.4  
适用工程：`Armor_Cmzy26_081`

本文档是装甲板 CAN 报文的最终定义。协议使用经典 CAN、标准 11 位 ID、数据帧（`CAN_RTR_DATA`）、500 kbps；多字节整数均为小端序。CAN ID 越小，仲裁优先级越高。不使用 ISO-TP、TLV 或应用层封包。

## 当前生效：四板自动 NodeID

四块装甲板烧录完全相同的固件。每块板从 STM32 UID 读取固定 12 字节身份；`NodeID` 只保存在 RAM，`0` 表示未分配。未完成分配时，装甲板不发送或处理业务 CAN 帧。

| ID | 名称 | 方向 | DLC | 数据 |
|---:|---|---|---:|---|
| `0x120` | `ENUM_START` | 电管→装甲板 | 1 | Byte0=`session` |
| `0x121` | `ENUM_ANNOUNCE` | 装甲板→电管 | 8 | Byte0=`session`，Byte1=`fragment`(0~3)，Byte2~4=UID 三字节，Byte5~7=UID FNV-1a hash 低 24 bit |
| `0x122` | `NODE_ASSIGN` | 电管→装甲板 | 8 | Byte0=`session`，Byte1=`NodeID`(1~4)，Byte2~4=`token`，Byte5~6=UID CRC16-CCITT 小端，Byte7=0 |
| `0x123` | `NODE_ACK` | 装甲板→电管 | 8 | 与 `NODE_ASSIGN` 对应，Byte7=`0xA5` |

UID 按 `HAL_GetUIDw0/w1/w2` 依次转换为每个 word 小端字节。FNV-1a 计算 32-bit hash，低 24 bit 为 `token`；UID CRC16 使用 CCITT，初值 `0xFFFF`、多项式 `0x1021`。电管只在所需数量的 UID 均完整且校验通过后，按 UID 12 字节无符号字典序分配 NodeID 1~4。正式固件所需数量固定为 4；测试固件可将 L431PM 的 `APP_ARMOR_ENUM_REQUIRED_NODE_COUNT` 编译宏设为 `1~3`，以支持部分装甲板联调。

收到 `ENUM_START` 后，装甲板立即清除 NodeID，在普通任务中等待 `5 + (uid_hash % 50)` ms 后发送四个 `ENUM_ANNOUNCE` 分片。发送失败或等待分配超时会以 UID 参与计算的二次退避重发；1 秒仍未获得分配则保持未分配，等待电管下一轮枚举。新的 `ENUM_START` 会使已运行节点重新枚举。

电管每次只对一个 UID 发送 `NODE_ASSIGN` 并等待 ACK，再继续下一个节点，避免多个板同时发送 `0x123`。缺板、多板、无效 UID 分片或 ACK 超时都会使电管重新发起枚举，不进入业务轮询。

### NodeID 业务槽位

业务 ID 公式为：`0x130 + (NodeID - 1) × 0x10 + offset`。Node 1 为 `0x130~0x139`，Node 4 为 `0x160~0x169`，均在 `0x1xx`，不会占用枪管的 `0x2xx`。

| offset | 报文 |
|---:|---|
| `0x0` | 异常复位原因 |
| `0x1` | 运行故障状态 |
| `0x2` | 初始化完成 |
| `0x3` | 有效击打事件 |
| `0x4` | 状态查询 |
| `0x5` | 状态回复 |
| `0x6` | LED 控制 |
| `0x7` | LED 控制 ACK |
| `0x8` | NodeID 身份查询 |
| `0x9` | NodeID 身份回复 |
| `0xA` | 有效击打事件 ACK |

### NodeID 身份确认

电管向目标节点的 `offset=0x8` ID 发送 `DLC=0` 数据帧；只有该 NodeID 的 READY 装甲板处理。装甲板通过同一 NodeID 的 `offset=0x9` ID 回复 `DLC=8`：Byte0=自身 NodeID，Byte1~3=UID FNV-1a token 低 24 bit（小端），Byte4~5=UID CRC16-CCITT（小端），Byte6~7=0。电管必须同时核对回复 ID、Byte0、token、CRC16 与枚举表，全部一致才判定成功。

电管串口调试命令：

- `ARMOR=LIST`：只读取当前 `armor_enum_diag` 和 `referee_can_monitor.armor_nodes`，逐行输出四个 NodeID 的 UID、token、ACK、在线、最后接收时间、状态、故障位；不会重新枚举。
- `ARMOR=ID?=<1..4>`：立即输出 `PENDING`，随后在普通任务输出目标 NodeID、回复 NodeID、token、CRC 和 `OK`、`MISMATCH` 或 `TIMEOUT`。未枚举、非法编号、重复请求和发送失败会输出明确错误。

电管在四块板全部 `ACK` 后开始运行；每 25 ms 查询下一块板，因此每块板都以 10 Hz 收到状态查询。装甲板 CAN 接收滤波覆盖 `0x100~0x17F`。

### 联调

1. 四块板烧录同一 bin，上电后观察装甲板串口 `[CAN ENUM] UID=...`、`assigned NodeID=...`、`ready`，并在电管调试器查看 `armor_enum_diag` 和 `referee_can_monitor.armor_nodes[4]`。
2. 确认四个 `node_id` 恰为 1、2、3、4；重新上电后，UID 排序不变时编号不变。
3. 确认四个节点状态回复 ID 分别落在对应槽位，且每块间隔约 100 ms。
4. 人工发送新的 `0x120`，确认所有业务帧暂停、重新枚举后恢复。
5. 在电管串口执行 `ARMOR=LIST`，确认输出四项枚举表；依次执行 `ARMOR=ID?=1` 至 `ARMOR=ID?=4`，确认均返回匹配的身份回复。

## v1.2 历史固定 ID 定义（不再生效）

## 1. ID 分层与分配

| 层 | ID 范围 | 说明 |
|---|---:|---|
| 调试层 | `0x100~0x10F` | 预留 |
| 故障层 | `0x110~0x11F` | 故障和启动状态 |
| 设备层 | `0x120~0x12F` | 预留给后续标定与参数读写 |
| 应用层 | `0x130~0x13F` | 命中事件、状态和灯光控制 |

| ID | 方向 | 报文 | DLC | 行为 |
|---:|---|---|---:|---|
| `0x110` | 装甲板→核心板 | 异常复位原因 | 1 | 异常复位后的初始化完成时发送 |
| `0x111` | 装甲板→核心板 | 运行故障状态 | 2 | 变化立即发送；非零时每 100 ms 重发 |
| `0x112` | 装甲板→核心板 | 初始化完成 | 0 | 初始化后发送一次，失败时后续循环重试 |
| `0x130` | 装甲板→核心板 | 有效击打事件 | 8 | 每次有效 HIT 入队发送 |
| `0x131` | 核心板→装甲板 | 状态查询 | 0 或 8 | 收到后回复 `0x132` |
| `0x132` | 装甲板→核心板 | 状态回复 | 8 | 回复 `0x131` |
| `0x133` | 核心板→装甲板 | LED 控制 | 8 | 合法命令执行后回复 `0x134` |
| `0x134` | 装甲板→核心板 | LED 控制 ACK | 8 | 原样回显 `0x133` 数据 |

## 2. 故障与启动

`0x100~0x10F` 为预留范围，当前固件不发送也不接受其中任何帧。协议不含链路心跳和通信超时判定。

### `0x110` 异常复位原因

`DLC=1`。Byte0 为复位原因组合位图：bit0=HardFault，bit1=独立/窗口看门狗复位，bit2=CAN Bus-Off 触发的受控复位（当前固件不启用该策略），bit3=MemManage、BusFault、UsageFault、选项字节或低功耗等其他异常复位。

HardFault 与其他 Cortex-M fault 会先把原因写入 RTC 备份寄存器，再执行系统复位。重新初始化成功后，装甲板按 `0x110 → 0x112` 顺序发送。普通上电、NRST 引脚复位和常规软件复位不发送 `0x110`。无 CAN ACK 或单次 Bus-Off 不会导致 MCU 复位。

### `0x111` 运行故障状态

`DLC=2`，Byte0~1 为 `HitDetect_GetFaultFlags()` 的 `uint16_t` 原始故障位图，小端序。故障位与 `App/detect/hit_detect.h` 的 `FAULT_*` 定义一致。故障清除时发送 `0x0000`。

### `0x112` 初始化完成

`DLC=0`。所有应用模块初始化完成、固定帧接收回调注册后发送一次。

## 3. 应用层

### `0x130` 有效击打事件

`DLC=8`。消息来自 `hit_event_t`；事件先进入四项 FIFO，直到收到电管应用 ACK 才出队。CAN 邮箱暂忙或 ACK 丢失时按 `100 ms` 重传同一 sequence；电管必须只对同一 NodeID 的新 sequence 扣血。

| 字节 | 字段 | 编码 |
|---|---|---|
| Byte0 | `sequence` | 本板递增 `uint8_t`；重传时保持不变 |
| Byte1 | `channel` | 当前恒为 `0xFF`，表示四通道融合判定 |
| Byte2 | `intensity` | `0~100`，未完成力度标定时为 0 |
| Byte3~4 | `force_01n` | `uint16_t` 小端，单位 0.01 N；未标定时为 0 |
| Byte5~7 | `sum_peak` | `uint24_t` 小端，对应串口 `sum_peak` 的低 24 bit |

### `offset=0xA` 有效击打 ACK

电管使用同一 NodeID 的业务 ID、`DLC=1` 下发 `Byte0=sequence`。装甲板仅在 sequence 等于当前 FIFO 首项时移除该受击；未知或过期 ACK 静默忽略。电管收到重复 HIT 必须重发 ACK，但不得重复扣血。

### `0x131` 查询与 `0x132` 回复

`0x131` 仅接受标准数据帧，支持两种格式：`DLC=0`，或 `DLC=8` 且 Byte0~7 均为 0（兼容固定 8 字节 CAN 工具）。其他格式静默忽略。

`0x132` 的 `DLC=8`：

| 字节 | 字段 | 编码 |
|---|---|---|
| Byte0 | `state` | `sm_state_t`：0=BOOT，1=NORMAL，2=HIT，3=FAULT，4=COMM_LOST，5=ID_SETUP，6=ID_CONFLICT |
| Byte1 | `reset_cause` | 与 `0x110` Byte0 相同的最近一次启动复位原因；正常启动为 0 |
| Byte2~3 | `hit_fault_flags` | `HitDetect_GetFaultFlags()`，`uint16_t` 小端 |
| Byte4~5 | `temp_fault_flags` | `TempMon_GetFaultFlags()`，`uint16_t` 小端；温度监测尚未实现时为 0 |
| Byte6 | `can_health` | bit0=错误警告，bit1=错误被动，bit2=Bus-Off；其余为 0 |
| Byte7 | 保留 | 固定为 0 |

### `0x133` LED 控制与 `0x134` ACK

`0x133` 必须为标准数据帧且 `DLC=8`；Byte2~7 必须为 0。未知命令、参数越界、保留字节非零、或当前处于 `FAULT`/`COMM_LOST` 时静默忽略且不回复 ACK。有效命令执行后，`0x134` 原样回显全部 8 个字节。当前 CAN 协议不会主动进入 `COMM_LOST`。

| Byte0 命令 | Byte1 参数 | 行为 |
|---:|---:|---|
| `0x00` | 忽略 | 切换为 `LED_EFF_NORMAL` |
| `0x01` | 忽略 | 设置红队 |
| `0x02` | 忽略 | 设置蓝队 |
| `0x03` | `0~100` | 设置 LED 亮度百分比 |
| `0x04` | `0~6` | 设置现有 `led_effect_t` 灯效值 |

## 4. 未实现的预留功能

`0x120~0x12F` 不被当前固件接受。CAN 参数读写、零偏标定以及 Flash 双槽 CRC 掉电保存尚未实现；当前参数在每次启动时恢复为固件默认值。为保护采样链路，通信层每次主循环最多发出一帧 CAN 报文。

## 5. 联调检查

1. 普通上电后观察一次 `0x112`，并确认静止、无故障时没有周期 CAN 帧；异常复位后应先看到 `0x110` 再看到 `0x112`。
2. 触发 HIT 后确认电管只扣一次血，并观察同一 NodeID 的 `offset=0xA` ACK；人为阻断 ACK 时应看到相同 sequence 的 HIT 每 100 ms 重发。检查 `0x111` 的故障变化和清除上报。
3. 分别以 DLC=0、DLC=8 且全 0 发送 `0x131`，确认收到 DLC=8 的 `0x132`；发送合法与非法 `0x133`，确认仅合法命令有 `0x134` 回显。
