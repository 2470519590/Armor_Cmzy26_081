# App/ 工程结构说明（装甲击打检测模块固件）

> 本文档说明**代码放在哪、按什么规矩写**；"为什么这么做/算法与协议细节"见 **`Docs/装甲模块固件设计方案.md`**（当前 v1.5，以下称《设计方案》，章节号均指该文档）。

---

## 1. 总览

- `App/` 是应用层唯一目录：**不参与 CubeMX 重生成**（重生成只覆盖 `Core/` 与 Makefile），修改 CubeMX 配置不会丢应用代码。
- 分层：`bsp/`（硬件抽象）→ `detect/`、`comm/`（功能与协议）→ `app/`（编排与状态）。**禁止反向依赖**。
- 全部文件经 `app.mk` 加入构建（见 §8）。
- 当前状态：**工程骨架**——所有模块接口已按《设计方案》定型，实现为桩（`TODO(Step N)` 标记），可编译可烧录（PB1 1Hz 闪烁自证 tick 链路）。按 §12 步骤逐个填充。

## 2. 完整目录树

```
App/
├─ board.h                      板级公共定义（外设句柄 extern 唯一入口、常量、极性宏）←《设计方案》2/6.7
├─ bsp/
│  ├─ ads131m04.h/.c            ADS131M04 驱动【桩 Step1】← 第 6 章
│  ├─ ws2812_uart.h/.c          WS2812 灯带驱动（USART2+DMA）【桩 Step3】← 8.1
│  └─ temp_mon.h/.c             基准温度监测（ADC1 TIM1 触发）【桩 Step5】← 7.5
├─ detect/
│  ├─ hit_detect.h/.c           击打检测管线（融合模型）【桩 Step2】← 第 7 章
│  └─ calibration.h/.c          参数表 P01~P24 + Flash 持久化【参数表已就绪，持久化桩 Step5】← 7.4/7.7
├─ comm/
│  ├─ app_can.h                 CAN 平台抽象接口
│  ├─ app_can.c                 CAN 适配层 L432 绑定（500kbps、过滤、Bus-Off 恢复）
│  ├─ app_log.h                 日志接口
│  ├─ app_log.c                 USART1 阻塞日志【可用】← 9.2
│  └─ board_comm.h/.c           NodeID 槽位业务帧（0x130~0x167、故障/击打/控制）← Docs/CAN_PROTOCOL.md
└─ app/
   ├─ main_app.h/.c             编排：App_Init/App_Loop/App_OnTick1ms + TIM 回调【骨架自证可用】← 5.3/第 10 章
   ├─ state_machine.h/.c        七状态机【桩 Step2/5】← 第 11 章
   └─ led_status.h/.c           六灯效联动【桩 Step3】← 8.2
```

标注说明：**【桩 StepN】**＝接口已定型、实现待第 N 步填充；其余为可正常工作的骨架实现。

## 3. 分层与依赖规则

- 依赖方向（只允许向下）：`app/` → `detect/`+`comm/`+`bsp/`；`detect/`、`comm/` → `bsp/`；`bsp/` → HAL + `board.h`。
- **`detect/` 不得依赖 `comm/`**（检测算法不感知通信）；状态/灯联动经 `app/` 层编排。
- 允许的 include：`"main.h"`、`"board.h"`、`<stdint.h>` 等标准头、本层头 + 下层头（如 `"bsp/ads131m04.h"`、`"detect/calibration.h"`）。**不要** include 其他模块的 `.c` 或 CubeMX 生成文件内部头。

## 4. 命名与代码规范

- 函数/全局符号前缀＝模块名：`ADS131M04_`、`Ws2812_`、`TempMon_`、`HitDetect_`、`Cal_`、`App_Can_`、`App_Log_`、`BoardComm_`、`StateMachine_`、`LedStatus_`、`App_`（编排）。
- 类型后缀 `_t`（如 `ads_frame_t`、`hit_event_t`、`hit_param_t`）；模块内私有符号一律 `static`。
- 注释用中文；桩实现标注 `TODO(Step N)：…`（N＝《设计方案》第 12 章实现步骤），完成一步删除对应 TODO。
- 常量/宏全大写；位域用 `(1u << n)` 风格并注释含义。

## 5. 头文件规范

- include guard：`APP_<路径>_H`（如 `APP_BSP_ADS131M04_H`）。
- 包含顺序：先本模块头，再 `"board.h"`/`"main.h"`，再标准库，再下层模块头。
- **`board.h` 是引用生成代码外设句柄（`hspi1` 等 extern）的唯一入口**——新增外设引用一律在 `board.h` 加 extern，禁止各文件自行 extern。
- 生成代码的引脚宏（`ADC_CS_Pin` 等）来自 `main.h`（CubeMX 重生成后自动更新，代码直接用宏名，不写死 GPIO_PIN_x）。

## 6. 中断与回调归属表

| weak 回调 | 定义位置 | 触发源（it.c 已生成） | 处理规则 |
|---|---|---|---|
| `HAL_TIM_PeriodElapsedCallback` | app/main_app.c | TIM2_IRQHandler（1kHz） | 仅 `htim->Instance==TIM2` 时调 `App_OnTick1ms()` |
| `HAL_GPIO_EXTI_Callback` | bsp/ads131m04.c | EXTI15_10_IRQHandler（PA15 DRDY，it.c 生成） | 仅 `ADC_NDRDY_Pin` 时调 `ADS131M04_IrqDrdy()` |
| `HAL_SPI_TxRxCpltCallback` | bsp/ads131m04.c | SPI1/DMA1_Ch2/3 IRQ | 仅 SPI1 时调 `ADS131M04_IrqDmaDone()` |
| `HAL_UART_TxCpltCallback` | bsp/ws2812_uart.c | USART2/DMA1_Ch7 IRQ | 仅 USART2 时调 `Ws2812_IrqTxDone()` |
| `HAL_CAN_RxFifo0MsgPendingCallback` | comm/app_can.c | CAN1_RX0_IRQHandler | 排空 FIFO0 分发（Step4 实现） |
| `HAL_CAN_ErrorCallback` | comm/app_can.c | CAN1_SCE_IRQHandler | 记录 LEC/Bus-Off 恢复（Step4 实现） |

**中断内铁律**：ISR 只做置标志/启动 DMA/计数，**禁止调用协议栈**（调度器非 ISR 安全）；共享数据结构——写指针仅 ISR 改、读指针仅主循环改、槽状态用原子标志（《设计方案》R17）。

## 7. main.c USER CODE 接入点清单

| USER CODE 块 | 内容 | 说明 |
|---|---|---|
| `Includes` | `#include "app/main_app.h"` | CubeMX 重生成后需恢复 |
| `2` | `App_Init();`（在 `MX_TIM2_Init();` 之后） | 同上 |
| `3`（while 循环内） | `App_Loop();` | 同上 |

重生成后恢复步骤：① Makefile 重新追加 `-include app.mk`（§8）；② 检查上述三个 USER CODE 块内容仍在（CubeMX 保留 USER CODE 块，通常无需重做）。

## 8. app.mk 与构建

- `app.mk` 位于仓库根目录：`APP_SOURCES`（17 个源文件）+ `C_INCLUDES`（`-IApp/...` 六条）。
- Makefile 中 `-include app.mk` 的位置有讲究：**必须在 Makefile 自身 `C_SOURCES`/`C_INCLUDES` 定义之后、`OBJECTS` 计算之前**（否则 `=` 赋值覆盖追加、或新源不参与编译）。当前位于 C_INCLUDES 块之后。
- **CubeMX 重生成会覆写 Makefile → 必须重新追加 `-include app.mk`**（已写入 CLAUDE.md 提醒）。
- **文件名全局唯一**：Makefile 按 `notdir` 扁平化目标，`APP_SOURCES` 内任何两个文件不得同名（当前 17 个均唯一）。
- 新增模块三步：① 建 `App/<层>/xxx.c/.h`；② `app.mk` 的 `APP_SOURCES`/`C_INCLUDES` 各加一行；③ `make -j` 验证。
- 常用命令：
  ```bash
  make -j                      # 构建 → build/Armor_Cmzy26_081.{elf,hex,bin,map}
  make clean
  make GCC_PATH=<toolchain>    # 工具链不在 PATH 时
  arm-none-eabi-size build/Armor_Cmzy26_081.elf    # 内存核对
  ```
- 烧录（openocd，ST-Link）：
  ```bash
  openocd -f interface/stlink.cfg -f target/stm32l4x.cfg \
      -c "program build/Armor_Cmzy26_081.elf verify reset exit"
  ```
- `compile_commands.json`（clangd 用）在新增源文件后需重新生成（见 §8.1 脚本思路：遍历 `make -pn` 的 C_SOURCES/APP_SOURCES 输出条目）。

## 9. 参数表约定

- 唯一权威：`detect/calibration.h` 的 `hit_param_t`，字段与《设计方案》7.4 的 P01~P24 一一对应（默认值在 `Cal_GetDefaults()`，标注了依据）。
- 修改参数默认值 → 只改 `calibration.c` 的 `Cal_GetDefaults()`；新增参数 → 结构体加字段 + 默认值 + 同步《设计方案》7.4 表与下行命令参数 ID。
- Flash 持久化：`Cal_Load`/`Cal_Save`（双槽 + CRC，Step5 实现）；标定值（K_N/K_ch/极性掩码）经此恢复。

## 10. 状态与灯效枚举

- 状态机（`app/state_machine.h`）：`SM_STATE_BOOT/NORMAL/HIT/FAULT/COMM_LOST/ID_SETUP/ID_CONFLICT`（《设计方案》第 11 章状态表）。
- 灯效（`app/led_status.h`）：`LED_EFF_NORMAL/HIT/ID_SETUP/FAULT/COMM_LOST/ID_CONFLICT/BOOT`；频率：快闪 5Hz、慢闪 1Hz、FAULT 红蓝 500ms、ID_CONFLICT 红蓝紫 333ms；帧刷新 20Hz。
- 指示 LED：**PB0=超温指示、PB1=系统正常指示**（NORMAL/HIT/ID_SETUP 亮；**高电平点亮、低电平灭**——硬件实测确认）。骨架阶段 PB1 由 main_app 1Hz 闪烁自证（Step3 起移交 LedStatus）。
- 检测调试变量（hit_detect.c）：`s_hit_dif`（实时差分信号）、`s_hit_sum`/`s_hit_bl[4]`（合力/基线）、`s_hit_state`（0=IDLE/1=ARMED/2=FIRING/3=REFRACTORY）、`s_hit_event`/`s_hit_count`/`s_hit_dif_peak`、`s_wave_ready`/`s_wave_frozen_*`（锁帧波形）、`s_hit_faults`。
- 自检变量（ads131m04.c）：`s_st_dc[4]`（直流偏置，随姿态变化正常）、`s_st_rms[4]`（**去均值后**的真实噪声，期望 <100 counts）。

## 11. 数据流与缓冲约定

- **K=32 帧环形缓冲**（`ADS_RING_DEPTH`，32×18B=576B）：DRDY EXTI 取空闲槽写、槽满跳帧 + `drop_cnt` 计数；主循环按读指针消费（写指针仅 ISR、读指针仅主循环，R17）。
- WS2812 帧缓冲：`WS2812_FRAME_BYTES`=118B（13×8+14），DMA 发送 ≈404µs，20Hz 刷新。
- CAN 接收队列和击打事件队列均为 4 项固定帧缓冲；CAN 暂时发送失败时击打事件按 10ms 节拍重试。
- 事件流：ADS 帧 → `HitDetect_Feed` → `hit_event_t`（合力峰值 + `peak_ch[4]` + 预留 impulse/duration）→ `BoardComm_ReportHitEvent`。

## 12. 扩展配方（step-by-step 文件清单）

| 场景 | 步骤 |
|---|---|
| **新增 BSP 设备** | ① `App/bsp/xxx.h/.c`（模块前缀 `Xxx_`）；② 外设句柄 extern 加入 `board.h`；③ `app.mk` 两行；④ `main_app.c` 的 `App_Init` 调 `Xxx_Init()` |
| **新增协议消息** | ① `board_comm.h` 分配固定 CAN ID；② `board_comm.c` 实现固定 DLC 的构造/解析；③ 同步 `Docs/CAN_PROTOCOL.md` 与《设计方案》第 9 章 |
| **新增可调参数** | ① `hit_param_t` 加字段；② `Cal_GetDefaults` 默认值；③ 下行参数 ID 映射（Step4 的 CMD_PARAM_SET/GET）；④ 同步《设计方案》7.4 |
| **新增灯效/状态** | ① `led_effect_t`/`sm_state_t` 加枚举值；② `led_status.c` 灯效生成；③ `state_machine.c` 转移条件；④ 同步《设计方案》8.2/11 章表 |

## 13. 设计文档映射表

| 《设计方案》章节 | 工程落点 |
|---|---|
| 第 2 章 硬件事实 | `App/board.h`（句柄/常量/极性宏） |
| 第 4 章 CubeMX 配置 | `.ioc`（已完成，见 v1.5 配置表） |
| 第 6 章 ADC 驱动 | `App/bsp/ads131m04.h/.c` |
| 第 7 章 检测算法 | `App/detect/hit_detect.c`、`App/detect/calibration.c`、`App/bsp/temp_mon.c` |
| 第 8 章 灯驱动 | `App/bsp/ws2812_uart.c`、`App/app/led_status.c` |
| 第 9 章 通信 | `App/comm/{app_can,board_comm,app_log}.c` + `transport/protocol/service/` |
| 第 10 章 调度 | `App/app/main_app.c`（App_Loop/App_OnTick1ms） |
| 第 11 章 状态机 | `App/app/state_machine.c` |
| 第 12 章 里程碑 | 本文 §12 步骤与 `TODO(Step N)` 标记 |
