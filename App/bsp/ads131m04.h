/**
 ******************************************************************************
 * @file    ads131m04.h
 * @brief   ADS131M04 设备驱动接口（SPI Mode1 / DRDY EXTI / TX+RX DMA / K 帧环形缓冲）。
 *
 * 设计文档 v1.5 第 6 章为唯一权威来源：接口事实 6.1、寄存器表 6.2、
 * 初始化序列 6.3、帧读取方案 A 6.4、极性 6.7。
 * 寄存器位定义以 Driver_Examples/ads131m08/ads131m0x.h 宏为准。
 ******************************************************************************
 */
#ifndef APP_ADS131M04_H
#define APP_ADS131M04_H

#include <stdint.h>
#include <stdbool.h>

/* ---- 初始化寄存器写入开关（2026-09-09） ---- */
/* 主开关：1=按 SBAS890D 表写入除 MODE 外全部寄存器（CLOCK 优先写）——
 * 修复新批次默认 OSR 异常等场景；0=退回只读回验证（旧策略，量产兜底）。 */
#define ADS_REG_INIT_ENABLE       1u
/* MODE 写入子开关（实验用，默认 0）：0x0510→0x0410 两笔。
 * 警告：REVID=0x05 实测写 MODE 会把字长清成 16bit 破坏帧流——仅在新
 * 数据手册确认布局后或牺牲芯片做实验时打开。 */
#define ADS_REG_INIT_WRITE_MODE   0u

/* 单样本类型（24bit 有符号，符号已按极性宏修正后的 32 位表示） */
typedef int32_t ads_sample_t;

/* 一帧数据：状态字 + 4 通道 + 时间戳（设计文档 5.3/6.4） */
typedef struct
{
    uint32_t status;        /* 帧内 STATUS 字（含 DRDY3:0、RESET、LOCK、CRC 标志位） */
    int32_t  ch[4];         /* 4 通道，ch1/ch3 已按极性宏取反 */
    uint32_t tick_ms;       /* 帧就绪时刻 HAL_GetTick() */
} ads_frame_t;

/* 诊断计数（纳入 STATUS_REPORT 与调试变量表，6.5） */
typedef struct
{
    uint32_t spi_err;       /* SPI/DMA 错误 */
    uint32_t crc_err;       /* CRC 校验错误（CRC 开启后有效） */
    uint32_t drdy_cnt;      /* DRDY 中断计数 */
    uint32_t drop_cnt;      /* 环形缓冲满而跳帧计数 */
    uint32_t frames_read;   /* 主循环成功消费的帧数 */
} ads_diag_t;

/* 初始化：复位 + 只读回验证（6.3，REVID=0x05 适配），失败置全局故障。 */
void ADS131M04_Init(void);

/* 初始化结果：0=失败（main_app 据此置 sticky FAULT_SPI，见 app/faults.h）。 */
bool ADS131M04_IsInitOk(void);

/* 消费 DMA 就绪帧（K 帧环形缓冲按读指针取）；0=OK，-1=无帧。 */
int  ADS131M04_ReadFrame(ads_frame_t *f);

/* 寄存器读写（"命令帧 + 响应下一帧"两拍，6.5） */
int  ADS131M04_RegWrite(uint8_t addr, uint16_t val);
int  ADS131M04_RegRead(uint8_t addr, uint16_t *val);

/* 自检：回读/DRDY/偏移/注入检查（7.6） */
int  ADS131M04_RunSelfTest(void);

/* 通道极性运行时覆盖（6.7）：bit0-3=ch0-3，1=取反 */
void ADS131M04_SetChannelSign(uint8_t sign_mask);

/* 诊断计数快照 */
void ADS131M04_GetDiag(ads_diag_t *d);

/* 中断服务接口（由 weak 回调桥接，6.5）：
 * - IrqDrdy：EXTI15_10 下降沿（HAL_GPIO_EXTI_Callback 分发）
 * - IrqDmaDone：SPI TX/RX DMA 完成（HAL_SPI_TxRxCpltCallback 分发） */
void ADS131M04_IrqDrdy(void);
void ADS131M04_IrqDmaDone(void);

/* 帧就绪标志（主循环查询） */
bool ADS131M04_IsFrameReady(void);

#endif /* APP_ADS131M04_H */
