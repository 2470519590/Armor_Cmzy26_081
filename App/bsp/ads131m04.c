/**
 ******************************************************************************
 * @file    ads131m04.c
 * @brief   ADS131M04 设备驱动实现（设计文档 v1.5 第 6 章）。
 *
 * 实现要点：
 *  - CLKIN：TIM16 CH1N PWM 8.0MHz（HR 模式跑满），HAL_TIMEx_PWMN_Start 启动；
 *  - 初始化序列 6.3：PC15 复位脉冲 → 自愈盲发序列（UNLOCK/NULL/RESET/NULL，
 *    救 DRDY 卡低的坏芯片）→ 等 DRDY 上升沿 → 就绪后补发 RESET（双保险，
 *    保证落地）→ 等 DRDY 上升沿 → UNLOCK → 探针回读校验（ID/MODE/CLOCK）
 *    → 寄存器表写入（ADS_REG_INIT_ENABLE 开关，CLOCK 优先、MODE 默认不写）
 *    → 开流采集
 *  - 帧读取方案 A（6.4）：DRDY（ADC_NDRDY）下降沿 EXTI → 拉低 CS、启动 SPI TX+RX DMA
 *    （TX=静态 18B NULL）→ DMA 完成拉高 CS、槽标记满 → 主循环按读指针消费；
 *    K=32 帧环形缓冲（写指针仅 ISR 改、读指针仅主循环改、槽状态标志）；
 *  - 寄存器读写："命令帧 + 响应下一帧"两拍（6.5），流采集中自动暂停/恢复；
 *  - 极性（6.7）：ADC_CH_SIGN_MASK 宏（bit0-3=ch0-3，1=取反）+ 运行时覆盖。
 *
 * 注意：.ioc 中 DRDY 脚（ADC_NDRDY，当前为 PA15）的 EXTI 边沿配的是上升沿
 * （GPIO_MODE_IT_RISING），而 DRDY 低有效、数据就绪发生在下降沿——本驱动在
 * Init 中重配为下降沿；引脚定义以 CubeMX 为准（CS/DRDY 对调后已自动跟随宏）。
 ******************************************************************************
 */
#include "bsp/ads131m04.h"
#include "board.h"
#include "comm/app_log.h"

#include <math.h>
#include <string.h>

/* ==================== 命令字（SBAS890D 表 8-11） ==================== */
#define ADS_CMD_NULL     0x0000u
#define ADS_CMD_RESET    0x0011u
#define ADS_CMD_UNLOCK   0x0655u
#define ADS_CMD_LOCK     0x0555u
#define ADS_CMD_RREG(addr)   (0xA000u | ((uint16_t)(addr) << 7))
#define ADS_CMD_WREG(addr)   (0x6000u | ((uint16_t)(addr) << 7))

/* 状态字健康位（16 位有效数据 MSB 对齐于 24bit 字，取高 16 位后按位判断） */
#define STATUS_RESET_BIT   0x0400u   /* 意外复位 */
#define STATUS_REG_MAP_BIT 0x2000u   /* 寄存器表 CRC 异常 */
#define STATUS_CRC_ERR_BIT 0x1000u   /* 输入帧 CRC 错误 */
#define STATUS_DRDY_MASK   0x000Fu   /* DRDY3:0 */

/* 初始化需显式写入的全部可写寄存器（6.2 表，含默认值；地址按 SBAS890D §8.6） */
typedef struct
{
    uint8_t  addr;
    uint16_t val;
} ads_reg_cfg_t;

/* 写入顺序（2026-09-09 定稿）：CLOCK 排第一——修复新批次默认 OSR 异常的关键
 * 寄存器，先落先验；其余为复位默认值的无害空写（布局一致时）；MODE 条目排在
 * 末尾且由 ADS_REG_INIT_WRITE_MODE 子开关控制（REVID=0x05 实测写 MODE 会把
 * 字长清成 16bit 破坏帧流，默认不写——若实验打开，其失步也不会污染前面写入）。 */
static const ads_reg_cfg_t s_reg_init[] =
{
    { 0x03u, 0x0F0Eu },   /* CLOCK：4 通道使能 + OSR=1024 + PWR=HR（旧批默认即此值） */
    { 0x04u, 0x0000u },   /* GAIN1：4×PGA=1（标定后提升，6.6） */
    { 0x06u, 0x0600u },   /* CFG：默认 */
    { 0x07u, 0x0000u },   /* THRSHLD_MSB */
    { 0x08u, 0x0000u },   /* THRSHLD_LSB（DCBLOCK=关） */
    { 0x09u, 0x0000u },   /* CH0_CFG */
    { 0x0Eu, 0x0000u },   /* CH1_CFG */
    { 0x13u, 0x0000u },   /* CH2_CFG */
    { 0x18u, 0x0000u },   /* CH3_CFG */
    { 0x0Au, 0x0000u }, { 0x0Bu, 0x0000u },   /* CH0_OCAL */
    { 0x0Fu, 0x0000u }, { 0x10u, 0x0000u },   /* CH1_OCAL */
    { 0x14u, 0x0000u }, { 0x15u, 0x0000u },   /* CH2_OCAL */
    { 0x19u, 0x0000u }, { 0x1Au, 0x0000u },   /* CH3_OCAL */
    { 0x0Cu, 0x8000u }, { 0x0Du, 0x0000u },   /* CH0_GCAL = 单位增益 */
    { 0x11u, 0x8000u }, { 0x12u, 0x0000u },   /* CH1_GCAL */
    { 0x16u, 0x8000u }, { 0x17u, 0x0000u },   /* CH2_GCAL */
    { 0x1Bu, 0x8000u }, { 0x1Cu, 0x0000u },   /* CH3_GCAL */
    { 0x3Fu, 0x0000u },   /* RESERVED：手册要求恒写 0 */
    { 0x02u, 0x0510u },   /* MODE：24bit 字长、CRC 关、TIMEOUT 开、RESET 标志置位（仅子开关打开时写） */
    { 0x02u, 0x0410u },   /* MODE：清 RESET 标志（第二笔，仅子开关打开时写） */
};

/* ==================== 环形帧缓冲（方案 A，6.4） ==================== */
#define SLOT_FREE 0u
#define SLOT_FULL 1u

static uint8_t  s_ring[ADS_RING_DEPTH][ADS_FRAME_BYTES];   /* K=32 × 18B = 576B */
static volatile uint8_t s_slot_state[ADS_RING_DEPTH];
static uint32_t s_wr_idx;     /* 写指针：仅 ISR 侧修改 */
static uint32_t s_rd_idx;     /* 读指针：仅主循环侧修改 */
static volatile uint8_t s_dma_busy;   /* 帧传输进行中 */
static volatile uint8_t s_streaming;  /* 采集使能（Init 完成后置 1；寄存器读写时暂停） */
static uint8_t s_discard;     /* 恢复采集后丢弃的前 N 帧（响应延迟一帧） */
static uint8_t s_tx_null[ADS_FRAME_BYTES];   /* 静态 NULL 发送缓冲（全 0） */

/* ==================== 运行状态 ==================== */
static volatile ads_diag_t s_diag;
static volatile uint8_t   s_sign_mask = ADC_CH_SIGN_MASK;   /* 极性宏（6.7） */
static uint8_t s_init_ok;    /* 初始化结果（main_app 失败时置 sticky FAULT_SPI，见 app/faults.h） */
static uint16_t s_chip_id;   /* 最近一次读到的 ID 寄存器值（调试用，对应串口打印的 ID） */
static volatile uint8_t  s_fail_addr;     /* 最近一次写回读失败的寄存器地址（调试用） */
static volatile uint16_t s_fail_readback; /* 失败时的实际回读值（调试用：0xFFFF=悬空，错乱值=时序） */
static volatile uint16_t s_probe_mode;    /* 调试探针：任何 WREG 之前读到的 MODE 值 */
static volatile uint16_t s_probe_clock;   /* 调试探针：任何 WREG 之前读到的 CLOCK 值 */

static volatile uint8_t s_st_reset_flag;  /* STATUS.RESET 标志观察（本芯片预期恒 1，信息性） */

/* 帧结构观察（调试用，每 256µs 更新）：
 * s_frame_w[6] = 最新一帧的 6 个原始 24bit 字（word0=响应/STATUS、word1~4=ch0~3、word5=CRC）
 * s_frame_status = 解析出的 STATUS 16 位值（帧间应恒定） */
static volatile uint32_t s_frame_w[6];
static volatile uint16_t s_frame_status;

/* 自检结果固化（调试器直接 watch，无需串口） */
static volatile uint32_t s_st_drdy_hz;    /* 实测 DRDY 频率（期望 ≈3906） */
static volatile int32_t  s_st_mean[4];    /* 4 通道均值 = 直流偏置（随重力/姿态变化属正常） */
static volatile int32_t  s_st_dc[4];      /* 4 通道直流偏置（s_st_mean 的语义别名，调试用） */
static volatile int32_t  s_st_rms[4];     /* 4 通道噪声 RMS（去均值后，期望 <100 counts） */
static volatile uint8_t  s_st_result;     /* 0=通过 1=失败 */

/* 初始化调试 RX 捕获（调试器直接 watch，无需逻辑分析仪）：
 * s_init_rx[n] = 第 n+1 次 SPI 交换的 18B 原始 MISO 字节。
 * 帧序号对照：[0]=UNLOCK [1]=UNLOCK后NULL [2]=WREG MODE [3]=WREG后NULL
 *            [4]=RREG MODE [5]=RREG后NULL（当前失败帧，字0 应为 05 10 00）…… */
#define ADS_INIT_RX_FRAMES 128u   /* 128 帧 × 18B = 2304B（初始化 122 帧 + ping 4 帧） */
static uint8_t  s_init_rx[ADS_INIT_RX_FRAMES][ADS_FRAME_BYTES];
static volatile uint32_t s_init_rx_w0[ADS_INIT_RX_FRAMES]; /* 每帧字0（首3字节）标量化，Ozone CSV 可直接导出数值 */
static volatile uint16_t s_init_rx_count;    /* 已捕获帧数 */
static volatile uint8_t  s_init_rx_overflow; /* 超过 128 帧时置 1 */

/* 初始化阶段标记（调试用，直接 watch 即可定位失败阶段）：
 * 0=入口 1=EXTI重配完成 2=CLKIN启动 3=复位脉冲完成
 * 4=自愈盲发完成+DRDY等待通过 5=UNLOCK已发 6=探针校验通过 9=流采集已开（成功） */
static volatile uint8_t s_init_stage = 0u;

/* ==================== 内部工具 ==================== */

static void Ads_CsLow(void)
{
    uint32_t i;
    HAL_GPIO_WritePin(ADC_CS_GPIO_Port, ADC_CS_Pin, GPIO_PIN_RESET);
    /* CS 建立延时：保证芯片侧 CS 输入完全低于 VIL 后才开始第一个时钟，
     * 消除"首时钟被吞"导致的帧漂移（td(CSSC)≥16ns，取 ~2.5µs 大裕量）。 */
    for (i = 0u; i < 200u; i++)
    {
        __NOP();
    }
}

static void Ads_CsHigh(void)
{
    HAL_GPIO_WritePin(ADC_CS_GPIO_Port, ADC_CS_Pin, GPIO_PIN_SET);
}

/* 阻塞式收发一帧（18B；仅寄存器读写用，调用前须暂停流采集） */
static void Ads_ExchangeFrame(const uint8_t *tx, uint8_t *rx)
{
    Ads_CsLow();
    if (HAL_SPI_TransmitReceive(&hspi1, (uint8_t *)tx, rx, ADS_FRAME_BYTES, 10u) != HAL_OK)
    {
        s_diag.spi_err++;
    }
    Ads_CsHigh();

    /* 调试捕获：记录每次交换的 RX 原始字节（Init 入口清零） */
    if (s_init_rx_count < ADS_INIT_RX_FRAMES)
    {
        memcpy(s_init_rx[s_init_rx_count], rx, ADS_FRAME_BYTES);
        s_init_rx_w0[s_init_rx_count] = ((uint32_t)rx[0] << 16) | ((uint32_t)rx[1] << 8) | (uint32_t)rx[2];
    }
    else
    {
        s_init_rx_overflow = 1u;
    }
    s_init_rx_count++;
}

/* 把 16 位命令/数据放进 24bit 字（MSB 对齐，低 8 位补零） */
static void Ads_PutWord(uint8_t *dst, uint16_t word)
{
    dst[0] = (uint8_t)(word >> 8);
    dst[1] = (uint8_t)(word);
    dst[2] = 0u;
}

/* 取 24bit 字（无符号） */
static uint32_t Ads_GetWord(const uint8_t *src)
{
    return ((uint32_t)src[0] << 16) | ((uint32_t)src[1] << 8) | (uint32_t)src[2];
}

/* 24bit 二进制补码 → int32 符号扩展 */
static int32_t Ads_WordToSigned(uint32_t word)
{
    int32_t v = (int32_t)word;
    if (word & 0x800000u)
    {
        v |= (int32_t)0xFF000000u;
    }
    return v;
}

/* 暂停流采集（等当前 DMA 帧结束），返回前丢弃计数清零 */
static void Ads_PauseStream(void)
{
    s_streaming = 0u;
    while (s_dma_busy != 0u)
    {
        /* 等待进行中的帧传输完成（~7.2µs，最多一帧时间） */
    }
}

/* 恢复流采集：器件响应延迟一帧，丢弃前 2 帧 */
static void Ads_ResumeStream(void)
{
    s_discard = 2u;
    s_streaming = 1u;

    /* 关键：DRDY 为电平式（新数据就绪拉低、读走整帧才拉高）。
     * 初始化/暂停期间设备持续转换而无人读帧，DRDY 早已拉低并保持——
     * 此时若只等"下降沿"会永远等不到（死锁）。开启采集时若电平已低，
     * 立即补启动一次读取，之后恢复正常"下降沿→读→DRDY 拉高→下一次下降沿"循环。 */
    if (HAL_GPIO_ReadPin(ADC_NDRDY_GPIO_Port, ADC_NDRDY_Pin) == GPIO_PIN_RESET)
    {
        ADS131M04_IrqDrdy();
    }
}

/* ==================== 寄存器读写（6.5：命令帧 + 响应下一帧两拍） ==================== */

int ADS131M04_RegRead(uint8_t addr, uint16_t *val)
{
    uint8_t tx[ADS_FRAME_BYTES] = {0};
    uint8_t rx[ADS_FRAME_BYTES];
    uint32_t was_streaming;

    if (val == NULL)
    {
        return -1;
    }
    was_streaming = s_streaming;
    if (was_streaming != 0u)
    {
        Ads_PauseStream();
    }

    Ads_PutWord(tx, ADS_CMD_RREG(addr));   /* 帧 1：RREG（读 1 个寄存器） */
    Ads_ExchangeFrame(tx, rx);             /* 响应为上一帧，忽略 */
    memset(tx, 0, sizeof(tx));
    Ads_ExchangeFrame(tx, rx);             /* 帧 2：NULL，取 RREG 响应（word0 高 16 位） */
    *val = (uint16_t)(Ads_GetWord(rx) >> 8);

    if (was_streaming != 0u)
    {
        Ads_ResumeStream();
    }
    return 0;
}

int ADS131M04_RegWrite(uint8_t addr, uint16_t val)
{
    uint8_t tx[ADS_FRAME_BYTES] = {0};
    uint8_t rx[ADS_FRAME_BYTES];
    uint16_t readback;
    uint32_t was_streaming;

    was_streaming = s_streaming;
    if (was_streaming != 0u)
    {
        Ads_PauseStream();
    }

    Ads_PutWord(&tx[0], ADS_CMD_WREG(addr));   /* word0：WREG 命令 */
    Ads_PutWord(&tx[3], val);                  /* word1：数据（其余 4 字为 0） */
    Ads_ExchangeFrame(tx, rx);                 /* 响应下一帧 */
    memset(tx, 0, sizeof(tx));
    Ads_ExchangeFrame(tx, rx);                 /* 推进流水线 */

    /* 只写不校验的寄存器：
     * - 0x3F（RESERVED）：手册只要求"恒写 0"，未定义回读值；
     * - 0x02（MODE）：本板芯片 REVID=0x05（新硅片），MODE 的 WLENGTH/TIMEOUT 位
     *   （bit8/bit4）不回写（写入 0x0510 后回读 0x0400），与 SBAS890D（REV A）布局
     *   不一致；但 24 位帧行为经 ID/CLOCK 读写实测正常，CRC 位回读为 0（关），
     *   跳过 MODE 回读校验是安全的。 */
    if (addr == 0x3Fu || addr == 0x02u)
    {
        if (was_streaming != 0u)
        {
            Ads_ResumeStream();
        }
        return 0;
    }

    if (ADS131M04_RegRead(addr, &readback) != 0 || readback != val)
    {
        s_fail_addr = addr;
        s_fail_readback = readback;
        if (was_streaming != 0u)
        {
            Ads_ResumeStream();
        }
        return -2;   /* 回读校验失败 */
    }

    if (was_streaming != 0u)
    {
        Ads_ResumeStream();
    }
    return 0;
}

/* ==================== 初始化（6.3 时序严格） ==================== */

void ADS131M04_Init(void)
{
    uint32_t i;

    s_init_ok = 0u;
    s_streaming = 0u;
    s_dma_busy = 0u;
    s_discard = 0u;
    s_wr_idx = 0u;
    s_rd_idx = 0u;
    memset((void *)s_slot_state, SLOT_FREE, sizeof(s_slot_state));
    memset(s_tx_null, 0, sizeof(s_tx_null));
    memset((void *)&s_diag, 0, sizeof(s_diag));
    s_init_rx_count = 0u;
    s_init_rx_overflow = 0u;
    s_fail_addr = 0u;
    s_fail_readback = 0u;

    /* 1) CS 默认高（空闲）；DRDY 脚 EXTI 重配为下降沿（.ioc 配的是上升沿，DRDY 低有效） */
    Ads_CsHigh();
    {
        GPIO_InitTypeDef g = {0};
        g.Pin  = ADC_NDRDY_Pin;
        g.Mode = GPIO_MODE_IT_FALLING;
        g.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(ADC_NDRDY_GPIO_Port, &g);
    }
    s_init_stage = 1u;

    /* 2) 先启动 CLKIN（TIM16 CH1N 8MHz PWM）——复位脉宽以 tCLKIN 计 */
    HAL_TIMEx_PWMN_Start(&htim16, TIM_CHANNEL_1);
    s_init_stage = 2u;

    /* 3) PC15（SYNC/RESET#）复位脉冲：必须在 8~2048 tCLKIN 窗口内（8MHz 下 = 1µs~256µs）。
     *    超过 256µs 会被器件解释为"转换同步"而非"复位"——故不能用 HAL_Delay(1ms)，
     *    用 ~12.5µs 忙等待（1000 次 NOP @80MHz ≈ 12.5µs，落在窗口内且满足 ≥8 tCLKIN）。 */
    HAL_GPIO_WritePin(ADC_SYNC_GPIO_Port, ADC_SYNC_Pin, GPIO_PIN_RESET);
    for (i = 0u; i < 1000u; i++)
    {
        __NOP();
    }
    HAL_GPIO_WritePin(ADC_SYNC_GPIO_Port, ADC_SYNC_Pin, GPIO_PIN_SET);
    s_init_stage = 3u;

    /* 4) 自愈盲发序列（前移，2026-09-09 加固）：UNLOCK→NULL→RESET→NULL。
     * 排障结论（2026-08-21）：上次运行可能把 MODE 写坏（WLENGTH→16bit）或
     * 写入低功耗态（CLOCK.PWR→LP/VLP）、或接口被 LOCK，芯片带坏状态持续
     * 运行且 DRDY 可能卡低——旧顺序"先等 DRDY 上升沿再发 RESET"会永远等不到，
     * 自愈序列发不出去（死结）。改为复位脉冲后立即盲发：
     * - 坏状态芯片：接口早已就绪（就绪窗口只存在于 POR 之后），RESET 保证
     *   被解码——即使 16bit 帧模式，"命令帧+NULL 帧"两拍下命令仍落在芯片
     *   命令槽上（LCM(144,96)=288 位 = 2 帧/3 帧对齐）；RESET 后全部寄存器
     *   恢复默认（24bit/HR/解锁），1.8V 核心等低功耗停摆一并恢复；
     * - 刚 POR 的健康芯片：接口未就绪窗口内 SPI 被忽略，序列无害；若恰在
     *   序列中途就绪，至多等于对默认态芯片再发一次无害 RESET。
     * 注：SYNC 引脚复位不可依赖（曾实测不生效），RESET 命令是可靠复位方式。 */
    {
        uint8_t tx[ADS_FRAME_BYTES] = {0};
        uint8_t rx[ADS_FRAME_BYTES];

        Ads_PutWord(tx, ADS_CMD_UNLOCK);
        Ads_ExchangeFrame(tx, rx);
        memset(tx, 0, sizeof(tx));
        Ads_ExchangeFrame(tx, rx);

        Ads_PutWord(tx, ADS_CMD_RESET);
        Ads_ExchangeFrame(tx, rx);
        memset(tx, 0, sizeof(tx));
        Ads_ExchangeFrame(tx, rx);
    }
    s_init_stage = 4u;

    /* 5) 等 DRDY 上升沿（POR / RESET 命令后接口就绪标志），超时 100ms → 无响应故障 */
    {
        uint32_t t0 = HAL_GetTick();
        while ((HAL_GetTick() - t0) < 100u)
        {
            if (HAL_GPIO_ReadPin(ADC_NDRDY_GPIO_Port, ADC_NDRDY_Pin) == GPIO_PIN_SET)
            {
                break;
            }
        }
        if ((HAL_GetTick() - t0) >= 100u)
        {
            App_Log_Printf("[ADS] DRDY wait timeout\r\n");

            /* 诊断（硬件调试期）：设备未就绪也强制打 4 帧 NULL——
             * 保证 SCLK/CS 有波形可测，并打印 MISO 原始字节判断线路状态
             *（全 FF=悬空/被上拉，全 00=被拉低，有数据=设备在响应）。 */
            {
                uint8_t tx[ADS_FRAME_BYTES] = {0};
                uint8_t rx[ADS_FRAME_BYTES];
                for (i = 0u; i < 4u; i++)
                {
                    Ads_ExchangeFrame(tx, rx);
                    App_Log_Printf("[ADS] ping%lu: %02X %02X %02X %02X %02X %02X\r\n",
                                   (unsigned long)i,
                                   rx[0], rx[1], rx[2], rx[3], rx[4], rx[5]);
                }
            }
            return;
        }
    }

    /* 5.5) 就绪后补发 RESET（双保险，2026-09-09）：
     * 上一步的盲发序列可能在芯片"未就绪窗口"内被整体忽略（PC15 引脚复位
     * 又不可依赖），坏状态芯片（上次运行被写成 LP/VLP 低功耗、16bit 字长）
     * 就会带着残留寄存器状态继续跑——此时接口已就绪，补发的 RESET 保证
     * 落地，CLOCK.PWR 等全部恢复默认。对已复位成功的芯片重复 RESET 无害。 */
    {
        uint8_t tx[ADS_FRAME_BYTES] = {0};
        uint8_t rx[ADS_FRAME_BYTES];

        Ads_PutWord(tx, ADS_CMD_UNLOCK);
        Ads_ExchangeFrame(tx, rx);
        memset(tx, 0, sizeof(tx));
        Ads_ExchangeFrame(tx, rx);

        Ads_PutWord(tx, ADS_CMD_RESET);
        Ads_ExchangeFrame(tx, rx);
        memset(tx, 0, sizeof(tx));
        Ads_ExchangeFrame(tx, rx);

        /* RESET 后重新等 DRDY 上升沿（接口就绪标志） */
        {
            uint32_t t0 = HAL_GetTick();
            while ((HAL_GetTick() - t0) < 100u)
            {
                if (HAL_GPIO_ReadPin(ADC_NDRDY_GPIO_Port, ADC_NDRDY_Pin) == GPIO_PIN_SET)
                {
                    break;
                }
            }
            if ((HAL_GetTick() - t0) >= 100u)
            {
                App_Log_Printf("[ADS] DRDY wait (post-RESET) timeout\r\n");
                return;
            }
        }
    }

    /* 5) UNLOCK（复位后本就解锁，显式发送防异常状态） */
    {
        uint8_t tx[ADS_FRAME_BYTES] = {0};
        uint8_t rx[ADS_FRAME_BYTES];
        Ads_PutWord(tx, ADS_CMD_UNLOCK);
        Ads_ExchangeFrame(tx, rx);
        memset(tx, 0, sizeof(tx));
        Ads_ExchangeFrame(tx, rx);
    }
    s_init_stage = 5u;

    /* 5.5) 【调试探针】在任何 WREG 之前读取 ID/MODE/CLOCK 三个寄存器：
     * - ID 只读、值已知（0x24xx）——命令链路探针
     * - MODE 复位默认应为 0x0510（24bit）——若读回 0x0400（WLENGTH=00=16bit），
     *   说明复位没生效或芯片当前就是 16bit 模式 → WREG 的数据字会半字错位写坏寄存器
     * - CLOCK 默认 0x0F0E */
    {
        uint16_t id_probe = 0u;
        uint16_t mode_probe = 0u;
        uint16_t clock_probe = 0u;
        ADS131M04_RegRead(0x00u, &id_probe);
        ADS131M04_RegRead(0x02u, &mode_probe);
        ADS131M04_RegRead(0x03u, &clock_probe);
        s_chip_id = id_probe;
        s_probe_mode = mode_probe;
        s_probe_clock = clock_probe;
        App_Log_Printf("[ADS] probe: ID=0x%04X MODE=0x%04X CLOCK=0x%04X\r\n",
                       id_probe, mode_probe, clock_probe);
    }

    /* 6) 配置验证（REVID=0x05 关键教训：此芯片上写寄存器会破坏配置——
     *    实测 MODE 写入（旧手册布局）把字长清成 16 位，导致帧流错位。
     *    本芯片复位默认值 = 目标配置（MODE=0510h 24bit、CLOCK=0F0Eh、GAIN1=1、
     *    CHn_CFG=0、GCAL=单位增益），因此策略改为【只读回验证，不写入】。
     *    旧的 s_reg_init 全量写入表保留为注释（见文件上方），待 TI 新版数据手册
     *    确认 REVID=0x05 布局后再恢复。 */
    if ((s_chip_id & 0x0F00u) != 0x0400u)
    {
        App_Log_Printf("[ADS] ID check fail: 0x%04X\r\n", s_chip_id);
        return;
    }
    if (s_probe_mode != 0x0510u)
    {
        App_Log_Printf("[ADS] MODE default check fail: 0x%04X (expect 0x0510)\r\n", s_probe_mode);
        return;
    }
    /* CLOCK 默认值：REVID=0x05 硅片布局与 SBAS890D 不一致（MODE 的坑重现在
     * CLOCK 上），实测两种复位默认——0x0F0E=旧批次（曾被旧固件写入过，见调试
     * 文档）、0x0F02=新批次真实默认；两者行为等效（DRDY≈3906Hz 实测确认，
     * 即 OSR 等效 1024/HR）。仅接受已知集合，其余值仍判失败（防读错位）；
     * 实际数据率由自检 ST_DRDY（3125~4687Hz）行为验证兜底。 */
    if (s_probe_clock != 0x0F0Eu && s_probe_clock != 0x0F02u)
    {
        App_Log_Printf("[ADS] CLOCK default check fail: 0x%04X (expect 0x0F0E/0x0F02)\r\n", s_probe_clock);
        return;
    }
    s_init_stage = 6u;

    /* 6.5) 寄存器表写入（编译开关 ADS_REG_INIT_ENABLE，2026-09-09 恢复）：
     * CLOCK 优先写（修复新批次默认 OSR 异常的关键）；MODE 由
     * ADS_REG_INIT_WRITE_MODE 子开关控制（默认不写，见 s_reg_init 注释）。
     * 此刻流未启动，写前无需暂停。写入失败只记日志不判初始化失败——
     * 帧流未失步即继续，实际数据率由上电自检 ST_DRDY（3125~4687Hz）
     * 行为关卡判定并经 CAN 上报。 */
#if ADS_REG_INIT_ENABLE
    {
        uint32_t n;
        for (n = 0u; n < (sizeof(s_reg_init) / sizeof(s_reg_init[0])); n++)
        {
            int rc;

            if (s_reg_init[n].addr == 0x02u && (ADS_REG_INIT_WRITE_MODE == 0u))
            {
                continue;   /* MODE 默认跳过（REVID=0x05 实测写坏字长） */
            }
            rc = ADS131M04_RegWrite(s_reg_init[n].addr, s_reg_init[n].val);
            if (rc == 0)
            {
                App_Log_Printf("[ADS] WREG 0x%02X=0x%04X OK\r\n",
                               s_reg_init[n].addr, s_reg_init[n].val);
            }
            else
            {
                App_Log_Printf("[ADS] WREG 0x%02X=0x%04X rc=%d readback=0x%04X\r\n",
                               s_reg_init[n].addr, s_reg_init[n].val, rc, s_fail_readback);
            }
        }
        /* 写后再探针 CLOCK：观察新批次是否写粘（不判失败，供调试判读） */
        {
            uint16_t clock_after = 0u;
            (void)ADS131M04_RegRead(0x03u, &clock_after);
            s_probe_clock = clock_after;
            App_Log_Printf("[ADS] CLOCK after WREG: 0x%04X\r\n", clock_after);
        }
    }
#endif

    /* 7) 开启流采集（丢弃前 2 帧：器件响应延迟一帧，首帧为旧数据） */
    s_init_ok = 1u;
    Ads_ResumeStream();
    s_init_stage = 9u;

    App_Log_Printf("[ADS] init OK, ID=0x%04X\r\n", s_chip_id);
}

/* ==================== 方案 A 帧读取（6.4） ==================== */

void ADS131M04_IrqDrdy(void)
{
    if (s_streaming == 0u)
    {
        return;   /* 初始化/寄存器读写期间忽略 */
    }
    s_diag.drdy_cnt++;

    if (s_dma_busy != 0u)
    {
        return;   /* 理论不可达（帧间 256µs ≫ 传输 7.2µs），防御 */
    }
    if (s_slot_state[s_wr_idx] != SLOT_FREE)
    {
        s_diag.drop_cnt++;   /* 环形缓冲满：主循环消费滞后，跳帧 */
        return;
    }

    s_dma_busy = 1u;
    Ads_CsLow();
    if (HAL_SPI_TransmitReceive_DMA(&hspi1, s_tx_null, s_ring[s_wr_idx], ADS_FRAME_BYTES) != HAL_OK)
    {
        Ads_CsHigh();
        s_dma_busy = 0u;
        s_diag.spi_err++;
    }
}

void ADS131M04_IrqDmaDone(void)
{
    if (s_dma_busy == 0u)
    {
        return;
    }
    Ads_CsHigh();
    s_slot_state[s_wr_idx] = SLOT_FULL;
    s_wr_idx = (s_wr_idx + 1u) % ADS_RING_DEPTH;
    s_dma_busy = 0u;
}

bool ADS131M04_IsFrameReady(void)
{
    return (s_slot_state[s_rd_idx] == SLOT_FULL);
}

int ADS131M04_ReadFrame(ads_frame_t *f)
{
    uint32_t i;

    if (f == NULL || s_slot_state[s_rd_idx] != SLOT_FULL)
    {
        return -1;
    }

    if (s_discard > 0u)
    {
        /* 恢复采集后的旧数据帧，直接丢弃 */
        s_discard--;
        s_slot_state[s_rd_idx] = SLOT_FREE;
        s_rd_idx = (s_rd_idx + 1u) % ADS_RING_DEPTH;
        return -1;
    }

    {
        uint32_t status_word = Ads_GetWord(&s_ring[s_rd_idx][0]);
        uint16_t status16    = (uint16_t)(status_word >> 8);   /* 16 位有效数据 MSB 对齐 */

        /* 帧结构观察：记录 6 个原始字（调试用） */
        for (i = 0u; i < 6u; i++)
        {
            s_frame_w[i] = Ads_GetWord(&s_ring[s_rd_idx][i * 3u]);
        }
        s_frame_status = status16;

        f->status  = status_word;
        f->tick_ms = HAL_GetTick();

        for (i = 0u; i < 4u; i++)
        {
            int32_t v = Ads_WordToSigned(Ads_GetWord(&s_ring[s_rd_idx][3 + i * 3]));
            if (s_sign_mask & (1u << i))
            {
                v = -v;   /* 极性宏取反（6.7） */
            }
            f->ch[i] = v;
        }

        /* 状态字健康位检查（6.5）：
         * - RESET 位：仅"发生过复位"的信息性标志。本板芯片（REVID=0x05）MODE
         *   写不进去、该标志无法清除，会每帧置位——不作为错误计数（s_st_reset_flag
         *   供观察），真正的 SPI 错误由 HAL 返回值统计；
         * - REG_MAP/CRC_ERR 位：真实故障，计 crc_err。 */
        if (status16 & STATUS_RESET_BIT)
        {
            s_st_reset_flag = 1u;
        }
        if (status16 & (STATUS_REG_MAP_BIT | STATUS_CRC_ERR_BIT))
        {
            s_diag.crc_err++;
        }
    }

    s_slot_state[s_rd_idx] = SLOT_FREE;
    s_rd_idx = (s_rd_idx + 1u) % ADS_RING_DEPTH;
    s_diag.frames_read++;
    return 0;
}

/* ==================== 自检（7.6，Step 1 版本） ==================== */

int ADS131M04_RunSelfTest(void)
{
    uint16_t id = 0u, mode = 0u, clock_reg = 0u;
    uint32_t t0;
    int64_t  sum[4] = {0, 0, 0, 0};
    int64_t  sqsum[4] = {0, 0, 0, 0};
    uint32_t n = 0u;
    uint32_t drdy0;
    uint32_t i;

    if (s_init_ok == 0u)
    {
        return 1;
    }

    /* 1) 寄存器回读 */
    ADS131M04_RegRead(0x00u, &id);
    ADS131M04_RegRead(0x02u, &mode);
    ADS131M04_RegRead(0x03u, &clock_reg);
    s_chip_id = id;
    App_Log_Printf("[SELFTEST] ID=0x%04X MODE=0x%04X CLOCK=0x%04X\r\n", id, mode, clock_reg);

    /* 2) DRDY 频率实测：统计 1s 内中断计数，期望 ≈3906（M1 验证 R7）。
     *    窗口内持续排空环形缓冲，避免 1s 堆积 3906 帧把 K=32 环形打满而误报 drop。 */
    drdy0 = s_diag.drdy_cnt;
    t0 = HAL_GetTick();
    while ((HAL_GetTick() - t0) < 1000u)
    {
        ads_frame_t f;
        (void)ADS131M04_ReadFrame(&f);   /* 排空 */
    }
    s_st_drdy_hz = s_diag.drdy_cnt - drdy0;
    App_Log_Printf("[SELFTEST] DRDY = %lu Hz\r\n", (unsigned long)s_st_drdy_hz);

    /* 3) 采集 ~100ms（256 帧）统计各通道均值/RMS（期望 RMS < 50 counts@Gain1） */
    while (n < 256u)
    {
        ads_frame_t f;
        if (ADS131M04_ReadFrame(&f) == 0)
        {
            for (i = 0u; i < 4u; i++)
            {
                sum[i]   += f.ch[i];
                sqsum[i] += (int64_t)f.ch[i] * f.ch[i];
            }
            n++;
        }
    }
    for (i = 0u; i < 4u; i++)
    {
        s_st_mean[i] = (int32_t)(sum[i] / (int64_t)n);
        s_st_dc[i]   = s_st_mean[i];
        /* 噪声 RMS 必须先去均值（修正 2026-08-21）：对直流偏置 ~±1M counts 的通道，
         * 不去均值时 sqrt(Σx²/N)≈|直流值|，根本不是噪声 */
        {
            double m = (double)sum[i] / (double)n;
            double v = (double)sqsum[i] / (double)n - m * m;
            s_st_rms[i] = (int32_t)sqrt((v > 0.0) ? v : 0.0);
        }
        App_Log_Printf("[SELFTEST] CH%lu mean=%ld RMS=%ld\r\n",
                       (unsigned long)i, (long)s_st_mean[i], (long)s_st_rms[i]);
    }
    App_Log_Printf("[SELFTEST] spi_err=%lu crc_err=%lu drop=%lu\r\n",
                   (unsigned long)s_diag.spi_err, (unsigned long)s_diag.crc_err,
                   (unsigned long)s_diag.drop_cnt);

    s_st_result = (s_diag.spi_err == 0u && s_diag.drop_cnt == 0u) ? 0u : 1u;
    return (int)s_st_result;
}

/* ==================== 其他接口 ==================== */

bool ADS131M04_IsInitOk(void)
{
    return s_init_ok != 0u;
}

void ADS131M04_SetChannelSign(uint8_t sign_mask)
{
    s_sign_mask = sign_mask & 0x0Fu;
}

void ADS131M04_GetDiag(ads_diag_t *d)
{
    if (d != NULL)
    {
        d->spi_err  = s_diag.spi_err;
        d->crc_err  = s_diag.crc_err;
        d->drdy_cnt = s_diag.drdy_cnt;
        d->drop_cnt = s_diag.drop_cnt;
        d->frames_read = s_diag.frames_read;
    }
}

/* ==================== weak 回调桥接（6.5，it.c 已备好 EXTI15_10 handler） ==================== */

void HAL_GPIO_EXTI_Callback(uint16_t pin)
{
    if (pin == ADC_NDRDY_Pin)
    {
        ADS131M04_IrqDrdy();
    }
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI1)
    {
        ADS131M04_IrqDmaDone();
    }
}
