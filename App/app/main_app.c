/**
 ******************************************************************************
 * @file    main_app.c
 * @brief   应用层编排（设计文档 v1.5 第 5.3/10 章）。
 *
 * 骨架阶段自证：TIM2 1kHz tick 驱动 PB1 1Hz 闪烁（tick 链路存活证明），
 * 上电经 USART1 打印 "App skeleton OK"。各模块桩按步骤填充：
 *   Step 1 ADS131M04 / Step 2 检测 / Step 3 灯 / Step 4 通信 / Step 5 收尾。
 ******************************************************************************
 */
#include "app/main_app.h"

#include "board.h"
#include "bsp/ads131m04.h"
#include "bsp/temp_mon.h"
#include "bsp/ws2812_uart.h"
#include "detect/calibration.h"
#include "detect/hit_detect.h"
#include "comm/app_can.h"
#include "comm/app_log.h"
#include "comm/board_comm.h"
#include "app/state_machine.h"
#include "app/led_status.h"
#include "app/faults.h"
#include "app/self_test.h"
#include "app/reset_cause.h"

/* ---- 运行参数（Step 5 起从 Flash 载入标定值） ---- */
static hit_param_t s_param;
static uint32_t    s_tick_ms = 0u;

/* ---- 诊断：1s 速率快照（调试器直接 watch，检查漏跑情况） ---- */
static volatile uint32_t s_rate_drdy;     /* 每秒 DRDY 中断数（期望 ≈3906） */
static volatile uint32_t s_rate_frames;   /* 每秒消费帧数（应 ≈s_rate_drdy） */
static volatile uint32_t s_rate_spi_err;  /* 每秒 SPI 错误增量 */
static volatile uint32_t s_rate_drop;     /* 每秒跳帧增量 */
static uint32_t s_last_drdy;
static uint32_t s_last_frames;
static uint32_t s_last_spi_err;
static uint32_t s_last_drop;

/* ---- 主循环阻塞诊断（DWT 周期计数，1 cycle = 12.5ns @80MHz） ---- */
static volatile uint32_t s_iter_us;        /* 最近一次 App_Loop 迭代耗时（µs） */
static volatile uint32_t s_loop_max_us;    /* 1s 窗口内单次迭代最大耗时 */
static uint32_t s_loop_max_cyc;

/* PVD 欠压恢复：FAULT_POWER sticky 后 5s 自动重试自检
 * （原 44bfd88 board_comm 职责，随固定帧通信层替换上移到固件层）。 */
static uint32_t s_last_st_retry_ms;

/* 最近一帧解析结果（调试器实时观察 ADC 数据用，每 256µs 更新一次） */
static volatile ads_frame_t s_last_frame;

void App_Init(void)
{
    ResetCause_Init();

    /* 参数：骨架阶段用默认值；TODO(Step 5)：Cal_Load 从 Flash 恢复标定值。 */
    Cal_GetDefaults(&s_param);

    App_Log_Init();
    App_Log_Printf("\r\nApp skeleton OK (v0.1)\r\n");

    Fault_Init();              /* 故障注册表 + PVD（FAULT_POWER 通路），先于故障源初始化 */
    App_Can_Init();            /* CAN 启动 + 通知使能（Step 4 补齐发送/分发） */
    ADS131M04_Init();          /* Step 1：复位 + 只读回验证（6.3）+ 方案 A 读取链 */
    if (!ADS131M04_IsInitOk())
    {
        /* 初始化失败 → sticky FAULT_SPI（仅自检通过/复位可清；live 侧由 DRDY 停滞检测兜底） */
        Fault_SetSticky(FAULT_SPI);
        App_Log_Printf("[ADS] init FAIL -> FAULT_SPI sticky\r\n");
    }
    TempMon_Init(&s_param);    /* Step 5：TIM1 触发链 + ADC1 DMA circular 启动 */
    Ws2812_Init();             /* TODO(Step 3)：发送实现 */
    HitDetect_Init(&s_param);  /* TODO(Step 2)：检测管线 */
    LedStatus_Init();          /* Step 3：灯效生成（WS2812 编码 + DMA） */
    LedStatus_SetTeamColor(s_param.team_color);
    LedStatus_SetBrightness(s_param.brightness);
    StateMachine_Init();       /* Step 3：BOOT→NORMAL/HIT/FAULT 基础版 */
    BoardComm_Init();          /* 固定帧 CAN 协议初始化 */

    /* 上电自检（7.6）：非阻塞，结果经故障位图/STATUS_REPLY 反映；
     * ADC 初始化失败时不发起（sticky FAULT_SPI 已直接进入 FAULT 链路）。 */
    SelfTest_Init(&s_param);
    if (ADS131M04_IsInitOk())
    {
        (void)SelfTest_Request();
    }
    s_last_st_retry_ms = HAL_GetTick();

    /* DWT 周期计数器使能（主循环阻塞测量，调试器 halt 期间 CYCCNT 冻结，不影响测量） */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0u;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;

    /* 启动 1kHz 系统时基（TIM2 更新中断）——CubeMX 只生成 Init 不 Start，必须显式启动；
     * 放在全部模块初始化之后，避免首个 tick 命中半初始化模块。 */
    HAL_TIM_Base_Start_IT(&htim2);
}

void App_Loop(void)
{
    uint32_t cyc0 = DWT->CYCCNT;

    /* --- 自检步进（IDLE 立即返回）；运行期间自检独占帧消费 --- */
    SelfTest_Poll();

    /* --- 帧消费 + 检测管线（Step 1/2 起启用；自检运行期由 SelfTest_Poll 消费并喂管线） --- */
    if (!SelfTest_IsRunning() && ADS131M04_IsFrameReady())
    {
        ads_frame_t frame;
        if (ADS131M04_ReadFrame(&frame) == 0)
        {
            s_last_frame = frame;   /* 调试观察点（Ozone 2Hz 刷新即可看到流动数据） */
            HitDetect_Feed(&frame);
        }
    }

    /* 击打事件输出（Step 3：状态机联动 + 调试日志；P18 bit0 事件使能开关） */
    if ((s_param.evt_enable & 0x01u) != 0u)
    {
        hit_event_t ev;
        if (HitDetect_GetEvent(&ev))
        {
            StateMachine_OnHitEvent(&ev);
            /* Fault and CAN-link-loss states may still observe a vibration,
             * but must never turn it into a referee hit/HP deduction. */
            if (StateMachine_Get() == SM_STATE_HIT)
            {
                BoardComm_ReportHitEvent(&ev);   /* 击打事件经 CAN 上报核心板 */
            }
            App_Log_Printf("[HIT] sum_peak=%lu p0=%lu p1=%lu p2=%lu p3=%lu I=%u%%\r\n",
                           (unsigned long)ev.peak,
                           (unsigned long)ev.peak_ch[0], (unsigned long)ev.peak_ch[1],
                           (unsigned long)ev.peak_ch[2], (unsigned long)ev.peak_ch[3],
                           (unsigned int)ev.intensity);
        }
    }
    else
    {
        hit_event_t ev;
        while (HitDetect_GetEvent(&ev)) { /* 事件关闭时仍取走，避免滞留 */ }
    }

    BoardComm_Loop();

    /* PVD 欠压恢复（自 44bfd88 通信层上移，行为不变）：
     * FAULT 态 + sticky FAULT_POWER + 自检空闲 → 每 5s 自动重试自检。 */
    if (StateMachine_Get() == SM_STATE_FAULT &&
        (Fault_GetSticky() & FAULT_POWER) != 0u &&
        !SelfTest_IsRunning() &&
        (int32_t)(HAL_GetTick() - s_last_st_retry_ms) >= 5000)
    {
        s_last_st_retry_ms = HAL_GetTick();
        (void)SelfTest_Request();
        App_Log_Printf("[SELFTEST] auto-retry after PVD\r\n");
    }

    /* 非阻塞日志刷出：每圈 2 字符（≈174µs），帧消费优先、日志其次（2026-08-21） */
    App_Log_FlushSmall(2u);

    /* 迭代耗时统计（80MHz：1 cycle = 12.5ns） */
    {
        uint32_t dt = DWT->CYCCNT - cyc0;   /* uint32 减法天然处理回绕 */
        s_iter_us = dt / 80u;
        if (dt > s_loop_max_cyc)
        {
            s_loop_max_cyc = dt;
        }
    }
}

void App_OnTick1ms(void)
{
    s_tick_ms++;

    StateMachine_Tick();    /* 1kHz 状态机步进（第 11 章） */
    LedStatus_Tick();       /* 1kHz 灯效相位推进（20Hz 帧刷新） */

    if ((s_tick_ms % 50u) == 0u)
    {
        TempMon_Tick();     /* 20Hz 健康评估 + TEMP 窗检（7.5，Step 5 桩） */
        HitDetect_Tick();   /* 20Hz 传感器健康评估（7.5，Step 2） */
    }

    /* PB1 已移交 LedStatus/状态机驱动（Step 3），骨架闪烁移除 */

    /* 诊断：每秒快照 DRDY 速率/错误增量/帧消费速率（Step 1 验证用） */
    if ((s_tick_ms % 1000u) == 0u)
    {
        ads_diag_t d;
        ADS131M04_GetDiag(&d);
        s_rate_drdy    = d.drdy_cnt - s_last_drdy;
        s_last_drdy    = d.drdy_cnt;
        s_rate_frames  = d.frames_read - s_last_frames;
        s_last_frames  = d.frames_read;
        s_rate_spi_err = d.spi_err - s_last_spi_err;
        s_last_spi_err = d.spi_err;
        s_rate_drop    = d.drop_cnt - s_last_drop;
        s_last_drop    = d.drop_cnt;

        s_loop_max_us  = s_loop_max_cyc / 80u;
        s_loop_max_cyc = 0u;
    }
}

/* ==================== weak 回调覆写（it.c 已备好 TIM2_IRQHandler） ==================== */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2)
    {
        App_OnTick1ms();
    }
}
