/**
 ******************************************************************************
 * @file    state_machine.c
 * @brief   模块状态机实现（TIM2 1kHz 中断驱动，设计文档 v1.5 第 11 章）。
 *
 * 七状态：BOOT / NORMAL / HIT / FAULT / COMM_LOST / ID_SETUP / ID_CONFLICT。
 * 转移条件：
 *   BOOT   300ms 自检白灯 → NORMAL
 *   NORMAL 击打事件（StateMachine_OnHitEvent）→ HIT（快闪 300ms 后回 NORMAL）
 *   任一态 Fault_GetBitmap() 非零 → FAULT（故障优先，≤1ms 响应）；清零 → NORMAL
 *   COMM_LOST/ID_* 由通信事件进出（board_comm 驱动）
 ******************************************************************************
 */
#include "app/state_machine.h"
#include "app/led_status.h"
#include "app/faults.h"
#include "detect/hit_detect.h"

#define SM_BOOT_MS   300u    /* BOOT 白灯时长 */
#define SM_HIT_MS    300u    /* HIT 快闪时长（5Hz × 3 个周期 = 300ms） */

static volatile sm_state_t s_state = SM_STATE_BOOT;
static volatile uint32_t   s_state_ms;   /* 当前状态已驻留 ms 数（1kHz 累计） */
static volatile uint8_t    s_comm_lost;  /* 通信层置位；在 BOOT 结束后仍须持续生效 */

/* 进入 FAULT（故障灯效由 led_status 按 Fault_GetBitmap 分类生成） */
static void Sm_ToFault(void)
{
    s_state = SM_STATE_FAULT;
    s_state_ms = 0u;
    LedStatus_SetEffect(LED_EFF_FAULT);
}

void StateMachine_Init(void)
{
    s_state = SM_STATE_BOOT;
    s_state_ms = 0u;
    s_comm_lost = 0u;
    LedStatus_SetEffect(LED_EFF_BOOT);
}

void StateMachine_Tick(void)
{
    s_state_ms++;

    /* 安全状态优先于 BOOT/HIT/ID 等暂态。这样即使 BOOT 在掉线后完成，
     * 也不会把 COMM_LOST 覆盖为 NORMAL。
     * 合并自 44bfd88(faults 统一注册表) 与 83bd9e7(COMM_LOST 锁存)：
     * 顶层门统一以 Fault_GetBitmap() 判定，满足「任一态故障 ≤1ms 进 FAULT」。 */
    if (Fault_GetBitmap() != 0u)
    {
        if (s_state != SM_STATE_FAULT)
        {
            s_state = SM_STATE_FAULT;
            s_state_ms = 0u;
            LedStatus_SetEffect(LED_EFF_FAULT);
        }
        return;
    }
    if (s_comm_lost != 0u)
    {
        if (s_state != SM_STATE_COMM_LOST)
        {
            s_state = SM_STATE_COMM_LOST;
            s_state_ms = 0u;
            LedStatus_SetEffect(LED_EFF_COMM_LOST);
        }
        return;
    }

    switch (s_state)
    {
    case SM_STATE_BOOT:
        if (s_state_ms >= SM_BOOT_MS)
        {
            s_state = SM_STATE_NORMAL;
            s_state_ms = 0u;
            LedStatus_SetEffect(LED_EFF_NORMAL);
        }
        break;

    case SM_STATE_NORMAL:
        if (Fault_GetBitmap() != 0u)
        {
            Sm_ToFault();
        }
        break;

    case SM_STATE_HIT:
        /* 故障优先：受击闪灯期间新故障 ≤1ms 内转入 FAULT */
        if (Fault_GetBitmap() != 0u)
        {
            Sm_ToFault();
        }
        else if (s_state_ms >= SM_HIT_MS)
        {
            s_state = SM_STATE_NORMAL;
            s_state_ms = 0u;
            LedStatus_SetEffect(LED_EFF_NORMAL);
        }
        break;

    case SM_STATE_FAULT:
        if (Fault_GetBitmap() == 0u)
        {
            s_state = SM_STATE_NORMAL;
            s_state_ms = 0u;
            LedStatus_SetEffect(LED_EFF_NORMAL);
        }
        break;

    case SM_STATE_COMM_LOST:
        /* 故障优先于通信状态；无故障保持紫常亮（恢复由 OnCommLost(false) 驱动） */
        if (Fault_GetBitmap() != 0u)
        {
            Sm_ToFault();
        }
        break;

    case SM_STATE_ID_SETUP:
    case SM_STATE_ID_CONFLICT:
        if (Fault_GetBitmap() != 0u)
        {
            Sm_ToFault();
        }
        else if (s_state_ms >= 1000u)   /* 1s 显示后自动回 NORMAL */
        {
            s_state = SM_STATE_NORMAL;
            s_state_ms = 0u;
            LedStatus_SetEffect(LED_EFF_NORMAL);
        }
        break;

    default:
        break;
    }
}

void StateMachine_OnCommLost(bool lost)
{
    if (lost)
    {
        s_comm_lost = 1u;
        if (s_state != SM_STATE_FAULT)
        {
            s_state = SM_STATE_COMM_LOST;
            s_state_ms = 0u;
            LedStatus_SetEffect(LED_EFF_COMM_LOST);
        }
    }
    else
    {
        s_comm_lost = 0u;
        if (s_state == SM_STATE_COMM_LOST)
        {
            s_state = SM_STATE_NORMAL;
            s_state_ms = 0u;
            LedStatus_SetEffect(LED_EFF_NORMAL);
        }
    }
}

void StateMachine_OnIdSet(bool conflict)
{
    if (conflict)
    {
        s_state = SM_STATE_ID_CONFLICT;
    }
    else
    {
        s_state = SM_STATE_ID_SETUP;
    }
    s_state_ms = 0u;
    LedStatus_SetEffect(conflict ? LED_EFF_ID_CONFLICT : LED_EFF_ID_SETUP);
}

void StateMachine_OnHitEvent(const hit_event_t *e)
{
    (void)e;
    /* NORMAL 或 HIT 中再受击 → 刷新快闪计时（HIT 中重击不重新开始也可，取刷新） */
    if (s_state == SM_STATE_NORMAL || s_state == SM_STATE_HIT)
    {
        s_state = SM_STATE_HIT;
        s_state_ms = 0u;
        LedStatus_SetEffect(LED_EFF_HIT);
    }
    /* FAULT/COMM_LOST 态不发送只计数（11 章语义）——事件仍由 main_app 日志输出 */
}

sm_state_t StateMachine_Get(void)
{
    return (sm_state_t)s_state;
}
