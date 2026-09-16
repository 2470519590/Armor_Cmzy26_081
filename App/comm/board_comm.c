/**
 ******************************************************************************
 * @file    board_comm.c
 * @brief   装甲板固定帧 CAN 协议实现。
 *
 * 接收中断只复制小帧；所有控制、应答与发送均在主循环执行，
 * 防止 CAN 事务阻塞 DRDY/SPI 采样链路。
 ******************************************************************************
 */
#include "comm/board_comm.h"

#include "board.h"
#include "comm/app_can.h"
#include "comm/can_node.h"
#include "app/led_status.h"
#include "app/reset_cause.h"
#include "app/state_machine.h"
#include "bsp/temp_mon.h"
#include "detect/hit_detect.h"

#include <string.h>

#define BOARD_FAULT_REPEAT_MS    100u
#define BOARD_INIT_RETRY_MS      500u
#define BOARD_RX_QUEUE_DEPTH     4u
#define BOARD_HIT_QUEUE_DEPTH    4u
#define BOARD_CAN_SEND_TIMEOUT   2u
#define BOARD_COMM_LOST_MS        300u
#define BOARD_HIT_ACK_RETRY_MS    100u

typedef struct { uint16_t id; uint8_t dlc; uint8_t data[8]; } board_rx_item_t;
typedef struct { uint8_t data[8]; uint8_t sequence; } board_hit_item_t;

static board_rx_item_t s_rx_queue[BOARD_RX_QUEUE_DEPTH];
static volatile uint8_t s_rx_wr;
static uint8_t s_rx_rd;
static volatile uint32_t s_rx_drop;

static board_hit_item_t s_hit_queue[BOARD_HIT_QUEUE_DEPTH];
static uint8_t s_hit_wr;
static uint8_t s_hit_rd;
static uint8_t s_hit_count;
static volatile uint32_t s_hit_drop;

static uint32_t s_last_fault_tx_ms;
static uint32_t s_last_fault_attempt_ms;
static uint32_t s_last_reset_attempt_ms;
static uint32_t s_last_init_attempt_ms;
static uint32_t s_last_hit_attempt_ms;
static volatile uint32_t s_last_l431_rx_ms;
static uint8_t s_hit_sequence;
static uint16_t s_fault_last_sent;
static uint8_t s_init_pending;
static uint8_t s_reset_pending;

/* 调试观察值。 */
static volatile uint32_t s_sent_count;
static volatile uint32_t s_rx_count;

static void Bc_CanRxBridge(AppCanPort port, const AppCanFrame *frame);

static bool Bc_Send(uint16_t id, const uint8_t *data, uint8_t dlc)
{
    if (id == 0u || !CanNode_IsReady())
    {
        return false;
    }
    if (App_Can_SendStd(APP_CAN_PORT_1, id, data, dlc, BOARD_CAN_SEND_TIMEOUT) == APP_CAN_STATUS_OK)
    {
        s_sent_count++;
        return true;
    }
    return false;
}

static void Bc_PutU16Le(uint8_t *dst, uint16_t value)
{
    dst[0] = (uint8_t)(value & 0xFFu);
    dst[1] = (uint8_t)(value >> 8);
}

static bool Bc_SendStatusReply(void)
{
    AppCanDiag diag;
    uint8_t data[8] = {0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u};
    data[0] = (uint8_t)StateMachine_Get();
    data[1] = ResetCause_GetLatched();
    Bc_PutU16Le(&data[2], HitDetect_GetFaultFlags());
    Bc_PutU16Le(&data[4], TempMon_GetFaultFlags());
    App_Can_GetDiag(APP_CAN_PORT_1, &diag);
    data[6] = (uint8_t)((diag.error_warning != 0u ? 0x01u : 0u) |
                        (diag.error_passive != 0u ? 0x02u : 0u) |
                        (diag.bus_off != 0u ? 0x04u : 0u));
    return Bc_Send(CanNode_BusinessId(CAN_NODE_OFFSET_STATUS_REPLY), data, sizeof(data));
}

static bool Bc_SendNodeIdReply(void)
{
    uint8_t data[8] = {0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u};
    data[0] = CanNode_GetId();
    data[1] = (uint8_t)can_node_diag.token;
    data[2] = (uint8_t)(can_node_diag.token >> 8);
    data[3] = (uint8_t)(can_node_diag.token >> 16);
    Bc_PutU16Le(&data[4], can_node_diag.uid_crc16);
    return Bc_Send(CanNode_BusinessId(CAN_NODE_OFFSET_NODE_ID_REPLY), data, sizeof(data));
}

static bool Bc_AllZero(const uint8_t data[8])
{
    uint8_t i;
    for (i = 0u; i < 8u; i++)
    {
        if (data[i] != 0u) { return false; }
    }
    return true;
}

static bool Bc_ExecuteLedControl(const uint8_t data[8])
{
    uint8_t i;

    /* 故障/掉线指示属于安全状态机，远端灯控不得覆盖。 */
    if (StateMachine_Get() == SM_STATE_FAULT || StateMachine_Get() == SM_STATE_COMM_LOST)
    {
        return false;
    }

    for (i = 2u; i < 8u; i++)
    {
        if (data[i] != 0u) { return false; }
    }

    switch (data[0])
    {
    case BOARD_CAN_CMD_LED_NORMAL: LedStatus_SetEffect(LED_EFF_NORMAL); break;
    case BOARD_CAN_CMD_TEAM_RED:   LedStatus_SetTeamColor(0u); break;
    case BOARD_CAN_CMD_TEAM_BLUE:  LedStatus_SetTeamColor(1u); break;
    case BOARD_CAN_CMD_BRIGHTNESS:
        if (data[1] > 100u) { return false; }
        LedStatus_SetBrightness(data[1]);
        break;
    case BOARD_CAN_CMD_EFFECT:
        if (data[1] >= (uint8_t)LED_EFF_COUNT) { return false; }
        LedStatus_SetEffect((led_effect_t)data[1]);
        break;
    default: return false;
    }
    return true;
}

/* 一次最多处理一个会产生发送的请求；避免同步 CAN 发送在单圈累积。 */
static bool Bc_ProcessRxQueue(void)
{
    while (s_rx_rd != s_rx_wr)
    {
        board_rx_item_t item = s_rx_queue[s_rx_rd];
        s_rx_rd = (uint8_t)((s_rx_rd + 1u) % BOARD_RX_QUEUE_DEPTH);
        if (item.id == CanNode_BusinessId(CAN_NODE_OFFSET_STATUS_QUERY))
        {
            (void)Bc_SendStatusReply();
            return true;
        }
        else if (item.id == CanNode_BusinessId(CAN_NODE_OFFSET_NODE_ID_QUERY))
        {
            (void)Bc_SendNodeIdReply();
            return true;
        }
        else if (item.id == CanNode_BusinessId(CAN_NODE_OFFSET_LED_CONTROL) && Bc_ExecuteLedControl(item.data))
        {
            (void)Bc_Send(CanNode_BusinessId(CAN_NODE_OFFSET_CONTROL_ACK), item.data, 8u);
            return true;
        }
        else if (item.id == CanNode_BusinessId(CAN_NODE_OFFSET_HIT_ACK) && item.dlc == 1u &&
                 s_hit_count != 0u && item.data[0] == s_hit_queue[s_hit_rd].sequence)
        {
            s_hit_rd = (uint8_t)((s_hit_rd + 1u) % BOARD_HIT_QUEUE_DEPTH);
            s_hit_count--;
        }
    }
    return false;
}

static bool Bc_ProcessHitQueue(uint32_t now)
{
    if (s_hit_count != 0u && (now - s_last_hit_attempt_ms) >= BOARD_HIT_ACK_RETRY_MS)
    {
        s_last_hit_attempt_ms = now;
        if (Bc_Send(CanNode_BusinessId(CAN_NODE_OFFSET_HIT_EVENT), s_hit_queue[s_hit_rd].data, 8u))
        {
            /* Keep the event until the L431 application confirms this
             * sequence. CAN TXOK alone cannot prove its software consumed it. */
        }
        return true;
    }
    return false;
}

void BoardComm_Init(void)
{
    uint32_t now = HAL_GetTick();
    s_rx_wr = 0u; s_rx_rd = 0u; s_rx_drop = 0u;
    s_hit_wr = 0u; s_hit_rd = 0u; s_hit_count = 0u; s_hit_drop = 0u;
    s_last_fault_tx_ms = now;
    s_last_fault_attempt_ms = now - BOARD_FAULT_REPEAT_MS;
    s_last_reset_attempt_ms = now - BOARD_INIT_RETRY_MS;
    s_last_init_attempt_ms = now - BOARD_INIT_RETRY_MS;
    s_last_hit_attempt_ms = now - BOARD_HIT_ACK_RETRY_MS;
    s_hit_sequence = 0u;
    s_last_l431_rx_ms = now;
    s_fault_last_sent = 0xFFFFu; /* 保证首次循环发送当前状态。 */
    s_init_pending = 1u;
    s_reset_pending = (ResetCause_GetLatched() != 0u) ? 1u : 0u;
    s_sent_count = 0u; s_rx_count = 0u;
    CanNode_Init();
    App_Can_SetRxCallback(Bc_CanRxBridge);
}

void BoardComm_Loop(void)
{
    uint32_t now = HAL_GetTick();
    uint16_t faults;
    bool tx_attempted;

    App_Can_RecoverBusOff();
    StateMachine_OnCommLost((uint32_t)(now - s_last_l431_rx_ms) > BOARD_COMM_LOST_MS);
    tx_attempted = CanNode_Task();

    if (!CanNode_IsReady())
    {
        return;
    }

    /* 异常复位启动时必须先发送 0x110，再发送初始化完成帧。 */
    if (s_reset_pending != 0u && (now - s_last_reset_attempt_ms) >= BOARD_INIT_RETRY_MS)
    {
        uint8_t data = ResetCause_GetLatched();
        s_last_reset_attempt_ms = now;
        if (Bc_Send(CanNode_BusinessId(CAN_NODE_OFFSET_RESET_CAUSE), &data, 1u)) { s_reset_pending = 0u; }
        tx_attempted = true;
    }

    if (!tx_attempted && s_init_pending != 0u && (now - s_last_init_attempt_ms) >= BOARD_INIT_RETRY_MS)
    {
        s_last_init_attempt_ms = now;
        if (Bc_Send(CanNode_BusinessId(CAN_NODE_OFFSET_INIT_DONE), NULL, 0u)) { s_init_pending = 0u; }
        tx_attempted = true;
    }
    if (!tx_attempted)
    {
        tx_attempted = Bc_ProcessRxQueue();
    }
    faults = HitDetect_GetFaultFlags();
    if (!tx_attempted && ((faults != s_fault_last_sent) || ((faults != 0u) && ((now - s_last_fault_tx_ms) >= BOARD_FAULT_REPEAT_MS))) &&
        ((now - s_last_fault_attempt_ms) >= BOARD_FAULT_REPEAT_MS))
    {
        uint8_t data[2];
        s_last_fault_attempt_ms = now;
        Bc_PutU16Le(data, faults);
        if (Bc_Send(CanNode_BusinessId(CAN_NODE_OFFSET_FAULT_STATUS), data, sizeof(data)))
        {
            s_fault_last_sent = faults;
            s_last_fault_tx_ms = now;
        }
        tx_attempted = true;
    }

    if (!tx_attempted)
    {
        (void)Bc_ProcessHitQueue(now);
    }
}

void BoardComm_ReportHitEvent(const hit_event_t *e)
{
    board_hit_item_t *item;
    if (e == NULL) { return; }
    if (s_hit_count >= BOARD_HIT_QUEUE_DEPTH) { s_hit_drop++; return; }

    item = &s_hit_queue[s_hit_wr];
    item->sequence = s_hit_sequence++;
    /* HIT v2 keeps the useful diagnostics while adding a de-duplication key:
     * seq, channel, intensity, force[2], peak low 24 bits. */
    item->data[0] = item->sequence;
    item->data[1] = e->ch;
    item->data[2] = e->intensity;
    Bc_PutU16Le(&item->data[3], e->force_01n);
    item->data[5] = (uint8_t)e->peak;
    item->data[6] = (uint8_t)(e->peak >> 8);
    item->data[7] = (uint8_t)(e->peak >> 16);
    s_hit_wr = (uint8_t)((s_hit_wr + 1u) % BOARD_HIT_QUEUE_DEPTH);
    s_hit_count++;
}

void BoardComm_ReportStatus(uint16_t fault_flags)
{
    (void)fault_flags;
    /* 故障位图由 BoardComm_Loop 统一仲裁发送，保证每圈最多一帧。 */
    s_fault_last_sent = 0xFFFFu;
}

/* CAN RX ISR context: only supported small-frame queue handling. */
static void Bc_CanRxBridge(AppCanPort port, const AppCanFrame *frame)
{
    uint8_t next;
    if (port != APP_CAN_PORT_1 || frame == NULL || frame->is_extended_id != 0u) { return; }

    CanNode_OnRxIsr(frame);
    if (!CanNode_IsReady()) { return; }

    if (!((frame->can_id == CanNode_BusinessId(CAN_NODE_OFFSET_STATUS_QUERY) &&
           (frame->dlc == 0u || (frame->dlc == 8u && Bc_AllZero(frame->data)))) ||
          (frame->can_id == CanNode_BusinessId(CAN_NODE_OFFSET_NODE_ID_QUERY) && frame->dlc == 0u) ||
          (frame->can_id == CanNode_BusinessId(CAN_NODE_OFFSET_LED_CONTROL) && frame->dlc == 8u) ||
          (frame->can_id == CanNode_BusinessId(CAN_NODE_OFFSET_HIT_ACK) && frame->dlc == 1u))) { return; }

    /* A correctly addressed L431 request is the heartbeat.  Do not count
     * unrelated CAN traffic as proof that the referee/power manager is alive. */
    s_last_l431_rx_ms = HAL_GetTick();

    next = (uint8_t)((s_rx_wr + 1u) % BOARD_RX_QUEUE_DEPTH);
    if (next == s_rx_rd) { s_rx_drop++; return; }
    s_rx_queue[s_rx_wr].id = (uint16_t)frame->can_id;
    s_rx_queue[s_rx_wr].dlc = frame->dlc;
    memcpy(s_rx_queue[s_rx_wr].data, frame->data, sizeof(frame->data));
    s_rx_wr = next;
    s_rx_count++;
}
