#include "comm/can_node.h"
#include "comm/app_log.h"
#include "board.h"
#include <string.h>

#define CAN_NODE_RX_QUEUE_DEPTH 4u
#define CAN_NODE_SEND_TIMEOUT_MS 2u

/* 枚举时序（多板同 ID 0x121 announce 防碰撞调优）：
 * 四块板收同一条 0x120 后按 5+hash%50 同步起跑、每 2ms 一帧连续发 4 个分片；
 * 同 ID 帧一旦重叠，CAN 仲裁无法区分发送方，数据场位错 → 双方整帧作废且发送方
 * 无感知。故：(1) 加宽初始错开窗；(2) 重试延迟按 attempt 伪随机拉大，让后续尝试
 * 与其它板错开；(3) 拉长单轮 session 生存期，避免 1000ms 掉线错过本轮收集。 */
#define CAN_NODE_INITIAL_DELAY_BASE_MS 5u
#define CAN_NODE_INITIAL_DELAY_SPAN_MS 100u   /* 50→100 */
#define CAN_NODE_RETRY_DELAY_BASE_MS 20u
#define CAN_NODE_RETRY_DELAY_SPAN_MS 700u     /* 180→700 */
#define CAN_NODE_ANNOUNCE_INTERVAL_MS 2u
#define CAN_NODE_ASSIGN_WAIT_MS 180u
#define CAN_NODE_ENUM_TIMEOUT_MS 4000u        /* 1000→4000 */
#define CAN_NODE_ANNOUNCE_FRAGMENT_COUNT 4u
#define CAN_NODE_ACK_READY_MARKER 0xA5u

typedef struct { uint16_t id; uint8_t dlc; uint8_t data[8]; } can_node_rx_item_t;
volatile CanNodeDiag_t can_node_diag;
static can_node_rx_item_t s_rx_queue[CAN_NODE_RX_QUEUE_DEPTH];
static volatile uint8_t s_rx_wr;
static uint8_t s_rx_rd;
static uint32_t s_due_ms;
static uint32_t s_enum_deadline_ms;
static uint8_t s_fragment;
static uint8_t s_ack_pending;

static uint32_t CanNode_Fnv1a(const uint8_t *data, uint8_t len)
{
    uint32_t hash = 2166136261u; uint8_t i;
    for (i = 0u; i < len; i++) { hash ^= data[i]; hash *= 16777619u; }
    return hash;
}

static uint16_t CanNode_Crc16Ccitt(const uint8_t *data, uint8_t len)
{
    uint16_t crc = 0xFFFFu; uint8_t i;
    while (len-- != 0u) { crc ^= (uint16_t)(*data++) << 8; for (i = 0u; i < 8u; i++) { crc = ((crc & 0x8000u) != 0u) ? (uint16_t)((crc << 1) ^ 0x1021u) : (uint16_t)(crc << 1); } }
    return crc;
}

static void CanNode_SetState(CanNodeState_t state, uint32_t now)
{
    can_node_diag.state = state; can_node_diag.last_state_change_ms = now;
}

static uint32_t CanNode_RetryDelayMs(void)
{
    uint8_t index = (uint8_t)((can_node_diag.announce_attempt * 5u) % CAN_NODE_UID_BYTES);
    uint32_t mixed = can_node_diag.uid_hash ^ ((uint32_t)can_node_diag.uid[index] << 16) ^ ((uint32_t)can_node_diag.announce_attempt * 0x45D9F3Bu);
    return CAN_NODE_RETRY_DELAY_BASE_MS + (mixed % CAN_NODE_RETRY_DELAY_SPAN_MS);
}

static void CanNode_ScheduleRetry(uint32_t now)
{
    can_node_diag.announce_attempt++; s_fragment = 0u; s_due_ms = now + CanNode_RetryDelayMs(); CanNode_SetState(NODE_WAIT_ANNOUNCE, now);
}

static void CanNode_StartEnumeration(uint8_t session, uint32_t now)
{
    can_node_diag.node_id = 0u; can_node_diag.session = session; can_node_diag.announce_attempt = 0u;
    s_fragment = 0u; s_ack_pending = 0u;
    s_due_ms = now + CAN_NODE_INITIAL_DELAY_BASE_MS + (can_node_diag.uid_hash % CAN_NODE_INITIAL_DELAY_SPAN_MS);
    s_enum_deadline_ms = now + CAN_NODE_ENUM_TIMEOUT_MS;
    CanNode_SetState(NODE_WAIT_ANNOUNCE, now);
    App_Log_Printf("[CAN ENUM] start session=%u\r\n", (unsigned int)session);
}

static bool CanNode_SendAnnounceFragment(uint32_t now)
{
    uint8_t data[8]; uint8_t uid_offset = (uint8_t)(s_fragment * 3u);
    data[0] = can_node_diag.session; data[1] = s_fragment;
    data[2] = can_node_diag.uid[uid_offset];
    data[3] = can_node_diag.uid[(uint8_t)(uid_offset + 1u)];
    data[4] = can_node_diag.uid[(uint8_t)(uid_offset + 2u)];
    data[5] = (uint8_t)can_node_diag.token; data[6] = (uint8_t)(can_node_diag.token >> 8); data[7] = (uint8_t)(can_node_diag.token >> 16);
    if (App_Can_SendStd(APP_CAN_PORT_1, CAN_NODE_ID_ENUM_ANNOUNCE, data, sizeof(data), CAN_NODE_SEND_TIMEOUT_MS) != APP_CAN_STATUS_OK)
    { can_node_diag.tx_fail_count++; CanNode_ScheduleRetry(now); return true; }
    s_fragment++;
    if (s_fragment >= CAN_NODE_ANNOUNCE_FRAGMENT_COUNT)
    { can_node_diag.announce_count++; s_due_ms = now + CAN_NODE_ASSIGN_WAIT_MS; CanNode_SetState(NODE_WAIT_ASSIGN, now); App_Log_Printf("[CAN ENUM] announce\r\n"); }
    else { s_due_ms = now + CAN_NODE_ANNOUNCE_INTERVAL_MS; }
    return true;
}

static bool CanNode_SendAck(void)
{
    uint8_t data[8] = {0u};
    data[0] = can_node_diag.session; data[1] = can_node_diag.node_id;
    data[2] = (uint8_t)can_node_diag.token; data[3] = (uint8_t)(can_node_diag.token >> 8); data[4] = (uint8_t)(can_node_diag.token >> 16);
    data[5] = (uint8_t)can_node_diag.uid_crc16; data[6] = (uint8_t)(can_node_diag.uid_crc16 >> 8); data[7] = CAN_NODE_ACK_READY_MARKER;
    if (App_Can_SendStd(APP_CAN_PORT_1, CAN_NODE_ID_ACK, data, sizeof(data), CAN_NODE_SEND_TIMEOUT_MS) != APP_CAN_STATUS_OK)
    { can_node_diag.tx_fail_count++; return true; }
    s_ack_pending = 0u; return true;
}

static void CanNode_ProcessAssign(const can_node_rx_item_t *item, uint32_t now)
{
    uint32_t token; uint16_t crc; uint8_t assigned_id;
    if (item->dlc != 8u || item->data[0] != can_node_diag.session || item->data[7] != 0u) { return; }
    assigned_id = item->data[1]; token = (uint32_t)item->data[2] | ((uint32_t)item->data[3] << 8) | ((uint32_t)item->data[4] << 16);
    crc = (uint16_t)item->data[5] | ((uint16_t)item->data[6] << 8);
    if (assigned_id == 0u || assigned_id > CAN_NODE_COUNT || token != can_node_diag.token || crc != can_node_diag.uid_crc16) { return; }
    can_node_diag.node_id = assigned_id; s_ack_pending = 1u;
    if (can_node_diag.state != NODE_READY) { CanNode_SetState(NODE_READY, now); App_Log_Printf("[CAN ENUM] assigned NodeID=%u\r\n", (unsigned int)assigned_id); App_Log_Printf("[CAN ENUM] ready\r\n"); }
}

void CanNode_Init(void)
{
    uint32_t uid_words[3]; uint8_t word; uint8_t byte;
    memset((void *)&can_node_diag, 0, sizeof(can_node_diag));
    uid_words[0] = HAL_GetUIDw0(); uid_words[1] = HAL_GetUIDw1(); uid_words[2] = HAL_GetUIDw2();
    for (word = 0u; word < 3u; word++) for (byte = 0u; byte < 4u; byte++) can_node_diag.uid[(uint8_t)(word * 4u + byte)] = (uint8_t)(uid_words[word] >> (byte * 8u));
    can_node_diag.uid_hash = CanNode_Fnv1a((const uint8_t *)can_node_diag.uid, CAN_NODE_UID_BYTES);
    can_node_diag.token = can_node_diag.uid_hash & 0x00FFFFFFu;
    can_node_diag.uid_crc16 = CanNode_Crc16Ccitt((const uint8_t *)can_node_diag.uid, CAN_NODE_UID_BYTES);
    CanNode_SetState(NODE_UNASSIGNED, HAL_GetTick()); s_rx_wr = 0u; s_rx_rd = 0u; s_ack_pending = 0u;
    App_Log_Printf("[CAN ENUM] UID=%08lX%08lX%08lX\r\n", (unsigned long)uid_words[2], (unsigned long)uid_words[1], (unsigned long)uid_words[0]);
}

void CanNode_OnRxIsr(const AppCanFrame *frame)
{
    uint8_t next;
    if (frame == NULL || frame->is_extended_id != 0u || !((frame->can_id == CAN_NODE_ID_ENUM_START && frame->dlc == 1u) || (frame->can_id == CAN_NODE_ID_ASSIGN && frame->dlc == 8u))) return;
    next = (uint8_t)((s_rx_wr + 1u) % CAN_NODE_RX_QUEUE_DEPTH);
    if (next == s_rx_rd) { can_node_diag.rx_drop_count++; return; }
    s_rx_queue[s_rx_wr].id = (uint16_t)frame->can_id; s_rx_queue[s_rx_wr].dlc = frame->dlc; memcpy(s_rx_queue[s_rx_wr].data, frame->data, sizeof(frame->data)); s_rx_wr = next;
}

bool CanNode_Task(void)
{
    uint32_t now = HAL_GetTick();
    while (s_rx_rd != s_rx_wr) { can_node_rx_item_t item = s_rx_queue[s_rx_rd]; s_rx_rd = (uint8_t)((s_rx_rd + 1u) % CAN_NODE_RX_QUEUE_DEPTH); if (item.id == CAN_NODE_ID_ENUM_START) CanNode_StartEnumeration(item.data[0], now); else CanNode_ProcessAssign(&item, now); }
    if (can_node_diag.state == NODE_READY && s_ack_pending != 0u) return CanNode_SendAck();
    if ((can_node_diag.state == NODE_WAIT_ANNOUNCE || can_node_diag.state == NODE_WAIT_ASSIGN) && (int32_t)(now - s_enum_deadline_ms) >= 0) { can_node_diag.node_id = 0u; CanNode_SetState(NODE_UNASSIGNED, now); App_Log_Printf("[CAN ENUM] timeout\r\n"); return false; }
    if (can_node_diag.state == NODE_WAIT_ANNOUNCE && (int32_t)(now - s_due_ms) >= 0) return CanNode_SendAnnounceFragment(now);
    if (can_node_diag.state == NODE_WAIT_ASSIGN && (int32_t)(now - s_due_ms) >= 0) CanNode_ScheduleRetry(now);
    return false;
}

bool CanNode_IsReady(void) { return can_node_diag.state == NODE_READY && can_node_diag.node_id != 0u; }
uint8_t CanNode_GetId(void) { return CanNode_IsReady() ? can_node_diag.node_id : 0u; }
uint16_t CanNode_BusinessId(uint8_t offset)
{
    if (!CanNode_IsReady() || offset > CAN_NODE_OFFSET_PARAM_ACK) return 0u;
    return (uint16_t)(CAN_NODE_BUSINESS_BASE_ID + ((uint16_t)(can_node_diag.node_id - 1u) * CAN_NODE_BUSINESS_STRIDE) + offset);
}
