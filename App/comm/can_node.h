#ifndef APP_CAN_NODE_H
#define APP_CAN_NODE_H

#include <stdint.h>
#include <stdbool.h>
#include "comm/app_can.h"

#define CAN_NODE_UID_BYTES 12u
#define CAN_NODE_COUNT 4u
#define CAN_NODE_ID_ENUM_START 0x120u
#define CAN_NODE_ID_ENUM_ANNOUNCE 0x121u
#define CAN_NODE_ID_ASSIGN 0x122u
#define CAN_NODE_ID_ACK 0x123u
#define CAN_NODE_BUSINESS_BASE_ID 0x130u
#define CAN_NODE_BUSINESS_STRIDE 0x10u
#define CAN_NODE_OFFSET_RESET_CAUSE 0x0u
#define CAN_NODE_OFFSET_FAULT_STATUS 0x1u
#define CAN_NODE_OFFSET_INIT_DONE 0x2u
#define CAN_NODE_OFFSET_HIT_EVENT 0x3u
#define CAN_NODE_OFFSET_STATUS_QUERY 0x4u
#define CAN_NODE_OFFSET_STATUS_REPLY 0x5u
#define CAN_NODE_OFFSET_LED_CONTROL 0x6u
#define CAN_NODE_OFFSET_CONTROL_ACK 0x7u
#define CAN_NODE_OFFSET_NODE_ID_QUERY 0x8u
#define CAN_NODE_OFFSET_NODE_ID_REPLY 0x9u
#define CAN_NODE_OFFSET_HIT_ACK 0xAu

typedef enum { NODE_UNASSIGNED = 0, NODE_WAIT_ANNOUNCE, NODE_WAIT_ASSIGN, NODE_READY } CanNodeState_t;

typedef struct
{
    uint8_t uid[CAN_NODE_UID_BYTES];
    uint32_t uid_hash;
    uint32_t token;
    uint16_t uid_crc16;
    uint8_t node_id;
    CanNodeState_t state;
    uint8_t session;
    uint8_t announce_attempt;
    uint32_t announce_count;
    uint32_t tx_fail_count;
    uint32_t rx_drop_count;
    uint32_t last_state_change_ms;
} CanNodeDiag_t;

extern volatile CanNodeDiag_t can_node_diag;
void CanNode_Init(void);
void CanNode_OnRxIsr(const AppCanFrame *frame);
bool CanNode_Task(void);
bool CanNode_IsReady(void);
uint8_t CanNode_GetId(void);
uint16_t CanNode_BusinessId(uint8_t offset);

#endif
