/**
 ******************************************************************************
 * @file    board_comm.h
 * @brief   装甲板固定帧 CAN 协议接口。
 *
 * 所有报文均为 500kbps、标准 11 位 CAN 数据帧；完整定义见
 * Docs/CAN_PROTOCOL.md。协议不使用 ISO-TP、TLV 或应用层封包。
 ******************************************************************************
 */
#ifndef APP_BOARD_COMM_H
#define APP_BOARD_COMM_H

#include <stdint.h>
#include "detect/hit_detect.h"

/* NodeID 分配完成后，每块板使用自己的 CAN ID 槽位。 */
#define BOARD_CAN_CMD_LED_NORMAL     0x00u
#define BOARD_CAN_CMD_TEAM_RED       0x01u
#define BOARD_CAN_CMD_TEAM_BLUE      0x02u
#define BOARD_CAN_CMD_BRIGHTNESS     0x03u
#define BOARD_CAN_CMD_EFFECT         0x04u

void BoardComm_Init(void);
void BoardComm_Loop(void);
void BoardComm_ReportHitEvent(const hit_event_t *e);
void BoardComm_ReportStatus(uint16_t fault_flags);

#endif /* APP_BOARD_COMM_H */
