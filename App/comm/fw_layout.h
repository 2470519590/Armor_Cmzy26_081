#ifndef APP_FW_LAYOUT_H
#define APP_FW_LAYOUT_H

#include <stdint.h>

#define FW_BOOT_BASE 0x08000000u
#define FW_BOOT_SIZE 0x00004000u
#define FW_SLOT_A_BASE 0x08004000u
#define FW_SLOT_B_BASE 0x08010000u
#define FW_SLOT_SIZE 0x0000C000u
#define FW_META_BASE 0x0801C000u
#define FW_META_SIZE 0x00004000u
#define FW_FLASH_END 0x08020000u
#define FW_PAGE_SIZE 2048u
#define FW_BOOT_SIGNATURE_ADDR (FW_BOOT_BASE + 0x400u)
#define FW_BOOT_SIGNATURE 0x31424F46u

#define FW_CAN_CMD_ID 0x100u
#define FW_CAN_DATA_ID 0x101u
#define FW_CAN_ACK_ID 0x102u
#define FW_CAN_STATUS_ID 0x103u

#define FW_CMD_DISCOVER 1u
#define FW_CMD_SELECT 2u
#define FW_CMD_ENTER 3u
#define FW_CMD_BEGIN 4u
#define FW_CMD_END 5u
#define FW_CMD_ABORT 6u
#define FW_CMD_INFO 7u
#define FW_CMD_CONFIRM 8u
#define FW_CMD_DATA_ACK 9u
#define FW_CMD_DISCOVER_REPLY 10u
#define FW_CMD_BEGIN_SIZE 0u
#define FW_CMD_BEGIN_CRC 1u
#define FW_STATUS_OK 0u
#define FW_STATUS_INVALID 1u
#define FW_STATUS_BUSY 2u
#define FW_STATUS_FLASH 3u
#define FW_STATUS_CRC 4u
#define FW_CONFIRM_MAGIC 0x46574301u
#define FW_TRIAL_MAGIC 0x46575401u

static inline uint32_t Fw_OtherSlot(uint32_t slot)
{
    return slot == FW_SLOT_A_BASE ? FW_SLOT_B_BASE : FW_SLOT_A_BASE;
}

#endif
