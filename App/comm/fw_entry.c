#include "comm/fw_entry.h"
#include "comm/fw_layout.h"
#include "board.h"
#include <string.h>

#define FW_REQUEST_MAGIC 0x46574201u
#define FW_SELECT_TIMEOUT_MS 500u

typedef struct {
    uint8_t data[8];
    uint8_t dlc;
} fw_rx_t;

static volatile uint8_t s_pending;
static fw_rx_t s_rx;
static uint8_t s_uid[12];
static uint8_t s_mask;
static uint8_t s_session;
static uint32_t s_last_ms;
static uint8_t s_discovery_session;
static uint8_t s_discovery_part;
static uint32_t s_discovery_due;
static volatile uint8_t s_maintenance;

static uint16_t FwEntry_UidCrc(void)
{
    uint16_t crc = 0xFFFFu;
    uint8_t i, bit;
    for (i = 0u; i < 12u; ++i)
    {
        crc ^= (uint16_t)s_uid[i] << 8;
        for (bit = 0u; bit < 8u; ++bit)
            crc = (uint16_t)((crc << 1) ^ ((crc & 0x8000u) ? 0x1021u : 0u));
    }
    return crc;
}

void FwEntry_Init(void)
{
    uint32_t words[3] = {HAL_GetUIDw0(), HAL_GetUIDw1(), HAL_GetUIDw2()};
    uint8_t i;
    for (i = 0u; i < 12u; ++i) s_uid[i] = (uint8_t)(words[i / 4u] >> (8u * (i % 4u)));
    s_pending = 0u;
    s_mask = 0u;
    s_session = 0u;
    s_last_ms = 0u;
    s_discovery_due = 0u;
    s_maintenance = 0u;
}

void FwEntry_OnRxIsr(const AppCanFrame *frame)
{
    if (frame == NULL || frame->is_extended_id != 0u || frame->can_id != FW_CAN_CMD_ID ||
        frame->dlc != 8u || s_pending != 0u) return;
    memcpy(s_rx.data, frame->data, 8u);
    s_rx.dlc = frame->dlc;
    s_pending = 1u;
}

bool FwEntry_Task(void)
{
    uint8_t data[8];
    uint8_t i;
    uint8_t fragment;
    uint32_t now = HAL_GetTick();
    if (s_discovery_due != 0u && s_discovery_part < 4u &&
        (int32_t)(now - s_discovery_due) >= 0)
    {
        uint8_t reply[8] = {FW_CMD_DISCOVER_REPLY, s_discovery_session, s_discovery_part, 0u};
        uint16_t crc = FwEntry_UidCrc();
        for (i = 0u; i < 3u; ++i) reply[3u + i] = s_uid[s_discovery_part * 3u + i];
        reply[6] = (uint8_t)crc;
        reply[7] = (uint8_t)(crc >> 8);
        if (App_Can_SendStd(APP_CAN_PORT_1, FW_CAN_STATUS_ID, reply, 8u, 2u) == APP_CAN_STATUS_OK)
        {
            ++s_discovery_part;
            s_discovery_due = s_discovery_part < 4u ? now + 2u : 0u;
            return true;
        }
    }
    if (s_mask != 0u && (uint32_t)(now - s_last_ms) > FW_SELECT_TIMEOUT_MS) s_mask = 0u;
    if (s_pending == 0u) return false;
    memcpy(data, s_rx.data, 8u);
    s_pending = 0u;
    if (data[0] == FW_CMD_DISCOVER && data[2] == 0u && data[3] == 0u &&
        data[4] == 0u && data[5] == 0u && data[6] == 0u && data[7] == 0u)
    {
        s_discovery_session = data[1];
        s_discovery_part = 0u;
        s_discovery_due = now + 5u + ((uint32_t)s_uid[0] + s_uid[5] * 17u + s_uid[11] * 31u + data[1]) % 180u;
        return false;
    }
    if (data[0] == FW_CMD_SELECT && data[1] < 3u)
    {
        fragment = data[1];
        if (s_mask != 0u && s_session != data[2]) s_mask = 0u;
        s_session = data[2];
        s_last_ms = now;
        for (i = 0u; i < 4u; ++i)
        {
            if (data[3u + i] != s_uid[fragment * 4u + i]) { s_mask = 0u; return false; }
        }
        if (data[7] != 0u) { s_mask = 0u; return false; }
        s_mask |= (uint8_t)(1u << fragment);
        s_maintenance = 1u;
        return false;
    }
    if (data[0] != FW_CMD_ENTER || data[1] != s_session || s_mask != 0x07u ||
        data[2] != 0u || data[3] != 0u || data[4] != 0u ||
        data[5] != 0u || data[6] != 0u || data[7] != 0u) return false;
    s_mask = 0u;
    if (*(const volatile uint32_t *)FW_BOOT_SIGNATURE_ADDR != FW_BOOT_SIGNATURE) return false;
    RCC->APB1ENR1 |= RCC_APB1ENR1_PWREN | RCC_APB1ENR1_RTCAPBEN;
    PWR->CR1 |= PWR_CR1_DBP;
    RTC->BKP1R = FW_REQUEST_MAGIC;
    __DSB();
    NVIC_SystemReset();
    return true;
}

bool FwEntry_IsMaintenance(void)
{
    return s_maintenance != 0u;
}
