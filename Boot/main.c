#include "stm32l4xx_hal.h"
#include "comm/fw_layout.h"
#include <string.h>

#define BOOT_REQUEST_MAGIC 0x46574201u
#define META_MAGIC 0x3141544Du
#define META_PAGE_A FW_META_BASE
#define META_PAGE_B (FW_META_BASE + FW_PAGE_SIZE)
#define IMAGE_PENDING 1u
#define IMAGE_CONFIRMED 2u
#define IMAGE_DOWNLOADING 3u
#define IMAGE_TRIAL 4u
#define FW_DATA_WINDOW 32u

typedef struct {
    uint32_t magic;
    uint32_t sequence;
    uint32_t active_slot;
    uint32_t pending_slot;
    uint32_t a_size;
    uint32_t a_crc;
    uint32_t b_size;
    uint32_t b_crc;
    uint32_t state;
    uint32_t record_crc;
} boot_meta_t;

__attribute__((used, section(".boot_signature"))) const uint64_t boot_signature =
    ((uint64_t)~FW_BOOT_SIGNATURE << 32) | FW_BOOT_SIGNATURE;

static CAN_HandleTypeDef s_can;
static boot_meta_t s_meta;
static uint32_t s_meta_page;
static uint32_t s_target;
static uint32_t s_size;
static uint32_t s_expected_crc;
static uint32_t s_offset;
static uint8_t s_session;
static uint8_t s_selected;
static uint8_t s_uid[12];
static uint8_t s_select_mask;
static uint8_t s_select_session;
static uint8_t s_receiving;
static uint8_t s_begin_size;
static uint8_t s_begin_crc;
static uint8_t s_staging[8];
static uint8_t s_staging_count;
static uint16_t s_packet;
static uint32_t s_erased_page;
static uint32_t s_last_activity;
static uint32_t s_discovery_due;
static uint8_t s_discovery_part;
static uint8_t s_discovery_session;

/* HAL_Init() starts the 1 ms SysTick used by HAL_GetTick()/HAL_Delay().
   The standalone Bootloader does not link the application's interrupt file,
   so it must provide this handler itself; otherwise the weak default handler
   traps on the first tick and CAN processing never runs. */
void SysTick_Handler(void)
{
    HAL_IncTick();
}

static uint16_t UidCrc(void)
{
    uint16_t crc = 0xFFFFu;
    uint8_t i, bit;
    for (i = 0u; i < 12u; ++i) {
        crc ^= (uint16_t)s_uid[i] << 8;
        for (bit = 0u; bit < 8u; ++bit)
            crc = (uint16_t)((crc << 1) ^ ((crc & 0x8000u) ? 0x1021u : 0u));
    }
    return crc;
}

static void ClockInit(void)
{
    RCC_OscInitTypeDef osc = {0};
    RCC_ClkInitTypeDef clk = {0};
    osc.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    osc.HSIState = RCC_HSI_ON;
    osc.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    osc.PLL.PLLState = RCC_PLL_OFF;
    if (HAL_RCC_OscConfig(&osc) != HAL_OK) while (1) { }
    clk.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK |
                    RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
    clk.AHBCLKDivider = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV1;
    clk.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_0) != HAL_OK) while (1) { }
}

static uint32_t Crc32(const uint8_t *data, uint32_t length)
{
    uint32_t crc = 0xFFFFFFFFu;
    uint32_t i;
    for (i = 0u; i < length; ++i) {
        uint8_t bit;
        crc ^= data[i];
        for (bit = 0u; bit < 8u; ++bit)
            crc = (crc >> 1) ^ ((crc & 1u) != 0u ? 0xEDB88320u : 0u);
    }
    return ~crc;
}

static uint8_t ImageVectorOk(uint32_t base)
{
    uint32_t sp = *(const uint32_t *)base;
    uint32_t pc = *(const uint32_t *)(base + 4u);
    return ((sp >= 0x20000000u && sp <= 0x2000C000u) ||
            (sp >= 0x10000000u && sp <= 0x10004000u)) &&
           (pc & 1u) != 0u && (pc & ~1u) >= base &&
           (pc & ~1u) < base + FW_SLOT_SIZE;
}

static uint8_t ImageOk(uint32_t base, uint32_t size, uint32_t crc)
{
    return size >= 8u && size <= FW_SLOT_SIZE && ImageVectorOk(base) &&
           Crc32((const uint8_t *)base, size) == crc;
}

static uint8_t MetaOk(const boot_meta_t *m)
{
    return m->magic == META_MAGIC &&
           (m->active_slot == FW_SLOT_A_BASE || m->active_slot == FW_SLOT_B_BASE) &&
           (m->pending_slot == 0u || m->pending_slot == FW_SLOT_A_BASE || m->pending_slot == FW_SLOT_B_BASE) &&
           m->record_crc == Crc32((const uint8_t *)m, sizeof(*m) - 4u);
}

static void MetaLoad(void)
{
    const boot_meta_t *a = (const boot_meta_t *)META_PAGE_A;
    const boot_meta_t *b = (const boot_meta_t *)META_PAGE_B;
    uint8_t av = MetaOk(a), bv = MetaOk(b);
    if (av && (!bv || (int32_t)(a->sequence - b->sequence) > 0)) {
        s_meta = *a; s_meta_page = META_PAGE_A;
    } else if (bv) {
        s_meta = *b; s_meta_page = META_PAGE_B;
    } else {
        memset(&s_meta, 0, sizeof(s_meta));
        s_meta.magic = META_MAGIC;
        s_meta.active_slot = FW_SLOT_A_BASE;
        s_meta_page = META_PAGE_B;
    }
}

static uint8_t MetaSave(void)
{
    FLASH_EraseInitTypeDef erase = {0};
    uint32_t error = 0u;
    uint32_t page = s_meta_page == META_PAGE_A ? META_PAGE_B : META_PAGE_A;
    uint32_t i;
    const uint64_t *words;
    s_meta.magic = META_MAGIC;
    ++s_meta.sequence;
    s_meta.record_crc = Crc32((const uint8_t *)&s_meta, sizeof(s_meta) - 4u);
    erase.TypeErase = FLASH_TYPEERASE_PAGES;
    erase.Banks = FLASH_BANK_1;
    erase.Page = (page - FW_BOOT_BASE) / FW_PAGE_SIZE;
    erase.NbPages = 1u;
    if (HAL_FLASH_Unlock() != HAL_OK) return 0u;
    if (HAL_FLASHEx_Erase(&erase, &error) != HAL_OK) { HAL_FLASH_Lock(); return 0u; }
    words = (const uint64_t *)&s_meta;
    for (i = 0u; i < sizeof(s_meta) / 8u; ++i) {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, page + i * 8u, words[i]) != HAL_OK) {
            HAL_FLASH_Lock(); return 0u;
        }
    }
    HAL_FLASH_Lock();
    if (!MetaOk((const boot_meta_t *)page)) return 0u;
    s_meta_page = page;
    return 1u;
}

static void Jump(uint32_t base)
{
    uint32_t sp = *(const uint32_t *)base;
    uint32_t pc = *(const uint32_t *)(base + 4u);
    __disable_irq();
    if (s_can.Instance == CAN1 && HAL_CAN_GetState(&s_can) == HAL_CAN_STATE_LISTENING) {
        (void)HAL_CAN_Stop(&s_can);
        (void)HAL_CAN_DeInit(&s_can);
    }
    SysTick->CTRL = 0u;
    NVIC->ICER[0] = 0xFFFFFFFFu;
    NVIC->ICER[1] = 0xFFFFFFFFu;
    NVIC->ICPR[0] = 0xFFFFFFFFu;
    NVIC->ICPR[1] = 0xFFFFFFFFu;
    SCB->VTOR = base;
    /* Jump() disables IRQs while tearing down the Bootloader.  The
       application expects normal interrupt delivery after HAL_Init(); leave
       PRIMASK clear before entering its reset handler. */
    __enable_irq();
    __set_MSP(sp);
    ((void (*)(void))pc)();
    while (1) { }
}

static void CanInit(void)
{
    GPIO_InitTypeDef gpio = {0};
    CAN_FilterTypeDef filter = {0};
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_CAN1_CLK_ENABLE();
    gpio.Pin = GPIO_PIN_11 | GPIO_PIN_12;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOA, &gpio);
    s_can.Instance = CAN1;
    s_can.Init.Prescaler = 2u;
    s_can.Init.Mode = CAN_MODE_NORMAL;
    s_can.Init.SyncJumpWidth = CAN_SJW_1TQ;
    s_can.Init.TimeSeg1 = CAN_BS1_13TQ;
    s_can.Init.TimeSeg2 = CAN_BS2_2TQ;
    s_can.Init.AutoRetransmission = ENABLE;
    s_can.Init.ReceiveFifoLocked = DISABLE;
    if (HAL_CAN_Init(&s_can) != HAL_OK) while (1) { }
    filter.FilterBank = 0u;
    /* Receive only the two firmware transport IDs.  The previous 32-bit
       mask (0xFFC0) admitted the whole 0x100..0x13F range, including the
       L431/competition traffic at 0x120+, which could fill FIFO0 while a
       data window was arriving and make packet numbers appear to skip. */
    filter.FilterMode = CAN_FILTERMODE_IDLIST;
    filter.FilterScale = CAN_FILTERSCALE_16BIT;
    filter.FilterIdHigh = (uint16_t)(FW_CAN_CMD_ID << 5);
    filter.FilterIdLow = (uint16_t)(FW_CAN_DATA_ID << 5);
    filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    filter.FilterActivation = ENABLE;
    if (HAL_CAN_ConfigFilter(&s_can, &filter) != HAL_OK || HAL_CAN_Start(&s_can) != HAL_OK) while (1) { }
}

static void Send(uint16_t id, const uint8_t data[8])
{
    CAN_TxHeaderTypeDef header = {0};
    uint32_t mailbox;
    uint32_t start = HAL_GetTick();
    header.StdId = id;
    header.IDE = CAN_ID_STD;
    header.RTR = CAN_RTR_DATA;
    header.DLC = 8u;
    while (HAL_CAN_GetTxMailboxesFreeLevel(&s_can) == 0u) {
        if (HAL_GetTick() - start > 20u) return;
    }
    if (HAL_CAN_AddTxMessage(&s_can, &header, (uint8_t *)data, &mailbox) != HAL_OK)
        return;
    /* END acknowledgement is followed by a software reset.  Wait until the
       controller has accepted the frame so the reset cannot drop the ACK. */
    start = HAL_GetTick();
    while (HAL_CAN_IsTxMessagePending(&s_can, mailbox) != 0u) {
        /* At 500 kbps an 8-byte classic CAN frame is well below 1 ms.
           Keep a small guard for the USB-CAN bridge, but do not impose the
           former 20 ms delay on every 4-byte data acknowledgement. */
        if (HAL_GetTick() - start > 3u) break;
    }
}

static void Reply(uint8_t code, uint8_t result)
{
    uint8_t data[8] = {0};
    data[0] = code; data[1] = result; data[2] = s_session;
    data[3] = (uint8_t)s_offset; data[4] = (uint8_t)(s_offset >> 8);
    data[5] = (uint8_t)(s_offset >> 16); data[6] = (uint8_t)(s_offset >> 24);
    Send(FW_CAN_ACK_ID, data);
}

static void SendInfo(void)
{
    uint8_t data[8] = {0};
    data[0] = FW_CMD_INFO;
    data[1] = s_session;
    data[2] = s_meta.state == IMAGE_PENDING ? 1u : 0u;
    data[3] = s_meta.active_slot == FW_SLOT_A_BASE ? 0u : 1u;
    data[4] = s_target == FW_SLOT_A_BASE ? 0u : 1u;
    Send(FW_CAN_STATUS_ID, data);
}

static void SendDiscoveryPart(void)
{
    uint8_t data[8] = {0};
    uint8_t i;
    uint16_t crc = UidCrc();
    data[0] = FW_CMD_DISCOVER_REPLY;
    data[1] = s_discovery_session;
    data[2] = s_discovery_part;
    for (i = 0u; i < 3u; ++i) data[3u + i] = s_uid[s_discovery_part * 3u + i];
    data[6] = (uint8_t)crc;
    data[7] = (uint8_t)(crc >> 8);
    Send(FW_CAN_STATUS_ID, data);
    ++s_discovery_part;
    s_discovery_due = HAL_GetTick() + 2u;
}

static uint8_t ErasePage(uint32_t address)
{
    FLASH_EraseInitTypeDef erase = {0};
    uint32_t error = 0u;
    if (address < FW_SLOT_A_BASE || address >= FW_META_BASE) return 0u;
    erase.TypeErase = FLASH_TYPEERASE_PAGES;
    erase.Banks = FLASH_BANK_1;
    erase.Page = (address - FW_BOOT_BASE) / FW_PAGE_SIZE;
    erase.NbPages = 1u;
    if (HAL_FLASH_Unlock() != HAL_OK) return 0u;
    if (HAL_FLASHEx_Erase(&erase, &error) != HAL_OK) { HAL_FLASH_Lock(); return 0u; }
    HAL_FLASH_Lock();
    return 1u;
}

static uint8_t EraseSlot(uint32_t base)
{
    uint32_t address;
    for (address = base; address < base + FW_SLOT_SIZE; address += FW_PAGE_SIZE) {
        if (!ErasePage(address)) return 0u;
    }
    return 1u;
}

static uint8_t ProgramStaging(void)
{
    uint64_t word = 0xFFFFFFFFFFFFFFFFull;
    uint32_t address = s_target + s_offset - s_staging_count;
    uint8_t i;
    if (s_staging_count == 0u || (address & 7u) != 0u ||
        address < s_target || address + 8u > s_target + FW_SLOT_SIZE) return 0u;
    for (i = 0u; i < s_staging_count; ++i) {
        word &= ~((uint64_t)0xFFu << (8u * i));
        word |= (uint64_t)s_staging[i] << (8u * i);
    }
    if (HAL_FLASH_Unlock() != HAL_OK) return 0u;
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, address, word) != HAL_OK) {
        HAL_FLASH_Lock(); return 0u;
    }
    HAL_FLASH_Lock();
    if (*(const uint64_t *)address != word) return 0u;
    s_staging_count = 0u;
    return 1u;
}

static void OnData(const uint8_t d[8])
{
    uint16_t packet = (uint16_t)d[0] | ((uint16_t)d[1] << 8);
    uint8_t count;
    uint8_t i;
    if (!s_receiving) return;
    /* Data frames are the activity during a long image transfer.  Refresh
       the session watchdog here; otherwise the 10 s command timeout aborts
       an otherwise healthy download before END is received. */
    s_last_activity = HAL_GetTick();
    if (packet < s_packet) {
        /* A host may replay a window after its cumulative ACK was lost.
           Ignore duplicate packets quietly; only the last packet in a
           window re-issues the cumulative ACK, avoiding an ACK storm. */
        if (((packet + 1u) % FW_DATA_WINDOW) == 0u) Reply(FW_CMD_DATA_ACK, FW_STATUS_OK);
        return;
    }
    if (packet != s_packet) { Reply(FW_CMD_DATA_ACK, FW_STATUS_INVALID); return; }
    count = (uint8_t)((s_size - s_offset) > 6u ? 6u : (s_size - s_offset));
    if (count == 0u) { Reply(FW_CMD_DATA_ACK, FW_STATUS_INVALID); return; }
    for (i = 0u; i < count; ++i) {
        s_staging[s_staging_count++] = d[2u + i];
        ++s_offset;
        if (s_staging_count == 8u && !ProgramStaging()) {
            s_receiving = 0u; Reply(FW_CMD_DATA_ACK, FW_STATUS_FLASH); return;
        }
    }
    ++s_packet;
    if ((s_packet % FW_DATA_WINDOW) == 0u || s_offset == s_size)
        Reply(FW_CMD_DATA_ACK, FW_STATUS_OK);
}

static void OnCommand(const uint8_t d[8])
{
    uint8_t i;
    if (d[0] == FW_CMD_DISCOVER && d[2] == 0u && d[3] == 0u &&
        d[4] == 0u && d[5] == 0u && d[6] == 0u && d[7] == 0u) {
        s_discovery_part = 0u;
        s_discovery_session = d[1];
        s_discovery_due = HAL_GetTick() + 5u + ((uint32_t)s_uid[0] + s_uid[5] * 17u + s_uid[11] * 31u + d[1]) % 180u;
        return;
    }
    if (d[0] == FW_CMD_SELECT && d[1] < 3u) {
        if (s_select_mask != 0u && HAL_GetTick() - s_last_activity > 500u) s_select_mask = 0u;
        if (s_select_mask != 0u && s_select_session != d[2]) s_select_mask = 0u;
        s_select_session = d[2];
        s_last_activity = HAL_GetTick();
        for (i = 0u; i < 4u; ++i) if (d[3u + i] != s_uid[d[1] * 4u + i]) { s_select_mask = 0u; return; }
        if (d[7] != 0u) { s_select_mask = 0u; return; }
        s_select_mask |= (uint8_t)(1u << d[1]);
        if (s_select_mask == 7u) { s_selected = 1u; s_session = d[2]; Reply(FW_CMD_SELECT, 0u); }
        return;
    }
    if (!s_selected || d[1] != s_session) return;
    s_last_activity = HAL_GetTick();
    if (d[0] == FW_CMD_DISCOVER || d[0] == FW_CMD_INFO) {
        SendInfo();
    } else if (d[0] == FW_CMD_BEGIN && d[2] == FW_CMD_BEGIN_SIZE) {
        s_size = (uint32_t)d[3] | ((uint32_t)d[4] << 8) | ((uint32_t)d[5] << 16) | ((uint32_t)d[6] << 24);
        s_target = d[7] == 0u ? FW_SLOT_A_BASE : FW_SLOT_B_BASE;
        s_begin_size = s_size >= 8u && s_size <= FW_SLOT_SIZE &&
            (s_meta.state != IMAGE_CONFIRMED || s_target != s_meta.active_slot);
        Reply(FW_CMD_BEGIN, s_begin_size ? FW_STATUS_OK : FW_STATUS_INVALID);
    } else if (d[0] == FW_CMD_BEGIN && d[2] == FW_CMD_BEGIN_CRC) {
        s_expected_crc = (uint32_t)d[3] | ((uint32_t)d[4] << 8) |
                         ((uint32_t)d[5] << 16) | ((uint32_t)d[6] << 24);
        s_begin_crc = 1u;
        Reply(FW_CMD_BEGIN, FW_STATUS_OK);
    } else if (d[0] == FW_CMD_BEGIN && d[2] == 2u) {
        if (!s_begin_size || !s_begin_crc || s_meta.state == IMAGE_PENDING) {
            Reply(FW_CMD_BEGIN, FW_STATUS_INVALID); return;
        }
        s_offset = 0u; s_packet = 0u; s_staging_count = 0u;
        /* Erase the complete inactive slot before streaming data.  Erasing
           on each 2 KB boundary caused a visible multi-second stall and
           allowed the host ACK timeout to dominate throughput. */
        if (!EraseSlot(s_target)) { Reply(FW_CMD_BEGIN, FW_STATUS_FLASH); return; }
        s_erased_page = 0xFFFFFFFFu;
        s_meta.state = IMAGE_DOWNLOADING;
        s_meta.pending_slot = 0u;
        if (!MetaSave()) { Reply(FW_CMD_BEGIN, FW_STATUS_FLASH); return; }
        s_receiving = 1u;
        Reply(FW_CMD_BEGIN, FW_STATUS_OK);
    } else if (d[0] == FW_CMD_END) {
        if (!s_receiving || s_offset != s_size ||
            (s_staging_count != 0u && !ProgramStaging())) {
            Reply(FW_CMD_END, FW_STATUS_INVALID); return;
        }
        s_receiving = 0u;
        if (!ImageOk(s_target, s_size, s_expected_crc)) {
            Reply(FW_CMD_END, FW_STATUS_CRC); return;
        }
        s_meta.pending_slot = s_target;
        s_meta.state = IMAGE_PENDING;
        if (s_target == FW_SLOT_A_BASE) { s_meta.a_size = s_size; s_meta.a_crc = s_expected_crc; }
        else { s_meta.b_size = s_size; s_meta.b_crc = s_expected_crc; }
        if (!MetaSave()) { Reply(FW_CMD_END, FW_STATUS_FLASH); return; }
        Reply(FW_CMD_END, FW_STATUS_OK);
        HAL_Delay(20u);
        NVIC_SystemReset();
    } else if (d[0] == FW_CMD_ABORT) {
        s_selected = 0u; s_select_mask = 0u; s_receiving = 0u;
        Reply(FW_CMD_ABORT, 0u);
    }
}

int main(void)
{
    uint32_t words[3];
    uint8_t i;
    uint8_t request;
    HAL_Init();
    ClockInit();
    words[0] = HAL_GetUIDw0(); words[1] = HAL_GetUIDw1(); words[2] = HAL_GetUIDw2();
    for (i = 0u; i < 12u; ++i) s_uid[i] = (uint8_t)(words[i / 4u] >> (8u * (i % 4u)));
    RCC->APB1ENR1 |= RCC_APB1ENR1_PWREN | RCC_APB1ENR1_RTCAPBEN;
    PWR->CR1 |= PWR_CR1_DBP;
    request = RTC->BKP1R == BOOT_REQUEST_MAGIC;
    RTC->BKP1R = 0u;
    MetaLoad();
    if (s_meta.state == IMAGE_TRIAL) {
        if (RTC->BKP2R == FW_CONFIRM_MAGIC && s_meta.pending_slot != 0u) {
            s_meta.active_slot = s_meta.pending_slot;
            s_meta.state = IMAGE_CONFIRMED;
        } else {
            s_meta.state = IMAGE_CONFIRMED;
        }
        s_meta.pending_slot = 0u;
        RTC->BKP2R = 0u;
        (void)MetaSave();
    }
    if (s_meta.state == IMAGE_PENDING && s_meta.pending_slot != 0u) {
        uint32_t trial = s_meta.pending_slot;
        uint32_t size = trial == FW_SLOT_A_BASE ? s_meta.a_size : s_meta.b_size;
        uint32_t crc = trial == FW_SLOT_A_BASE ? s_meta.a_crc : s_meta.b_crc;
        if (ImageOk(trial, size, crc)) {
            s_meta.state = IMAGE_TRIAL;
            if (MetaSave()) {
                RTC->BKP2R = FW_TRIAL_MAGIC;
                Jump(trial);
            }
        }
    }
    if (!request && s_meta.state != IMAGE_PENDING && s_meta.state != IMAGE_TRIAL) {
        uint32_t size = s_meta.active_slot == FW_SLOT_A_BASE ? s_meta.a_size : s_meta.b_size;
        uint32_t crc = s_meta.active_slot == FW_SLOT_A_BASE ? s_meta.a_crc : s_meta.b_crc;
        if (ImageOk(s_meta.active_slot, size, crc)) Jump(s_meta.active_slot);
    }
    CanInit();
    for (;;) {
        CAN_RxHeaderTypeDef header;
        uint8_t data[8];
        if (s_selected && HAL_GetTick() - s_last_activity > 10000u) {
            s_selected = 0u; s_select_mask = 0u; s_receiving = 0u;
        }
        if (s_discovery_due != 0u && s_discovery_part < 4u &&
            (int32_t)(HAL_GetTick() - s_discovery_due) >= 0) {
            SendDiscoveryPart();
            if (s_discovery_part == 4u) s_discovery_due = 0u;
        }
        if (HAL_CAN_GetRxFifoFillLevel(&s_can, CAN_RX_FIFO0) == 0u) continue;
        if (HAL_CAN_GetRxMessage(&s_can, CAN_RX_FIFO0, &header, data) != HAL_OK) continue;
        if (header.IDE != CAN_ID_STD || header.RTR != CAN_RTR_DATA || header.DLC != 8u) continue;
        if (header.StdId == FW_CAN_CMD_ID) OnCommand(data);
        else if (header.StdId == FW_CAN_DATA_ID) OnData(data);
    }
}
