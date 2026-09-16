/**
 ******************************************************************************
 * @file    reset_cause.c
 * @brief   STM32L432 复位原因采集与异常上下文最小化记录。
 *
 * RTC BKP0R 在系统复位后保留，可区分 HardFault/其他 Cortex-M 异常触发的
 * 软件复位；RCC->CSR 补充独立/窗口看门狗及其他硬件复位标志。
 ******************************************************************************
 */
#include "app/reset_cause.h"

#include "main.h"

#define RESET_CAUSE_BKP_MAGIC      0x41520000u
#define RESET_CAUSE_BKP_MAGIC_MASK 0xFFFF0000u

static uint8_t s_latched_cause;

static void ResetCause_EnableBackupAccess(void)
{
    RCC->APB1ENR1 |= RCC_APB1ENR1_PWREN | RCC_APB1ENR1_RTCAPBEN;
    (void)RCC->APB1ENR1;
    PWR->CR1 |= PWR_CR1_DBP;
    __DSB();
}

static void ResetCause_RecordAndReset(uint8_t cause) __attribute__((noreturn));

static void ResetCause_RecordAndReset(uint8_t cause)
{
    ResetCause_EnableBackupAccess();
    RTC->BKP0R = RESET_CAUSE_BKP_MAGIC | cause;
    __DSB();
    NVIC_SystemReset();
    while (1) { }
}

void ResetCause_Init(void)
{
    uint32_t csr = RCC->CSR;
    uint32_t backup;

    s_latched_cause = 0u;
    ResetCause_EnableBackupAccess();
    backup = RTC->BKP0R;
    if ((backup & RESET_CAUSE_BKP_MAGIC_MASK) == RESET_CAUSE_BKP_MAGIC)
    {
        s_latched_cause |= (uint8_t)(backup & 0xFFu);
        RTC->BKP0R = 0u; /* 消费一次，避免后续正常复位重复上报。 */
    }

    if ((csr & (RCC_CSR_IWDGRSTF | RCC_CSR_WWDGRSTF)) != 0u)
    {
        s_latched_cause |= RESET_CAUSE_WATCHDOG;
    }
    if ((csr & (RCC_CSR_OBLRSTF | RCC_CSR_LPWRRSTF)) != 0u)
    {
        s_latched_cause |= RESET_CAUSE_OTHER;
    }
    RCC->CSR |= RCC_CSR_RMVF;
}

uint8_t ResetCause_GetLatched(void)
{
    return s_latched_cause;
}

void ResetCause_RecordHardFaultAndReset(void)
{
    ResetCause_RecordAndReset(RESET_CAUSE_HARDFAULT);
}

void ResetCause_RecordOtherExceptionAndReset(void)
{
    ResetCause_RecordAndReset(RESET_CAUSE_OTHER);
}
