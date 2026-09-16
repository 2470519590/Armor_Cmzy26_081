/**
 ******************************************************************************
 * @file    reset_cause.h
 * @brief   复位原因采集与异常复位记录。
 ******************************************************************************
 */
#ifndef APP_RESET_CAUSE_H
#define APP_RESET_CAUSE_H

#include <stdint.h>

/* 0x110 Byte0 复位原因位图。 */
#define RESET_CAUSE_HARDFAULT  (1u << 0)
#define RESET_CAUSE_WATCHDOG   (1u << 1)
#define RESET_CAUSE_CAN_BUSOFF (1u << 2) /* 当前策略不因 Bus-Off 复位，保留。 */
#define RESET_CAUSE_OTHER      (1u << 3)

void ResetCause_Init(void);
uint8_t ResetCause_GetLatched(void);

/* Cortex-M 异常处理程序调用：写备份寄存器后立即执行系统复位。 */
void ResetCause_RecordHardFaultAndReset(void) __attribute__((noreturn));
void ResetCause_RecordOtherExceptionAndReset(void) __attribute__((noreturn));

#endif /* APP_RESET_CAUSE_H */
