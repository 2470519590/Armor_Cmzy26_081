/**
 ******************************************************************************
 * @file    main_app.h
 * @brief   应用层编排接口（初始化 / 主循环 / 1ms 时基）。
 *
 * 设计文档 v1.5 第 10 章：主循环承担帧消费与协议轮询；状态维护由
 * TIM2 1kHz 中断驱动（App_OnTick1ms），ISR 只做计算与置标志。
 ******************************************************************************
 */
#ifndef APP_MAIN_APP_H
#define APP_MAIN_APP_H

#include <stdint.h>

#define APP_HIT_THRESHOLD_MIN 1000u
#define APP_HIT_THRESHOLD_MAX 67108864u

/* 初始化：外设→ADS→检测→灯→状态机→CAN→协议栈 依序（设计文档 5.3）。 */
void App_Init(void);

/* 主循环一次迭代（while(1) 内调用，10.3 任务表）。 */
void App_Loop(void);

/* TIM2 1kHz 中断调用（HAL_TIM_PeriodElapsedCallback 桥接）：状态机/灯/健康节拍。 */
void App_OnTick1ms(void);

/* Update only P04 in RAM. The value is intentionally not persisted. */
uint8_t App_SetHitThreshold(uint32_t threshold, uint32_t *applied);

#endif /* APP_MAIN_APP_H */
