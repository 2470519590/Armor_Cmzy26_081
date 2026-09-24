#ifndef APP_FW_ENTRY_H
#define APP_FW_ENTRY_H

#include <stdbool.h>
#include "comm/app_can.h"

void FwEntry_Init(void);
void FwEntry_OnRxIsr(const AppCanFrame *frame);
bool FwEntry_Task(void);
bool FwEntry_IsMaintenance(void);

#endif
