#ifndef __MODESWITCH_TASK_H__
#define __MODESWITCH_TASK_H__

#include "stdint.h"
#include "comm_type.h"

void mode_switch_task(void const *argu);

extern uint8_t lock_flag;
extern ctrl_mode_e ctrl_mode;

#endif
