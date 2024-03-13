#ifndef __MODESWITCH_TASK_H__
#define __MODESWITCH_TASK_H__

#include "stdint.h"

typedef enum
{
    PROTECT_MODE,   //保护模式
    REMOTER_MODE,   //遥控模式
    KEYBOARD_MODE,  //键盘模式
    VISION_MODE,     //视觉模式(鼠标右键开启)
    AUTO_MODE      //自动模式
} ctrl_mode_e;

void mode_switch_task(void const *argu);

extern uint8_t lock_flag;
extern ctrl_mode_e ctrl_mode;

#endif
