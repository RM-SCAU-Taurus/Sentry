#ifndef __MODESWITCH_TASK_H__
#define __MODESWITCH_TASK_H__

#include "stdint.h"

typedef enum
{
    PROTECT_MODE,
    LOCK_MODE,					//云台锁定在中点，底盘自身运动
    SEPARATE_MODE,      //底盘云台分离，底盘跟随云台
    DANCE_MODE,         //摇摆模式
    REMOTER_MODE,
    STANDARD_MODE,
    KEYBOARD_MODE,      //键盘模式
    VISION_MODE,        //视觉辅助模式
    ENERGY_MODE,				//能量机关模式
	AUTO_MODE,
} ctrl_mode_e;

extern ctrl_mode_e ctrl_mode;
extern uint8_t lock_flag;
extern int kb_status[11];
void mode_switch_task(void const *argu);
void get_sw_mode(void);
void unlock_init(void);
void sw1_mode_handler(void);
void sw2_mode_handler(void);

#endif
