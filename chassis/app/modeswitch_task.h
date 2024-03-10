#ifndef __MODESWITCH_TASK_H__
#define __MODESWITCH_TASK_H__

#include "stdint.h"

typedef enum
{
    PROTECT_MODE,
    LOCK_MODE,					//��̨�������е㣬���������˶�
    SEPARATE_MODE,      //������̨���룬���̸�����̨
    DANCE_MODE,         //ҡ��ģʽ
    REMOTER_MODE,
    STANDARD_MODE,
    KEYBOARD_MODE,      //����ģʽ
    VISION_MODE,        //�Ӿ�����ģʽ
    ENERGY_MODE,				//��������ģʽ
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
