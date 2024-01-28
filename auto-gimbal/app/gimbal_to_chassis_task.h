#ifndef __GIMBAL_TO_CHASSIS_TASK_H__
#define __GIMBAL_TO_CHASSIS_TASK_H__


#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_can.h"


#define GIMBAL_TO_CHASSIS_TASK_PERIOD 10

typedef void (*Send_ctrl_msg)(int16_t, int16_t, int16_t, int16_t , uint8_t , uint8_t );
typedef void (*Send_judge_msg)(int16_t, CAN_HandleTypeDef *);

typedef struct
{
	Send_ctrl_msg Send_ctrl_callback;
   Send_judge_msg Send_judge_callback; // 解析收到的数据的回调函数
}Gim_to_Cha_send_t;


void test_task(void const *argu);
void gimbal_to_chassic_task(void const *argu);//��������
#endif

