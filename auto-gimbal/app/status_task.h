#ifndef _STATUS_TASK_H_
#define _STATUS_TASK_H_

#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

typedef struct
{
    uint8_t gyro_status[2];     //LED_A
    uint8_t chassis_status[4];  //LED_B
    uint8_t gimbal_status[3];   //LED_C
    uint8_t vision_status;      //LED_D
    uint8_t power_control;      //LED_D
    uint8_t debug_status;       //流水灯
    uint8_t rc_status;          //遥控器通信
} status_t;

extern status_t status;
void status_task(void const *argu);
void status_init(void);

#endif
