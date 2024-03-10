#ifndef __COMM_TASK_H__
#define __COMM_TASK_H__

#include "stm32f4xx_hal.h"

#define CHASSIS_MOTOR_MSG_SEND    ( 1 << 0 )
#define CHASSIS_CAN_TX_ID 	0x200
#define GIMBAL_CAN_TX_ID  	0x1ff
#define SUPERCAP_CAN_TX_ID	0x010
/* motor current parameter structure */
typedef struct
{
    /* 4 chassis motor current */
    int16_t chassis_cur_3508[4];
    int16_t chassis_cur_6020[4];
} motor_current_t;

typedef __packed struct
{
    __packed struct
    {
        uint8_t  vx_sign:1;
        uint8_t  vy_sign:1;
        uint8_t  vw_sign:1;
        uint8_t	 mode   :2;
        uint8_t  C  :1;
    } sign_msg;
    uint8_t  power_max;

    uint16_t spd_vx;
    uint16_t spd_vy;
    uint16_t spd_vw;
} chassis_msg_t;

extern motor_current_t motor_cur;

void can_msg_send_task(void const *argu);;
void judge_msg_get(CAN_HandleTypeDef* hcan, uint8_t * CAN_Rx_data);
void get_chassis_msg(CAN_HandleTypeDef* hcan, uint8_t * CAN_Rx_data);
void can1_send_supercap(int16_t TX_ID);
#endif


