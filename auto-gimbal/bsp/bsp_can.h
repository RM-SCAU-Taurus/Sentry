/*
 * @Author: your name
 * @Date: 2021-12-19 14:37:59
 * @LastEditTime: 2022-01-03 22:06:12
 * @LastEditors: Please set LastEditors
 * @Description: 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 * @FilePath: \MDK-ARMd:\RM\auto-Infantry\rm-Infantry-20211026\bsp\bsp_can.h
 */
#ifndef _BSP_CAN_H_
#define _BSP_CAN_H_

#ifdef  __BSP_CAN_GLOBALS
#define __BSP_CAN_EXT
#else
#define __BSP_CAN_EXT extern
#endif
#include "can.h"
#include "comm_task.h"
#include "bsp_powerlimit.h"
#include "bsp_judge.h"
/* CAN send and receive ID */
typedef enum
{
    CAN_3508_M1_ID       = 0x201,
    CAN_3508_M2_ID       = 0x202,
    CAN_3508_M3_ID       = 0x203,
    CAN_3508_M4_ID       = 0x204,

    CAN_YAW_MOTOR_ID     = 0x205,
    CAN_PIT_MOTOR_ID     = 0x206,
    CAN_TRIGGER_MOTOR1_ID= 0x207,
   POWER_CONTROL_ID     = 0x003,
    CHASSIS_MSG_ID       =0x005,
    GAME1_MSG_ID          =0x006,
		GAME2_MSG_ID  				=0x007,
		GAME3_MSG_ID  				=0x008,
		GAME4_MSG_ID  				=0x009,
		GAME5_MSG_ID  				=0x010,
		GAME6_MSG_ID  				=0x011,
   
} can_msg_id_e;

typedef struct
{
    uint16_t ecd;
    uint16_t last_ecd;

    int16_t  speed_rpm;
    int16_t  given_current;

    int32_t  round_cnt;
    int32_t  total_ecd;

    uint16_t offset_ecd;
    uint32_t msg_cnt;
} moto_measure_t;
extern uint8_t GAME_STATE;
__BSP_CAN_EXT moto_measure_t moto_chassis[4] ;
__BSP_CAN_EXT moto_measure_t moto_pit;
__BSP_CAN_EXT moto_measure_t moto_yaw;
__BSP_CAN_EXT moto_measure_t motor_trigger;
void encoder_data_handler(moto_measure_t* ptr, CAN_HandleTypeDef* hcan, uint8_t * CAN_Rx_data);
void get_moto_offset(moto_measure_t* ptr, CAN_HandleTypeDef* hcan, uint8_t * CAN_Rx_data);

void can_device_init(void);
void game_data_handler(robot_judge_msg_t * robot_judge_msg_e);
void can1_send_chassis_message(int16_t TX_ID, int16_t iq1, int16_t iq2, int16_t iq3, uint8_t iq4,uint8_t iq5);
void can1_send_message(int16_t TX_ID, int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4);
void can2_send_message(int16_t TX_ID, int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4);
void pit_data_handler(moto_measure_t* ptr, CAN_HandleTypeDef* hcan, uint8_t * CAN_Rx_data);
void send_judge_msg(int16_t TX_ID,CAN_HandleTypeDef* hcan);
#endif
