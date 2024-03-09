/*
 * @Author: your name
 * @Date: 2021-12-19 14:37:59
 * @LastEditTime: 2022-01-14 22:54:36
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

		CAN_6020_M1_ID    = 0x205,
		CAN_6020_M2_ID    = 0x206,
		CAN_6020_M3_ID    = 0x207,
		CAN_6020_M4_ID    = 0x208,

    POWER_CONTROL_ID     = 0x003,
    CHASSIS_MSG_ID       =0x004,
    GAME_MSG_ID          =0x005,
    CHASSIS_CTRl_MSG_ID  =0x1f1,
		CAN_JUDGE_MSG_ID   = 0x09,
   
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
typedef struct
{
    uint8_t game_type;				
    uint8_t game_progress;				
    uint16_t shooter_id1_17mm_cooling_rate;
    uint16_t shooter_id1_17mm_speed_limit;		
    uint16_t shooter_id1_17mm_cooling_limit;	
    uint8_t robot_id;		
    uint16_t remain_HP;	
	  uint16_t max_HP;		
    uint8_t armor_id;	
    uint8_t hurt_type;		
    uint8_t bullet_freq;
		uint16_t bullet_speed;
		uint16_t bullet_remaining_num_17mm;
		float blue_x;
		float blue_y;
		float blue_confiden;
		float red_x;
		float red_y;
		float red_confiden;
} Game_Status_t;

__BSP_CAN_EXT moto_measure_t moto_chassis[4] ;
__BSP_CAN_EXT moto_measure_t moto_chassis_6020[4] ;
__BSP_CAN_EXT moto_measure_t moto_pit;
__BSP_CAN_EXT moto_measure_t moto_yaw;
__BSP_CAN_EXT moto_measure_t motor_trigger;
extern Game_Status_t Game_Status;
void encoder_data_handler(moto_measure_t* ptr, CAN_HandleTypeDef* hcan, uint8_t * CAN_Rx_data);
void get_moto_offset(moto_measure_t* ptr, CAN_HandleTypeDef* hcan, uint8_t * CAN_Rx_data);
void can_device_init(void);
void can1_send_chassis_message(int16_t TX_ID, int16_t iq1, int16_t iq2, int16_t iq3, uint8_t iq4,uint8_t iq5);
void can1_send_spd_msg(int16_t TX_ID);
void can1_send_game_msg(int16_t TX_ID,robot_judge_msg_t * robot_judge_msg_e);
void can1_send_message(int16_t TX_ID, int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4);
void can2_send_message(int16_t TX_ID, int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4);
void pit_data_handler(moto_measure_t* ptr, CAN_HandleTypeDef* hcan, uint8_t * CAN_Rx_data);
void judge_msg_get(CAN_HandleTypeDef* hcan, uint8_t * CAN_Rx_data);
void can1_send_supercap(int16_t TX_ID);


void can_device_init(void);
void canx_init(CAN_HandleTypeDef * hcan, uint32_t  * id, void (*pFunc)(uint32_t,uint8_t*));
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);



extern uint8_t gimbal_status;
#endif
