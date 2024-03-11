/** 
  * @file bap_can.h
  * @version 2.0
  * @date Jun,6th 2021
  *
  * @brief  
  *
  * @author YY
  *
  */
#ifndef _BSP_CAN_H_
#define _BSP_CAN_H_

#include "can.h"
#include "comm_task.h"

/* CAN receive ID */
typedef enum
{
	CAN_3508_M1_ID    = 0x201,
    CAN_3508_M2_ID    = 0x202,
    CAN_3508_M3_ID    = 0x203,
    CAN_3508_M4_ID    = 0x204,

	CAN_6020_M1_ID    = 0x205,
	CAN_6020_M2_ID    = 0x206,
	CAN_6020_M3_ID    = 0x207,
	CAN_6020_M4_ID    = 0x208,
	
	CAN_POWER_ID       = 0x003,
	CAN_CHASSIS_MSG_ID = 0x1f1,
	CAN_JUDGE_MSG_ID   = 0x09,
} can2_id_e;

/* can receive motor parameter structure */
typedef struct
{
  uint16_t offset_ecd;
	uint16_t ecd;
  uint16_t last_ecd;
	int32_t  total_ecd;

  int16_t  speed_rpm;
  int16_t  given_current;

  int32_t  round_cnt;

  uint32_t msg_cnt;
} moto_measure_t;

/*moto received messages structure*/
typedef struct
{
	moto_measure_t chassis_3508[4];
	moto_measure_t chassis_6020[4];
} MOTO_RecvMsg_t;

extern MOTO_RecvMsg_t	moto_msg;
extern CAN_TxHeaderTypeDef Tx1Message;

void encoder_data_handler(moto_measure_t* ptr, uint8_t* rx_message);

void can_device_init(void);
void can1_send_message(int16_t TX_ID, int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4);
void can2_send_message(int16_t TX_ID, int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4);

#endif

