#include "bsp_can.h"
#include "pid.h"
#include "chassis_task.h"
#include "string.h"
#include "bsp_powerlimit.h"
#include "comm_task.h"
#include "status_task.h"
#include "modeswitch_task.h"
MOTO_RecvMsg_t	moto_msg;

CAN_TxHeaderTypeDef Tx1Message;
CAN_RxHeaderTypeDef Rx1Message;
CAN_TxHeaderTypeDef Tx2Message;
CAN_RxHeaderTypeDef Rx2Message;

uint8_t CAN1_Rx_data[8];
uint8_t CAN1_Tx_data[8];
uint8_t CAN2_Rx_data[8];
uint8_t CAN2_Tx_data[8];
float test_time,last_time,time=0;
/**
  * @brief     CAN接受中断回调函数
  * @param     CAN_Rx_data ：CAN节点反馈的数据帧
  * @attention 
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	if(hcan == &hcan1)
	{
		HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &Rx1Message, CAN1_Rx_data);
		switch (Rx1Message.StdId)
		{
			case CAN_POWER_ID:
			{
				Power_data_handler(CAN_POWER_ID, CAN1_Rx_data);
				break;
			}
			case CAN_CHASSIS_MSG_ID:
			{
			           chassis.spd_input.vx =(int16_t)( CAN1_Rx_data[0]<<8|CAN1_Rx_data[1]);
           chassis.spd_input.vy = (int16_t)(CAN1_Rx_data[2]<<8|CAN1_Rx_data[3]);
           chassis.spd_input.vw =(int16_t)( CAN1_Rx_data[4]<<8|CAN1_Rx_data[5]);
					//tent1++;
           if (CAN1_Rx_data[6]==0x00)
           {
               ctrl_mode=PROTECT_MODE;
               /* code */
           }
           else if (CAN1_Rx_data[6]==0x01)
           {
               ctrl_mode=REMOTER_MODE;
               /* code */
           }
           else if (CAN1_Rx_data[6]==0x02)
           {
               ctrl_mode=AUTO_MODE;
               /* code */
           }
            break;
			}
			case CAN_JUDGE_MSG_ID:
			{
                judge_msg_get(&hcan1, CAN1_Rx_data);
                break;
			}
//			case CAN_3508_M1_ID:
//			case CAN_3508_M2_ID:
			case CAN_3508_M3_ID:
//			case CAN_3508_M4_ID:
			{
				static uint8_t i;
				i = Rx1Message.StdId - CAN_3508_M1_ID;
				encoder_data_handler(&moto_msg.chassis_3508[i], CAN1_Rx_data);
				break;
			}
		
			default:
			{
				break;
			}
		}
		__HAL_CAN_ENABLE_IT(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
	}

	else if(hcan == &hcan2)
	{
		HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &Rx2Message, CAN2_Rx_data);
		switch (Rx2Message.StdId)
		{
			case CAN_3508_M1_ID://00
			case CAN_3508_M2_ID://01
//			case CAN_3508_M3_ID://02
			case CAN_3508_M4_ID://03
			{
				static uint8_t i;
				i = Rx2Message.StdId - CAN_3508_M1_ID;
				encoder_data_handler(&moto_msg.chassis_3508[i], CAN2_Rx_data);
				
				break;
			}
			case CAN_6020_M1_ID:
			case CAN_6020_M2_ID:
			case CAN_6020_M3_ID:
			case CAN_6020_M4_ID:				
			{
				static uint8_t j;
				j = Rx2Message.StdId - CAN_6020_M1_ID;
				encoder_data_handler(&moto_msg.chassis_6020[j], CAN2_Rx_data);
				
				break;
			}
			
			default:
			{
				break;
			}
		}
		__HAL_CAN_ENABLE_IT(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
	}
}


/**
  * @brief     get motor rpm and calculate motor round_count/total_encoder/total_angle
  * @param     ptr: Pointer to a moto_measure_t structure
  * @attention this function should be called after get_moto_offset() function
  */
void encoder_data_handler(moto_measure_t* ptr, uint8_t* rx_message)
{
	/* 转子转速 */
	ptr->speed_rpm     = (int16_t)(rx_message[2] << 8 | rx_message[3]);
	ptr->given_current = (int16_t)(rx_message[4] << 8 | rx_message[5]);

	/* 减速前编码值 */
	ptr->last_ecd = ptr->ecd;
	ptr->ecd      = (uint16_t)(rx_message[0] << 8 | rx_message[1]);

	/* 转子旋转圈数 */
	if (ptr->ecd - ptr->last_ecd >= 4096)
		ptr->round_cnt--;
	else if (ptr->ecd - ptr->last_ecd <= -4096)
		ptr->round_cnt++;

	/* 开机后总编码值 */
	ptr->total_ecd = ptr->round_cnt * 8192 + ptr->ecd - ptr->offset_ecd;
}

/**
  * @brief  send calculated current to motor
  * @param  CAN1 motor current
  */
void can1_send_message(int16_t TX_ID, int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4)
{
//	uint8_t FreeTxNum = 0; 
	static uint32_t txmailbox;
	Tx1Message.StdId = TX_ID;
	Tx1Message.IDE 	 = CAN_ID_STD;
	Tx1Message.RTR   = CAN_RTR_DATA;
  Tx1Message.DLC   = 0x08;
	
	CAN1_Tx_data[0] = iq1 >> 8;
	CAN1_Tx_data[1] = iq1;
	CAN1_Tx_data[2] = iq2 >> 8 ;
	CAN1_Tx_data[3] = iq2;
	CAN1_Tx_data[4] = iq3 >> 8;
	CAN1_Tx_data[5] = iq3;
	CAN1_Tx_data[6] = iq4 >> 8;
	CAN1_Tx_data[7] = iq4;
	
	/* 查询发送邮箱是否为空 */
//	FreeTxNum = HAL_CAN_GetTxMailboxesFreeLevel(&hcan1);  
//	while(FreeTxNum == 0) 
//	{  
//    FreeTxNum = HAL_CAN_GetTxMailboxesFreeLevel(&hcan1);  
//  }
		while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0);      //如果三个邮箱都阻塞了就等一会儿，直到其中某个邮箱空闲
	if ((hcan1.Instance->TSR & CAN_TSR_TME0) != RESET)     //如果邮箱0空闲
	{
		txmailbox =CAN_TX_MAILBOX0;
	}

	/* Check Tx Mailbox 1 status */
	else if ((hcan1.Instance->TSR & CAN_TSR_TME1) != RESET)
	{
		txmailbox =CAN_TX_MAILBOX1;
	}

	/* Check Tx Mailbox 2 status */
	else if ((hcan1.Instance->TSR & CAN_TSR_TME2) != RESET)
	{
		txmailbox =CAN_TX_MAILBOX2;
	}
	HAL_CAN_AddTxMessage(&hcan1, &Tx1Message, CAN1_Tx_data, (uint32_t *)txmailbox);
	
}


/**
  * @brief  send calculated current to motor
  * @param  CAN2 motor current
  */
void can2_send_message(int16_t TX_ID, int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4)
{
//	uint8_t FreeTxNum = 0; 
	static uint32_t txmailbox;
	Tx2Message.StdId = TX_ID;
	Tx2Message.IDE 	 = CAN_ID_STD;
	Tx2Message.RTR   = CAN_RTR_DATA;
  Tx2Message.DLC   = 0x08;
	
	CAN2_Tx_data[0] = iq1 >> 8;
	CAN2_Tx_data[1] = iq1;
	CAN2_Tx_data[2] = iq2 >> 8 ;
	CAN2_Tx_data[3] = iq2;
	CAN2_Tx_data[4] = iq3 >> 8;
	CAN2_Tx_data[5] = iq3;
	CAN2_Tx_data[6] = iq4 >> 8;
	CAN2_Tx_data[7] = iq4;
	
	/* 查询发送邮箱是否为空 */
		while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan2) == 0);      //如果三个邮箱都阻塞了就等一会儿，直到其中某个邮箱空闲
	if ((hcan2.Instance->TSR & CAN_TSR_TME0) != RESET)     //如果邮箱0空闲
	{
		txmailbox =CAN_TX_MAILBOX0;
	}

	/* Check Tx Mailbox 1 status */
	else if ((hcan2.Instance->TSR & CAN_TSR_TME1) != RESET)
	{
		txmailbox =CAN_TX_MAILBOX1;
	}

	/* Check Tx Mailbox 2 status */
	else if ((hcan2.Instance->TSR & CAN_TSR_TME2) != RESET)
	{
		txmailbox =CAN_TX_MAILBOX2;
	}
	
	HAL_CAN_AddTxMessage(&hcan2, &Tx2Message, CAN2_Tx_data, (uint32_t*)txmailbox);
}

/**
  * @brief  init the can transmit and receive
  * @param  None
  */
void can_device_init(void)
{
  /* can filter config */
  CAN_FilterTypeDef  can_filter;

	can_filter.FilterBank           = 0;
  can_filter.FilterMode           = CAN_FILTERMODE_IDMASK;
  can_filter.FilterScale          = CAN_FILTERSCALE_32BIT;
  can_filter.FilterIdHigh         = 0x0000;
  can_filter.FilterIdLow          = 0x0000;
  can_filter.FilterMaskIdHigh     = 0x0000;
  can_filter.FilterMaskIdLow      = 0x0000;
  can_filter.FilterFIFOAssignment = CAN_FilterFIFO0;
	can_filter.SlaveStartFilterBank = 0;  
  can_filter.FilterActivation     = ENABLE;

  HAL_CAN_ConfigFilter(&hcan1, &can_filter);
  while (HAL_CAN_ConfigFilter(&hcan1, &can_filter) != HAL_OK);

	can_filter.FilterBank           = 14;
  HAL_CAN_ConfigFilter(&hcan2, &can_filter);  
	while (HAL_CAN_ConfigFilter(&hcan2, &can_filter) != HAL_OK);

	HAL_Delay(100);

	/* start the can transmit and receive */
	HAL_CAN_Start(&hcan1);
 	HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING);
	HAL_CAN_Start(&hcan2);
	HAL_CAN_ActivateNotification(&hcan2,CAN_IT_RX_FIFO0_MSG_PENDING);
}

