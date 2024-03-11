#define __BSP_CAN_GLOBALS
#include "bsp_can.h"
#include "pid.h"
#include "chassis_task.h"
#include "status_task.h"
#include "string.h"
#include "bsp_T_imu.h"
#include "bsp_powerlimit.h"
#include "bsp_judge.h"
#include "protocol_camp.h"
#include "math_calcu.h"
#include "comm_task.h"
#include "modeswitch_task.h"


CAN_TxHeaderTypeDef Tx1Message;
CAN_RxHeaderTypeDef Rx1Message;
CAN_TxHeaderTypeDef Tx2Message;
CAN_RxHeaderTypeDef Rx2Message;

uint8_t CAN1_Rx_data[8];
uint8_t CAN1_Tx_data[8];
uint8_t CAN2_Rx_data[8];
uint8_t CAN2_Tx_data[8];
uint8_t GAME_STATE;

uint8_t gimbal_status;

Game_Status_t Game_Status;

extern Game_Status_t Game_Status;



struct can_rx_buff
{
    CAN_RxHeaderTypeDef header;
    uint8_t             data[8];
} can_rx_data;

/* 本地变量，CAN中断回调函数指针 */
static void (*pCAN1_RxFifo0CpltCallback)(uint32_t,uint8_t*);
static void (*pCAN2_RxFifo1CpltCallback)(uint32_t,uint8_t*);

static void CAN_Filter_IDList_Config(CAN_HandleTypeDef * hcan, uint32_t  * ID);
static void CAN1_FilterEnd_Config(CAN_HandleTypeDef * hcan,CAN_FilterTypeDef  *Can_Fliter,uint8_t num);
static void CAN2_FilterEnd_Config(CAN_HandleTypeDef * hcan,CAN_FilterTypeDef  *Can_Fliter,uint8_t num);
static void CAN1_IDList_Config(CAN_HandleTypeDef * hcan,CAN_FilterTypeDef  *Can_Fliter,uint32_t  * ID);
static void CAN2_IDList_Config(CAN_HandleTypeDef * hcan,CAN_FilterTypeDef  *Can_Fliter,uint32_t  * ID);


static void User_can1_callback(uint32_t ID, uint8_t* CAN_RxData);
static void User_can2_callback(uint32_t ID, uint8_t* CAN_RxData);




/******************************MOVE************************************************/
void can_device_init(void)
{
    uint32_t can_ID1[] = {CAN_3508_M2_ID,CAN_3508_M3_ID,POWER_CONTROL_ID, CAN_JUDGE_MSG_ID,CHASSIS_CTRl_MSG_ID,0xFFF};
    uint32_t can_ID2[] = {CAN_6020_M4_ID,CAN_6020_M3_ID,CAN_6020_M2_ID, CAN_6020_M1_ID, CAN_3508_M1_ID,CAN_3508_M4_ID, 0xFFF};
    canx_init(&hcan1, can_ID1, User_can1_callback);
    canx_init(&hcan2, can_ID2, User_can2_callback);
}

/**
* @brief  Initialize CAN Bus
* @param  hcan: CANx created by CubeMX. id:an array of ListID. (*pFunc):USER_Callback_Func
* @return None.
*/
void canx_init(CAN_HandleTypeDef * hcan, uint32_t  * id, void (*pFunc)(uint32_t,uint8_t*))
{
    CAN_Filter_IDList_Config(hcan,id);
    HAL_CAN_Start(hcan);
    if (CAN1 == hcan->Instance)
    {
        pCAN1_RxFifo0CpltCallback = pFunc;
        HAL_CAN_ActivateNotification(hcan,CAN_IT_RX_FIFO0_MSG_PENDING);
        __HAL_CAN_ENABLE_IT(hcan,CAN_IT_RX_FIFO0_MSG_PENDING);
    }
    else if (CAN2 == hcan->Instance)
    {
        pCAN2_RxFifo1CpltCallback = pFunc;
        HAL_CAN_ActivateNotification(hcan,CAN_IT_RX_FIFO1_MSG_PENDING);
        __HAL_CAN_ENABLE_IT(hcan,CAN_IT_RX_FIFO1_MSG_PENDING);
    }
}



/**
* @brief  Initialize CAN Filter
* @param  hcan: CANx created by CubeMX. id:an array of ListID.
* @return None.
*/
static void CAN_Filter_IDList_Config(CAN_HandleTypeDef * hcan, uint32_t  * ID)
{
    /* can filter config */
    CAN_FilterTypeDef  can_filter;

    can_filter.FilterMode = CAN_FILTERMODE_IDLIST;
    can_filter.FilterScale = CAN_FILTERSCALE_16BIT;
    can_filter.SlaveStartFilterBank = 14;
    //Config IDList
    if (CAN1 == hcan->Instance)
        CAN1_IDList_Config(hcan,&can_filter,ID);
    else if (CAN2 == hcan->Instance)
        CAN2_IDList_Config(hcan,&can_filter,ID);
}

/**
* @brief  Initialize Register about IDlist for CAN1
* @param  hcan: CANx created by CubeMX. Can_Fliter: CAN filter configuration structure.id:an array of ListID.
* @return None.
*/
static void CAN1_IDList_Config(CAN_HandleTypeDef * hcan,CAN_FilterTypeDef  *Can_Fliter,uint32_t  * ID)
{
    uint8_t i = 0;
    while(ID[i]!=0xFFF)
    {
        switch (i%4)
        {
        case 0:
            Can_Fliter->FilterIdHigh = ID[i]<<5;
            break;
        case 1:
            Can_Fliter->FilterIdLow = ID[i]<<5;
            break;
        case 2:
            Can_Fliter->FilterMaskIdHigh = ID[i]<<5;
            break;
        case 3:
        {
            Can_Fliter->FilterMaskIdLow = ID[i]<<5;
            CAN1_FilterEnd_Config(hcan,Can_Fliter,i);
            break;
        }
        }
        if (ID[i+1]==0xFFF)
        {
            CAN1_FilterEnd_Config(hcan,Can_Fliter,i);
        }
        i++;
    }
}

/**
* @brief  Initialize Register about IDlist for CAN2
* @param  hcan: CANx created by CubeMX. Can_Fliter: CAN filter configuration structure.id:an array for ListID.
* @return None.
*/
static void CAN2_IDList_Config(CAN_HandleTypeDef * hcan,CAN_FilterTypeDef  *Can_Fliter,uint32_t  * ID)
{
    uint8_t i = 0;
    while(ID[i]!=0xFFF)
    {
        switch (i%4)
        {
        case 0:
            Can_Fliter->FilterIdHigh = ID[i]<<5;
            break;
        case 1:
            Can_Fliter->FilterIdLow = ID[i]<<5;
            break;
        case 2:
            Can_Fliter->FilterMaskIdHigh = ID[i]<<5;
            break;
        case 3:
        {
            Can_Fliter->FilterMaskIdLow = ID[i]<<5;
            CAN2_FilterEnd_Config(hcan,Can_Fliter,i);
            break;
        }
        }
        if (ID[i+1]==0xFFF)
        {
            CAN2_FilterEnd_Config(hcan,Can_Fliter,i);
        }
        i++;
    }
}

/**
* @brief  Finish initialize register about IDlist for CAN1
* @param  hcan: CANx created by CubeMX. Can_Fliter: CAN filter configuration structure. num: the index of ID array
* @return None.
*/
static void CAN1_FilterEnd_Config(CAN_HandleTypeDef * hcan,CAN_FilterTypeDef  *Can_Fliter,uint8_t num)
{
    Can_Fliter->FilterBank  = num/4;
    Can_Fliter->FilterFIFOAssignment = CAN_FILTER_FIFO0;
    Can_Fliter->FilterActivation = CAN_FILTER_ENABLE;
    HAL_CAN_ConfigFilter(hcan,Can_Fliter);
}

/**
* @brief  Finish initialize register about IDlist for CAN2
* @param  hcan: CANx created by CubeMX. Can_Fliter: CAN filter configuration structure. num: the index of ID array
* @return None.
*/
static void CAN2_FilterEnd_Config(CAN_HandleTypeDef * hcan,CAN_FilterTypeDef  *Can_Fliter,uint8_t num)
{
    Can_Fliter->FilterBank  = num/4+14;
    Can_Fliter->FilterFIFOAssignment = CAN_FILTER_FIFO1;
    Can_Fliter->FilterActivation = CAN_FILTER_ENABLE;
    HAL_CAN_ConfigFilter(hcan,Can_Fliter);
}


/*HAL库FIFO0中断*/
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    if (hcan->Instance ==CAN1)
    {
        if (HAL_CAN_GetRxMessage(hcan,CAN_FILTER_FIFO0,&can_rx_data.header,can_rx_data.data) == HAL_ERROR) {};
        pCAN1_RxFifo0CpltCallback(can_rx_data.header.StdId,can_rx_data.data);
    }
}

/*HAL库FIFO1中断*/
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    if (hcan->Instance == CAN2)
    {
        if (HAL_CAN_GetRxMessage(hcan,CAN_FILTER_FIFO1,&can_rx_data.header,can_rx_data.data)==HAL_ERROR) {};
        pCAN2_RxFifo1CpltCallback(can_rx_data.header.StdId,can_rx_data.data);
    }
}



/************************************************************************************/



/***************CALLBACK***********************/


static void User_can1_callback(uint32_t ID, uint8_t* CAN_RxData)
{
    switch (ID)
    {

//        case CAN_3508_M1_ID:
        case CAN_3508_M2_ID:
        case CAN_3508_M3_ID:
//        case CAN_3508_M4_ID:
        {
            static uint8_t i;
            i = ID- CAN_3508_M1_ID;
            encoder_data_handler(&moto_chassis[i],&hcan1,CAN_RxData);
            status.chassis_status[i] = 1;
            break;
        }
        case POWER_CONTROL_ID:
        {
            Power_data_handler(ID,CAN_RxData);
            status.power_control = 1;
            break;
        }
					case CAN_JUDGE_MSG_ID:
			{
                judge_msg_get(&hcan1, CAN_RxData);
                break;
			}
				 case CHASSIS_CTRl_MSG_ID:
        {
					 gimbal_status=1;
           chassis.spd_input.vx =(int16_t)( CAN_RxData[0]<<8|CAN_RxData[1]);
           chassis.spd_input.vy = (int16_t)(CAN_RxData[2]<<8|CAN_RxData[3]);
           chassis.spd_input.vw =(int16_t)( CAN_RxData[4]<<8|CAN_RxData[5]);
					//tent1++;
           if (CAN_RxData[6]==0x00)
           {
               ctrl_mode=PROTECT_MODE;
               /* code */
           }
           else if (CAN_RxData[6]==0x01)
           {
               ctrl_mode=REMOTER_MODE;
               /* code */
           }
           else if (CAN_RxData[6]==0x02)
           {
               ctrl_mode=AUTO_MODE;
               /* code */
           }
            break;
        }

        default:
        {
            break;
        }
			}		
				
}


static void User_can2_callback(uint32_t ID, uint8_t* CAN_RxData)
{





    switch (ID)
    {
				case CAN_3508_M1_ID:
        case CAN_3508_M2_ID: //2
        case CAN_3508_M3_ID:
        case CAN_3508_M4_ID:
        {
						
            static uint8_t i;
            i = ID- CAN_3508_M1_ID;
            encoder_data_handler(&moto_chassis[i],&hcan2,CAN_RxData);
            status.chassis_status[i] = 1;
            break;
        }

			case CAN_6020_M1_ID:
			case CAN_6020_M2_ID:
			case CAN_6020_M3_ID:
			case CAN_6020_M4_ID:				
			{
				static uint8_t j;
				j = ID - CAN_6020_M1_ID;
				encoder_data_handler(&moto_chassis_6020[j],&hcan2, CAN_RxData);
				
				break;
			}
        

						default:
        {
            break;
        }
				
    }
}


/***************CALLBACK***********************/

/**
  * @brief     get motor initialize offset value
  * @param     ptr: Pointer to a moto_measure_t structure
  * @retval    None
  * @attention this function should be called after system can init
  */
void get_moto_offset(moto_measure_t* ptr, CAN_HandleTypeDef* hcan, uint8_t * CAN_Rx_data)
{
    ptr->ecd        = (uint16_t)(CAN_Rx_data[0] << 8 | CAN_Rx_data[1]);
    ptr->offset_ecd = ptr->ecd;
}

/**
  * @brief     get motor rpm and calculate motor round_count/total_encoder/total_angle
  * @param     ptr: Pointer to a moto_measure_t structure
  * @attention this function should be called after get_moto_offset() function
  */
void encoder_data_handler(moto_measure_t* ptr, CAN_HandleTypeDef* hcan, uint8_t * CAN_Rx_data)
{
    //转子转速
    ptr->speed_rpm     = (int16_t)(CAN_Rx_data[2] << 8 | CAN_Rx_data[3]);
    ptr->given_current = (int16_t)(CAN_Rx_data[4] << 8 | CAN_Rx_data[5]);

    //机械角度
    ptr->last_ecd = ptr->ecd;
    ptr->ecd      = (uint16_t)(CAN_Rx_data[0] << 8 | CAN_Rx_data[1]);

    //相对开机后的角度
    if (ptr->ecd - ptr->last_ecd > 4096)
        ptr->round_cnt--;
    else if (ptr->ecd - ptr->last_ecd < -4096)
        ptr->round_cnt++;

    ptr->total_ecd = ptr->round_cnt * 8192 + ptr->ecd - ptr->offset_ecd;
}
void can1_send_chassis_message(int16_t TX_ID, int16_t iq1, int16_t iq2, int16_t iq3, uint8_t iq4,uint8_t iq5)
{
    uint8_t FreeTxNum = 0;

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
    CAN1_Tx_data[6] = iq4;
    CAN1_Tx_data[7] = iq5;

    //查询发送邮箱是否为空
    FreeTxNum = HAL_CAN_GetTxMailboxesFreeLevel(&hcan1);
    while(FreeTxNum == 0)
    {
        FreeTxNum = HAL_CAN_GetTxMailboxesFreeLevel(&hcan1);
    }

    HAL_CAN_AddTxMessage(&hcan1, &Tx1Message,CAN1_Tx_data,(uint32_t*)CAN_TX_MAILBOX0);
}
/**
  * @brief  send calculated current to motor
  * @param  CAN1 motor current
  */
void can1_send_message(int16_t TX_ID, int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4)
{
//	uint8_t FreeTxNum = 0; 
	static uint32_t txmailbox;
	static uint16_t time =0;
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
		while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0)
		{
				time++;
			if(time > 5)
			{
			time=0;
				return;
			}
		
		}      //如果三个邮箱都阻塞了就等一会儿，直到其中某个邮箱空闲
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

void can2_send_message(int16_t TX_ID, int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4)
{
//    uint8_t FreeTxNum = 0;
			static uint32_t txmailbox;
	static uint16_t time =0;
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

//		while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan2) == 0);      //如果三个邮箱都阻塞了就等一会儿，直到其中某个邮箱空闲
	
			while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan2) == 0)
		{
				time++;
			if(time > 5)
			{
			time=0;
				return;
			}
		
		}  
		
		
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
	HAL_CAN_AddTxMessage(&hcan2, &Tx2Message, CAN2_Tx_data, (uint32_t *)txmailbox);
}


void pit_data_handler(moto_measure_t* ptr, CAN_HandleTypeDef* hcan, uint8_t * CAN_Rx_data)
{
    //转子转速
    ptr->speed_rpm     = (int16_t)(CAN_Rx_data[2] << 8 | CAN_Rx_data[3]);
    ptr->given_current = (int16_t)(CAN_Rx_data[4] << 8 | CAN_Rx_data[5]);

    //机械角度
    ptr->last_ecd = ptr->ecd;
    ptr->ecd      = (uint16_t)(CAN_Rx_data[0] << 8 | CAN_Rx_data[1]);
    if(ptr->ecd<=2000)	ptr->ecd+=8191;
    //相对开机后的角度
    if (ptr->ecd - ptr->last_ecd > 4096)
        ptr->round_cnt--;
    else if (ptr->ecd - ptr->last_ecd < -4096)
        ptr->round_cnt++;

    ptr->total_ecd = ptr->round_cnt * 8192 + ptr->ecd - ptr->offset_ecd;
}
void can1_send_spd_msg(int16_t TX_ID)
{
    uint8_t FreeTxNum = 0;

    Tx1Message.StdId = TX_ID;                                                                                                                                                                                                                                                                                                                                                                                       ;
    Tx1Message.IDE 	 = CAN_ID_STD;
    Tx1Message.RTR   = CAN_RTR_DATA;
    Tx1Message.DLC   = 0x08;
    
    CAN1_Tx_data[0] = chassis.spd_tx_data[0];
    CAN1_Tx_data[1] = chassis.spd_tx_data[1];
    CAN1_Tx_data[2] = chassis.spd_tx_data[2];
    CAN1_Tx_data[3] = chassis.spd_tx_data[3];
    CAN1_Tx_data[4] = chassis.spd_tx_data[4];
    CAN1_Tx_data[5] = chassis.spd_tx_data[5];
    CAN1_Tx_data[6] = chassis.spd_tx_data[6];
    CAN1_Tx_data[7] = chassis.spd_tx_data[7];
    //查询发送邮箱是否为空
    FreeTxNum = HAL_CAN_GetTxMailboxesFreeLevel(&hcan1);
    while(FreeTxNum == 0)
    {
        FreeTxNum = HAL_CAN_GetTxMailboxesFreeLevel(&hcan1);
    }

    HAL_CAN_AddTxMessage(&hcan1, &Tx1Message,CAN1_Tx_data,(uint32_t*)CAN_TX_MAILBOX0);
}
void can1_send_supercap(int16_t TX_ID)
{
    uint8_t supercap_msg_uint8_t[8]= {0};
    uint8_t FreeTxNum = 0;

    Tx1Message.StdId = TX_ID;
    Tx1Message.IDE 	 = CAN_ID_STD;
    Tx1Message.RTR   = CAN_RTR_DATA;
    Tx1Message.DLC   = 0x08;

    supercap_msg_uint8_t[0] = supercap.mode;		//超级电容模式
    memcpy(supercap_msg_uint8_t+1,&supercap.charge_current_set,sizeof(supercap.charge_current_set));
    //查询发送邮箱是否为空
    FreeTxNum = HAL_CAN_GetTxMailboxesFreeLevel(&hcan1);
    while(FreeTxNum == 0)
    {
        FreeTxNum = HAL_CAN_GetTxMailboxesFreeLevel(&hcan1);
    }

    HAL_CAN_AddTxMessage(&hcan1, &Tx1Message,supercap_msg_uint8_t,(uint32_t*)CAN_TX_MAILBOX1);
}

void judge_msg_get(CAN_HandleTypeDef* hcan, uint8_t * CAN_Rx_data)
{
		
    powercontrol.power_buffer = CAN_Rx_data[0];
		powercontrol.max_power = CAN_Rx_data[1];
    memcpy(&powercontrol.chassis_power,CAN_Rx_data+2,sizeof(float));
}
void can1_send_game_msg(int16_t TX_ID,robot_judge_msg_t * robot_judge_msg_e)
{
   uint8_t FreeTxNum = 0;
	/**/
	
		 Game_Status.game_type=robot_judge_msg_e->Game_State_data.game_type;				
    Game_Status.game_progress=robot_judge_msg_e->Game_State_data.game_progress;				
    Game_Status.shooter_id1_17mm_cooling_rate=robot_judge_msg_e->Game_Robot_Status_data.shooter_id1_17mm_speed_limit;
    Game_Status.shooter_id1_17mm_speed_limit=robot_judge_msg_e->Game_Robot_Status_data.shooter_id1_17mm_speed_limit;		
    Game_Status.shooter_id1_17mm_cooling_limit=robot_judge_msg_e->Game_Robot_Status_data.shooter_id1_17mm_cooling_limit;	
    Game_Status.robot_id=robot_judge_msg_e->Game_Robot_Status_data.robot_id;		
    Game_Status.remain_HP=robot_judge_msg_e->Game_Robot_Status_data.remain_HP;	
	  Game_Status.max_HP=robot_judge_msg_e->Game_Robot_Status_data.max_HP;		
    Game_Status.armor_id=robot_judge_msg_e->Robot_Hurt_data.armor_id;	
    Game_Status.hurt_type=robot_judge_msg_e->Robot_Hurt_data.hurt_type;		
    Game_Status.bullet_freq=robot_judge_msg_e->Shoot_Data_data.bullet_freq;
		Game_Status.bullet_speed=robot_judge_msg_e->Shoot_Data_data.bullet_speed;
		Game_Status.bullet_remaining_num_17mm=robot_judge_msg_e->Bullet_Remaining_data.bullet_remaining_num_17mm;
		Game_Status.blue_x=robot_judge_msg_e->Radar_Data.blue_x;
		Game_Status.blue_y=robot_judge_msg_e->Radar_Data.blue_y;
		Game_Status.blue_confiden=robot_judge_msg_e->Radar_Data.blue_confiden;
		Game_Status.red_x=robot_judge_msg_e->Radar_Data.red_x;
		Game_Status.red_y=robot_judge_msg_e->Radar_Data.red_y;
		Game_Status.red_confiden=robot_judge_msg_e->Radar_Data.red_confiden;
    Tx1Message.StdId = TX_ID;                                                                                                                                                                                                                                                                                                                                                                                       ;
    Tx1Message.IDE 	 = CAN_ID_STD;
    Tx1Message.RTR   = CAN_RTR_DATA;
    Tx1Message.DLC   = 0x08;
    if (TX_ID==GAME1_CAN_TX_ID)
    {
				memcpy(CAN1_Tx_data,&Game_Status.game_type,sizeof(Game_Status.game_type));//比赛类型
				memcpy(CAN1_Tx_data+1,&Game_Status.game_progress,sizeof(Game_Status.game_progress));//比赛状态
				memcpy(CAN1_Tx_data+2,&(Game_Status.shooter_id1_17mm_cooling_rate),sizeof(Game_Status.shooter_id1_17mm_cooling_rate));//17mm冷却速率
			memcpy(CAN1_Tx_data+4,&Game_Status.shooter_id1_17mm_speed_limit,sizeof(Game_Status.shooter_id1_17mm_speed_limit));//17mm射速限制
			memcpy(CAN1_Tx_data+6,& Game_Status.shooter_id1_17mm_cooling_limit,sizeof(Game_Status.shooter_id1_17mm_cooling_limit));//17mm冷却上限
    }
		
    else if (TX_ID==GAME2_CAN_TX_ID)
    {
				memcpy(CAN1_Tx_data,&Game_Status.robot_id,sizeof(Game_Status.robot_id));//id
				memcpy(CAN1_Tx_data+1,&Game_Status.remain_HP,sizeof(Game_Status.remain_HP));//剩余血量
				memcpy(CAN1_Tx_data+3,&Game_Status.max_HP,sizeof(Game_Status.max_HP));//最大血量
				memcpy(CAN1_Tx_data+5,&Game_Status.armor_id,sizeof(Game_Status.armor_id));//装甲板id
				memcpy(CAN1_Tx_data+6,&Game_Status.hurt_type,sizeof(Game_Status.hurt_type));//扣血类型
//        CAN1_Tx_data[0] = robot_judge_msg_e->Game_Robot_Status_data.robot_id;//id
//        CAN1_Tx_data[1] = robot_judge_msg_e->Game_Robot_Status_data.remain_HP>>8;//剩余血量
//        CAN1_Tx_data[2] = robot_judge_msg_e->Game_Robot_Status_data.remain_HP;
//				CAN1_Tx_data[3] = robot_judge_msg_e->Game_Robot_Status_data.max_HP>>8;//最大血量
//				CAN1_Tx_data[4] = robot_judge_msg_e->Game_Robot_Status_data.max_HP;
//				CAN1_Tx_data[5] = robot_judge_msg_e->Robot_Hurt_data.armor_id;//装甲板id
//				CAN1_Tx_data[6] = robot_judge_msg_e->Robot_Hurt_data.hurt_type;//扣血类型
    }
		  else if (TX_ID==GAME3_CAN_TX_ID)
    {
				memcpy(CAN1_Tx_data,&Game_Status.bullet_freq,sizeof(Game_Status.bullet_freq));//射频
				memcpy(CAN1_Tx_data+1,&Game_Status.bullet_speed,sizeof(Game_Status.bullet_speed));//射速
				memcpy(CAN1_Tx_data+3,&Game_Status.bullet_remaining_num_17mm,sizeof(Game_Status.bullet_remaining_num_17mm));//剩余17mm发射量

    }
			  else if (TX_ID==GAME4_CAN_TX_ID)//blue
    {
			memcpy(CAN1_Tx_data,&Game_Status.blue_x,sizeof(Game_Status.blue_x));
				memcpy(CAN1_Tx_data+4,&Game_Status.blue_y,sizeof(Game_Status.blue_y));
    }
			  else if (TX_ID==GAME5_CAN_TX_ID)//red
    {
				memcpy(CAN1_Tx_data,&Game_Status.red_x,sizeof(Game_Status.red_x));//
				memcpy(CAN1_Tx_data+4,&Game_Status.red_y,sizeof(Game_Status.red_y));//
    }
				  else if (TX_ID==GAME6_CAN_TX_ID)//red
    {
				memcpy(CAN1_Tx_data,&Game_Status.blue_confiden,sizeof(Game_Status.blue_confiden));
				memcpy(CAN1_Tx_data+4,&Game_Status.red_confiden,sizeof(Game_Status.red_confiden));//
    }
    
    
    
   
    //查询发送邮箱是否为空
    FreeTxNum = HAL_CAN_GetTxMailboxesFreeLevel(&hcan1);
    while(FreeTxNum == 0)
    {
        FreeTxNum = HAL_CAN_GetTxMailboxesFreeLevel(&hcan1);
    }

    HAL_CAN_AddTxMessage(&hcan1, &Tx1Message,CAN1_Tx_data,(uint32_t*)CAN_TX_MAILBOX0);

}
