#define __BSP_CAN_GLOBALS
#include "bsp_can.h"
#include "pid.h"
#include "chassis_task.h"
#include "status_task.h"
#include "string.h"
#include "bsp_T_imu.h"
#include "bsp_powerlimit.h"
#include "modeswitch_task.h"
#include "bsp_judge.h"
#include "comm_task.h"
CAN_TxHeaderTypeDef Tx1Message;
CAN_RxHeaderTypeDef Rx1Message;
CAN_TxHeaderTypeDef Tx2Message;
CAN_RxHeaderTypeDef Rx2Message;

uint8_t CAN1_Rx_data[8];
uint8_t CAN1_Tx_data[8];
uint8_t CAN2_Rx_data[8];
uint8_t CAN2_Tx_data[8];
uint8_t gimbal_status;
Game_Status_t Game_Status;
/**
  * @brief     CAN接受中断回调函数
  * @param     CAN_Rx_data ：CAN节点反馈的数据帧
  * @attention
  */
	int tent1;
	int judge_tent;
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    if(hcan == &hcan1)
    {
        HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO0, &Rx1Message, CAN1_Rx_data);
        switch (Rx1Message.StdId)
        {
//        case CAN_3508_M1_ID:
        case CAN_3508_M2_ID:
        case CAN_3508_M3_ID:
//        case CAN_3508_M4_ID:
        {
            static uint8_t i;
            i = Rx1Message.StdId - CAN_3508_M1_ID;
            encoder_data_handler(&moto_chassis[i],&hcan1,CAN1_Rx_data);
            status.chassis_status[i] = 1;
            break;
        }
        case POWER_CONTROL_ID:
        {
            Power_data_handler(Rx1Message.StdId,CAN1_Rx_data);
            status.power_control = 1;
            break;
        }
					case CAN_JUDGE_MSG_ID:
			{
                judge_msg_get(&hcan1, CAN1_Rx_data);
								judge_tent++;
                break;
			}
        case CHASSIS_CTRl_MSG_ID:
        {
						gimbal_status=1;
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
        default:
        {
            break;
        }
        };
        __HAL_CAN_ENABLE_IT(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
    }
    else if( hcan == &hcan2 )
    {
        HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO0, &Rx2Message, CAN2_Rx_data);
        switch( Rx2Message.StdId )//1 3
        {
				case CAN_3508_M1_ID:
//        case CAN_3508_M2_ID: //2
//        case CAN_3508_M3_ID:
        case CAN_3508_M4_ID:
        {
						
            static uint8_t i;
            i = Rx2Message.StdId - CAN_3508_M1_ID;
            encoder_data_handler(&moto_chassis[i],&hcan2,CAN2_Rx_data);
            status.chassis_status[i] = 1;
            break;
        }
        case TIMU_PALSTANCE_ID:
        {
            T_imu_calcu(Rx2Message.StdId, CAN2_Rx_data);
            status.gyro_status[0] = 1;
					break;
        }
        case TIMU_ANGLE_ID:
        {
            T_imu_calcu(Rx2Message.StdId, CAN2_Rx_data);
            status.gyro_status[1] = 1;
            break;
        }
			case CAN_6020_M1_ID:
			case CAN_6020_M2_ID:
			case CAN_6020_M3_ID:
			case CAN_6020_M4_ID:				
			{
				static uint8_t j;
				j = Rx2Message.StdId - CAN_6020_M1_ID;
				encoder_data_handler(&moto_chassis_6020[j],&hcan2, CAN2_Rx_data);
				
				break;
			}
        }
        __HAL_CAN_ENABLE_IT(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
    }
}

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
void can1_send_message(int16_t TX_ID, int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4)
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
    CAN1_Tx_data[6] = iq4 >> 8;
    CAN1_Tx_data[7] = iq4;

    //查询发送邮箱是否为空
    FreeTxNum = HAL_CAN_GetTxMailboxesFreeLevel(&hcan1);
    while(FreeTxNum == 0)
    {
        FreeTxNum = HAL_CAN_GetTxMailboxesFreeLevel(&hcan1);
    }

    HAL_CAN_AddTxMessage(&hcan1, &Tx1Message,CAN1_Tx_data,(uint32_t*)CAN_TX_MAILBOX0);
}

void can2_send_message(int16_t TX_ID, int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4)
{
    uint8_t FreeTxNum = 0;

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

    //查询发送邮箱是否为空
    FreeTxNum = HAL_CAN_GetTxMailboxesFreeLevel(&hcan2);
    while(FreeTxNum == 0)
    {
        FreeTxNum = HAL_CAN_GetTxMailboxesFreeLevel(&hcan2);
    }

    HAL_CAN_AddTxMessage(&hcan2, &Tx2Message,CAN2_Tx_data,(uint32_t*)CAN_TX_MAILBOX0);
}

/**
  * @brief  init the can transmit and receive
  * @param  None
  */
void can_device_init(void)
{
    //can1 filter config
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
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING);
    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan2,CAN_IT_RX_FIFO0_MSG_PENDING);
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
