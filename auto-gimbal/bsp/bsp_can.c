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
#include "bsp_Mf_Motor.h"
#include "math_calcu.h"

CAN_TxHeaderTypeDef Tx1Message;
CAN_RxHeaderTypeDef Rx1Message;
CAN_TxHeaderTypeDef Tx2Message;
CAN_RxHeaderTypeDef Rx2Message;

uint8_t CAN1_Rx_data[8];
uint8_t CAN1_Tx_data[8];
uint8_t CAN2_Rx_data[8];
uint8_t CAN2_Tx_data[8];
uint8_t GAME_STATE;

/* 定义中值滤波变量 */
float RM6020_array[9] = {0};
float follow_yaw_data=0;
extern Game_Status_t Game_Status;
moto_mf_t YAW_9025;

#define return_time 2

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
    uint32_t can_ID1[] = {CAN_YAW_9025_MOTOR_ID, CHASSIS_MSG_ID, POWER_CONTROL_ID,0xFFF};
    uint32_t can_ID2[] = {TIMU_PALSTANCE_ID,TIMU_9025_ID,TIMU_ANGLE_ID, CAN_TRIGGER_MOTOR1_ID,CAN_YAW_6020_MOTOR_ID,0xFFF};
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
//					case CAN_YAW_6020_MOTOR_ID:
//        {
//            encoder_data_handler(&moto_yaw, &hcan1, CAN_RxData);
//            status.gimbal_status[0] = 1;
//            break;
//        }
				 case CAN_YAW_9025_MOTOR_ID:
        {
		  	encoder_data_receive(&YAW_9025,CAN_RxData);
//            status.gimbal_status[0] = 1;
            break;
        }
        case CHASSIS_MSG_ID:
        {
            chassis.msg_handle(ID, CAN_RxData);
            //  can_msg_read(Rx1Message.StdId,CAN1_Rx_data);
            break;
        }

        case POWER_CONTROL_ID:
        {
            Power_data_handler(ID, CAN_RxData);
            status.power_control = 1;
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
			case CAN_YAW_6020_MOTOR_ID:
        {
            encoder_data_handler(&moto_yaw, &hcan2, CAN_RxData);
            status.gimbal_status[0] = 1;
            break;
        }
				
				case TIMU_PALSTANCE_ID:
        {
            T_imu_calcu(ID, CAN_RxData);
            status.gyro_status[0] = 1;
        }
        case TIMU_ANGLE_ID:
        {
            T_imu_calcu(ID, CAN_RxData);
            status.gyro_status[1] = 1;
            break;
        }

        case CAN_PIT_MOTOR_ID:
        {
            encoder_data_handler(&moto_pit, &hcan2, CAN_RxData);
            status.gimbal_status[1] = 1;
            /* 当机械零点在pit轴运动范围内,拓展编码值范围 */
            //            if( gimbal.pid.pit_ecd_fdb < 5000 )  //确定电机编码值在零点附近时，当正向越界，从8191到1阶跃
            //            {
            //                gimbal.pid.pit_ecd_fdb += 8191;
            //            }
            break;
        }
				
			 case CAN_TRIGGER_MOTOR1_ID: // 上枪管拨盘2006
        {
            motor_trigger.msg_cnt++ <= 50 ? get_moto_offset(&motor_trigger, &hcan2, CAN_RxData) : encoder_data_handler(&motor_trigger, &hcan2, CAN_RxData);
            status.gimbal_status[2] = 1;
            break;
        }
			case TIMU_9025_ID:
        {
            T_imu_calcu(ID, CAN_RxData);
            break;
        }
			case CAN_3508_L_ID:
			case CAN_3508_R_ID:
		  	{
														
            static uint8_t i;
            i = ID- CAN_3508_L_ID;
            encoder_data_handler(&moto_fric[i],&hcan2,CAN_RxData);
            status.moto_fric[i] = 1;
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
void get_moto_offset(moto_measure_t *ptr, CAN_HandleTypeDef *hcan, uint8_t *CAN_Rx_data)
{
    ptr->ecd = (uint16_t)(CAN_Rx_data[0] << 8 | CAN_Rx_data[1]);
    ptr->offset_ecd = ptr->ecd;
}

/**
 * @brief     get the imformation of game
 * @param     ptr: Pointer to a Game_Status_t structure
 * @attention this function should be called after the Game_id OF CAN
 */
uint32_t test_run_period1 = 0;
void game_data_handler(robot_judge_msg_t *robot_judge_msg_e)
{
    static uint32_t run_time, last_run_time = 0;
    run_time = HAL_GetTick();
    test_run_period1 = run_time - last_run_time;
    robot_judge_msg_copy(); // 裁判系统结构体拷贝
    /*裁判系统数据赋值*/
    Game_Status.game_type = (float)robot_judge_msg_e->Game_State_data.game_type;
    Game_Status.game_progress = (float)robot_judge_msg_e->Game_State_data.game_progress;
    Game_Status.shooter_id1_17mm_cooling_rate = robot_judge_msg_e->Game_Robot_Status_data.shooter_id1_17mm_speed_limit;
    Game_Status.shooter_id1_17mm_speed_limit = robot_judge_msg_e->Game_Robot_Status_data.shooter_id1_17mm_speed_limit;
    Game_Status.shooter_id1_17mm_cooling_limit = robot_judge_msg_e->Game_Robot_Status_data.shooter_id1_17mm_cooling_limit;
    Game_Status.robot_id = robot_judge_msg_e->Game_Robot_Status_data.robot_id;
    Game_Status.remain_HP = robot_judge_msg_e->Game_Robot_Status_data.remain_HP;
    Game_Status.max_HP = robot_judge_msg_e->Game_Robot_Status_data.max_HP;
    Game_Status.armor_id = robot_judge_msg_e->Robot_Hurt_data.armor_id;
    Game_Status.hurt_type = robot_judge_msg_e->Robot_Hurt_data.hurt_type;
    Game_Status.bullet_freq = robot_judge_msg_e->Shoot_Data_data.bullet_freq;
    Game_Status.bullet_speed = robot_judge_msg_e->Shoot_Data_data.bullet_speed;
    Game_Status.bullet_remaining_num_17mm = robot_judge_msg_e->Bullet_Remaining_data.bullet_remaining_num_17mm;
    Game_Status.game_time = (float)robot_judge_msg_e->Game_State_data.stage_remain_time;

    if (robot_judge_msg_e->Game_Robot_Status_data.robot_id < 100) // 发送敌方哨兵数据
    {
        Game_Status.Enemy_Sentry_HP = robot_judge_msg_e->RobotHP_data.blue_7_robot_HP;  // 敌方哨兵
        Game_Status.Self_outpost_HP = robot_judge_msg_e->RobotHP_data.red_outpost_HP;   // 我方前哨战
        Game_Status.Enemy_outpost_HP = robot_judge_msg_e->RobotHP_data.blue_outpost_HP; // 敌方前哨战
    }
    else
    {
        Game_Status.Enemy_Sentry_HP = robot_judge_msg_e->RobotHP_data.red_7_robot_HP;  // 敌方哨兵
        Game_Status.Enemy_outpost_HP = robot_judge_msg_e->RobotHP_data.red_outpost_HP; // 敌方前哨战
        Game_Status.Self_outpost_HP = robot_judge_msg_e->RobotHP_data.blue_outpost_HP; // 我方前哨战
    }
    Game_Status.commd_keyboard = robot_judge_msg_e->Robot_command_Data.commd_keyboard;
    Game_Status.target_robot_ID = robot_judge_msg_e->Robot_command_Data.target_robot_ID;
    Game_Status.target_position_x = robot_judge_msg_e->Robot_command_Data.target_position_x;
    Game_Status.target_position_y = robot_judge_msg_e->Robot_command_Data.target_position_y;
    Game_Status.target_position_z = robot_judge_msg_e->Robot_command_Data.target_position_z;
    Game_Status.red_y = robot_judge_msg_e->Radar_Data.red_y;
    Game_Status.red_confiden = robot_judge_msg_e->Radar_Data.red_confiden;
}

/**
 * @brief     get motor rpm and calculate motor round_count/total_encoder/total_angle
 * @param     ptr: Pointer to a moto_measure_t structure
 * @attention this function should be called after get_moto_offset() function
 */

void encoder_data_handler(moto_measure_t *ptr, CAN_HandleTypeDef *hcan, uint8_t *CAN_Rx_data)
{
    // 转子转速
    ptr->speed_rpm = (int16_t)(CAN_Rx_data[2] << 8 | CAN_Rx_data[3]);
    ptr->given_current = (int16_t)(CAN_Rx_data[4] << 8 | CAN_Rx_data[5]);

    // 机械角度
    ptr->last_ecd = ptr->ecd;
    ptr->ecd = (uint16_t)(CAN_Rx_data[0] << 8 | CAN_Rx_data[1]);

//		follow_yaw_data = GildeAverageValueFilter(ptr->ecd, RM6020_array);

    // 相对开机后的角度
    if (ptr->ecd - ptr->last_ecd > 4096)
        ptr->round_cnt--;
    else if (ptr->ecd - ptr->last_ecd < -4096)
        ptr->round_cnt++;

    ptr->total_ecd = ptr->round_cnt * 8192 + ptr->ecd - ptr->offset_ecd;
}
void can1_send_chassis_message(int16_t TX_ID, int16_t iq1, int16_t iq2, int16_t iq3, uint8_t iq4, uint8_t iq5)
{
    uint8_t FreeTxNum = 0;

    Tx1Message.StdId = TX_ID;
    Tx1Message.IDE = CAN_ID_STD;
    Tx1Message.RTR = CAN_RTR_DATA;
    Tx1Message.DLC = 0x08;

    CAN1_Tx_data[0] = iq1 >> 8;
    CAN1_Tx_data[1] = iq1;
    CAN1_Tx_data[2] = iq2 >> 8;
    CAN1_Tx_data[3] = iq2;
    CAN1_Tx_data[4] = iq3 >> 8;
    CAN1_Tx_data[5] = iq3;
    CAN1_Tx_data[6] = iq4;
    CAN1_Tx_data[7] = iq5;

    // 查询发送邮箱是否为空
    FreeTxNum = HAL_CAN_GetTxMailboxesFreeLevel(&hcan1);
    while (FreeTxNum == 0)
    {
        FreeTxNum = HAL_CAN_GetTxMailboxesFreeLevel(&hcan1);
    }

    HAL_CAN_AddTxMessage(&hcan1, &Tx1Message, CAN1_Tx_data, (uint32_t *)CAN_TX_MAILBOX0);
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
			if(time > return_time)
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
			if(time > return_time)
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



void send_judge_msg(int16_t TX_ID, CAN_HandleTypeDef *hcan)
{
//    uint8_t FreeTxNum = 0;
    CAN_TxHeaderTypeDef TxMessage;
    uint8_t judeg_msg_uint8_t[8] = {0};
    static uint32_t MailBox;
		static uint16_t time;
    TxMessage.StdId = TX_ID;
    TxMessage.IDE = CAN_ID_STD;
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.DLC = 0x08;

    judeg_msg_uint8_t[0] = (uint8_t)Power_Heat_Data.chassis_power_buffer;
    judeg_msg_uint8_t[1] = (uint8_t)powercontrol.max_power;
    memcpy(judeg_msg_uint8_t + 2, (void *)&Power_Heat_Data.chassis_power, sizeof(Power_Heat_Data.chassis_power));
		
			while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0)
		{
				time++;
			if(time > return_time)
			{
			time=0;
				return;
			}
		
		}      //如果三个邮箱都阻塞了就等一会儿，直到其中某个邮箱空闲
	if ((hcan1.Instance->TSR & CAN_TSR_TME0) != RESET)     //如果邮箱0空闲
	{
		MailBox =CAN_TX_MAILBOX0;
	}

	/* Check Tx Mailbox 1 status */
	else if ((hcan1.Instance->TSR & CAN_TSR_TME1) != RESET)
	{
		MailBox =CAN_TX_MAILBOX1;
	}

	/* Check Tx Mailbox 2 status */
	else if ((hcan1.Instance->TSR & CAN_TSR_TME2) != RESET)
	{
		MailBox =CAN_TX_MAILBOX2;
	}
	HAL_CAN_AddTxMessage(&hcan1, &TxMessage, judeg_msg_uint8_t, (uint32_t *)MailBox);
	
}
/**
 * @brief  init the can transmit and receive
 * @param  None
 */
//void can_device_init(void)
//{
//    // can1 filter config
//    CAN_FilterTypeDef can_filter;

//    can_filter.FilterBank = 0;
//    can_filter.FilterMode = CAN_FILTERMODE_IDMASK;
//    can_filter.FilterScale = CAN_FILTERSCALE_32BIT;
//    can_filter.FilterIdHigh = 0x0000;
//    can_filter.FilterIdLow = 0x0000;
//    can_filter.FilterMaskIdHigh = 0x0000;
//    can_filter.FilterMaskIdLow = 0x0000;
//    can_filter.FilterFIFOAssignment = CAN_FilterFIFO0;
//    can_filter.SlaveStartFilterBank = 0;
//    can_filter.FilterActivation = ENABLE;

//    HAL_CAN_ConfigFilter(&hcan1, &can_filter);
//    while (HAL_CAN_ConfigFilter(&hcan1, &can_filter) != HAL_OK)
//        ;

//    can_filter.FilterBank = 14;
//    HAL_CAN_ConfigFilter(&hcan2, &can_filter);
//    while (HAL_CAN_ConfigFilter(&hcan2, &can_filter) != HAL_OK)
//        ;

//    HAL_Delay(100);
//    HAL_CAN_Start(&hcan1);
//    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
//    HAL_CAN_Start(&hcan2);
//    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
//}

void pit_data_handler(moto_measure_t *ptr, CAN_HandleTypeDef *hcan, uint8_t *CAN_Rx_data)
{
    // 转子转速
    ptr->speed_rpm = (int16_t)(CAN_Rx_data[2] << 8 | CAN_Rx_data[3]);
    ptr->given_current = (int16_t)(CAN_Rx_data[4] << 8 | CAN_Rx_data[5]);

    // 机械角度
    ptr->last_ecd = ptr->ecd;
    ptr->ecd = (uint16_t)(CAN_Rx_data[0] << 8 | CAN_Rx_data[1]);
    if (ptr->ecd <= 2000)
        ptr->ecd += 8191;
    // 相对开机后的角度
    if (ptr->ecd - ptr->last_ecd > 4096)
        ptr->round_cnt--;
    else if (ptr->ecd - ptr->last_ecd < -4096)
        ptr->round_cnt++;

    ptr->total_ecd = ptr->round_cnt * 8192 + ptr->ecd - ptr->offset_ecd;
}

void send_message_mf(int16_t TX_ID,uint8_t Command_byte,int16_t iq1,int16_t iq2,int16_t iq3)
{
    iq1 = data_limit(iq1,2024,-2024);
    CAN_TxHeaderTypeDef Tx1Message;
		static uint32_t txmailbox;
    uint8_t CAN1_Tx_data[8];
//    uint8_t FreeTxNum = 0;
		static uint16_t time;
    Tx1Message.StdId = TX_ID;
    Tx1Message.ExtId = 0x00;
    Tx1Message.RTR   = CAN_RTR_DATA;
    Tx1Message.IDE 	 = CAN_ID_STD;
    Tx1Message.DLC   = 8;

    CAN1_Tx_data[0] = Command_byte;
    CAN1_Tx_data[1] = 0x00;
    CAN1_Tx_data[2] = iq2 ;
    CAN1_Tx_data[3] = iq2>> 8;
    CAN1_Tx_data[4] = iq1;
    CAN1_Tx_data[5] = iq1 >> 8;
    CAN1_Tx_data[6] = iq3;
    CAN1_Tx_data[7] = iq3>> 8;

    //查询发送邮箱是否为空
//    FreeTxNum = HAL_CAN_GetTxMailboxesFreeLevel(&hcan1);
//    while(FreeTxNum == 0)
//    {
//        FreeTxNum = HAL_CAN_GetTxMailboxesFreeLevel(&hcan1);
//    }

//    HAL_CAN_AddTxMessage(&hcan1, &Tx1Message,CAN1_Tx_data,(uint32_t*)CAN_TX_MAILBOX2);
//		
				while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0)
		{
				time++;
			if(time > return_time)
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

