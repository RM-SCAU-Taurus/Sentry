#include "comm_task.h"
#include "bsp_can.h"
#include "bsp_vision.h"
#include "cmsis_os.h"
#include "modeswitch_task.h"
#include "bsp_can.h"
#include "bsp_T_imu.h"
#include "control_def.h"

#include "bsp_usart.h"
#include "remote_msg.h"
#include "bsp_judge.h"
#include "shoot_task.h"
#include "protocol_camp.h"
#include "iwdg.h"
int size;
motor_current_t motor_cur;
vision_tx_msg_t vision_tx_msg = {0};
extern chassis_odom_info_t chassis_odom;
extern game_status_info_t game_state;
extern chassis_ctrl_info_t chassis_ctrl;
//extern void rm_queue_data(uint16_t cmd_id,void* buf,uint16_t len );
/**
  * @brief can_msg_send_task
  * @param
  * @attention
  * @note
  */
int tent=0;
void can_msg_send_task(void const *argu)
{
//    osEvent event;
    for(;;)
    {
							if( ctrl_mode==PROTECT_MODE || !lock_flag )
          {
                for(int i=0; i<4; i++)		motor_cur.chassis_cur_3508[i]= 0;
								for(int i=0; i<4; i++)		motor_cur.chassis_cur_6020[i]= 0;
						can2_send_message(CHASSIS_CAN_TX_ID,motor_cur.chassis_cur_3508[0], motor_cur.chassis_cur_3508[1], motor_cur.chassis_cur_3508[2], motor_cur.chassis_cur_3508[3]);
						can1_send_message(CHASSIS_CAN_TX_ID,motor_cur.chassis_cur_3508[0],motor_cur.chassis_cur_3508[1],motor_cur.chassis_cur_3508[2],motor_cur.chassis_cur_3508[3]);
            can2_send_message(GIMBAL_CAN_TX_ID,motor_cur.chassis_cur_6020[0],motor_cur.chassis_cur_6020[1],motor_cur.chassis_cur_6020[2],motor_cur.chassis_cur_6020[3]);
        }
            else if( lock_flag )  //解锁才给电流
            {
                   if(ctrl_mode==AUTO_MODE)
									 { 
											can2_send_message(CHASSIS_CAN_TX_ID,motor_cur.chassis_cur_3508[0],0,motor_cur.chassis_cur_3508[2],0);
										can1_send_message(CHASSIS_CAN_TX_ID,0,motor_cur.chassis_cur_3508[1],0,motor_cur.chassis_cur_3508[3]);
                    can2_send_message(GIMBAL_CAN_TX_ID,motor_cur.chassis_cur_6020[0],motor_cur.chassis_cur_6020[1],motor_cur.chassis_cur_6020[2],motor_cur.chassis_cur_6020[3]);
									 }
										 else if(ctrl_mode==REMOTER_MODE)
										 {
										can2_send_message(CHASSIS_CAN_TX_ID,motor_cur.chassis_cur_3508[0],0,0,motor_cur.chassis_cur_3508[3]);
										can1_send_message(CHASSIS_CAN_TX_ID,0,motor_cur.chassis_cur_3508[1],motor_cur.chassis_cur_3508[2],0);
//										can2_send_message(CHASSIS_CAN_TX_ID,motor_cur.chassis_cur_3508[0], motor_cur.chassis_cur_3508[1], motor_cur.chassis_cur_3508[2], motor_cur.chassis_cur_3508[3]);
//										can1_send_message(CHASSIS_CAN_TX_ID,motor_cur.chassis_cur_3508[0],motor_cur.chassis_cur_3508[1],motor_cur.chassis_cur_3508[2],motor_cur.chassis_cur_3508[3]);
                    can2_send_message(GIMBAL_CAN_TX_ID,motor_cur.chassis_cur_6020[0],motor_cur.chassis_cur_6020[1],motor_cur.chassis_cur_6020[2],motor_cur.chassis_cur_6020[3]);
									}
								}
//								}
					osDelay(1);
//        }
//    }
}
		}
int tent2;
void can_msg_chassis_to_gimbal(void const *argu)
{
    uint32_t wake_up_time = osKernelSysTick();
    for(;;)
    {
				size = xPortGetFreeHeapSize();
				chassis.msg_send(CHASSIS_TO_Gimbal_CAN_TX_ID,chassis.spd_fdb.vx,chassis.spd_fdb.vy,chassis.spd_fdb.vw,0x00,0);//CAN1发送
				can1_send_supercap(SUPERCAP_CAN_TX_ID);
					//tent2++;
        osDelayUntil(&wake_up_time, 10);
    }
}


/**
  * @brief usart_msg_send_task
  */
float pit_angle_error_buf = 0;
void usart_msg_send_task(void const *argu)
{
//    uint8_t vision_tx_buf[8];
//    int a;
    uint32_t mode_wake_time = osKernelSysTick();
    for(;;)
    {
        taskENTER_CRITICAL();
        /* 帧头 */
     //   vision_tx_msg.SOF = 0x11;
        /* 获取PIT与水平角度差 */
//        pit_angle_error_buf = ((moto_pit.ecd - GIMBAL_PIT_CENTER_OFFSET) / (22.752f));
//        memcpy(vision_tx_msg.pit_angle_error, &pit_angle_error_buf, 4);
      //  memcpy(vision_tx_msg.pit_angle_error, &imu_data.wz, 4);
        /* 获取射速信息 */
      //  vision_tx_msg.mode_msg.shooter_speed = shoot.shoot_speed_vision;
        /* 视觉状态检测 */
      /*  if( vision_mode == VISION_MODE_sENERGY )
        {
            key_status_clear(KEY_VISION_sENERGY);  //清除小能量键盘按键状态
            vision_tx_msg.mode_msg.aiming_status = 1;	//大能量
        }
        else if( vision_mode == VISION_MODE_bENERGY )
        {
            vision_tx_msg.mode_msg.aiming_status = 2;	//小能量
        }
        else if( vision_mode == VISION_MODE_ANTIROTATE )
            vision_tx_msg.mode_msg.aiming_status = 0;   //反小陀螺
        else
            vision_tx_msg.mode_msg.aiming_status = 0;	//自瞄（小预测） 或 打哨兵模式（大预测）
        if( Game_Robot_Status.robot_id > 100 )		vision_tx_msg.mode_msg.camp = 1;
        else 										vision_tx_msg.mode_msg.camp = 0;
          
        帧尾 */
      /*  vision_tx_msg.EOF1 = 0x22;
        vision_tx_msg.EOF2 = 0x33;*/

        /* 打包视觉发送信息 */
      /*  memcpy(vision_tx_buf, &vision_tx_msg, 8);
        HAL_UART_Transmit_DMA(&huart4, vision_tx_buf, 8);*/
       // rm_queue_data(GAME_STATUS_FDB_ID,&game_state,sizeof(game_state));
      //  rm_queue_data(CHASSIS_ODOM_FDB_ID,&chassis_odom,sizeof(chassis_odom));
       // a++;
        taskEXIT_CRITICAL();
        osDelayUntil(&mode_wake_time, USART_SEND_PERIOD);
     
    }
}

