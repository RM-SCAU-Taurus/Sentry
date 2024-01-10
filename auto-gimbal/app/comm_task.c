#include "comm_task.h"
#include "bsp_can.h"
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
motor_current_t motor_cur;
vision_tx_msg_t vision_tx_msg = {0};
extern chassis_odom_info_t chassis_odom;
 extern Game_Status_t Game_Status;
extern chassis_ctrl_info_t chassis_ctrl;
int i=0;
/**
  * @brief can_msg_send_task
  * @param
  * @attention
  * @note
  */
void can_msg_send_task(void const *argu)
{
    osEvent event;
    for(;;)
    {
        event = osSignalWait(GIMBAL_MOTOR_MSG_SEND  | \
                             CHASSIS_MOTOR_MSG_SEND | \
                             SHOOT_MOTOR_MSG_SEND, osWaitForever);
        if( event.status == osEventSignal )
        {
            if( ctrl_mode==PROTECT_MODE || !lock_flag )
            {
                for(int i=0; i<4; i++)		motor_cur.chassis_cur[i]= 0;
                for(int i=0; i<2; i++)		motor_cur.gimbal_cur[i] = 0;
                motor_cur.trigger_cur = 0;
                can1_send_message(GIMBAL_CAN_TX_ID, 0, 0, 0, 0);
                can2_send_message(GIMBAL_CAN_TX_ID, 0, 0, 0, 0);
								send_judge_msg(0x09,&hcan1);
						}
            else if( lock_flag )  //有陀螺仪数据才给电流
            {
                if( event.value.signals)
                {
                    /* 小步兵 */
//									can1_send_message(GIMBAL_CAN_TX_ID, 0, 0, 0, 0);
//                can2_send_message(GIMBAL_CAN_TX_ID, 0, 0, 0, 0);
                    can1_send_message(GIMBAL_CAN_TX_ID, motor_cur.gimbal_cur[0], 0, motor_cur.trigger_cur, 0);
                    can2_send_message(GIMBAL_CAN_TX_ID, 0, motor_cur.gimbal_cur[1], 0, 0);
										send_judge_msg(0x09,&hcan1);
								}
                if( event.value.signals & CHASSIS_MOTOR_MSG_SEND )
                {
								}
               // can1_send_supercap();
            }
        }
    }
}
