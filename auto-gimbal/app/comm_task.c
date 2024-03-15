
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
unsigned portBASE_TYPE uxHighWaterMark;
/**
  * @brief can_msg_send_task
  * @param
  * @attention
  * @note
  */
void can_msg_send_task(void const *argu)
{
    osEvent event;
    static uint8_t mf_times;
    static uint8_t mf_times_protect;
    static uint16_t fric_protect_times;
		static uint16_t fric_times;
    for (;;)
    {
        event = osSignalWait(GIMBAL_MOTOR_MSG_SEND | \
                             CHASSIS_MOTOR_MSG_SEND | \
                             SHOOT_MOTOR_MSG_SEND, osWaitForever);
        if (event.status != osEventSignal)
            continue;

        if (ctrl_mode == PROTECT_MODE || !lock_flag)
        {
            if (event.value.signals & GIMBAL_MOTOR_MSG_SEND)
            {
                for (int i = 0; i < 4; i++)
                    motor_cur.chassis_cur[i] = 0;
                for (int i = 0; i < 2; i++)
                    motor_cur.gimbal_cur[i] = 0;
                motor_cur.trigger_cur = 0;
                can2_send_message(GIMBAL_CAN_TX_ID, 0, 0, 0, 0);
                if (++mf_times_protect >= 2)
                {
                    send_message_mf(CAN_9025_YAW_TX_ID, TORQUE_COMMAND, 0, 0, 0);
                    mf_times_protect = 0;
                }
            }
            else if (event.value.signals & SHOOT_MOTOR_MSG_SEND)
            {
                if (fric.protect_flag == FRIC_SLOW_TO_PROTECT && (fric_protect_times++ <= 50))
								{ 	can2_send_message(Fric_CAN_TX_ID, motor_cur.fric_cur[0], motor_cur.fric_cur[1], 0, 0);
										can1_send_message(CAN_Fric_Ctrl_ID,0,fric.Fric_Pid_Set[0].fric_spd_ref,fric.Fric_Pid_Set[1].fric_spd_ref,0);}
                else
                {
                    can2_send_message(Fric_CAN_TX_ID, 0, 0, 0, 0);
                    fric.protect_flag = FRIC_PROTECT;
                    fric_protect_times = 0;
                }
            }
        }
        else if (lock_flag)
        {
            if (event.value.signals & GIMBAL_MOTOR_MSG_SEND)
            {
                
                can2_send_message(GIMBAL_CAN_TX_ID, motor_cur.gimbal_cur[0], motor_cur.gimbal_cur[1], 0, 0);
                if (++mf_times >= 2)
                {
                    send_message_mf(CAN_9025_YAW_TX_ID, TORQUE_COMMAND, motor_cur.gimbal_cur[2], 0,0);
                    mf_times = 0;
                }
//                send_judge_msg(0x09, &hcan1);
            }
            else if (event.value.signals & SHOOT_MOTOR_MSG_SEND)
            {
                can2_send_message(Fric_CAN_TX_ID,0,0, motor_cur.trigger_cur, 0);
							  if (++fric_times >= 100)
								{
								can1_send_message(CAN_Fric_Ctrl_ID,0,fric.Fric_Pid_Set[0].fric_spd_ref,fric.Fric_Pid_Set[1].fric_spd_ref,0);
								fric_times=0;
								}
            }
        }
    }
}
