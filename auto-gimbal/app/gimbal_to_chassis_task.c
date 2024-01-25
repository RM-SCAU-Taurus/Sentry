#include "gimbal_to_chassis_task.h"
#include "cmsis_os.h"
#include "func_generator.h"
#include "chassis_task.h"
#include "comm_task.h"
#include "modeswitch_task.h"
#include "protocol_camp.h"
#include "bsp_can.h"
#include "bsp_judge.h"
#define GIMBAL_TO_CHASSIS_TASK_PERIOD 10

extern chassis_ctrl_info_t chassis_ctrl;

void gimbal_to_chassic_task(void const *argu)//��������
{
    uint32_t wake_up_time = osKernelSysTick();
    for(;;)
    {
			game_data_handler(&robot_judge_msg);//����ϵͳ��ֵ
			if(ctrl_mode==PROTECT_MODE)
			{
			chassis.msg_send(CHASSIS_CTRL_CAN_TX_ID,0,0,0,0,0);//CAN1����
			send_judge_msg(0x09,&hcan1);
			}
			if(ctrl_mode==AUTO_MODE)
			{
				chassis.msg_send(CHASSIS_CTRL_CAN_TX_ID,chassis.spd_input.vx,chassis.spd_input.vy,chassis.spd_input.vw,0x02,chassis_ctrl.super_cup);
				send_judge_msg(0x09,&hcan1);
			}            
    else if(ctrl_mode==REMOTER_MODE)
		{
			chassis.msg_send(CHASSIS_CTRL_CAN_TX_ID,chassis.spd_input.vx,chassis.spd_input.vy,chassis.spd_input.vw,0x01,chassis.spin_dir);
			send_judge_msg(0x09,&hcan1);
		}			
			 osDelayUntil(&wake_up_time, GIMBAL_TO_CHASSIS_TASK_PERIOD);
    }
}
