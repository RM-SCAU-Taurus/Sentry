#include "test_task.h"
#include "cmsis_os.h"
#include "func_generator.h"
#include "chassis_task.h"
#include "comm_task.h"
#include "modeswitch_task.h"
#include "protocol_camp.h"
#include "bsp_can.h"
#include "bsp_judge.h"
#define TEST_PERIOD 5
extern chassis_ctrl_info_t chassis_ctrl;
//FGT_sin_t test_sin_func = 
//{
//    .Td = TEST_PERIOD,
//    .time = 0,
//    .max = 0 + 0,
//    .min = 0 - 0,
//    .dc = 0,
//    .T = 800,
//    .A = 250,
//    .phi = 0,
//    .out = 0
//};

void gimbal_to_chassic_task(void const *argu)//发给底盘
{
    uint32_t wake_up_time = osKernelSysTick();
    for(;;)
    {
			game_data_handler(&robot_judge_msg);//裁判系统赋值
			if(ctrl_mode==PROTECT_MODE)
			{
			chassis.msg_send(CHASSIS_CTRL_CAN_TX_ID,0,0,0,0,0);//CAN1发送
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
			  osDelay(10);
    }
}
