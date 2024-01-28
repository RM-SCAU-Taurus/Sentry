/* HAL库 ---------------------------------------------------------------------------*/
/* USER CODE BEGIN */
#include "gimbal_to_chassis_task.h"
#include "cmsis_os.h"
#include "comm_task.h"
/* USER CODE END */

/* C库 ---------------------------------------------------------------------------*/
/* USER CODE BEGIN */

/* USER CODE END */

/* 硬件外设库 --------------------------------------------------------------------*/
/* USER CODE BEGIN */

/* USER CODE END */

/* 任务库 ------------------------------------------------------------------------*/
/* USER CODE BEGIN */

/* USER CODE END */

/* 数学库 ------------------------------------------------------------------------*/
/* USER CODE BEGIN */

/* USER CODE END */

/* 数据处理库 --------------------------------------------------------------------*/
/* USER CODE BEGIN */
#include "protocol_camp.h"
#include "msg_center.h"
#include "comm_type.h"
/* USER CODE END */

/* 类型定义库 --------------------------------------------------------------------*/
/* USER CODE BEGIN */

/* USER CODE END */

/* 板级支持库 --------------------------------------------------------------------*/
/* USER CODE BEGIN */
#include "bsp_can.h"
#include "bsp_judge.h"
/* USER CODE END */

/* 外部变量声明 ------------------------------------------------------------------*/
/* USER CODE BEGIN */

/* USER CODE END */

/* 外部函数声明 ------------------------------------------------------------------*/
/* USER CODE BEGIN */

/* USER CODE END */

/* 静态函数声明 ------------------------------------------------------------------*/
/* USER CODE BEGIN */
static void gimbal_to_chassic_init(void);
/* USER CODE END */

/* 宏定义声明 --------------------------------------------------------------------*/
/* USER CODE BEGIN */

/* USER CODE END */

/* 结构体定义 --------------------------------------------------------------------*/
/* USER CODE BEGIN */

/* USER CODE END */

/* 变量声明 ----------------------------------------------------------------------*/
/* USER CODE BEGIN */
static Gimbal_to_Chassis_t Gimbal_to_Chassis;

static Subscriber_t *chassis_ctrl_send_sub; // 用于订阅底盘的发送命令

static Gim_to_Cha_send_t Gim_to_Cha_send;
/* USER CODE END */

/* 测试变量声明 ------------------------------------------------------------------*/
/* USER CODE BEGIN */

/* USER CODE END */


void gimbal_to_chassic_task(void const *argu) // 发给底盘
{
	gimbal_to_chassic_init();
	uint32_t wake_up_time = osKernelSysTick();
	for (;;)
	{
		SubGetMessage(chassis_ctrl_send_sub, &Gimbal_to_Chassis);
		game_data_handler(&robot_judge_msg); // 裁判系统赋值
		if (Gimbal_to_Chassis.ctrl_mode_sys == PROTECT_MODE)
		{
			Gim_to_Cha_send.Send_ctrl_callback(CHASSIS_CTRL_CAN_TX_ID, 0, 0, 0, 0, 0); // CAN1发送
			Gim_to_Cha_send.Send_judge_callback(0x09, &hcan1);
		}
		if (Gimbal_to_Chassis.ctrl_mode_sys == AUTO_MODE)
		{
			Gim_to_Cha_send.Send_ctrl_callback(CHASSIS_CTRL_CAN_TX_ID, Gimbal_to_Chassis.spd_input.vx, Gimbal_to_Chassis.spd_input.vy, Gimbal_to_Chassis.spd_input.vw, 0x02,Gimbal_to_Chassis.super_cup);
			Gim_to_Cha_send.Send_judge_callback(0x09, &hcan1);
		}
		else if (Gimbal_to_Chassis.ctrl_mode_sys == REMOTER_MODE)
		{
			Gim_to_Cha_send.Send_ctrl_callback(CHASSIS_CTRL_CAN_TX_ID, Gimbal_to_Chassis.spd_input.vx, Gimbal_to_Chassis.spd_input.vy, Gimbal_to_Chassis.spd_input.vw, 0x01, Gimbal_to_Chassis.spd_input.spin_dir);
			Gim_to_Cha_send.Send_judge_callback(0x09, &hcan1);
		}
		osDelayUntil(&wake_up_time, GIMBAL_TO_CHASSIS_TASK_PERIOD);
	}
}

static void gimbal_to_chassic_init(void)
{
	Gim_to_Cha_send.Send_ctrl_callback = can1_send_chassis_message;
	Gim_to_Cha_send.Send_judge_callback = send_judge_msg;
	chassis_ctrl_send_sub = SubRegister("Chassis_spd_send", sizeof(Gimbal_to_Chassis_t));
}
