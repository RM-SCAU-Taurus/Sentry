/* HAL�� ---------------------------------------------------------------------------*/
/* USER CODE BEGIN */
#include "gimbal_to_chassis_task.h"
#include "cmsis_os.h"
#include "comm_task.h"
/* USER CODE END */

/* C�� ---------------------------------------------------------------------------*/
/* USER CODE BEGIN */

/* USER CODE END */

/* Ӳ������� --------------------------------------------------------------------*/
/* USER CODE BEGIN */

/* USER CODE END */

/* ����� ------------------------------------------------------------------------*/
/* USER CODE BEGIN */

/* USER CODE END */

/* ��ѧ�� ------------------------------------------------------------------------*/
/* USER CODE BEGIN */

/* USER CODE END */

/* ���ݴ���� --------------------------------------------------------------------*/
/* USER CODE BEGIN */
#include "protocol_camp.h"
#include "msg_center.h"
#include "comm_type.h"
/* USER CODE END */

/* ���Ͷ���� --------------------------------------------------------------------*/
/* USER CODE BEGIN */

/* USER CODE END */

/* �弶֧�ֿ� --------------------------------------------------------------------*/
/* USER CODE BEGIN */
#include "bsp_can.h"
#include "bsp_judge.h"
/* USER CODE END */

/* �ⲿ�������� ------------------------------------------------------------------*/
/* USER CODE BEGIN */

/* USER CODE END */

/* �ⲿ�������� ------------------------------------------------------------------*/
/* USER CODE BEGIN */

/* USER CODE END */

/* ��̬�������� ------------------------------------------------------------------*/
/* USER CODE BEGIN */
static void gimbal_to_chassic_init(void);
/* USER CODE END */

/* �궨������ --------------------------------------------------------------------*/
/* USER CODE BEGIN */

/* USER CODE END */

/* �ṹ�嶨�� --------------------------------------------------------------------*/
/* USER CODE BEGIN */

/* USER CODE END */

/* �������� ----------------------------------------------------------------------*/
/* USER CODE BEGIN */
static Gimbal_to_Chassis_t Gimbal_to_Chassis;

static Subscriber_t *chassis_ctrl_send_sub; // ���ڶ��ĵ��̵ķ�������

static Gim_to_Cha_send_t Gim_to_Cha_send;
/* USER CODE END */

/* ���Ա������� ------------------------------------------------------------------*/
/* USER CODE BEGIN */

/* USER CODE END */


void gimbal_to_chassic_task(void const *argu) // ��������
{
	gimbal_to_chassic_init();
	uint32_t wake_up_time = osKernelSysTick();
	for (;;)
	{
		SubGetMessage(chassis_ctrl_send_sub, &Gimbal_to_Chassis);
		game_data_handler(&robot_judge_msg); // ����ϵͳ��ֵ
		if (Gimbal_to_Chassis.ctrl_mode_sys == PROTECT_MODE)
		{
			Gim_to_Cha_send.Send_ctrl_callback(CHASSIS_CTRL_CAN_TX_ID, 0, 0, 0, 0, 0); // CAN1����
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
