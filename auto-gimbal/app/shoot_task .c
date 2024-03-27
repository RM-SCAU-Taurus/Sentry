/**********C��****************/
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
/**********Ӳ�������******/
#include "tim.h"
#include "usart.h"
/**********�����*************/

#include "comm_task.h"
#include "shoot_task.h"
#include "freertos.h"
/**********��ѧ��*************/

/**********���ݴ����**********/
#include "remote_msg.h"
/**********���Ͷ����**********/
#include "control_def.h"
#include "protocol_camp.h"
/**********�弶֧�ֿ�**********/
#include "bsp_CoverServo.h"
#include "bsp_FricMotor.h"
#include "bsp_TriggerMotor.h"
#include "bsp_judge.h"
#include "bsp_shoot_class.h"
/**********�ⲿ��������********/
extern vision_ctrl_info_t vision_ctrl;
/**********�ⲿ��������********/

/**********��̬��������********/
static void ShootParam_Update(void);
static shoot_class_parent_t *shoot_mode_check(void);
static void Shoot_power_check(void);
static void Shoot_hz_ctrl(void);
/**********�궨������**********/
#define __SHOOT_TASK_GLOBALS
/**********�ṹ�嶨��**********/
fric_t fric;
shoot_t shoot;
extern osThreadId can_msg_send_task_t;
/**********��������*************/

/**********���Ա�������********/
int flag = 0;
int cnt = 0;

void shoot_task(void const *argu)
{
    uint32_t mode_wake_time = osKernelSysTick();
    static shoot_class_parent_t *Action_ptr = NULL;
	    for (;;)
    {
        taskENTER_CRITICAL();
        /* �����ʼ�� */
        Shoot_power_check();
        /* ������ģʽ�л� ,����ָ�븳ֵ*/
        Action_ptr = shoot_mode_check();
        /*����ָ��������ຯ��*/
        Action_ptr->Fric_action();
        Action_ptr->Trigger_action();
			
        osSignalSet(can_msg_send_task_t, SHOOT_MOTOR_MSG_SEND);
        taskEXIT_CRITICAL();
        osDelayUntil(&mode_wake_time, SHOOT_PERIOD);
    }
}

void shoot_init(void)
{
    /* �������ײ��ʼ�� */
    FricMotor_init();    // Ħ���ֳ�ʼ��
    TriggerMotor_init(); // ���̳�ʼ��

    shoot.firc_mode = FIRC_MODE_STOP;
    shoot.fric_protect_mode = FRIC_PROTECT;
    shoot.stir_mode = STIR_MODE_PROTECT;
    shoot.house_mode = HOUSE_MODE_PROTECT;
    /* ǹ�ܲ�����ʼ�� */
    shoot.trigger_period = TRIGGER_PERIOD;
    shoot.barrel.cooling_rate = 80;
    shoot.barrel.heat_max = 400;
    shoot.shoot_speed = 30;
}

static shoot_class_parent_t *shoot_mode_check(void)
{
    if (!fric.init_flag)
    {
        fric.protect_flag = FRIC_PROTECT;
    }
    /* ���²���ϵͳ���� */
    ShootParam_Update();
    /* �õ���������Ƶ */
		Shoot_hz_ctrl(); 
		
    static shoot_class_parent_t *p_return = NULL;

    switch (ctrl_mode)

    {

    case PROTECT_MODE:

    {

        p_return = (shoot_class_parent_t *)&PROTECT_choice;
    }

    break;

    case REMOTER_MODE:
    case AUTO_MODE:

    {

        /* Ħ���ֺͲ���ģʽ�л� */
        switch (rc.sw2)
        {
        case RC_UP:
        {
            p_return = (shoot_class_parent_t *)&RC_UP_choice;
        }
        break;
        case RC_MI:
        {
            p_return = (shoot_class_parent_t *)&RC_MI_choice;
        }
        break;
        case RC_DN:
        {
            p_return = (shoot_class_parent_t *)&RC_DN_choice;
        }
        break;
        default:
            break;
        }
    }
    break;
    default:
        break;
    }

    return p_return;
}

static void Shoot_power_check(void)
{
    static uint8_t last_fric_enable, fric_enable;
    fric_enable = !!Game_Robot_Status.mains_power_shooter_output;
    if (fric_enable && !last_fric_enable)
    {
        shoot.fric_protect_mode = FRIC_PROTECT;
    }
    last_fric_enable = fric_enable;
}
/* ����������ϵͳ���ݸ��� */
static void ShootParam_Update(void)
{

		
		    /* ���²���ϵͳ���� */
    if (Game_Robot_Status.shooter_barrel_cooling_value != 0)
    {
//        shoot.shoot_speed = Game_Robot_Status.shooter_id1_17mm_speed_limit;          //
        shoot.barrel.heat_max = Game_Robot_Status.shooter_barrel_heat_limit;    //
        shoot.barrel.cooling_rate = Game_Robot_Status.shooter_barrel_cooling_value; //
    }
    /* ���� ģ�����ϵͳ ���� */
    shoot.barrel.heat -= shoot.barrel.cooling_rate * SHOOT_PERIOD * 0.001f; //
    if (shoot.barrel.heat < 0)
        shoot.barrel.heat = 0;
    shoot.barrel.heat_remain = shoot.barrel.heat_max - shoot.barrel.heat; //
    //   		shoot.barrel.heat_remain = shoot.barrel.heat_max ;  //��������������
		
}

static void Shoot_hz_ctrl(void){

     
        if (vision_ctrl.shoot_cmd == 20) // ���25hz
        {
            shoot.trigger_hz = TRIGGER_20hz;
        }
        else if(vision_ctrl.shoot_cmd == 10)
        {
            shoot.trigger_hz = TRIGGER_10hz; // �Ӿ�ֱ�ӷ�hz
        }
				else if(vision_ctrl.shoot_cmd == 0 )
				{	
						shoot.trigger_hz = TRIGGER_20hz; // �Ӿ�ֱ�ӷ�hz
				}
				else
					shoot.trigger_hz = TRIGGER_20hz;

}
