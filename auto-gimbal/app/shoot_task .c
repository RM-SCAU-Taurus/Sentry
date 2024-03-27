/**********C库****************/
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
/**********硬件外设库******/
#include "tim.h"
#include "usart.h"
/**********任务库*************/

#include "comm_task.h"
#include "shoot_task.h"
#include "freertos.h"
/**********数学库*************/

/**********数据处理库**********/
#include "remote_msg.h"
/**********类型定义库**********/
#include "control_def.h"
#include "protocol_camp.h"
/**********板级支持库**********/
#include "bsp_CoverServo.h"
#include "bsp_FricMotor.h"
#include "bsp_TriggerMotor.h"
#include "bsp_judge.h"
#include "bsp_shoot_class.h"
/**********外部变量声明********/
extern vision_ctrl_info_t vision_ctrl;
/**********外部函数声明********/

/**********静态函数声明********/
static void ShootParam_Update(void);
static shoot_class_parent_t *shoot_mode_check(void);
static void Shoot_power_check(void);
static void Shoot_hz_ctrl(void);
/**********宏定义声明**********/
#define __SHOOT_TASK_GLOBALS
/**********结构体定义**********/
fric_t fric;
shoot_t shoot;
extern osThreadId can_msg_send_task_t;
/**********变量声明*************/

/**********测试变量声明********/
int flag = 0;
int cnt = 0;

void shoot_task(void const *argu)
{
    uint32_t mode_wake_time = osKernelSysTick();
    static shoot_class_parent_t *Action_ptr = NULL;
	    for (;;)
    {
        taskENTER_CRITICAL();
        /* 电调初始化 */
        Shoot_power_check();
        /* 发射器模式切换 ,父类指针赋值*/
        Action_ptr = shoot_mode_check();
        /*父类指针调用子类函数*/
        Action_ptr->Fric_action();
        Action_ptr->Trigger_action();
			
        osSignalSet(can_msg_send_task_t, SHOOT_MOTOR_MSG_SEND);
        taskEXIT_CRITICAL();
        osDelayUntil(&mode_wake_time, SHOOT_PERIOD);
    }
}

void shoot_init(void)
{
    /* 发射器底层初始化 */
    FricMotor_init();    // 摩擦轮初始化
    TriggerMotor_init(); // 拨盘初始化

    shoot.firc_mode = FIRC_MODE_STOP;
    shoot.fric_protect_mode = FRIC_PROTECT;
    shoot.stir_mode = STIR_MODE_PROTECT;
    shoot.house_mode = HOUSE_MODE_PROTECT;
    /* 枪管参数初始化 */
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
    /* 更新裁判系统参数 */
    ShootParam_Update();
    /* 让导航控制射频 */
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

        /* 摩擦轮和拨盘模式切换 */
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
/* 发射器裁判系统数据更新 */
static void ShootParam_Update(void)
{

		
		    /* 更新裁判系统数据 */
    if (Game_Robot_Status.shooter_barrel_cooling_value != 0)
    {
//        shoot.shoot_speed = Game_Robot_Status.shooter_id1_17mm_speed_limit;          //
        shoot.barrel.heat_max = Game_Robot_Status.shooter_barrel_heat_limit;    //
        shoot.barrel.cooling_rate = Game_Robot_Status.shooter_barrel_cooling_value; //
    }
    /* 更新 模拟裁判系统 数据 */
    shoot.barrel.heat -= shoot.barrel.cooling_rate * SHOOT_PERIOD * 0.001f; //
    if (shoot.barrel.heat < 0)
        shoot.barrel.heat = 0;
    shoot.barrel.heat_remain = shoot.barrel.heat_max - shoot.barrel.heat; //
    //   		shoot.barrel.heat_remain = shoot.barrel.heat_max ;  //无限热量测试用
		
}

static void Shoot_hz_ctrl(void){

     
        if (vision_ctrl.shoot_cmd == 20) // 最大25hz
        {
            shoot.trigger_hz = TRIGGER_20hz;
        }
        else if(vision_ctrl.shoot_cmd == 10)
        {
            shoot.trigger_hz = TRIGGER_10hz; // 视觉直接发hz
        }
				else if(vision_ctrl.shoot_cmd == 0 )
				{	
						shoot.trigger_hz = TRIGGER_20hz; // 视觉直接发hz
				}
				else
					shoot.trigger_hz = TRIGGER_20hz;

}
