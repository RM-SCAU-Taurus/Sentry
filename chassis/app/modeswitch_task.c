#include "modeswitch_task.h"
#include "gimbal_task.h"
//#include "shoot_task.h"
#include "bsp_can.h"
//#include "supercap_task.h"
#include "control_def.h"
//#include "bsp_TriggerMotor.h"
//#include "visionfire_task.h"
#include "remote_msg.h"
#include "cmsis_os.h"
//#include "bsp_vision.h"
//#include "bsp_judge.h"
//#include "bsp_FricMotor.h"
//#include "status_task.h"
#include "modeswitch_task.h"
ctrl_mode_e ctrl_mode;
/* 键盘按键状态标志位 */
int kb_status[11]= {0};
int last_kb_status[11]= {0};
uint8_t lock_flag=1;
uint8_t last_mains_power_shooter_output;
double fire_time;
void mode_switch_task(void const *argu)
{
    for(;;)
    {
        if(!lock_flag)
        {
            unlock_init();			//解锁操作
        }

        sw1_mode_handler();
        sw2_mode_handler();
        osDelay(5);
    }
}

void sw1_mode_handler(void)			//由拨杆1决定的模式切换，主要是云台和底盘
{
    switch (rc.sw1)
    {
    case RC_UP:
    {
        ctrl_mode = REMOTER_MODE;

        break;
    }
    case RC_MI:
    {
        ctrl_mode = PROTECT_MODE;
        break;
    }
    case RC_DN:
    {
        if(ctrl_mode != ENERGY_MODE || ctrl_mode != VISION_MODE)
            ctrl_mode = KEYBOARD_MODE;				//非大能量和视觉模式下重置为键盘模式
        break;
    }
    default:
        break;
    }
}

void sw2_mode_handler(void)		//由拨杆2决定的模式切换，主要是发射器
{
    if(ctrl_mode == REMOTER_MODE)
    {
        switch (rc.sw2)
        {
        case RC_UP:
        {
            break;
        }
        case RC_MI:
        {
            break;
        }
        case RC_DN:
        {

        }
        }
    }
}

void unlock_init(void)			//解锁函数
{
    if(rc.sw1 == RC_MI && rc.sw2 == RC_UP)
    {
        if(rc.ch4==-660)
        {
            if(rc.ch3==660)
            {
                lock_flag = 1;
            }
        }
    }
}
