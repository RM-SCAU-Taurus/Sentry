/**
  * @file     bsp_CoverServo.c
  * @version  v2.0
  * @date     July,8th 2019
  *
  * @brief    弹舱舵机基础配置包含1.舵机初始化
	*								                2.舵机初始位置校准
	*								                3.舵机控制
  *
  *	@author   Fatmouse,part of the code reference Link's code
  *
  */
#include "bsp_CoverServo.h"
#include "math_calcu.h"
#include "tim.h"
#include "shoot_task.h"
#include "remote_msg.h"
#include "control_def.h"
#include "modeswitch_task.h"
uint8_t house_switch_flag ;
Slope_Struct Cover_Servo;

void CoverServo_init(void)
{
    HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);
    CoverServo_ctrl(COVER_START);
    house_switch_flag = 1;
}

//开关弹仓盖
void CoverServo_switch(void)
{
    switch(ctrl_mode)
    {
    case REMOTER_MODE:
    {
        if((rc.sw2 == RC_MI) && house_switch_flag)
        {
            house_switch_flag  = 0;
            if(shoot_upper.house_switch)
            {
                shoot_upper.house_switch = COVER_OFF;
            }
            else
            {
                shoot_upper.house_switch = COVER_ON;
            }
        }
        else if(!(rc.sw2 == RC_MI))
        {
            house_switch_flag = 1;
        }
    }
    break;
    case KEYBOARD_MODE:
    {
        if(rc.kb.bit.R && house_switch_flag)
        {
            house_switch_flag  = 0;
            if(shoot_upper.house_switch)
            {
                shoot_upper.house_switch = COVER_OFF;
            }
            else
            {
                shoot_upper.house_switch = COVER_ON;
            }
        }
        else if(!(rc.kb.bit.R))
        {
            house_switch_flag = 1;
        }
    }
    break;
    default:
    {
    }
    }
    if(shoot_upper.house_switch)
    {
        CoverServo_ctrl(COVER_START);
    }
    else
    {
        CoverServo_ctrl(COVER_END);
    }
}

void CoverServo_ctrl(uint16_t pwm)
{
    TIM2->CCR2=pwm;
}
