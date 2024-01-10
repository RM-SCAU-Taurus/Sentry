#include "bsp_FricMotor.h"
#include "tim.h"
#include "pid.h"
#include "math_calcu.h"
#include "remote_msg.h"
#include "shoot_task.h"
#include "modeswitch_task.h"
#include "control_def.h"
#include "status_task.h"

static void FricGunControl(uint16_t pwm, int16_t err_1, int16_t err_2);

Slope_Struct shoot_Fric_pwm;

void FricMotor_init(void)
{
    //900-2000
    //启动时，油门打到最低
    HAL_TIM_PWM_Start(FricMotor_Time_CH1);
    HAL_TIM_PWM_Start(FricMotor_Time_CH2);

    FricMotor_PWM1 = Init_PWM;
    FricMotor_PWM2 = Init_PWM;
    /* 初始化摩擦轮斜坡函数 */
    shoot_Fric_pwm.limit_target = Init_PWM;
    shoot_Fric_pwm.real_target  = Init_PWM;
    shoot_Fric_pwm.change_scale = 0.5;

    /* 初始化射速 */
    shoot.shoot_speed = 15;
}

/**
  * @func   		void FricGunControl(uint8_t Control)
  * @bref			摩擦轮起停
  * @param[in]      Control：0为停止，1为启动
  * @retval         void
  * @note			900以上起转
  */
static void FricGunControl(uint16_t pwm, int16_t err_1, int16_t err_2)
{
    shoot_Fric_pwm.limit_target = Init_PWM+pwm;

    if( lock_flag )			//解锁后才能够输出PWM
    {
        Slope_On(&shoot_Fric_pwm); //上摩擦轮斜坡启动

        FricMotor_PWM1 = shoot_Fric_pwm.real_target + err_1;
        FricMotor_PWM2 = shoot_Fric_pwm.real_target + err_2;
    }
}

int16_t pwm_1 = 0, pwm_2 = 0;
uint16_t test_pwm = 590;
//左：

/**
  * @name     FricMotor_Control
  * @brief    摩擦轮控制
  * @param    None
  * @retval   None
  * @note     无论单双枪管，一经开火模式，摩擦轮都打开；拨盘会根据枪管模式不同而有区别
  */
void FricMotor_Control(void)
{
    uint16_t speed_pwm;
    switch( shoot.firc_mode )
    {
        case FIRC_MODE_STOP:
        {
            FricGunControl(0, 0, 0);		//摩擦轮停转
            break;
        }
        case FIRC_MODE_RUN:
        {
            if( shoot.shoot_speed == 15 )
            {
                speed_pwm = LOW_SPEED;
            }
            else if( shoot.shoot_speed == 18 )
            {
                speed_pwm = MID_SPEED;
            }
            else if( shoot.shoot_speed >= 30 )
            {
                speed_pwm = HIGH_SPEED;
            }
            else  //退弹模式
            {
                speed_pwm = LOW_SPEED;
            }
            break;
        }
        default:
        {
            break;
        }
    }
    if( rc_FSM_check(RC_RIGHT_LU) )
    {
        speed_pwm = HIGH_SPEED;
    }
    else 
    {
        speed_pwm = LOW_SPEED;
    }
    FricGunControl(speed_pwm, pwm_1, pwm_2);
}
