#include "bsp_TriggerMotor.h"
#include "shoot_task.h"
#include "bsp_can.h"
#include "pid.h"
#include "gimbal_task.h"
#include "remote_msg.h"
#include "modeswitch_task.h"
#include "control_def.h"
#include "shoot_task.h"
#include "math_calcu.h"
#include "string.h"
#include "bsp_judge.h"
#include "remote_msg.h"
#include "bsp_T_imu.h"
#include "status_task.h"
#include "protocol_camp.h"
#include "vision_predict.h"
extern shoot_t shoot;
extern vision_ctrl_info_t vision_ctrl; // 自动步兵控制
static void TriggerMotor_pidcal(void);
float shoot_spd = 2000;
extern TaskHandle_t can_msg_send_task_t;
uint8_t shoot_enable_flag = 0;
int shoot_number = 0;
uint32_t last_total_ecd;
float Fric_hz;
void TriggerMotor_init(void)
{
    PID_struct_init(&pid_trigger_hz, POSITION_PID, 6800, 0,
                    PID_TRIGGER_ECD_P, PID_TRIGGER_ECD_I, PID_TRIGGER_ECD_D);
    PID_struct_init(&pid_trigger_spd, POSITION_PID, 10000, 5500,
                    PID_TRIGGER_SPD_P, PID_TRIGGER_SPD_I, PID_TRIGGER_SPD_D);
}

static void TriggerMotor_pidcal(void)
{
    //    shoot.barrel.pid.trigger_hz_fdb = Fric_hz;
    //		shoot.barrel.pid.trigger_hz_ref = 1000.0f/shoot.trigger_period ;
    //    pid_calc(&pid_trigger_hz, shoot.barrel.pid.trigger_hz_fdb, shoot.barrel.pid.trigger_hz_ref);
    //    shoot.barrel.pid.trigger_spd_ref = pid_trigger_hz.pos_out; // 位置环

    shoot.barrel.pid.trigger_spd_fdb = motor_trigger.speed_rpm;
    pid_calc(&pid_trigger_spd, shoot.barrel.pid.trigger_spd_fdb, shoot.barrel.pid.trigger_spd_ref);
    shoot.barrel.current = pid_trigger_spd.pos_out; // 速度环

    motor_cur.trigger_cur = shoot.barrel.current;
}

void TriggerMotor_control(void)
{
    /* 局部全局变量 */
    static uint16_t frequency_cnt = 0;           // 射频计算
    static uint8_t shoot_enable = 1;             // 打能量机关单发使能标志
    static uint32_t shoot_time, shoot_last_time; // 计算射击周期

    switch (shoot.stir_mode)
    {
    case STIR_MODE_PROTECT: // 拨盘保护模式，保持惯性，无力
    case STIR_MODE_STOP:    // 拨盘停止模式，保持静止，有力
    {
        shoot.barrel.pid.trigger_spd_ref = 0;
        pid_trigger_spd.iout = 0;
        motor_cur.trigger_cur = 0;
    }
    break;
    case STIR_MODE_SINGLE: // 拨盘单发模式，连续开枪请求，只响应一次
    case STIR_MODE_SERIES: // 拨盘连发模式，连续开枪请求，连续响应
    {
        /* 利用遥控器切换单发连发模式 */
        if (rc_FSM_check(RC_LEFT_LU)) // 遥控器上电前，左拨杆置左上
        {
            shoot.stir_mode = STIR_MODE_SINGLE; // 单发
        }
        else if (rc_FSM_check(RC_LEFT_RU)) // 遥控器上电前，左拨杆置右上
        {
            shoot.stir_mode = STIR_MODE_SERIES; // 连发
        }
        frequency_cnt++;
        shoot.barrel.pid.trigger_ecd_error = shoot.barrel.pid.trigger_ecd_ref - shoot.barrel.pid.trigger_ecd_fdb;
        if (STIR_MODE_SINGLE == shoot.stir_mode) // 拨盘单发模式
        {
            if ((rc.mouse.l == 0 && ctrl_mode == AUTO_MODE) ||
                (rc.ch5 == 0 && ctrl_mode == REMOTER_MODE))
                shoot_enable = 1;
            if (shoot_enable && (rc.mouse.l || rc.ch5 == 660) && ABS(shoot.barrel.pid.trigger_ecd_error) < 0.2f * TRIGGER_MOTOR_ECD)
            {
                shoot_enable = 0;
                shoot.barrel.pid.trigger_ecd_ref += TRIGGER_MOTOR_ECD;
                shoot.barrel.heat += 10;
            }
        }
        else if (shoot.stir_mode == STIR_MODE_SERIES)
        {

            //            if (
            //                (ctrl_mode == REMOTER_MODE ||
            //                 (ctrl_mode == AUTO_MODE && vision.shoot_enable)) &&
            //                frequency_cnt * SHOOT_PERIOD >= shoot.trigger_period                  // 射频控制等够了时间才发一颗
            //                && ABS(shoot.barrel.pid.trigger_ecd_error) < 0.8f * TRIGGER_MOTOR_ECD // 拨盘误差控制
            //                && shoot.barrel.heat_remain >= MIN_HEAT                               // 热量控制
            //                )                                                                     // 一个周期打一颗  射频控制
            //            {
            //                frequency_cnt = 0;
            //                /* 拨一颗子弹 */
            //                shoot.barrel.pid.trigger_ecd_ref += TRIGGER_MOTOR_ECD;
            //                shoot.barrel.heat += 10;
            //                /* 获取射击周期 */
            //                shoot_time = osKernelSysTick();
            //                shoot.barrel.shoot_period = shoot_time - shoot_last_time;
            //                shoot_last_time = shoot_time;
            //            }

            if (
                (ctrl_mode == REMOTER_MODE ||
                 (ctrl_mode == AUTO_MODE && vision.shoot_enable)) &&
                shoot.barrel.heat_remain >= MIN_HEAT // 热量控制
                )                                    // 一个周期打一颗  射频控制
            {
                if (frequency_cnt > 49)
                {
                    frequency_cnt = 0;
                    shoot_time = osKernelSysTick();
                    shoot.barrel.shoot_period = shoot_time - shoot_last_time;

                    shoot.barrel.heat += 10 * (((motor_trigger.total_ecd - last_total_ecd) / TRIGGER_MOTOR_ECD_Circle) * 9.0f);

                    Fric_hz = (((motor_trigger.total_ecd - last_total_ecd) / TRIGGER_MOTOR_ECD_Circle) * 9.0f) / (shoot.barrel.shoot_period / 1000.0f);

                    shoot_last_time = shoot_time;
                    last_total_ecd = motor_trigger.total_ecd;
                }
                shoot.barrel.pid.trigger_spd_ref = TRIGGER_10hz;

                /* 获取射击周期 */
            }

            if ((ctrl_mode == AUTO_MODE && !vision.shoot_enable) || shoot.barrel.heat_remain < MIN_HEAT)
            {
                shoot.barrel.pid.trigger_spd_ref = 0;
                pid_trigger_spd.iout = 0;
                motor_cur.trigger_cur = 0;
            }
        }
    }
    break;
    default:
        break;
    }
    TriggerMotor_pidcal();
}
void Trigger_STOP_or_PROTECT(void)
{
    shoot.barrel.pid.trigger_spd_ref = 0;
    pid_trigger_spd.iout = 0;
    motor_cur.trigger_cur = 0;
}

void Trigger_SINGLE_or_SERIES(void)
{
    /* 局部全局变量 */
    static uint16_t frequency_cnt = 0;           // 射频计算
    static uint8_t shoot_enable = 1;             // 打能量机关单发使能标志
    static uint32_t shoot_time, shoot_last_time; // 计算射击周期

    /* 利用遥控器切换单发连发模式 */
    if (rc_FSM_check(RC_LEFT_LU)) // 遥控器上电前，左拨杆置左上
    {
        shoot.stir_mode = STIR_MODE_SINGLE; // 单发
    }
    else if (rc_FSM_check(RC_LEFT_RU)) // 遥控器上电前，左拨杆置右上
    {
        shoot.stir_mode = STIR_MODE_SERIES; // 连发
    }
    frequency_cnt++;
    
		
    if (STIR_MODE_SINGLE == shoot.stir_mode) // 拨盘单发模式
    {
        if ((rc.mouse.l == 0 && ctrl_mode == AUTO_MODE) ||
            (rc.ch5 == 0 && ctrl_mode == REMOTER_MODE))
            shoot_enable = 1;
        if (shoot_enable && (rc.mouse.l || rc.ch5 == 660))
        {
            shoot_enable = 0;
            shoot.barrel.heat += 10;
        }
    }
		
    else if (shoot.stir_mode == STIR_MODE_SERIES)
    {

        if (
            (ctrl_mode == REMOTER_MODE ||
             (ctrl_mode == AUTO_MODE && vision.shoot_enable)) &&
            shoot.barrel.heat_remain >= MIN_HEAT // 热量控制
            )                                    // 一个周期打一颗  射频控制
        {
            if (frequency_cnt > 49)
            {
                frequency_cnt = 0;
                shoot_time = osKernelSysTick();
                shoot.barrel.shoot_period = shoot_time - shoot_last_time;

                shoot.barrel.heat += 10 * (((motor_trigger.total_ecd - last_total_ecd) / TRIGGER_MOTOR_ECD_Circle) * 9.0f);

                Fric_hz = (((motor_trigger.total_ecd - last_total_ecd) / TRIGGER_MOTOR_ECD_Circle) * 9.0f) / (shoot.barrel.shoot_period / 1000.0f);

                shoot_last_time = shoot_time;
                last_total_ecd = motor_trigger.total_ecd;
            }
            shoot.barrel.pid.trigger_spd_ref = TRIGGER_10hz;

            /* 获取射击周期 */
        }

        if ((ctrl_mode == AUTO_MODE && !vision.shoot_enable) || shoot.barrel.heat_remain < MIN_HEAT)
        {
            shoot.barrel.pid.trigger_spd_ref = 0;
            pid_trigger_spd.iout = 0;
            motor_cur.trigger_cur = 0;
        }
    }
}
