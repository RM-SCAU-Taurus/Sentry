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
uint16_t tritestaa = 2;
uint32_t last_total_ecd;
float Fric_hz;


 uint16_t frequency_cnt = 0;           // 射频计算
		
#define AGL	
void TriggerMotor_init(void)
{
    PID_struct_init(&pid_trigger_ecd, POSITION_PID, 6800, 0,
                    PID_TRIGGER_ECD_P, PID_TRIGGER_ECD_I, PID_TRIGGER_ECD_D);
    PID_struct_init(&pid_trigger_spd, POSITION_PID, 10000, 5500,
                    PID_TRIGGER_SPD_P, PID_TRIGGER_SPD_I, PID_TRIGGER_SPD_D);
}

static void TriggerMotor_pidcal(void)
{
		#ifdef AGL
    shoot.barrel.pid.trigger_ecd_fdb = motor_trigger.total_ecd;
    pid_calc(&pid_trigger_ecd, shoot.barrel.pid.trigger_ecd_fdb, shoot.barrel.pid.trigger_ecd_ref);
    shoot.barrel.pid.trigger_spd_ref = pid_trigger_ecd.pos_out;  //位置环
		#endif
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
							
						#ifdef AGL
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

						#else
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

						#endif
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

#ifdef AGL

void Trigger_STOP_or_PROTECT(void)
{
         frequency_cnt=0;
            shoot.barrel.shoot_period = 0;
					
            pid_trigger_ecd.set[0] = motor_trigger.total_ecd;
						shoot.barrel.pid.trigger_ecd_ref=motor_trigger.total_ecd;
            pid_trigger_spd.iout  = 0;
            motor_cur.trigger_cur = 0;
	
	TriggerMotor_pidcal();
	
}

void Trigger_SINGLE_or_SERIES(void)
{
    
    static uint8_t shoot_enable = 1;             // 打能量机关单发使能标志
    static uint32_t shoot_time, shoot_last_time; // 计算射击周期

    frequency_cnt++;
		
 /* 利用遥控器切换单发连发模式 */
            if( rc_FSM_check(RC_LEFT_LU) )  //遥控器上电前，左拨杆置左上
            {
                shoot.stir_mode = STIR_MODE_SINGLE;  //单发
            }
            else if( rc_FSM_check(RC_LEFT_RU) )  //遥控器上电前，左拨杆置右上
            {
                shoot.stir_mode = STIR_MODE_SERIES;  //连发
            }
            frequency_cnt++;
            shoot.barrel.pid.trigger_ecd_error = shoot.barrel.pid.trigger_ecd_ref - shoot.barrel.pid.trigger_ecd_fdb;
            if(0)  //拨盘单发模式
            {
                    if( ( rc.mouse.l == 0 && ctrl_mode == AUTO_MODE )|| \
                    ( rc.ch5 == 0 && ctrl_mode == REMOTER_MODE ) )
                    shoot_enable = 1;
                if( shoot_enable \
                    && (rc.mouse.l || rc.ch5 == 660) \
                    && ABS(shoot.barrel.pid.trigger_ecd_error) < 0.2f * TRIGGER_MOTOR_ECD \
                    )  
                {
                    shoot_enable = 0;
                    shoot.barrel.pid.trigger_ecd_ref += TRIGGER_MOTOR_ECD;
                    shoot.barrel.heat += 10;
                }
            }
            else if(1)
            {

                     if(  
											 (ctrl_mode == REMOTER_MODE|| 
										 (ctrl_mode == AUTO_MODE&&vision.shoot_enable)) && 
                    frequency_cnt * SHOOT_PERIOD >= shoot.trigger_period //射频控制
                    && ABS(shoot.barrel.pid.trigger_ecd_error) < 0.8f * TRIGGER_MOTOR_ECD //拨盘误差控制
                   &&shoot.barrel.heat_remain>=MIN_HEAT  //热量控制
										 )//一个周期打一颗  射频控制  
                {
                        frequency_cnt = 0;
                        /* 拨一颗子弹 */
                        shoot.barrel.pid.trigger_ecd_ref += TRIGGER_MOTOR_ECD;
                        shoot.barrel.heat += 10;
                        /* 获取射击周期 */
                        shoot_time = osKernelSysTick();
                        shoot.barrel.shoot_period = shoot_time - shoot_last_time;
                        shoot_last_time = shoot_time;
                    }
								if((ctrl_mode == AUTO_MODE&&!vision.shoot_enable)||shoot.barrel.heat_remain<MIN_HEAT )
								{
								shoot.barrel.pid.trigger_ecd_ref=motor_trigger.total_ecd;
								pid_trigger_spd.iout  = 0;
								motor_cur.trigger_cur = 0;
								}
            
						}
						
						TriggerMotor_pidcal();
						
}


#else
void Trigger_STOP_or_PROTECT(void)
{
    shoot.barrel.pid.trigger_spd_ref = 0;
    pid_trigger_spd.iout = 0;
    motor_cur.trigger_cur = 0;
    TriggerMotor_pidcal();
}

void Trigger_SINGLE_or_SERIES(void)
{
    /* 局部全局变量 */
    static uint16_t frequency_cnt = 0;           // 射频计算
    static uint8_t shoot_enable = 1;             // 打能量机关单发使能标志
    static uint32_t shoot_time, shoot_last_time; // 计算射击周期

    frequency_cnt++;

    /* 利用遥控器切换单发连发模式 */
    if (rc_FSM_check(RC_LEFT_LU)) // 遥控器上电前，左拨杆置左上
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
    else if (1) // 遥控器上电前，左拨杆置右上
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
							
									if(rc.sw2 == RC_DN)
									{	static uint8_t back;
									static uint16_t cnt;
									if( (motor_trigger.speed_rpm >0 && motor_trigger.speed_rpm <50) || back == 1 )	
									{
										cnt++;
										if(back == 0)
										{	shoot.barrel.pid.trigger_spd_ref = -1000;
										back=1;
										}
										if( (cnt>100 && back == 1) )
										back=0;
									}
									else
									shoot.barrel.pid.trigger_spd_ref = shoot.trigger_hz;
//									shoot.barrel.pid.trigger_spd_ref = shoot.trigger_hz;
									
									
								}

            /* 获取射击周期 */
        }

        if ((ctrl_mode == AUTO_MODE && !vision.shoot_enable) || shoot.barrel.heat_remain < MIN_HEAT)
        {
            shoot.barrel.pid.trigger_spd_ref = 0;
            pid_trigger_spd.iout = 0;
            motor_cur.trigger_cur = 0;
        }
    }

    TriggerMotor_pidcal();
}


#endif
