#include "bsp_FricMotor.h"
#include "tim.h"
#include "pid.h"
#include "math_calcu.h"
#include "remote_msg.h"
#include "shoot_task.h"
#include "modeswitch_task.h"
#include "control_def.h"
#include "status_task.h"
#include "bsp_judge.h"
static void FricGunControl(uint16_t pwm, int16_t err_1, int16_t err_2);

Slope_Struct shoot_Fric_pwm;
FricMotor_Pid_Control Fric_pid;


int16_t pwm_1 = 0, pwm_2 = 0;
uint16_t test_pwm = 590;
//左：

///**
//  * @name     FricMotor_Control
//  * @brief    摩擦轮控制
//  * @param    None
//  * @retval   None
//  * @note     无论单双枪管，一经开火模式，摩擦轮都打开；拨盘会根据枪管模式不同而有区别
//  */
// uint16_t speed_pwm=0;
void FricMotor_Control(void)
{
	
}

void FricGunControl_PID(uint16_t pwm)
{
// if(fric.init_flag)			//解锁后电调校准完才能够输出PWM
//    {
//			if(fric.acc_flag==0)
//			{
//					shoot_Fric_pwm.limit_target = Init_PWM+pwm;
//					Slope_On(&shoot_Fric_pwm); //上摩擦轮斜坡启动
//					FricMotor_PWM1 = shoot_Fric_pwm.real_target ;
//					FricMotor_PWM2 = shoot_Fric_pwm.real_target ;
//					
//					if (Init_PWM + pwm- shoot_Fric_pwm.real_target < 0.6f) {
//						fric.acc_flag = 1;
//					} else {
//					fric.acc_flag = 0;
//								}
//				}
//		}
//			if(fric.acc_flag ==1&&Game_Robot_Status.shooter_id1_17mm_speed_limit!=0)//加速完成后且裁判系统有数据开启射速pid控制
//				{
//					Fric_pid.bullet_speed=Shoot_Data.bullet_speed;
//					if(Fric_pid.bullet_speed!=Fric_pid.last_bullet_speed)//通过两次速度不同来判断发射
//					{
//						Fric_pid.x[Fric_pid.shoot_num]=Fric_pid.shoot_num+1;
//						Fric_pid.Shoot_speed[Fric_pid.shoot_num]=Fric_pid.bullet_speed;
//						
//						
//						Fric_pid.last_bullet_speed=Fric_pid.bullet_speed;
//						Fric_pid.shoot_num++;
//						if(Fric_pid.shoot_num==30||Fric_pid.adapting_flag==1)//当射击数量到达30后，恒定30
//						{
//							Fric_pid.adapting_num=30;
//							Fric_pid.adapting_flag=1;
//						}
//						else
//						{
//							Fric_pid.adapting_num=Fric_pid.shoot_num;
//						}
//						Fric_pid.shoot_num=Fric_pid.shoot_num%30;//大小为30的回环数组
//					}
//						Fric_pid.sum_x2 =0;//拟合变量置0
//						Fric_pid.sum_y =0;
//						Fric_pid.sum_x =0;
//						Fric_pid.sum_xy=0;
//					if(Fric_pid.adapting_num>2)//发弹两次后开始拟合
//					{
//					  for (int i = 0; i < Fric_pid.adapting_num; ++i) {
//						Fric_pid.sum_x2 += Fric_pid.x[i] * Fric_pid.x[i];
//						Fric_pid.sum_y += Fric_pid.Shoot_speed[i];
//						Fric_pid.sum_x += Fric_pid.x[i];
//						Fric_pid.sum_xy += Fric_pid.x[i] * Fric_pid.Shoot_speed[i];
//					}
//					 Fric_pid.a = (Fric_pid.adapting_num * Fric_pid.sum_xy - Fric_pid.sum_x * Fric_pid.sum_y)
//												/ (Fric_pid.adapting_num * Fric_pid.sum_x2 - Fric_pid.sum_x * Fric_pid.sum_x);
//					 Fric_pid.b = (Fric_pid.sum_x2 * Fric_pid.sum_y - Fric_pid.sum_x * Fric_pid.sum_xy)
//												/ (Fric_pid.adapting_num * Fric_pid.sum_x2 - Fric_pid.sum_x * Fric_pid.sum_x);
//					 Fric_pid.Speed_pre=Fric_pid.a*(Fric_pid.shoot_num+1)+Fric_pid.b;
//					 
//					}
//					
//				if(Shoot_Data.bullet_speed<29.5f &&Fric_pid.Speed_pre!=0)// 超射速保护降低到一定射速
//				{
//					pid_calc(&pid_shoot_speed, Fric_pid.Speed_pre, 28.5);//稳定到28.5的PID控制
//					shoot_Fric_pwm.limit_target = Init_PWM+pwm+pid_shoot_speed.pos_out;
//					Slope_On(&shoot_Fric_pwm); //上摩擦轮斜坡启动
//					FricMotor_PWM1 = shoot_Fric_pwm.real_target ;
//					FricMotor_PWM2 = shoot_Fric_pwm.real_target ;
//				}
//				else//超射速保护降低到一定射速
//				{
//					shoot_Fric_pwm.limit_target = Init_PWM+pwm;
//					Slope_On(&shoot_Fric_pwm); //上摩擦轮斜坡启动
//					FricMotor_PWM1 = shoot_Fric_pwm.real_target ;
//					FricMotor_PWM2 = shoot_Fric_pwm.real_target ;
//				}
//					
//				}

}
