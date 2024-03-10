#ifndef _CONTROL_DEF_H_
#define _CONTROL_DEF_H_

#include "stm32f4xx_hal.h"

#define NO3_DEF //更改兵种

#ifdef NO3_DEF
#define Robot_Number 3
/***********shoot************/
#define COVER_START 800
#define COVER_END 1800
/***********chassis**********/
#define rc_ch2_scale 12
#define rc_ch1_scale 12
/***********gimbal***********/
//#define rc_ch4_scale 12.0f
//#define rc_ch3_scale 12.0f
#define rc_ch4_scale 6.0f
#define rc_ch3_scale 6.0f

#define gimbal_pit_center_offset 2200			
#define gimbal_yaw_center_offset 6240
#define GIMBAL_PIT_MAX           2990		
#define GIMBAL_PIT_MIN           1370	
/*********PID-Gimbal*********/

#define pid_pit_ecd_P 10.0f
#define pid_pit_ecd_I 0.0f
#define pid_pit_ecd_D 0.0f

#define pid_pit_spd_P 10.0f
#define pid_pit_spd_I 0.2f
#define pid_pit_spd_D 0.0f

#define pid_yaw_angle_P 200.0f
#define pid_yaw_angle_I 0.0f
#define pid_yaw_angle_D 0.0f

#define pid_yaw_spd_P 25.0f
#define pid_yaw_spd_I 0.3f
#define pid_yaw_spd_D 0.0f

#define Reduction_ratio			2.0f		//pit轴减速比

/*发射器参数*/
#define LOW_SPEED   			568		//15m/s射速 
#define LOW_SPEED_UPPER		541
#define MID_SPEED	  620		//18m/s射速
#define HIGH_SPEED	900		//28m/s射速		单枪管用
#define SHOOT_FREQUENCY  15 //射击频率   //单枪管的频率是该数除2
#define Upper_FricMotor_PWM	TIM12->CCR2		//上枪管
#define Under_FricMotor_PWM	TIM12->CCR1		//下枪管

/*底盘参数*/
#define POWER_OFFSET		-1.5f		//自制功率控制板大3.0W
#define SPEED_45W  			2900.0f
#define SPEED_50W				3200.0f
#define SPEED_55W				3500.0f
#define SPEED_60W				3700.0f
#define SPEED_80W				4600.0f
#define SPEED_100W      5800.0f
#define SPEED_SUPERCAP  8000.0f
#endif

#ifdef NO4_DEF
#define Robot_Number 4
/***********shoot************/
#define COVER_START 800
#define COVER_END 1800
/***********chassis**********/
#define rc_ch2_scale 12
#define rc_ch1_scale 12
/***********gimbal***********/
#define rc_ch4_scale -0.008f
#define rc_ch3_scale -0.0003f

#define gimbal_pit_center_offset 1450			
#define gimbal_yaw_center_offset 4747
#define GIMBAL_PIT_MAX           1860		
#define GIMBAL_PIT_MIN           720	
/*********PID-Gimbal*********/

#define pid_pit_ecd_P 10.0f
#define pid_pit_ecd_I 0.0f
#define pid_pit_ecd_D 0.0f

#define pid_pit_spd_P 15.0f
#define pid_pit_spd_I 0.2f
#define pid_pit_spd_D 0.0f

#define pid_yaw_angle_P 180.0f
#define pid_yaw_angle_I 0.0f
#define pid_yaw_angle_D 0.0f

#define pid_yaw_spd_P 25.0f
#define pid_yaw_spd_I 0.3f
#define pid_yaw_spd_D 0.0f

#define SUPERCAP_MAX_VOLAGE	25.5f		//超级电容最大电压
#define Reduction_ratio			1.0f		//pit轴减速比

/*发射器参数*/
#define LOW_SPEED   545		//15m/s射速 
#define LOW_SPEED_UPPER	545
#define MID_SPEED	  620		//18m/s射速
#define HIGH_SPEED	900		//28m/s射速		单枪管用
#define SHOOT_FREQUENCY  25 //射击频率   //单枪管的频率是该数除2
#define Upper_FricMotor_PWM	TIM3->CCR4		 
#define Under_FricMotor_PWM	TIM12->CCR2		//下枪管

/*底盘参数*/
#define POWER_OFFSET		-2.0f		//自制功率控制板大3.0W
#define SPEED_45W  			3400.0f
#define SPEED_50W				3600.0f
#define SPEED_55W				3900.0f
#define SPEED_60W				4200.0f
#define SPEED_80W				5600.0f
#define SPEED_100W      6800.0f
#define SPEED_SUPERCAP  8000.0f
#endif
#endif
