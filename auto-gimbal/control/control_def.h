#ifndef _CONTROL_DEF_H_
#define _CONTROL_DEF_H_

#include "stm32f4xx_hal.h"
#include "math_calcu.h"

//机器标号
//小钢炮1号   3
//小钢炮2号   4
//小钢炮3号   5
//舵轮单枪    6
#define Robot_Number 5

/*----------------------------- the whole system ----------------------------- */
// task period
#define GIMBAL_PERIOD	  1
#define CHASSIS_PERIOD    2
#define SHOOT_PERIOD      2
#define USART_SEND_PERIOD 2
#define MODESWITCH_PERIOD 6
#define STATUS_PERIOD     500

// gimbal test pid param
#define pid_yaw_mecd_P 10.0f
#define pid_yaw_mecd_I 0.0f
#define pid_yaw_mecd_D 0.0f

#define pid_yaw_mspd_P 10.0f
#define pid_yaw_mspd_I 0.0f
#define pid_yaw_mspd_D 0.0f

/*----------------------------- manipulator preference ----------------------------- */
/* special function key and key status definition */
// chassis control key (status) define
#define KEY_CHASSIS_FIGHT       KB_G  //操作手不用
#define FLAG_CHASSIS_FIGHT      kb_status[KEY_CHASSIS_FIGHT]

#define KEY_CHASSIS_ROTATE      KB_F
#define FLAG_CHASSIS_ROTATE     kb_status[KEY_CHASSIS_ROTATE]

// shoot control key (status) define
#define KEY_SHOOT_FRIC          KB_V
#define FLAG_SHOOT_FRIC         kb_status[KEY_SHOOT_FRIC]

#define KEY_SHOOT_HOUSE         KB_B
#define FLAG_SHOOT_HOUSE        kb_status[KEY_SHOOT_HOUSE]

// vision control key status define
#define KEY_VISION_sENERGY      KB_Q
#define FLAG_VISION_sENERGY     kb_status[KEY_VISION_sENERGY]

#define KEY_VISION_bENERGY      KB_E
#define FLAG_VISION_bENERGY     kb_status[KEY_VISION_bENERGY]

#define KEY_VISION_ENERGY_DIR   KB_Z

#define KEY_VISION_ANTIROTATE   KB_R
#define FLAG_VISION_ANTIROTATE  kb_status[KEY_VISION_ANTIROTATE]

#define KEY_VISION_SENTRY       KB_X
#define FLAG_VISION_SENTRY      kb_status[KEY_VISION_SENTRY]

/* ========================================== Infantry s1 ========================================== */

#if ( Robot_Number == 3 )

/* ========================================== Infantry s2 ========================================== */

#elif ( Robot_Number == 4 )

/*-----------------------------shoot-----------------------------*/
#define SPEED_PWM_14        600     //14m/s射速  射速上限为15m/s  连发时：13.8-14.6 单发时：14.3-14.6
#define SPEED_PWM_17        650     //16m/s射速  射速上限为18m/s  连发时：15.8-16.7 单发时：16.9-17.2
#define SPEED_PWM_28        920     //28m/s射速  射速上限为30m/s  连发时：25.0-28.2 单发时：28.0-28.5

#define LOW_SPEED           SPEED_PWM_14   //校准时改成1100 (烧程序时调拨杆) 调试拨盘时，可以设置成200
#define MID_SPEED	        SPEED_PWM_17	
#define HIGH_SPEED	        SPEED_PWM_28

/* 引出的PWM口及对应TIM */
#define PWM_SEND_TIM12   (&htim12)
#define PWM_SEND_CH1    TIM12->CCR1 //PB14
#define PWM_SEND_CH2    TIM12->CCR2 //PB15

#define PWM_SEND_TIM3   (&htim3)
#define PWM_SEND_CH3    TIM3->CCR3  //PB0
#define PWM_SEND_CH4    TIM3->CCR4  //PB1

/* 弹舱盖 */
#define COVER_PWM_OPEN      600
#define COVER_PWM_CLOSE     2250
#define Magazine_PWM        PWM_SEND_CH3
#define Magazine_Time_CH    PWM_SEND_TIM3, TIM_CHANNEL_3

/* 枪管PWM IO口 */
#define FricMotor_PWM1      PWM_SEND_CH1  //左摩擦轮
#define FricMotor_PWM2      PWM_SEND_CH4  //右摩擦轮
#define FricMotor_Time_CH1  PWM_SEND_TIM12, TIM_CHANNEL_1
#define FricMotor_Time_CH2  PWM_SEND_TIM3 , TIM_CHANNEL_4

/* 拨盘 PID 参数 */
#define PID_TRIGGER_ECD_P 0.3f
#define PID_TRIGGER_ECD_I 0.0f
#define PID_TRIGGER_ECD_D 0.3f

#define PID_TRIGGER_SPD_P 5.0f
#define PID_TRIGGER_SPD_I 0.008f
#define PID_TRIGGER_SPD_D 0.0f

/* 拨盘频率 */
#define TRIGGER_PERIOD    90  //射击周期（ms）

/*-----------------------------chassis---------------------------*/
#define RC_CH4_SCALE    12     
#define RC_CH3_SCALE    12

#define SPEED_0W        1000.0f  //裁判系统底盘功率调为规则外时
#define SPEED_SUPPLY    600.0f
#define SPEED_45W  		3400.0f
#define SPEED_50W		4800.0f
#define SPEED_55W		4900.0f
#define SPEED_60W		5000.0f
#define SPEED_80W		6000.0f
#define SPEED_100W      6200.0f
#define SPEED_120W      7600.0f
#define SPEED_SUPERCAP  8000.0f

#define SUPERCAP_MAX_VOLAGE	23.7f		//超级电容最大电压
/*-----------------------------gimbal--------- -------------------*/
#define Reduction_ratio			    1.0f	//pit轴减速比
#define RC_CH2_SCALE                0.003f
#define RC_CH1_SCALE                ( -0.0005f )

#define KEYBOARD_SCALE_PIT          ( -0.06f )
#define KEYBOARD_SCALE_YAW	        ( -0.001f )
#define KEYBOARD_SCALE_YAW_SUPPLY   ( -0.0006f )

#define GIMBAL_PIT_CENTER_OFFSET    2760
#define GIMBAL_PIT_MAX              3555
#define GIMBAL_PIT_MIN              2460

#define GIMBAL_YAW_CENTER_OFFSET    2700
#define GIMBAL_YAW_BETWEEN_ECD      ( 8191 / 8 )
#define FIGHT_OFFSET_ERR            ( -1.0f * GIMBAL_YAW_BETWEEN_ECD / 8191 * 2 * PI )

/* YAW轴PID系数 */
#define pid_yaw_angle_P 180.0f
#define pid_yaw_angle_I 0.0f
#define pid_yaw_angle_D 0.0f

#define pid_yaw_spd_P 20.0f
#define pid_yaw_spd_I 0.3f
#define pid_yaw_spd_D 0.0f

/* PIT轴PID系数 */
#define pid_pit_ecd_P 9.0f
#define pid_pit_ecd_I 0.0f
#define pid_pit_ecd_D 0.0f

#define pid_pit_spd_P 8.0f
#define pid_pit_spd_I 0.2f
#define pid_pit_spd_D 0.0f

/* ========================================== Infantry s3 ========================================== */

#elif ( Robot_Number == 5 )

/*-----------------------------shoot-----------------------------*/
//#define SPEED_PWM_14        510     //14m/s射速  射速上限为15m/s  连发时：13.8-14.6 单发时：14.3-14.6
#define SPEED_PWM_14        200     //14m/s射速  射速上限为15m/s  连发时：13.8-14.6 单发时：14.3-14.6
#define SPEED_PWM_17        570     //16m/s射速  射速上限为18m/s  连发时：15.8-16.7 单发时：16.9-17.2
#define SPEED_PWM_28        800//915     //28m/s射速  射速上限为30m/s  连发时：25.0-28.2 单发时：28.0-28.5
#define LOW_SPEED           SPEED_PWM_14   //校准时改成1100 (烧程序时调拨杆) 调试拨盘时，可以设置成200
#define MID_SPEED	        SPEED_PWM_17	
#define HIGH_SPEED	        SPEED_PWM_28

/* 引出的PWM口及对应TIM */
#define PWM_SEND_TIM12   (&htim12)
#define PWM_SEND_CH1    TIM12->CCR1 //PB14
#define PWM_SEND_CH2    TIM12->CCR2 //PB15

#define PWM_SEND_TIM3   (&htim3)
#define PWM_SEND_CH3    TIM3->CCR3  //PB0
#define PWM_SEND_CH4    TIM3->CCR4  //PB1

/* 弹舱盖 */
#define COVER_PWM_OPEN      500
#define COVER_PWM_CLOSE     2400
#define Magazine_PWM        PWM_SEND_CH3
#define Magazine_Time_CH    PWM_SEND_TIM3, TIM_CHANNEL_3

/* 枪管PWM IO口 */
#define FricMotor_PWM1      PWM_SEND_CH1  //左摩擦轮
#define FricMotor_PWM2      PWM_SEND_CH4  //右摩擦轮
#define FricMotor_Time_CH1  PWM_SEND_TIM12, TIM_CHANNEL_1
#define FricMotor_Time_CH2  PWM_SEND_TIM3 , TIM_CHANNEL_4

/* 拨盘 PID 参数 */
#define PID_TRIGGER_ECD_P 0.2f
#define PID_TRIGGER_ECD_I 0.0f
#define PID_TRIGGER_ECD_D 0.0f

#define PID_TRIGGER_SPD_P 5.0f
#define PID_TRIGGER_SPD_I 0.008f
#define PID_TRIGGER_SPD_D 0.0f

/* 拨盘频率 */
#define TRIGGER_PERIOD    200  //射击周期（ms）	15HZ66.7ms 11HZ 90ms

/*-----------------------------chassis---------------------------*/
#define RC_CH4_SCALE    12     
#define RC_CH3_SCALE    12

#define SPEED_0W        1000.0f  //裁判系统底盘功率调为规则外时
#define SPEED_SUPPLY    2000.0f
#define SPEED_45W  		4400.0f
#define SPEED_50W		4600.0f
#define SPEED_55W		4800.0f
#define SPEED_60W		5300.0f
#define SPEED_80W		6500.0f
#define SPEED_100W      7300.0f
#define SPEED_120W      8000.0f
#define SPEED_SUPERCAP  8000.0f

#define SUPERCAP_MAX_VOLAGE	23.7f		//超级电容最大电压
/*-----------------------------gimbal----------------------------*/
#define Reduction_ratio			    1.0f	//pit轴减速比
#define RC_CH2_SCALE                0.004f
#define RC_CH1_SCALE                ( -0.0005f )

#define KEYBOARD_SCALE_PIT          ( -0.06f )
#define KEYBOARD_SCALE_YAW	        ( -0.001f )
#define KEYBOARD_SCALE_YAW_SUPPLY   ( -0.0006f )

#define GIMBAL_PIT_CENTER_OFFSET    0
#define GIMBAL_PIT_MAX              3450
#define GIMBAL_PIT_MIN              2330

//#define GIMBAL_YAW_CENTER_OFFSET    6848
#define GIMBAL_YAW_CENTER_OFFSET   5588 //5568
#define GIMBAL_YAW_BETWEEN_ECD      ( 8191 / 8 )
#define FIGHT_OFFSET_ERR            ( -1.0f * GIMBAL_YAW_BETWEEN_ECD / 8191 * 2 * PI )

/* YAW轴PID系数 */
#define pid_yaw_angle_P 180.0f//180
#define pid_yaw_angle_I 0.0f
#define pid_yaw_angle_D 0.0f

#define pid_yaw_spd_P 13.0f//20
#define pid_yaw_spd_I 0.1f//0.3
#define pid_yaw_spd_D 0.0f

/* PIT轴PID系数 */


#define pid_pit_ecd_P 115.00f//150.0
#define pid_pit_ecd_I 0.0f
#define pid_pit_ecd_D 0.0f

#define pid_pit_spd_P 4.0f//10
#define pid_pit_spd_I 0.075f//0.1
#define pid_pit_spd_D 0.0f

#elif ( Robot_Number == 6 )

/*-----------------------------shoot-----------------------------*/
#define SPEED_PWM_14        540     //14m/s射速  射速上限为15m/s  连发时：13.8-14.6 单发时：14.3-14.6
#define SPEED_PWM_17        605     //16m/s射速  射速上限为18m/s  连发时：15.8-16.7 单发时：16.9-17.2
#define SPEED_PWM_28        900     //28m/s射速  射速上限为30m/s  连发时：25.0-28.2 单发时：28.0-28.5

#define LOW_SPEED           SPEED_PWM_14   //校准时改成1100 (烧程序时调拨杆) 调试拨盘时，可以设置成200
#define MID_SPEED	        SPEED_PWM_17	
#define HIGH_SPEED	        SPEED_PWM_28

/* 引出的PWM口及对应TIM */
#define PWM_SEND_TIM12   (&htim12)
#define PWM_SEND_CH1    TIM12->CCR1 //PB14
#define PWM_SEND_CH2    TIM12->CCR2 //PB15

#define PWM_SEND_TIM3   (&htim3)
#define PWM_SEND_CH3    TIM3->CCR3  //PB0
#define PWM_SEND_CH4    TIM3->CCR4  //PB1

/* 弹舱盖 */
#define COVER_PWM_OPEN      600
#define COVER_PWM_CLOSE     2250
#define Magazine_PWM        PWM_SEND_CH3
#define Magazine_Time_CH    PWM_SEND_TIM3, TIM_CHANNEL_3

/* 枪管PWM IO口 */
#define FricMotor_PWM1      PWM_SEND_CH1  //左摩擦轮
#define FricMotor_PWM2      PWM_SEND_CH4  //右摩擦轮
#define FricMotor_Time_CH1  PWM_SEND_TIM12, TIM_CHANNEL_1
#define FricMotor_Time_CH2  PWM_SEND_TIM3 , TIM_CHANNEL_4

/* 拨盘 PID 参数 */
#define PID_TRIGGER_ECD_P 0.3f
#define PID_TRIGGER_ECD_I 0.0f
#define PID_TRIGGER_ECD_D 0.3f

#define PID_TRIGGER_SPD_P 5.0f
#define PID_TRIGGER_SPD_I 0.008f
#define PID_TRIGGER_SPD_D 0.0f

/* 拨盘频率 */
#define TRIGGER_PERIOD    90  //射击周期（ms）

/*-----------------------------chassis---------------------------*/
#define RC_CH4_SCALE    12     
#define RC_CH3_SCALE    12

#define SPEED_0W        1000.0f  //裁判系统底盘功率调为规则外时
#define SPEED_SUPPLY    600.0f
#define SPEED_45W  		3400.0f
#define SPEED_50W		4800.0f
#define SPEED_55W		4900.0f
#define SPEED_60W		5000.0f
#define SPEED_80W		6000.0f
#define SPEED_100W      6200.0f
#define SPEED_120W      7600.0f
#define SPEED_SUPERCAP  8000.0f

#define SUPERCAP_MAX_VOLAGE	23.7f		//超级电容最大电压
/*-----------------------------gimbal----------------------------*/
#define Reduction_ratio			    1.0f	//pit轴减速比
#define RC_CH2_SCALE                0.003f
#define RC_CH1_SCALE                ( -0.0005f )

#define KEYBOARD_SCALE_PIT          ( -0.06f )
#define KEYBOARD_SCALE_YAW	        ( -0.001f )
#define KEYBOARD_SCALE_YAW_SUPPLY   ( -0.0006f )

#define GIMBAL_PIT_CENTER_OFFSET    2800
#define GIMBAL_PIT_MAX              3600
#define GIMBAL_PIT_MIN              2500

#define GIMBAL_YAW_CENTER_OFFSET    3454
#define GIMBAL_YAW_BETWEEN_ECD      ( 8191 / 8 )
#define FIGHT_OFFSET_ERR            ( -1.0f * GIMBAL_YAW_BETWEEN_ECD / 8191 * 2 * PI )

/* YAW轴PID系数 */
#define pid_yaw_angle_P 180.0f
#define pid_yaw_angle_I 0.0f
#define pid_yaw_angle_D 0.0f

#define pid_yaw_spd_P 20.0f
#define pid_yaw_spd_I 0.3f
#define pid_yaw_spd_D 0.0f

/* PIT轴PID系数 */
#define pid_pit_ecd_P -8.0f
#define pid_pit_ecd_I 0.0f
#define pid_pit_ecd_D 0.0f

#define pid_pit_spd_P -2.5f
#define pid_pit_spd_I -0.2f
#define pid_pit_spd_D 0.0f

#endif

#endif
