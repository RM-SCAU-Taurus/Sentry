#ifndef _CONTROL_DEF_H_
#define _CONTROL_DEF_H_

#include "stm32f4xx_hal.h"
#include "math_calcu.h"

#define Sentry 1 
#define Robot_Number Sentry

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

#if ( Robot_Number == Sentry )

/*-----------------------------shoot-----------------------------*/

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

#define GIMBAL_YAW_CENTER_OFFSET   5588 //5568
#define GIMBAL_YAW_BETWEEN_ECD      ( 8191 / 8 )
#define FIGHT_OFFSET_ERR            ( -1.0f * GIMBAL_YAW_BETWEEN_ECD / 8191 * 2 * PI )

/* YAW轴PID系数 */
#define pid_yaw_angle_6020_P 150.0f//180
#define pid_yaw_angle_6020_I 0.0f
#define pid_yaw_angle_6020_D 0.0f

#define pid_yaw_spd_6020_P 12.0f//20
#define pid_yaw_spd_6020_I 0.1f//0.3
#define pid_yaw_spd_6020_D 0.0f

#define pid_yaw_angle_9025_P 0.0f//180
#define pid_yaw_angle_9025_I 0.0f
#define pid_yaw_angle_9025_D 0.0f

#define pid_yaw_spd_9025_P 0.0f//20
#define pid_yaw_spd_9025_I 0.0f//0.3
#define pid_yaw_spd_9025_D 0.0f

/* PIT轴PID系数 */
#define pid_pit_angle_P 115.00f//180
#define pid_pit_angle_I 0.0f
#define pid_pit_angle_D 0.0f

#define pid_pit_ecd_P 200.00f//150.0
#define pid_pit_ecd_I 0.0f
#define pid_pit_ecd_D 0.0f

#define pid_pit_spd_P 5.0f//10
#define pid_pit_spd_I 0.075f//0.1
#define pid_pit_spd_D 0.0f

#endif

#endif
