/*
 * @Author: your name
 * @Date: 2021-12-19 14:37:59
 * @LastEditTime: 2022-01-15 17:40:13
 * @LastEditors: Please set LastEditors
 * @Description: 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 * @FilePath: \MDK-ARMd:\RM\auto-Infantry\rm-Infantry-20211026\app\chassis_task.h
 */
/**
  * @file     chassis_task.h
  * @version  v2.0
  * @date     July,6th 2019
  *
  * @brief
  *
  *	@author   Fatmouse
  *
  */
#ifndef __CHASSIS_TASK_H__
#define __CHASSIS_TASK_H__

#include "stm32f4xx_hal.h"
#include "bsp_powerlimit.h"
#include "shoot_task.h"
#include "bsp_judge.h"
#define CHASSIS_WZ_SET_SCALE	0.0f //底盘速度自旋补偿值
#define M3508_MOTOR_RPM_TO_VECTOR 0.000415809748903494517209f		//m3508转化成底盘速度(m/s)的比例

/* chassis parameter structure */
//typedef struct 
//{
    /* data */
 //   powercontrol_t power_msg;
 //   barrel_param_t barreel_msg;
    
//}robo;

typedef struct
{
    float vx;
    float vy;
    float vw;
} spd_t;
typedef struct
{
    /* data */
    float  x;
    float y;
}odom_t;
typedef enum 
{
    CHASSIS_MODE_PROTECT,
    
    CHASSIS_MODE_REMOTER_FOLLOW,
    CHASSIS_MODE_REMOTER_ROTATE,
    
    CHASSIS_MODE_KEYBOARD_FOLLOW,
    CHASSIS_MODE_KEYBOARD_ROTATE,
    CHASSIS_MODE_KEYBOARD_FIGHT,
    CHASSIS_MODE_KEYBOARD_SUPPLY,
    CHASSIS_MODE_AUTO
} chassis_mode_e;

typedef struct
{   
    chassis_mode_e mode;
    spd_t     spd_ref;
    spd_t     spd_input;
    spd_t     spd_fdb;
    odom_t    odom;
    int16_t   wheel_spd_ref[4];
    int16_t   wheel_spd_input[4];
    int16_t   wheel_spd_fdb[4];
    int16_t   current[4];
		int16_t   rudder_angle_ref[4];
    int16_t   spd_error;
    int16_t   position_ref;
		float 		rudder_ecd_offset[4];
		float   	rudder_ecd_ref[4];
		float     rudder_ecd_ref_ramp[4];
		float   	rudder_ecd_fdb[4];
		float			rudder_ecd_error[4];
		float   	rudder_spd_ref[4];
		float   	rudder_spd_fdb[4];
		int16_t		current_6020[4];
    float 	  position_error;
    float     angle_error;
    float     wheel_max;
    float     keyboard_input;
    int8_t    spin_dir;      //小陀螺方向
    int8_t    fight_dir;     //迎敌模式方向
    uint8_t   spd_tx_data[12];//0-3vx.4-7vy,8-11vw
    void(*power_control)(int16_t *  current);//功率控制
    void(*spd_distribution)();//运动学解算
    void(*Float_to_uint8)(float *target,unsigned char *buf,unsigned char beg);
    void (*msg_handle)(uint32_t can_id,uint8_t * data);
    void (*msg_send)(int16_t ID, int16_t iq1, int16_t iq2, int16_t iq3, uint8_t iq4,uint8_t iq5);
    void (*game_msg_send)(int16_t TX_ID,robot_judge_msg_t * robot_judge_msg_e);
    void (*spd_msg_send)(int16_t TX_ID);
} chassis_t;
//typedef void (*chassis_power_ctrol)(chassis_t * chassis_e);//功率控制
//typedef void (*chassis_spd_distributio_t)(chassis_t * chassis_e);//运动学解算
extern chassis_t chassis;
void chassis_task(void const *argu);
void can_msg_read(uint32_t can_id,uint8_t * data);
void chassis_init(void);
void mecanum_calc(float vx, float vy, float vw, int16_t speed[]);
void sparate_move(void);
void chassis_ramp(void);
void chassis_spd_distribution(void);
void chassis_sigmoid(void);
void steering_calc(float vx, float vy, float vw, int16_t angle[], int16_t speed[]);
void chassis_pid_calcu(void);
void rudder_odom_cal(void);
#endif
