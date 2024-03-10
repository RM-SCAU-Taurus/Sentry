#ifndef __CLASSIS_TASK_H__
#define __CLASSIS_TASK_H__

#include "stm32f4xx_hal.h"
#include "math_calcu.h"

#define CHASSIS_PERIOD  2

typedef struct
{
    float vx;
    float vy;
    float vw;
} spd_t;

/* chassis control message structure */
typedef __packed struct
{
    uint8_t 	SOF;
    uint16_t	vx_vy_vw[3];
    uint8_t		max_power;
    __packed struct
    {
        uint8_t  vx_sign : 1;
        uint8_t  vy_sign : 1;
        uint8_t  vw_sign : 1;
        uint8_t	 mode 	 : 1;
        uint8_t  shift   : 1;
    } sign_msg;
    uint8_t  	EOF;
} chassis_control_msg_t;

/* chassis parameter structure */
typedef struct
{
    uint8_t 	kb_C;

    spd_t     spd_ref;
    spd_t     spd_input;
    spd_t     spd_fdb; //(键盘/遥控器)输入的三轴向速度

    float   	wheel_spd_input[4]; //舵轮解算所得轮速输入
    float   	wheel_spd_ref[4];		//速度重分配后所得轮速目标
    float   	wheel_spd_fdb[4];		//电机轮速反馈
    int16_t		current_3508[4];

    float 		rudder_ecd_offset[4];
    float   	rudder_angle_ref[4];
    float   	rudder_ecd_ref[4];
    float     rudder_ecd_ref_ramp[4];
    float   	rudder_ecd_fdb[4];
    float			rudder_ecd_error[4];
    float   	rudder_spd_ref[4];
    float   	rudder_spd_fdb[4];
    int16_t		current_6020[4];
    uint8_t   mode;
    float     wheel_max;
		    void(*power_control)(int16_t *  current);//功率控制
    void(*spd_distribution)();//运动学解算
    void(*Float_to_uint8)(float *target,unsigned char *buf,unsigned char beg);
    void (*msg_handle)(uint32_t can_id,uint8_t * data);
    void (*msg_send)(int16_t ID, int16_t iq1, int16_t iq2, int16_t iq3, uint8_t iq4,uint8_t iq5);
} chassis_t;



extern chassis_t chassis;
void chassis_task(void const *argu);
void chassis_param_init(void);
void mecanum_calc(float vx, float vy, float vw, float speed[]);
void steering_calc(float vx, float vy, float vw, float angle[], float speed[]);
void chassis_spd_distribution(void);
void chassis_pid_calcu(void);
void MC_data_handler(uint8_t* MC_Control_Rx_data);

#endif
