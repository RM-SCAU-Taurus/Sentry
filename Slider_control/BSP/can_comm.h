#ifndef __CAN_COMM_H__
#define __CAN_COMM_H__

#include "can.h"

#define WEIGHT_MAX_ECD      7900
#define WEIGHT_MIN_ECD      400
#define WEIGHT_MID_ECD      ( (WEIGHT_MAX_ECD + WEIGHT_MIN_ECD) / 2 )

#define WEIGHT0_OFFSET_ECD  3933		//左平衡块初始值  1330--878
#define WEIGHT1_OFFSET_ECD	1930										//4441--4772


/* 摩擦轮保护模式 */
typedef enum
{
	FRIC_PROTECT 	= 0,
	FRIC_UNPROTECT 	= 1,
	FRIC_SLOW_TO_PROTECT = 2
} fric_protect_mode_e;

typedef struct 
{
	float fric_ecd_ref;
	float fric_ecd_fdb;
	float fric_ecd_err;
	
	float fric_spd_ref;
	float fric_spd_fdb;
	float fric_spd_err;
}fric_pid_t;

typedef struct {
    uint32_t 	protect_cnt;
    uint8_t 	protect_flag;
    uint8_t 	init_flag;
	  float 		set_speed;			//设置目标转速
	  fric_pid_t	Fric_Pid_Set[2];	//摩擦轮pid参数
} fric_t;



/* motor current parameter structure */
typedef struct
{
    int16_t balance_cur[2];  //平步平衡块
} motor_current_t;

typedef struct {
    float   ecd_ref;
    float   ecd_fdb;
    float   spd_ref;
    float   spd_fdb;
    int16_t current;
    float   offset;
} moto_pid_t;

typedef struct
{
    moto_pid_t weight[2];
} balance_t;
//extern moto_measure_t moto_balance[2];

//extern motor_current_t motor_cur;
//extern float chassis_last_torque[2], kal_detec_torque[2];
void Fric_msg_handle(uint8_t * CAN_Rx_data);
void Fric_init(void);
void can_device_init(void);
void can_msg_send_task(void);
 void Balance_slider_control(void);
 void can_out(void);
void Fric_control(void);
void mode_check(void);
//void can_msg_send_task(void const *argu); /* can信息发送任务 */

#endif
