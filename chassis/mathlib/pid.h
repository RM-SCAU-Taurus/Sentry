/******************************************************************************
  *@brief
  *@copyright Copyright (c) 2017 <dji-innovations, Corp. RM Dept.>
  *@license
*******************************************************************************/
#ifndef __pid_H
#define __pid_H

#ifdef  __PID_GLOBALS
#define __PID_EXT
#else
#define __PID_EXT extern
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

enum
{
	LLAST	= 0,
	LAST 	= 1,
	NOW 	= 2,

	POSITION_PID,
	DELTA_PID,
};

typedef struct __pid_t
{
	float p;
	float i;
	float d;

	float set[3];				//目标值,包含NOW， LAST， LLAST上上次
	float get[3];				//测量值
	float err[3];				//误差

	float pout;							//p输出
	float iout;							//i输出
	float dout;							//d输出

	float pos_out;						//本次位置式输出
	float last_pos_out;				//上次输出
	float delta_u;						//本次增量值
	float delta_out;					//本次增量式输出 = last_delta_out + delta_u
	float last_delta_out;

	float max_err;
	float deadband;				//err < deadband return
	uint32_t pid_mode;
	uint32_t MaxOutput;				//输出限幅
	uint32_t IntegralLimit;		//积分限幅

	void (*f_param_init)(struct __pid_t *pid,  //PID参数初始化
									uint32_t pid_mode,
									uint32_t maxOutput,
									uint32_t integralLimit,
									float p,
									float i,
									float d);
	void (*f_pid_reset)(struct __pid_t *pid, float p, float i, float d);		//pid三个参数修改
} pid_t;

__PID_EXT pid_t pid_chassis_3508_spd[4];

__PID_EXT pid_t pid_chassis_6020_ecd[4];
__PID_EXT pid_t pid_chassis_6020_spd[4];

void  PID_struct_init(pid_t* pid, uint32_t mode, uint32_t maxout, uint32_t intergral_limit, float kp, float ki, float kd);
float pid_calc(pid_t* pid, float fdb, float ref);

#endif

