/**
  * @file			pid.c
  * @version	V1.0.0
  * @date			2016年11月11日17:21:36
  * @brief  
  *  	
  *******************************************************************************/
#define __PID_GLOBALS
/* Includes ------------------------------------------------------------------*/
#include "pid.h"
#include <math.h>
#include "math_calcu.h"

#define ABS(x)	((x>0)? (x): (-x)) 

/*参数初始化--------------------------------------------------------------*/
static void pid_param_init(pid_t *pid, uint32_t mode, uint32_t maxout, uint32_t intergral_limit,
														float kp, float ki, float kd)
{
	pid->IntegralLimit = intergral_limit;
	pid->MaxOutput = maxout;
	pid->pid_mode = mode;

	pid->p = kp;
	pid->i = ki;
	pid->d = kd;
}
/*中途更改参数设定(调试)------------------------------------------------------------*/
static void pid_reset(pid_t	*pid, float kp, float ki, float kd)
{
	pid->p = kp;
	pid->i = ki;
	pid->d = kd;
}
/*pid总体初始化-----------------------------------------------------------------*/
void PID_struct_init(pid_t* pid, uint32_t mode, uint32_t maxout, uint32_t intergral_limit,
											float kp, float ki, float kd)
{
	/* init function pointer */
	pid->f_param_init = pid_param_init;
	pid->f_pid_reset  = pid_reset;

	/* init pid param */
	pid->f_param_init(pid, mode, maxout, intergral_limit, kp, ki, kd);
}

/**
    *@bref  calculate delta PID and position PID
    *@param[in] set： target
    *@param[in] real	measure
    */
float pid_calc(pid_t* pid, float get, float set)
{
	pid->get[NOW] = get;
	pid->set[NOW] = set;
	pid->err[NOW] = set - get;	//set - measure
	if (pid->max_err != 0 && ABS(pid->err[NOW]) >  pid->max_err)
	return 0;
	if (pid->deadband != 0 && ABS(pid->err[NOW]) < pid->deadband)
	return 0;

	/* 位置式PID */
	if(pid->pid_mode == POSITION_PID)
	{
		pid->pout = pid->p * pid->err[NOW];
		pid->iout += pid->i * pid->err[NOW];
		pid->dout = pid->d * (pid->err[NOW] - pid->err[LAST] );
		
		abs_limit(&(pid->iout), pid->IntegralLimit,0);
		pid->pos_out = pid->pout + pid->iout + pid->dout;
		abs_limit(&(pid->pos_out), pid->MaxOutput,0);
		pid->last_pos_out = pid->pos_out;	//update last time 
	}
	/* 增量式PID */
	else if(pid->pid_mode == DELTA_PID)
	{
		pid->pout = pid->p * (pid->err[NOW] - pid->err[LAST]);
		pid->iout = pid->i * pid->err[NOW];
		pid->dout = pid->d * (pid->err[NOW] - 2*pid->err[LAST] + pid->err[LLAST]);
		
		abs_limit(&(pid->iout), pid->IntegralLimit,0);
		pid->delta_u = pid->pout + pid->iout + pid->dout;
		pid->delta_out = pid->last_delta_out + pid->delta_u;
		abs_limit(&(pid->delta_out), pid->MaxOutput,0);
		pid->last_delta_out = pid->delta_out;	//update last time
	}

	pid->err[LLAST] = pid->err[LAST];
	pid->err[LAST]  = pid->err[NOW];
	pid->get[LLAST] = pid->get[LAST];
	pid->get[LAST]  = pid->get[NOW];
	pid->set[LLAST] = pid->set[LAST];
	pid->set[LAST]  = pid->set[NOW];

	return pid->pid_mode==POSITION_PID ? pid->pos_out : pid->delta_out;
}

