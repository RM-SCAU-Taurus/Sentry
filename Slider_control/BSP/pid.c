#include "pid.h"
#include "string.h"

#ifndef ABS
    #define ABS(x)  ( (x)>0? (x): (-(x)) )
#endif

enum
{
    LLAST   = 0,
    LAST    = 1,
    NOW     = 2,
};

/* 带偏移量的绝对值限幅函数 */
static void abs_limit(float *a, float abs_max,float offset)
{
    if(*a > abs_max+offset)
        *a = abs_max+offset;
    if(*a < -abs_max+offset)
        *a = -abs_max+offset;
}

/* 参数初始化 */
static void pid_param_init(
    pid_t *pid,
    uint8_t mode,
    float maxout,
    float intergral_limit,
    float deadband,
    float div,
    float 	kp,
    float 	ki,
    float 	kd)
{
    pid->IntegralLimit = intergral_limit;
    pid->MaxOutput = maxout;
    pid->deadband = deadband;
    pid->div = div;
    pid->pid_mode = mode;

    pid->p = kp;
    pid->i = ki;
    pid->d = kd;
}

/* 中途更改参数设定(调试) */
static void pid_param_reset(pid_t	*pid, float kp, float ki, float kd)
{
    pid->p = kp;
    pid->i = ki;
    pid->d = kd;
    pid->iout = 0;
}

/*pid总体初始化-----------------------------------------------------------------*/
void PID_struct_init(
    pid_t* pid, uint8_t mode, float maxout, 
    float intergral_limit, float deadband, float div,
    float kp, float ki, float kd)
{
    memset(pid, 0, sizeof(pid_t));
    /*init function pointer*/
    pid->f_param_init = pid_param_init;
    pid->f_pid_reset = pid_param_reset;
    /*init pid param */
    pid->f_param_init(pid, mode, maxout, intergral_limit, deadband, div, kp, ki, kd);
}

/**
  * @bref. calculate delta PID and position PID
  * @param[in] set： target
  * @param[in] real	measure
  */
float pid_calc(pid_t* pid, float get, float set)
{
    pid->get[NOW] = get;
    pid->set[NOW] = set;
    pid->err[NOW] = set - get;	//set - measure
    
    /* 误差超限控制 */
    if (pid->max_err != 0 && ABS(pid->err[NOW]) > pid->max_err)
        return pid->pid_mode==POSITION_PID ? pid->pos_out : pid->delta_out;

    if(pid->pid_mode == POSITION_PID) //位置式p
    {
        /* P输出 */
        pid->pout = pid->p * pid->err[NOW];
        /* D输出 */
        pid->d_error = pid->err[NOW] - pid->err[LAST];
        pid->dout = pid->d * pid->d_error;
        /* I输出 */
        if (pid->deadband != 0 && ABS(pid->err[NOW]) >= pid->deadband)//积分死区控制
        {
            if (pid->div != 0 && ABS(pid->err[NOW]) <= pid->div)//积分分离
                pid->iout += pid->i * pid->err[NOW];
            else if (pid->div == 0)
                pid->iout += pid->i * pid->err[NOW];
        }
        else if (pid->deadband == 0)//积分无死区控制
        {
            if (pid->div != 0 && ABS(pid->err[NOW]) <= pid->div)//积分分离
                pid->iout += pid->i * pid->err[NOW];
            else if (pid->div == 0)
                pid->iout += pid->i * pid->err[NOW];
        }
        abs_limit(&(pid->iout), pid->IntegralLimit,0);
        /* 位置式PID输出 */
        pid->pos_out = pid->pout + pid->iout + pid->dout;
        abs_limit(&(pid->pos_out), pid->MaxOutput,0);
        pid->last_pos_out = pid->pos_out;
    }
    else if(pid->pid_mode == DELTA_PID)//增量式P
    {
        pid->pout = pid->p * (pid->err[NOW] - pid->err[LAST]);
        pid->iout = pid->i * pid->err[NOW];
        pid->dout = pid->d * (pid->err[NOW] - 2*pid->err[LAST] + pid->err[LLAST]);

        abs_limit(&(pid->iout), pid->IntegralLimit,0);
        pid->delta_u = pid->pout + pid->iout + pid->dout;
        pid->delta_out = pid->last_delta_out + pid->delta_u;
        abs_limit(&(pid->delta_out), pid->MaxOutput,0);
        pid->last_delta_out = pid->delta_out;
    }

    pid->err[LLAST] = pid->err[LAST];
    pid->err[LAST] = pid->err[NOW];
    pid->get[LLAST] = pid->get[LAST];
    pid->get[LAST] = pid->get[NOW];
    pid->set[LLAST] = pid->set[LAST];
    pid->set[LAST] = pid->set[NOW];
    return pid->pid_mode==POSITION_PID ? pid->pos_out : pid->delta_out;
}
