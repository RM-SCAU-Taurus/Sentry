/******************************************************************************
/// @brief
/// @copyright Copyright (c) 2017 <dji-innovations, Corp. RM Dept.>
/// @license MIT License
/// Permission is hereby granted, free of charge, to any person obtaining a copy
/// of this software and associated documentation files (the "Software"), to deal
/// in the Software without restriction,including without limitation the rights
/// to use, copy, modify, merge, publish, distribute, sublicense,and/or sell
/// copies of the Software, and to permit persons to whom the Software is furnished
/// to do so,subject to the following conditions:
///
/// The above copyright notice and this permission notice shall be included in
/// all copies or substantial portions of the Software.
///
/// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
/// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
/// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
/// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
/// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
/// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
/// THE SOFTWARE.
*******************************************************************************/
/**
  ******************************************************************************
  * @file			pid.c
  * @version		V1.0.0
  * @date			2016年11月11日17:21:36
  * @brief   		对于PID， 反馈/测量习惯性叫get/measure/real/fdb,
                          期望输入一般叫set/target/ref
  *******************************************************************************/
#define __PID_GLOBALS
/* Includes ------------------------------------------------------------------*/
#include "pid.h"
#include "mytype.h"
#include <math.h>
#include "data_processing.h"
// #include "cmsis_os.h"

enum
{
    LLAST = 0,
    LAST = 1,
    NOW = 2,
};

pid_t pid_chassis_spd[4];
#ifndef ABS
#define ABS(x) ((x > 0) ? (x) : (-x))
#endif 


// void abs_limit(float *a, float ABS_MAX){
//     if(*a > ABS_MAX)
//         *a = ABS_MAX;
//     if(*a < -ABS_MAX)
//         *a = -ABS_MAX;
// }
/*参数初始化--------------------------------------------------------------*/
static void pid_param_init(
    pid_t *pid,
    uint32_t mode,
    uint32_t maxout,
    uint32_t intergral_limit,
    float kp,
    float ki,
    float kd)
{

    pid->IntegralLimit = intergral_limit;
    pid->MaxOutput = maxout;
    pid->pid_mode = mode;

    pid->p = kp;
    pid->i = ki;
    pid->d = kd;
}
/*中途更改参数设定(调试)------------------------------------------------------------*/
static void pid_reset(pid_t *pid, float kp, float ki, float kd)
{
    pid->p = kp;
    pid->i = ki;
    pid->d = kd;
}

/**
 *@bref. calculate delta PID and position PID
 *@param[in] set： target
 *@param[in] real	measure
 */
float pid_calc(pid_t *pid, float get, float set)
{
    pid->get[NOW] = get;
    pid->set[NOW] = set;
    pid->err[NOW] = set - get; // set - measure
    if (pid->max_err != 0 && ABS(pid->err[NOW]) > pid->max_err)
        return 0;
    if (pid->deadband != 0 && ABS(pid->err[NOW]) < pid->deadband)
        return 0;

    if (pid->pid_mode == POSITION_PID) // 位置式p
    {
        pid->pout = pid->p * pid->err[NOW];

        pid->iout += pid->i * pid->err[NOW];

        pid->d_error = pid->err[NOW] - pid->err[LAST];
        pid->dout = pid->d * (pid->err[NOW] - pid->err[LAST]);
        abs_limit(&(pid->iout), pid->IntegralLimit, 0);
        pid->pos_out = pid->pout + pid->iout + pid->dout;
        abs_limit(&(pid->pos_out), pid->MaxOutput, 0);
        pid->last_pos_out = pid->pos_out; // update last time
    }
    else if (pid->pid_mode == DELTA_PID) // 增量式P
    {
        pid->pout = pid->p * (pid->err[NOW] - pid->err[LAST]);
        pid->iout = pid->i * pid->err[NOW];
        pid->dout = pid->d * (pid->err[NOW] - 2 * pid->err[LAST] + pid->err[LLAST]);

        abs_limit(&(pid->iout), pid->IntegralLimit, 0);
        pid->delta_u = pid->pout + pid->iout + pid->dout;
        pid->delta_out = pid->last_delta_out + pid->delta_u;
        abs_limit(&(pid->delta_out), pid->MaxOutput, 0);
        pid->last_delta_out = pid->delta_out; // update last time
    }

    pid->err[LLAST] = pid->err[LAST];
    pid->err[LAST] = pid->err[NOW];
    pid->get[LLAST] = pid->get[LAST];
    pid->get[LAST] = pid->get[NOW];
    pid->set[LLAST] = pid->set[LAST];
    pid->set[LAST] = pid->set[NOW];
    return pid->pid_mode == POSITION_PID ? pid->pos_out : pid->delta_out;
    //
}

/**
 *@bref. special calculate position PID @attention @use @gyro data!!
 *@param[in] set： target
 *@param[in] real	measure
 */
float pid_sp_calc(pid_t *pid, float get, float set, float gyro)
{
    pid->get[NOW] = get;
    pid->set[NOW] = set;
    pid->err[NOW] = set - get; // set - measure

    if (pid->pid_mode == POSITION_PID) // 位置式p
    {
        pid->pout = pid->p * pid->err[NOW];
        if (fabs(pid->i) >= 0.001f)
            pid->iout += pid->i * pid->err[NOW];
        else
            pid->iout = 0;
        pid->dout = -pid->d * gyro / 100.0f;
        abs_limit(&(pid->iout), pid->IntegralLimit, 0);
        pid->pos_out = pid->pout + pid->iout + pid->dout;
        abs_limit(&(pid->pos_out), pid->MaxOutput, 0);
        pid->last_pos_out = pid->pos_out; // update last time
    }

    pid->err[LLAST] = pid->err[LAST];
    pid->err[LAST] = pid->err[NOW];
    pid->get[LLAST] = pid->get[LAST];
    pid->get[LAST] = pid->get[NOW];
    pid->set[LLAST] = pid->set[LAST];
    pid->set[LAST] = pid->set[NOW];
    return pid->pid_mode == POSITION_PID ? pid->pos_out : pid->delta_out;
}
/*pid总体初始化-----------------------------------------------------------------*/
void PID_struct_init(
    pid_t *pid,
    uint32_t mode,
    uint32_t maxout,
    uint32_t intergral_limit,

    float kp,
    float ki,
    float kd)
{
    /*init function pointer*/
    pid->f_param_init = pid_param_init;
    pid->f_pid_reset = pid_reset;
    /*init pid param */
    pid->f_param_init(pid, mode, maxout, intergral_limit, kp, ki, kd);
}

void abs_limit(float *a, float ABS_MAX, float offset)
{
    if (*a > ABS_MAX + offset)
        *a = ABS_MAX + offset;
    if (*a < -ABS_MAX + offset)
        *a = -ABS_MAX + offset;
}

/**********************************************************************************************************
 *函 数 名: FeedForward_Calc
 *功能说明: 前馈算法
 *形    参: PID_Struct *P  PID参数结构体
 *        ActualValue    PID计算反馈量（当前真实检测值）
 *返 回 值: PID反馈计算输出值
 **********************************************************************************************************/
void FeedForward_Calc(FeedForward_Typedef *FF, float Now_DeltIn)
{
    FF->Now_DeltIn = Now_DeltIn;
    FF->Out = FF->Now_DeltIn * FF->K1 + (FF->Now_DeltIn - FF->Last_DeltIn) * FF->K2;
    FF->Last_DeltIn = FF->Now_DeltIn;
    abs_limit(&(FF->Out), FF->OutMax, 0);
		
}

/**********************************************************************************************************
 *函 数 名: D_AGL_FeedForward_Calc
 *功能说明: 前馈算法(基于外环期望微分)
 *形    参: PID_Struct *P  PID参数结构体
 *        ActualValue    PID计算反馈量（当前真实检测值）
 *返 回 值: PID反馈计算输出值
 **********************************************************************************************************/
void D_AGL_FeedForward_Calc(D_AGL_FeedForward_Typedef *AGL_FF,float now_Ref,float period)
{
    AGL_FF->Now_ref = now_Ref;
		AGL_FF->xie =  (AGL_FF->Now_ref - AGL_FF->Last_ref);
    AGL_FF->Out = AGL_FF->k *  ( (AGL_FF->Now_ref - AGL_FF->Last_ref) / period );
    AGL_FF->Last_ref = AGL_FF->Now_ref;
		
		if( AGL_FF->xie >0.5 || AGL_FF->xie <-0.5 )
		AGL_FF->Out = 0;
		
}

// static void f_Changing_Integral_Rate(pid_t *pid,float ScalarB,float ScalarA)
// {

//         if (ABS(*pid->err) <= ScalarB)
//             pid->iout += pid->i * pid->err[NOW]; //完整积分
//         if (ABS(*pid->err) <= (ScalarA +ScalarB))
//             //使用线性函数过渡
//             pid->iout += (ScalarA- ABS(*pid->err) + ScalarB) / ScalarA * pid->i * pid->err[NOW]  ;
//         else
//             pid->iout += 0;//取消积分环节
// }
