#ifndef __MATH_PROCESS_H
#define __MATH_PROCESS_H


#include "stdbool.h"
#include "stdint.h"
#include "string.h"
#include "cmsis_os.h"


#define pi 3.14159265358979323846f

/*输出限幅*/
#define Output_Limit(output,max,min) \
        ((output)<=(max) && (output)>=(min)? output: ((output)>(max)? (output = max):(output = min)))

/*绝对值*/
#define ABS(x) ((x>0)? (x): (-x))

/**
  * @Brief   防止超限的加法运算符（+）
  * @Param  待被加变量，增量正值！！变量最大值
  */
#define ADD_LIMIT(var,delta,limit) \
        ( (limit)-(var)>=(delta)? (var)+=(delta): ((var)=(limit)) )

/**
  * @Brief   防止超限的减法运算符（-）
  * @Param  待减变量，增量正值！！变量最小值
  */
#define MIN_LIMIT(var,delta,limit) \
        ( (limit)-(var)<=(-delta)? (var)-=(delta): ((var)=(limit)) )


typedef struct
{
    float change_scale;  //斜率
    float real_target;   //当前值！！
    float limit_target;  //目标置
    uint32_t ticks;
    uint32_t last_ticks;
} Slope_Struct;

typedef struct
{
    float input;        //输入数据
    float out;          //输出数据
    float min_value;    //限幅最小值
    float max_value;    //限幅最大值
    float frame_period; //时间间隔
} ramp_function_source_t;

void    abs_limit(float *a, float ABS_MAX,float offset);
void    Slope_On(Slope_Struct *V);
void    slope_init(Slope_Struct *V, float change_scale,
                   float real_target, float limit_target);
void    ramp_calc(ramp_function_source_t *ramp_source_type,
                  float frame_period, float input, float max, float min);

float   circle_error(float set,float get,float circle_para);
bool    task_delay(uint16_t t[2], uint16_t task_T, uint16_t delay_ms);

#endif

