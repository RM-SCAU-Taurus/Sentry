#ifndef _MATH_CALCU_H_
#define _MATH_CALCU_H_

#include "math.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "arm_math.h"

#define N2 100
#define e  2.718282f
#define SIGMOID_PERIOD 0.133333f
#define SIGMOID_MAX    10

#define pi 3.14159265358979323846f

/*输出限幅*/
#define Output_Limit(output,max,min) \
        ((output)<=(max) && (output)>=(min)? output: ((output)>(max)? (output = max):(output = min)))

/*绝对值*/
#define ABS(x)		((x>0)? (x): (-x))

typedef struct
{
    float change_scale;
    uint16_t real_target;
    uint16_t limit_target;
    TickType_t ticks;
    TickType_t last_ticks;
} Slope_Struct;

typedef __packed struct
{
    float input;        //输入数据
    float out;          //滤波输出的数据
    float num[1];       //滤波参数
    float frame_period; //滤波的时间间隔 单位 s
} first_order_filter_type_t;

void Slope_On(Slope_Struct *V);
typedef struct
{
    float input;        //输入数据
    float out;          //输出数据
    float min_value;    //限幅最小值
    float max_value;    //限幅最大值
    float frame_period; //时间间隔
} ramp_function_source_t;

extern ramp_function_source_t chassis_x_ramp;
extern ramp_function_source_t chassis_y_ramp;
extern ramp_function_source_t chassis_w_ramp;
extern ramp_function_source_t chassis_super_x_ramp;
extern ramp_function_source_t chassis_super_y_ramp;

void ramp_calc(ramp_function_source_t *ramp_source_type, float frame_period, float input, float max, float min);
void Bubble_Sort(float *a,uint8_t n);
float GildeAverageValueFilter(float NewValue,float *Data);
float Sigmoid_function(float x);
float circle_error(float set,float get,float circle_para);
float data_limit(float data, float max, float min);
float lowpassfilter(float x);
#endif
