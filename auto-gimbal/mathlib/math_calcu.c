/**
  * @file     math_calcu.c
  * @version  v2.0
  * @date     July,6th 2019
	*
  * @brief    数学计算函数
	*
  *	@author   Fatmouse
  *
  */
#include "math_calcu.h"
#include "remote_msg.h"
#include "math.h"
#include "data_processing.h"

ramp_function_source_t chassis_x_ramp;
ramp_function_source_t chassis_y_ramp;
ramp_function_source_t chassis_w_ramp;
ramp_function_source_t chassis_super_x_ramp;
ramp_function_source_t chassis_super_y_ramp;
/**
  * @brief          斜波函数计算，根据输入的值进行叠加，输入单位为 /s 即一秒后增加输入的值
  * @author         RM
  * @param[in]      斜波函数结构体
  * @param[in]      输入值
  * @param[in]      滤波参数
  * @retval         返回空
  */
void ramp_calc(ramp_function_source_t *ramp_source_type, float frame_period, float input, float max, float min)
{
    ramp_source_type->max_value = max;
    ramp_source_type->min_value = min;
    ramp_source_type->frame_period = frame_period;

    ramp_source_type->input = input;

    ramp_source_type->out += ramp_source_type->input * ramp_source_type->frame_period;

    if (ramp_source_type->out > ramp_source_type->max_value)
    {
        ramp_source_type->out = ramp_source_type->max_value;
    }
    else if (ramp_source_type->out < ramp_source_type->min_value)
    {
        ramp_source_type->out = ramp_source_type->min_value;
    }
}

void Slope_On(Slope_Struct *V)
{
    V->last_ticks = V->ticks;
    V->ticks = osKernelSysTick();
    if(V->real_target !=V->limit_target)
    {
        if(V->real_target < V->limit_target)//加操作
        {
            V->real_target = V->real_target + V->change_scale* (V->ticks-V->last_ticks);
            if(V->real_target > V->limit_target)//限幅
            {
                V->real_target =  V->limit_target;
            }
        }
        else if(V->real_target > V->limit_target)//减操作
        {
            V->real_target =  V->real_target - V->change_scale* (V->ticks-V->last_ticks);
            if(V->real_target < V->limit_target)//限幅
            {
                V->real_target =  (short)V->limit_target;
            }
        }
    }
}

/**
  * @brief          冒泡排序函数
  * @param[in]      排序数组，数组大小
  * @retval         void
  */
void Bubble_Sort(float *a,uint8_t n)
{
    float buf;
    for(uint8_t i=0; i<n; ++i)
    {
        for(uint8_t j=0; j<n-1-i; ++j)
        {
            if(a[j]<a[j+1])
            {
                buf=a[j];
                a[j] = a[j+1];
                a[j+1] = buf;
            }
        }
    }
}

/**
  * @brief          滑动滤波函数   注：N2为滑动窗口大小
  * @author
  * @param[in]      滤波前的值、滤波缓存数组
  * @retval         滤波后的值
  */
float GildeAverageValueFilter(float NewValue,float *Data)
{
    float max,min;
    float sum;
    uint16_t i;
    Data[0]=NewValue;
    max=Data[0];
    min=Data[0];
    sum=Data[0];
    for(i=N2-1; i!=0; i--)
    {
        if(Data[i]>max) max=Data[i];
        else if(Data[i]<min) min=Data[i];
        sum+=Data[i];
        Data[i]=Data[i-1];
    }
    i=N2-2;
    sum=sum-max-min;
    sum=sum/i;
    return(sum);
}

/**
  * @brief          Sigmoid函数
  * @author
  * @param[in]      自变量
  * @retval         映射后的函数值
  */
float Sigmoid_function(float x)
{
    float y;
    float temp_x=ABS(x) - SIGMOID_MAX;			//将sigmoid函数向右平移最大区间值
    y= 1 / (1 + pow(e,-temp_x));
    return y;
}


/**
	*@func   float Circle_error(float set ,float get ,float circle_para)
	*@bref		环形数据计算偏差值
	*@param[in] set 设定值 get采样值 circle_para 一圈数值
	*@note	环形数据下，直接计算出PID中的偏差值
*/
float circle_error(float set,float get,float circle_para)
{
    float error;
    if(set > get)
    {
        if(set - get> circle_para/2)
            error = set - get - circle_para;
        else
            error = set - get;
    }
    else if(set < get)
    {
        if(set - get<-1*circle_para/2)
            error = set - get +circle_para;
        else
            error = set - get;
    }
    else	error = 0;

    return error;
}

float data_limit(float data, float max, float min)
{
    if(data >= max)					return max;
    else if(data <= min)		return min;
    else 										return data;
}

float lowpassfilter(float x)
{
	static float alpha = 0.9;
	static float y,y_prv;
	
	y = alpha * y_prv+ (1 - alpha) * x;
	y_prv = y;
	
	
	return y;

}