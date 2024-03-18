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
#include <stdlib.h>
#include <stdbool.h>
#define window 5 // 滑动窗口大小


#define M_PI_F 3.14159265358979f

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
    for(i=window-1; i!=0; i--)
    {
        if(Data[i]>max) max=Data[i];
        else if(Data[i]<min) min=Data[i];
        sum+=Data[i];
        Data[i]=Data[i-1];
    }
    i=window-2;
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

// Function to compare integers for qsort function
int compare(const void *a, const void *b)
{
	return (*(int *)a - *(int *)b);
}
// Function to perform median filter
void medianFilter(float *signal, float *result, uint8_t N)
{
	int i, j, filterSize = 5;
	int *temp = (int *)malloc(filterSize * sizeof(int));

	for (i = 0; i < N; ++i)
	{
		for (j = 0; j < filterSize; ++j)
		{
			if ((i + j) < N)
				temp[j] = signal[i + j];
			else
				temp[j] = 0;
		}

		qsort(temp, filterSize, sizeof(int), compare);
		result[i] = temp[filterSize / 2];
	}

	free(temp);
}






void Lpf2p_SetCutoffFreq(Lpf2p *lpf, float sample_freq, float cutoff_freq) {
    float fr = 0;
    float ohm = 0;
    float c = 0;

    fr = sample_freq / cutoff_freq;
    ohm = tanf(M_PI_F / fr);
    c = 1.0f + 2.0f * cosf(M_PI_F / 4.0f) * ohm + ohm * ohm;

    lpf->cutoff_freq1 = cutoff_freq;

    if (lpf->cutoff_freq1 > 0.0f) {
        lpf->b01 = ohm * ohm / c;
        lpf->b11 = 2.0f * lpf->b01;
        lpf->b21 = lpf->b01;
        lpf->a11 = 2.0f * (ohm * ohm - 1.0f) / c;
        lpf->a21 = (1.0f - 2.0f * cosf(M_PI_F / 4.0f) * ohm + ohm * ohm) / c;
    }
}

float Lpf2p_Apply(Lpf2p *lpf, float sample) {
    float delay_element_0 = 0, output = 0;
    if (lpf->cutoff_freq1 <= 0.0f) {
        // no filtering
        return sample;
    } else {
        delay_element_0 = sample - lpf->delay_element_11 * lpf->a11 - lpf->delay_element_21 * lpf->a21;
        // do the filtering
        if (isnan(delay_element_0) || isinf(delay_element_0)) {
            // don't allow bad values to propagate via the filter
            delay_element_0 = sample;
        }
        output = delay_element_0 * lpf->b01 + lpf->delay_element_11 * lpf->b11 + lpf->delay_element_21 * lpf->b21;

        lpf->delay_element_21 = lpf->delay_element_11;
        lpf->delay_element_11 = delay_element_0;

        // return the value. Should be no need to check limits
        return output;
    }
}
/*int main() {
    Lpf2p lpf;
    Lpf2p_SetCutoffFreq(&lpf, 1000.0f, 100.0f); // Sample frequency of 1000 Hz, cutoff frequency of 100 Hz
    float filtered_value = Lpf2p_Apply(&lpf, 1.0f); // Applying filter to input value of 1.0
    return 0;
}*/
