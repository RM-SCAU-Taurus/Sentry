#include "math_process.h"


/**
  * @brief  斜波结构体（再）初始化
  * @note   常在需要改变目标值时使用
  */
void slope_init(Slope_Struct *V, float change_scale,
                float real_target, float limit_target)
{
    /*防止重复初始化*/
    if(limit_target == V->limit_target)
        return;

    V->change_scale = change_scale;
    V->real_target = real_target;
    V->limit_target = limit_target;
}

/**
  * @brief          斜波函数计算
  * @param[in]      斜波结构体
  * @note           必须在系统任务中调用！！
  *                 可在变周期任务中运行，因为斜率确定，函数内主动获取两次执行时间增量
  * @note           使用准备：需要声明一个结构体变量，初始化初始值与目标值以及斜率
  *                 使用方法：调用此函数后，再调用结构体变量的 real_target
  */
void Slope_On(Slope_Struct *V)
{
    V->last_ticks = V->ticks;
    V->ticks = osKernelSysTick();
    if(V->real_target !=V->limit_target)  //是否已经到达目标值
    {
        if(V->real_target < V->limit_target)  //加操作
        {
            V->real_target = V->real_target + V->change_scale* (V->ticks-V->last_ticks);
            if(V->real_target > V->limit_target)  //限幅
            {
                V->real_target =  V->limit_target;
            }
        }
        else if(V->real_target > V->limit_target)  //减操作
        {
            V->real_target =  V->real_target - V->change_scale* (V->ticks-V->last_ticks);
            if(V->real_target < V->limit_target)  //限幅
            {
                V->real_target =  (short)V->limit_target;
            }
        }
    }
}

/**
  * @brief          斜波函数计算，根据输入的值进行叠加， 输入单位为 /s 即一秒后增加输入的值
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

/**
  * @brief          冒泡排序函数
  * @param[in]      排序数组，数组大小
  * @retval         void
  */
void Bubble_Sort(int *a,uint8_t n)
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
  *@Brief     环形数据偏差计算
  *@Param    set 设定值 get采样值 circle_para一圈数值
  *@Note     环形数据下，直接计算出PID中的偏差值 共四种情形 error 的方向是从get指向set的优弧
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

/**
  * @Brief   兼容包含偏移量的限幅
  * @Param  数据变量指针，最大值，偏移量（初始值）
  */
void abs_limit(float *a, float ABS_MAX,float offset)
{
    if(*a > ABS_MAX+offset)
        *a = ABS_MAX+offset;
    if(*a < -ABS_MAX+offset)
        *a = -ABS_MAX+offset;
}

/**
  * @Brief   非阻塞性延时
  * @Param  记录当前时间与结束延时时间的数组（初始化为{0，0}），任务周期，延时时间
  */
bool task_delay(uint16_t t[2], uint16_t task_T, uint16_t delay_ms)
{
    if( t[0] == 0 )
        t[1] = delay_ms/task_T;  //需要任务执行的次数
    t[0] += 1;
    if( t[0] >= t[1] )
    {
        t[0] = 0;
        return true;
    }
    else
        return false;
}

