
/**
  ******************************************************************************
  * @file			data_processing.c
  * @version		V1.0.0
  * @date			2018年12月5日
  * @brief   		一些数据处理函数
  *******************************************************************************/


#define __DATA_PROCESSING_GLOBALS
/* Includes ------------------------------------------------------------------*/
#include "mytype.h"
#include "data_processing.h"
#include <math.h>
//#include "cmsis_os.h"


void abs_limit(float *a, float ABS_MAX,float offset) {
    if(*a > ABS_MAX+offset)
        *a = ABS_MAX+offset;
    if(*a < -ABS_MAX+offset)
        *a = -ABS_MAX+offset;
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
///**
//	*@func   float Circle_error(float set ,float get ,float circle_para)
//	*@bref		环形数据计算偏差值
//	*@param[in] set 设定值 get采样值 circle_para 一圈数值
//	*@note	环形数据下，直接计算出PID中的偏差值
//*/
//float circle_error(float set ,float get ,float circle_para)
//{
//	float error;
//	if(set > get)
//	{
//		if(set - get> circle_para/2)
//			error = get - set + circle_para;
//		else
//			error = set - get;
//	}
//	else if(set < get)
//	{
//		if(set - get<-1*circle_para/2)
//			error = get - set - circle_para;
//		else
//			error = set - get;
//	}
//	else	error = 0;

//	return error;
//}
