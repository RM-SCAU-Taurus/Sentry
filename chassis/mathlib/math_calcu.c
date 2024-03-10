#include "math_calcu.h"
#include "bsp_can.h"
#include "chassis_task.h"

/**
  * @brief          б���������룬���������ֵ���е���
  * @author         
  * @param[in]      
  * @param[in]      
  * @param[in]      
  * @retval         ���ؿ�
  */
float ramp_input(float fdb, float ref, float period)
{
	if(fdb<ref)	     
	{		
		fdb += period;
		if(fdb>=ref)	fdb=ref;
	}
	else if(fdb>ref)  
	{
		fdb -= period;
		if(fdb<=ref)	fdb=ref;
	}	
	return fdb;
}

/**
  * @brief          ð��������
  * @param[in]      �������飬�����С
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
	*@bref		�������ݼ���ƫ��ֵ
	*@param[in]
	*@note
	*/
float circle_error(float *set, float *get, float circle_para)
{
	float error;

	if(*set > *get)
	{
		if(*set - *get> circle_para/2)
			error = *set - *get - circle_para;
		else
			error = *set - *get;
	}
	else if(*set < *get)
	{
		if(*set - *get<-1*circle_para/2)
			error = *set - *get +circle_para;
		else
			error = *set - *get;
	}
	else	error = 0;

	return error;
}

/**
  * @brief	�����޷�����
  * @param[in] 
  * @retval   
  */
float data_limit(float data, float max, float min)
{
	if(data >= max)					return max;
	else if(data <= min)		return min;
	else 										return data;
}

void abs_limit(float *a, float ABS_MAX, float offset)
{
	if(*a > (ABS_MAX+offset))
		*a = ABS_MAX + offset;
	if(*a < (-ABS_MAX+offset))
		*a = -ABS_MAX + offset;
}
