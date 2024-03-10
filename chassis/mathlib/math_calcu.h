#ifndef _MATH_CALCU_H_
#define _MATH_CALCU_H_

#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

#define PI       3.14159265358979f
#define RAD2DEG 57.29577951308f

#ifndef ABS
#define ABS(x)	((x>0)? (x): (-x))
#endif

typedef struct
{
	float input;        //��������
	float out;          //�������
	float min_value;    //�޷���Сֵ
	float max_value;    //�޷����ֵ
	float frame_period; //ʱ����
} ramp_function_source_t;

float ramp_input(float fdb, float ref, float period);
void Bubble_Sort(float *a,uint8_t n);
float circle_error(float *set, float *get, float circle_para);
void  abs_limit(float *a, float ABS_MAX, float offset);
float data_limit(float data, float max, float min);

#endif
