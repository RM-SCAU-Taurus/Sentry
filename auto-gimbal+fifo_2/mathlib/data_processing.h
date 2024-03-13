#ifndef __DATA_PROCESSING_H
#define __DATA_PROCESSING_H

#ifdef  __DATA_PROCESSING_GLOBALS
#define __DATA_PROCESSING_EXT
#else
#define __DATA_PROCESSING_EXT extern
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#define ABS(x)		((x>0)? (x): (-x))

float circle_error(float set,float get,float circle_para);
void abs_limit(float *a, float ABS_MAX,float offset);
#endif

