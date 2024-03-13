#ifndef __BSP_JY901_H
#define __BSP_JY901_H
#include "stm32f4xx_hal.h"
#include "main.h"
#include "usart.h"
#include "string.h"
#include "stdlib.h"
#define JY901_BUFLEN  20
typedef struct
{
	float roll;
	float pitch;
	float yaw;
	float wx;
	float wy;
	float wz;
}imu_typedef;
extern imu_typedef JY901_org_data;
void JY901_original_data_read(uint8_t imu_buf[]);
#endif

