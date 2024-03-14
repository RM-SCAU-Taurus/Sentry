#include "stdint.h"

#ifndef __BSP_TRIIGERMOTOR_H
#define __BSP_TRIIGERMOTOR_H

#define TRIGGER_MOTOR_ECD   32764.0f  //9齿 一颗编码值
#define TRIGGER_MOTOR_ECD_Circle   294876.0f  //9齿 一圈编码值 = 8191*36

#define MIN_HEAT		    80

void TriggerMotor_init(void);
void TriggerMotor_control(void);

#endif

