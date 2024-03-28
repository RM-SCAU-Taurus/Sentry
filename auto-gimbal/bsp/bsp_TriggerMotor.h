#include "stdint.h"

#ifndef __BSP_TRIIGERMOTOR_H
#define __BSP_TRIIGERMOTOR_H

#define TRIGGER_MOTOR_ECD   32764.0f  //9齿 一颗编码值
#define TRIGGER_MOTOR_ECD_Circle   294876.0f  //9齿 一圈编码值 = 8191*36

#define MIN_HEAT		    80

void TriggerMotor_init(void);
void TriggerMotor_control(void);
void Trigger_SINGLE_or_SERIES(void);
void Trigger_STOP_or_PROTECT(void);
uint8_t Trigger_Back(int16_t speed_rpm);
void Trigger_back_action(void);
#endif

