#ifndef __DRIVER_FRICMOTOR
#define __DRIVER_FRICMOTOR

#include "stm32f4xx_hal.h"
#include "shoot_task.h"

#define Init_PWM	 900

void FricMotor_init(void);
void FricMotor_Control(void);

#endif
