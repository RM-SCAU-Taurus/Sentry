#ifndef __DRIVER_FRICMOTOR
#define __DRIVER_FRICMOTOR

#include "stm32f4xx_hal.h"
#include "shoot_task.h"
uint8_t FricMotor_init(void); 
void FricMotor_speed_set(uint16_t speed);
void FricMotor_Control(void);
void Fric_protect(void);
void Fric_unprotect(void);
#endif
