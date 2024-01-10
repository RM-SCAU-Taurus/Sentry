/**
  * @file     bsp_CoverServo.h
  * @version  v2.0
  * @date     July,8th 2019
  *
  * @brief
  *
  *	@author   Fatmouse,part of the code reference Link's code
  *
  */
#ifndef __BSP_COVERSERVO
#define __BSP_COVERSERVO

#include "main.h"
#include "stm32f4xx_hal.h"


typedef enum
{
    COVER_OFF = 0,
    COVER_ON,
} cover_status_e;

void CoverServo_init(void);
void CoverServo_ctrl(uint16_t pwm);
void CoverServo_switch(void);

#endif
