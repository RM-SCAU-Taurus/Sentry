#include "stdint.h"

#ifndef __BSP_TRIIGERMOTOR_H
#define __BSP_TRIIGERMOTOR_H

#define TRIGGER_MOTOR_ECD   36859.0f  //����һ���ӵ�ת���ı���ֵ 8191 * 36 / 8 = 36859.5f
#define MIN_HEAT		    80

void TriggerMotor_init(void);
void TriggerMotor_control(void);

#endif

