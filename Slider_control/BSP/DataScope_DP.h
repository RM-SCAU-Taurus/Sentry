#ifndef __DATA_PRTOCOL_H
#define __DATA_PRTOCOL_H

#define VOFA

#include "stm32f4xx.h"

void DataWave(UART_HandleTypeDef* huart);
void DataScope_Get_Channel_Data(float Data);


#endif



