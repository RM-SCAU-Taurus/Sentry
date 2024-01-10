/*
 * @Author: your name
 * @Date: 2022-01-12 16:36:31
 * @LastEditTime: 2022-01-14 23:09:25
 * @LastEditors: your name
 * @Description: 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 * @FilePath: \chassis\mathlib\DataScope_DP.h
 */
#ifndef __DATA_PRTOCOL_H
#define __DATA_PRTOCOL_H

#include "stm32f4xx.h"

//若要使用VOFA+串口上位机，则注释下述宏
//#define MINIBALANCE

void DataWave(UART_HandleTypeDef* huart);
void Float2Byte(float *target,unsigned char *buf,unsigned char beg);
#endif

