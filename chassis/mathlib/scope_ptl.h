/** 
  * @file scope_ptl.h
  * @version 2.0
  * @date Jan,30th 2021
  *
  * @brief
  *
  *	@author
  *
  */
#ifndef __DATA_PROTOCOL_H__
#define __DATA_PROTOCOL_H__

#include "stm32f4xx.h"

typedef struct
{
	unsigned char DataScope_OutPut_Buffer[42];	//串口发送缓冲区
	unsigned char Send_Count; 									//串口需要发送的数据个数
	unsigned char DataCnt;          						//计数变量
} DataTypedfef;

extern DataTypedfef CK;

void Float2Byte(float *target,unsigned char *buf,unsigned char beg);
void DataScope_Get_Channel_Data(float Data,unsigned char Channel);
unsigned char DataScope_Data_Generate(unsigned char Channel_Number);

#endif

