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
	unsigned char DataScope_OutPut_Buffer[42];	//���ڷ��ͻ�����
	unsigned char Send_Count; 									//������Ҫ���͵����ݸ���
	unsigned char DataCnt;          						//��������
} DataTypedfef;

extern DataTypedfef CK;

void Float2Byte(float *target,unsigned char *buf,unsigned char beg);
void DataScope_Get_Channel_Data(float Data,unsigned char Channel);
unsigned char DataScope_Data_Generate(unsigned char Channel_Number);

#endif

