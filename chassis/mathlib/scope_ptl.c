/**
  * @file scope_ptl.c
  * @version 2.0
  * @date Jan,30th 2021
  *
  * @brief  示波器协议
  *
  *	@author
  *
  */
#include "scope_ptl.h"
#include "math.h"
#include "main.h"
#include "usart.h"

/* 传输数据用到的结构体 */
DataTypedfef CK;


/**
  * @brief 将单精度浮点数据转成4字节数据并存入指定地址
  * @param target:目标单精度数据; buf:待写入数组; beg:指定从数组第几个元素开始写入
  * @retval 无
  */
void Float2Byte(float *target,unsigned char *buf,unsigned char beg)
{
	unsigned char *point;
	/* 获取float的地址 */
	point = (unsigned char*)target;
	buf[beg]   = point[0];
	buf[beg+1] = point[1];
	buf[beg+2] = point[2];
	buf[beg+3] = point[3];
}

/**
  * @brief 将待发送通道的单精度浮点数据写入发送缓冲区
	* @param Data:通道数据; Channel:选择通道（1-10）
  * @retval 无
  */
void DataScope_Get_Channel_Data(float Data,unsigned char Channel)
{
	/* 通道个数大于10或等于0时不执行函数 */
	if( (Channel > 10) || (Channel == 0) )	return;
	else
	{
		switch (Channel)
		{
			case 1:  Float2Byte(&Data,CK.DataScope_OutPut_Buffer,1); 	break;
			case 2:  Float2Byte(&Data,CK.DataScope_OutPut_Buffer,5); 	break;
			case 3:  Float2Byte(&Data,CK.DataScope_OutPut_Buffer,9); 	break;
			case 4:  Float2Byte(&Data,CK.DataScope_OutPut_Buffer,13); break;
			case 5:  Float2Byte(&Data,CK.DataScope_OutPut_Buffer,17); break;
			case 6:  Float2Byte(&Data,CK.DataScope_OutPut_Buffer,21); break;
			case 7:  Float2Byte(&Data,CK.DataScope_OutPut_Buffer,25); break;
			case 8:  Float2Byte(&Data,CK.DataScope_OutPut_Buffer,29); break;
			case 9:  Float2Byte(&Data,CK.DataScope_OutPut_Buffer,33); break;
			case 10: Float2Byte(&Data,CK.DataScope_OutPut_Buffer,37); break;
		}
	}
}

/**
  * @brief  生成示波器上位机能正确识别的帧格式
	* @param  Channel_Number:需要发送的通道个数
	* @retval 0:帧格式生成失败; 其他:发送缓冲区数据个数
  */
unsigned char DataScope_Data_Generate(unsigned char Channel_Number)
{
	/* 通道个数大于10或等于0时不执行函数 */	
	if( (Channel_Number > 10) || (Channel_Number == 0) )	{return 0;}
	else
	{
		/* 帧头 */
		CK.DataScope_OutPut_Buffer[0] = '$';

		switch(Channel_Number)   
		{
			case 1:   CK.DataScope_OutPut_Buffer[5]  =  5; return  6;  
			case 2:   CK.DataScope_OutPut_Buffer[9]  =  9; return 10;
			case 3:   CK.DataScope_OutPut_Buffer[13] = 13; return 14; 
			case 4:   CK.DataScope_OutPut_Buffer[17] = 17; return 18;
			case 5:   CK.DataScope_OutPut_Buffer[21] = 21; return 22;  
			case 6:   CK.DataScope_OutPut_Buffer[25] = 25; return 26;
			case 7:   CK.DataScope_OutPut_Buffer[29] = 29; return 30; 
			case 8:   CK.DataScope_OutPut_Buffer[33] = 33; return 34; 
			case 9:   CK.DataScope_OutPut_Buffer[37] = 37; return 38;
			case 10:  CK.DataScope_OutPut_Buffer[41] = 41; return 42; 
		}
	}
	return 0;
}

