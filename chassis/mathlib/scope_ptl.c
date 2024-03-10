/**
  * @file scope_ptl.c
  * @version 2.0
  * @date Jan,30th 2021
  *
  * @brief  ʾ����Э��
  *
  *	@author
  *
  */
#include "scope_ptl.h"
#include "math.h"
#include "main.h"
#include "usart.h"

/* ���������õ��Ľṹ�� */
DataTypedfef CK;


/**
  * @brief �������ȸ�������ת��4�ֽ����ݲ�����ָ����ַ
  * @param target:Ŀ�굥��������; buf:��д������; beg:ָ��������ڼ���Ԫ�ؿ�ʼд��
  * @retval ��
  */
void Float2Byte(float *target,unsigned char *buf,unsigned char beg)
{
	unsigned char *point;
	/* ��ȡfloat�ĵ�ַ */
	point = (unsigned char*)target;
	buf[beg]   = point[0];
	buf[beg+1] = point[1];
	buf[beg+2] = point[2];
	buf[beg+3] = point[3];
}

/**
  * @brief ��������ͨ���ĵ����ȸ�������д�뷢�ͻ�����
	* @param Data:ͨ������; Channel:ѡ��ͨ����1-10��
  * @retval ��
  */
void DataScope_Get_Channel_Data(float Data,unsigned char Channel)
{
	/* ͨ����������10�����0ʱ��ִ�к��� */
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
  * @brief  ����ʾ������λ������ȷʶ���֡��ʽ
	* @param  Channel_Number:��Ҫ���͵�ͨ������
	* @retval 0:֡��ʽ����ʧ��; ����:���ͻ��������ݸ���
  */
unsigned char DataScope_Data_Generate(unsigned char Channel_Number)
{
	/* ͨ����������10�����0ʱ��ִ�к��� */	
	if( (Channel_Number > 10) || (Channel_Number == 0) )	{return 0;}
	else
	{
		/* ֡ͷ */
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

