#include "data_scope.h"

/* 定义示波器通道数 */
#ifndef MINIBALANCE
    #define DATASCOPE_MAX_SEND_NUM MAX_SEND_NUM //使用用户定义的最大通道数
#else
    #define DATASCOPE_MAX_SEND_NUM 10 //默认10个通道
#endif

/* 示波器数据结构体 */
static struct _DataTypedfef_t
{
    unsigned char OutPut_Buffer[4*DATASCOPE_MAX_SEND_NUM+4];//串口发送缓冲区
    unsigned char Send_Count;//串口需要发送的数据个数
    unsigned char Data_Num;//变量数量
} CK;

//函数说明：将单精度浮点数据转成4字节数据并存入指定地址
//target:目标单精度数据
//buf:待写入数组
//beg:指定从数组第几个元素开始写入
static void Float2Byte(float *target,unsigned char *buf,unsigned char beg)
{
    unsigned char *point;
    point = (unsigned char*)target;	  //得到float的地址
    buf[beg]   = point[0];
    buf[beg+1] = point[1];
    buf[beg+2] = point[2];
    buf[beg+3] = point[3];
}

//函数说明：将待发送通道的单精度浮点数据写入发送缓冲区
//Data：通道数据
void DataScope_Get_Channel_Data(float Data)
{
    CK.Data_Num++;
    if (CK.Data_Num > DATASCOPE_MAX_SEND_NUM)
        return;  //通道个数大于最大通道数，不执行函数
    else
    {
    #ifdef MINIBALANCE
        Float2Byte(&Data,CK.OutPut_Buffer,( (CK.Data_Num-1) * 4 +1 ) );  //留出帧头
    #else  //VOFA
        Float2Byte(&Data,CK.OutPut_Buffer,( (CK.Data_Num-1) * 4) );
    #endif
    }
}

//函数说明：生成能正确识别的帧格式
//Channel_Number，需要发送的通道个数
//返回发送缓冲区数据个数
static unsigned char DataScope_Data_Generate(unsigned char Channel_Number)
{
    if ( (Channel_Number > DATASCOPE_MAX_SEND_NUM) || (Channel_Number == 0) )
    {
        return 0;    //通道个数大于10或等于0，直接跳出，不执行函数
    }
    else
    {
    #ifdef MINIBALANCE
        CK.OutPut_Buffer[0] = '$';  //帧头
        uint8_t temp_cnt = Channel_Number*4+1;
        CK.OutPut_Buffer[temp_cnt]  =  temp_cnt;  //帧尾
        return (temp_cnt+1);  //返回一个数据包的字节数
    #else  //VOFA+
        uint8_t temp_cnt = Channel_Number*4+4;
        CK.OutPut_Buffer[4*Channel_Number + 0] = 0x00;
        CK.OutPut_Buffer[4*Channel_Number + 1] = 0x00;
        CK.OutPut_Buffer[4*Channel_Number + 2] = 0x80;
        CK.OutPut_Buffer[4*Channel_Number + 3] = 0x7f;
        return temp_cnt;  //返回一个数据包的字节数
    #endif
    }
}

/* This function is used to register the data to send */
__weak void DataWavePkg(void)
{
    /* the data needed to send:
     * DataScope_Get_Channel_Data(float_type_data1);
     * DataScope_Get_Channel_Data(float_type_data2);
     */
}

//函数说明：上位机通过串口打印数据波形
//附加说明：周期调用此函数
void DataWave(UART_HandleTypeDef* huart)
{
    DataWavePkg();
    CK.Send_Count = DataScope_Data_Generate(CK.Data_Num);
    for( uint8_t cnt = 0; cnt < CK.Send_Count; cnt++)
    {
        while((huart->Instance->SR&0X40)==0);
        huart->Instance->DR = CK.OutPut_Buffer[cnt];
    }
    CK.Data_Num=0;
    CK.Send_Count = 0;
}
