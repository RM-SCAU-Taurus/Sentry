//Channel：选择通道（1-10）
//传入单精度浮点数
//在函数的最下方的DataWave函数修改发送的数据
//其他函数无需修改
#include "DataScope_DP.h"
#include "usart.h"


#include "bsp_powerlimit.h"
#include "gimbal_task.h"
#include "pid.h"
#include "chassis_task.h"
#include "bsp_T_imu.h"
#include "bsp_vision.h"
#include "shoot_task.h"
#include "func_generator.h"
#include "bsp_TriggerMotor.h"

#define ABS(x)		((x>0)? (x): (-x))

typedef struct
{
    unsigned char DataScope_OutPut_Buffer[44];	   //串口发送缓冲区
    unsigned char Send_Count; //串口需要发送的数据个数
    unsigned char DataCnt;          //计数变量
} DataTypedfef;

DataTypedfef CK;	//传输数据用到的结构体

//函数说明：将单精度浮点数据转成4字节数据并存入指定地址
//target:目标单精度数据
//buf:待写入数组
//beg:指定从数组第几个元素开始写入
void Float2Byte(float *target,unsigned char *buf,unsigned char beg)
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
//Channel：选择通道（1-10）
void DataScope_Get_Channel_Data(float Data,unsigned char Channel)
{
    if ( (Channel > 10) || (Channel == 0) ) 
        return;  //通道个数大于10或等于0，直接跳出，不执行函数
    else
    {
    #ifdef MINIBALANCE
        Float2Byte(&Data,CK.DataScope_OutPut_Buffer,( (Channel-1) * 4 +1 ) );  //留出帧头
    #else  //VOFA
        Float2Byte(&Data,CK.DataScope_OutPut_Buffer,( (Channel-1) * 4) );
    #endif
    }
}


//函数说明：生成能正确识别的帧格式
//Channel_Number，需要发送的通道个数
//返回发送缓冲区数据个数
unsigned char DataScope_Data_Generate(unsigned char Channel_Number)
{
    if ( (Channel_Number > 10) || (Channel_Number == 0) )
    {
        return 0;    //通道个数大于10或等于0，直接跳出，不执行函数
    }
    else
    {
    #ifdef MINIBALANCE
        CK.DataScope_OutPut_Buffer[0] = '$';  //帧头
        uint8_t temp_cnt = Channel_Number*4+1;
        CK.DataScope_OutPut_Buffer[temp_cnt]  =  temp_cnt;  //帧尾
        return (temp_cnt+1);  //返回一个数据包的字节数
    #else  //VOFA+
        uint8_t temp_cnt = Channel_Number*4+4;
        CK.DataScope_OutPut_Buffer[4*Channel_Number + 0] = 0x00;
        CK.DataScope_OutPut_Buffer[4*Channel_Number + 1] = 0x00;
        CK.DataScope_OutPut_Buffer[4*Channel_Number + 2] = 0x80;
        CK.DataScope_OutPut_Buffer[4*Channel_Number + 3] = 0x7f;
        return temp_cnt;  //返回一个数据包的字节数
    #endif
    }
}


extern float vision_yaw_perdict;
extern float test_abs_speed_kp;
extern float test_angle_error;

//函数说明：上位机通过串口打印数据波形
//附加说明：直接在主函数中调用此函数
void DataWave(UART_HandleTypeDef* huart)
{
    /* Channel：选择通道（1-10）,传入单精度浮点数 */
    {
    /* 拨盘 */
//    DataScope_Get_Channel_Data((float) (test_trigger.shoot_wave_single), 1 );
//    DataScope_Get_Channel_Data((float) (test_trigger.shoot_wave_circle), 2 );

//    DataScope_Get_Channel_Data((float) (pid_trigger_ecd.pos_out), 1 );
//    DataScope_Get_Channel_Data((float) (pid_trigger_spd.pos_out), 2 );
//    DataScope_Get_Channel_Data((float) (pid_trigger_spd.iout), 3 );

//    DataScope_Get_Channel_Data((float) (pid_trigger_ecd.err[0]), 1 );
//    DataScope_Get_Channel_Data((float) (pid_trigger_spd.err[0]), 2 );
    /* 云台 PID调参 */
//    DataScope_Get_Channel_Data((float) gimbal.pid.pit_ecd_ref, 1 );
//    DataScope_Get_Channel_Data((float) gimbal.pid.pit_ecd_fdb, 2 );
//    DataScope_Get_Channel_Data((float) pid_pit_ecd.pout, 3 );
//    DataScope_Get_Channel_Data((float) pid_pit_ecd.dout, 4);
//    DataScope_Get_Channel_Data((float) gimbal.pid.pit_spd_ref, 5 );
//    DataScope_Get_Channel_Data((float) gimbal.pid.pit_spd_fdb, 6 );
//    DataScope_Get_Channel_Data((float) pid_pit_spd.pout, 7 );
//    DataScope_Get_Channel_Data((float) pid_pit_spd.iout, 8 );

//    DataScope_Get_Channel_Data((float) gimbal.pid.pit_ecd_ref, 1 );
//    DataScope_Get_Channel_Data((float) gimbal.pid.pit_ecd_fdb, 2 );
//    DataScope_Get_Channel_Data((float) gimbal.pid.pit_spd_ref, 3 );
//    DataScope_Get_Channel_Data((float) gimbal.pid.pit_spd_fdb, 4 );
//    DataScope_Get_Channel_Data((float) gimbal.pid.yaw_mecd_ref, 1 );
//    DataScope_Get_Channel_Data((float) gimbal.pid.yaw_mecd_fdb, 2 );
//    DataScope_Get_Channel_Data((float) gimbal.pid.yaw_mspd_ref, 3 );
//    DataScope_Get_Channel_Data((float) gimbal.pid.yaw_mspd_fdb, 4 );
    
    /* 波形生成测试 */
//    DataScope_Get_Channel_Data((float) test_sin.out, 1 );
//    DataScope_Get_Channel_Data((float) test_cos.out, 2 );
//    DataScope_Get_Channel_Data((float) test_sqr.out, 3 );
    /* 视觉 */
//    DataScope_Get_Channel_Data((float) vision.yaw.kal.imu_speed/test_abs_speed_kp, 1 );  // +-350范围，俯视逆时针为正
//    DataScope_Get_Channel_Data((float)(vision.yaw.kal.aim_speed[NOW]), 2 );  // (vision.yaw.aim_speed)
//    DataScope_Get_Channel_Data((float) vision.yaw.kal.abs_speed[NOW], 3 );
//    DataScope_Get_Channel_Data((float) vision.tof[NOW], 1 );
//    DataScope_Get_Channel_Data((float) vision.kal_tof, 2 );
//    DataScope_Get_Channel_Data((float) vision.yaw.kal.angle_error, 3 );

    /* 陀螺仪 */
    DataScope_Get_Channel_Data((float) imu_data.pitch, 1 );
    DataScope_Get_Channel_Data((float) imu_data.yaw, 2 );
    DataScope_Get_Channel_Data((float) 0.0f, 3 );
    }
    CK.Send_Count = DataScope_Data_Generate(3);//串口需要发送的数据个数

    for( CK.DataCnt = 0 ; CK.DataCnt < CK.Send_Count; CK.DataCnt++)
    {
        while((huart->Instance->SR&0X40)==0);
        huart->Instance->DR = CK.DataScope_OutPut_Buffer[CK.DataCnt];
    }
}

