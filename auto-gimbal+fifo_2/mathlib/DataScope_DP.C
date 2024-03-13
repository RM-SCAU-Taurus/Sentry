/*******************************************************************************
 * @file    DataScope_DP.c
 * @author  ZZJ
 * @date    2022.1.31
 * @brief   MiniBalance 和 VOFA+ 串口上位机驱动程序
 *          使用方法：
 *              0. 默认使用F4系列，否则需要修改.h文件中包含的头文件
 *              1. 在 DataScope_DP.h 选择使用的上位机，根据需要修改示波器通道数
 *                 其中，若使用 MiniBalance，则最大通道数默认10
 *              2. 在“Private includes”区， 包含外部变量声明头文件
 *              3. 在“Private variables”区，声明外部变量
 *              4. 在“Private user code”区，注册用户变量,如：
 *                  DataScope_Get_Channel_Data(JY901_data.Gyro_a_x);
 *                  DataScope_Get_Channel_Data(JY901_data.Gyro_a_y);
 *                  DataScope_Get_Channel_Data(JY901_data.Gyro_a_z);
 *              5. 周期执行 DataWave(UART_HandleTypeDef* huart)
 *                 函数即可一次性将注册数据当前值顺序发出
 *                  
 * @attention   只能修改.h文件中的宏定义和此文件中 BEGIN 与 END 之间的代码
 *              注册变量顺序决定上位机中的通道顺序
 * @note
 *          若使用 MiniBalance 上位机：或需要自行分频发送，避免卡机
 *          若使用 VOFA+ 上位机：上位机中需要使用 Justfloat 协议
 *******************************************************************************
 */
#include "DataScope_DP.h"
#include "gimbal_task.h"
#include "chassis_task.h"
/* 定义示波器通道数 */
#ifndef MINIBALANCE
    #define DATASCOPE_MAX_SEND_NUM MAX_SEND_NUM //使用用户定义的最大通道数
#else
    #define DATASCOPE_MAX_SEND_NUM 10 //默认10个通道
#endif

/* 定义绝对值函数 */
#ifndef ABS
    #define ABS(x)		((x>0)? (x): (-(x)))
#endif

/* 示波器数据结构体 */
struct _DataTypedfef_t
{
    unsigned char OutPut_Buffer[4*DATASCOPE_MAX_SEND_NUM+4];//串口发送缓冲区
    unsigned char Send_Count;//串口需要发送的数据个数
    unsigned char Data_Num;//变量数量
} CK;

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
unsigned char DataScope_Data_Generate(unsigned char Channel_Number)
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bsp_powerlimit.h"
#include "gimbal_task.h"
#include "pid.h"
#include "chassis_task.h"
#include "bsp_T_imu.h"
#include "shoot_task.h"
#include "bsp_TriggerMotor.h"
#include "protocol_camp.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
extern float vision_yaw_perdict;
extern float* last_imu_yaw_data;
/* USER CODE END PV */


//函数说明：上位机通过串口打印数据波形
//附加说明：周期调用此函数
void DataWave(UART_HandleTypeDef* huart)
{
	/*相位匹配*/
	
/* Private user code ---------------------------------------------------------*/
    /* USER CODE BEGIN 0 */
    /* 拨盘 */
//    DataScope_Get_Channel_Data((float) (pid_trigger_ecd.err[0]), 1 );
//    DataScope_Get_Channel_Data((float) (pid_trigger_spd.err[0]), 2 );
    /* 云台 PID调参 */
//    DataScope_Get_Channel_Data(gimbal.pid.pit_ecd_ref);
//    DataScope_Get_Channel_Data(gimbal.pid.pit_ecd_fdb);
//    DataScope_Get_Channel_Data(gimbal.pid.pit_spd_ref);
//    DataScope_Get_Channel_Data(gimbal.pid.pit_spd_fdb);
//    DataScope_Get_Channel_Data(pid_pit_spd.iout);
        
	DataScope_Get_Channel_Data(chassis.spd_input.vx);
	DataScope_Get_Channel_Data(chassis.spd_input.vy);
	DataScope_Get_Channel_Data(moto_yaw.ecd);
	
//    DataScope_Get_Channel_Data(gimbal.pid.yaw_angle_ref);
//    DataScope_Get_Channel_Data(gimbal.pid.yaw_angle_fdb);
//    DataScope_Get_Channel_Data(gimbal.pid.yaw_spd_ref);
//    DataScope_Get_Channel_Data(gimbal.pid.yaw_spd_fdb);

//    DataScope_Get_Channel_Data((float) gimbal.pid.yaw_mecd_ref, 1 );
//    DataScope_Get_Channel_Data((float) gimbal.pid.yaw_mecd_fdb, 2 );
//    DataScope_Get_Channel_Data((float) gimbal.pid.yaw_mspd_ref, 3 );
//    DataScope_Get_Channel_Data((float) gimbal.pid.yaw_mspd_fdb, 4 );
    
    /* 波形生成测试 */
//    DataScope_Get_Channel_Data((float) test_sin.out, 1 );
//    DataScope_Get_Channel_Data((float) test_cos.out, 2 );
//    DataScope_Get_Channel_Data((float) test_sqr.out, 3 );
    /* 视觉 */
//    DataScope_Get_Channel_Data((float) vision.yaw.angle_error[1]);  // +-350范围，俯视逆时针为正
//    DataScope_Get_Channel_Data((float)vision.yaw.kal.angle_error);  // (vision.yaw.aim_speed)
//    DataScope_Get_Channel_Data((float) vision.yaw.kal.abs_speed[NOW]);
//		DataScope_Get_Channel_Data((float) vision.yaw.abs_speed);
//    DataScope_Get_Channel_Data(vision.predict_angle);

//        DataScope_Get_Channel_Data(gimbal.pid.yaw_angle_ref);
//        DataScope_Get_Channel_Data(gimbal.pid.yaw_angle_fdb);
//        DataScope_Get_Channel_Data(gimbal.pid.yaw_spd_ref);
//        DataScope_Get_Channel_Data(gimbal.pid.yaw_spd_fdb);
//				DataScope_Get_Channel_Data(vision_ctrl.gimbal_yaw_cmd);
//        DataScope_Get_Channel_Data(vision.yaw.kal.abs_speed[NOW]);
//        DataScope_Get_Channel_Data(vision.distance);
//        DataScope_Get_Channel_Data(vision.kal_distance);
//        DataScope_Get_Channel_Data(vision.dr);
//        DataScope_Get_Channel_Data(vision.tof[1]);
//        DataScope_Get_Channel_Data(vision.kal_tof);

//        DataScope_Get_Channel_Data(vision.yaw.kal.abs_speed[NOW] * vision.distance * 1.0e-3f * (2*PI)/360.0f);
//        DataScope_Get_Channel_Data(vision.dr);
            
//        DataScope_Get_Channel_Data();
//        DataScope_Get_Channel_Data(vision.temp_line);
//        DataScope_Get_Channel_Data(vision.kal_distance/1000.0f);
//        DataScope_Get_Channel_Data(vision.predict_temp_line);
//        DataScope_Get_Channel_Data(vision.aim_2D_speed);
//        DataScope_Get_Channel_Data(vision.predict_distance);
//        DataScope_Get_Channel_Data(vision.aim_2D_spd_arc);
//        DataScope_Get_Channel_Data(vision.predict_angle);
//        DataScope_Get_Channel_Data(vision.kal_predict_angle);

    /* 陀螺仪 */
//    DataScope_Get_Channel_Data(imu_data.pitch);
//    DataScope_Get_Channel_Data(imu_data.yaw);
//    DataScope_Get_Channel_Data(imu_data.wy);
//    DataScope_Get_Channel_Data(imu_data.wz);

    /* Z轴云台 */
//    DataScope_Get_Channel_Data((float) (zgim.pid_param.spd_ref), 1 );
//    DataScope_Get_Channel_Data((float) (zgim.pid_param.spd_fdb), 2 );
//    DataScope_Get_Channel_Data((float) (zgim.pid_param.ecd_ref), 3 );
//    DataScope_Get_Channel_Data((float) (zgim.pid_param.ecd_fdb), 4 );
//    DataScope_Get_Channel_Data((float) (zgim.spd_pid.iout), 5 );

    /* z轴云台陀螺仪数据 */
//    DataScope_Get_Channel_Data(JY901_data.Gyro_a_x);
//    DataScope_Get_Channel_Data(JY901_data.Gyro_a_y);
//    DataScope_Get_Channel_Data(JY901_data.Gyro_a_z);
//    
//    DataScope_Get_Channel_Data(JY901_data.Gyro_x);
//    DataScope_Get_Channel_Data(JY901_data.Gyro_y);
//    DataScope_Get_Channel_Data(JY901_data.Gyro_z);
//    
//    DataScope_Get_Channel_Data(JY901_data.pitch);
//    DataScope_Get_Channel_Data(JY901_data.roll);
//    DataScope_Get_Channel_Data(JY901_data.yaw);


//    DataScope_Get_Channel_Data(imu_data.yaw);
//    DataScope_Get_Channel_Data(*last_imu_yaw_data);
    /* USER CODE END */

    CK.Send_Count = DataScope_Data_Generate(CK.Data_Num);
    for( uint8_t cnt = 0; cnt < CK.Send_Count; cnt++)
    {
        while((huart->Instance->SR&0X40)==0);
        huart->Instance->DR = CK.OutPut_Buffer[cnt];
    }
    CK.Data_Num=0;
    CK.Send_Count = 0;
}
