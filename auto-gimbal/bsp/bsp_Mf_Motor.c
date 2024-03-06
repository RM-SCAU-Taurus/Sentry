 /////////////////////////////////////////////////////////////////////////////////
// 电机型号：瓴控 MF9025 V2
// 最大转速：3000 rpm 或 3000*2*pi/60 ≈ 314.159 rad/s
// 最大电流：±16.5A 对应控制值 ±2048
//
//
//
/////////////////////////////////////////////////////////////////////////////////

#include "bsp_Mf_Motor.h"
#include <stdio.h>
#include <stdlib.h>
#include "math_calcu.h"
/* 圆周率 */
#ifndef PI
#define PI 3.14159265358979323846f
#endif

#ifndef ABS
#define ABS(x)  ( (x)>0? (x): (-(x)) )
#endif

#define  WHEEL_RADIUS 0.120f

/* 定义中值滤波变量 */
uint8_t wspeed_n = 0;
float wspeed[7] = {0};


void encoder_data_receive(moto_mf_t* ptr, uint8_t * CAN_Rx_data)
{
		static float wspeed_rec;
    //命令字节
    ptr->Command_byte = CAN_Rx_data[0];
    //转子转速
    ptr->RPM = ((int16_t)(CAN_Rx_data[5] << 8 | CAN_Rx_data[4]))/1.0f;
//    ptr->wspeed     = ((int16_t)(CAN_Rx_data[5] << 8 | CAN_Rx_data[4]))*0.104f/1.0f;
	  wspeed_rec     = ((int16_t)(CAN_Rx_data[5] << 8 | CAN_Rx_data[4]))*0.104f/1.0f;

		ptr->wspeed = GildeAverageValueFilter(wspeed_rec, wspeed);
		
    ptr->vspeed = ptr->wspeed * WHEEL_RADIUS;
    //转矩电流
    ptr->powercontrol = (int16_t)(CAN_Rx_data[3] << 8 | CAN_Rx_data[2]);
    ptr->torque_current = (ptr->powercontrol / 2048.0f) * 16.5f;
    ptr->torque = ptr->torque_current*0.32f;
    //机械角度&累加圈数&累加角度
    ptr->last_ecd = ptr->encoder;
    ptr->encoder      = (((uint16_t)(CAN_Rx_data[7] << 8 | CAN_Rx_data[6]))/65535.0f)*360.0f;
    if(ptr->last_ecd != ptr->encoder)
    {
        ptr->angle_changed = ptr->encoder-ptr->last_ecd;
        if(ABS(ptr->angle_changed)>355.0f)
        {
            if(ptr->encoder>ptr->last_ecd)
            {
                ptr->accumulated_angle +=(ptr->angle_changed-360.0f);
            }
            if(ptr->encoder<ptr->last_ecd)
            {
                ptr->accumulated_angle +=(ptr->angle_changed+360.0f);
            }
        }
        else
        {
            ptr->accumulated_angle +=ptr->angle_changed;
        }
        ptr->number_of_turns = ptr->accumulated_angle/360.0f;
    }
    //温度
    ptr->temperature_real = CAN_Rx_data[1];

    //计算总位移（需要轮径）
    ptr->total_position += ptr->RPM * 2 * PI /60 * 0.002f * WHEEL_RADIUS;
}
