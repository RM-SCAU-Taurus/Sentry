#ifndef __BSP_MF_MOTOR_H__
#define __BSP_MF_MOTOR_H__

#include "stdint.h"


typedef struct
{
    uint8_t Command_byte;//命令字节
    uint16_t encoder;
    uint16_t last_ecd;//编码器数值
    int16_t number_of_turns;
    int16_t angle_changed;
    int64_t accumulated_angle;//叠加角度
    int  temperature_real;
    int16_t  iq;//转矩电流值
    int32_t  acceleration;//加速度
    float  RPM;//角速度
    float  wspeed;//角速度
    float  vspeed;//角速度
    float  powercontrol;//开环功率控制
    float  torque_current;//current
    float  torque;//扭矩控制
    int32_t  Anti_skid_speed;//加速度


    float total_position;
} moto_mf_t;

void encoder_data_receive(moto_mf_t* ptr, uint8_t * CAN_Rx_data);


#endif
