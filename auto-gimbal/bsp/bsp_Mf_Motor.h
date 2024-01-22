#ifndef __BSP_MF_MOTOR_H__
#define __BSP_MF_MOTOR_H__

#include "stdint.h"


typedef struct
{
    uint8_t Command_byte;//�����ֽ�
    uint16_t encoder;
    uint16_t last_ecd;//��������ֵ
    int16_t number_of_turns;
    int16_t angle_changed;
    int64_t accumulated_angle;//���ӽǶ�
    int  temperature_real;
    int16_t  iq;//ת�ص���ֵ
    int32_t  acceleration;//���ٶ�
    float  RPM;//���ٶ�
    float  wspeed;//���ٶ�
    float  vspeed;//���ٶ�
    float  powercontrol;//�������ʿ���
    float  torque_current;//current
    float  torque;//Ť�ؿ���
    int32_t  Anti_skid_speed;//���ٶ�


    float total_position;
} moto_mf_t;

void encoder_data_receive(moto_mf_t* ptr, uint8_t * CAN_Rx_data);


#endif
