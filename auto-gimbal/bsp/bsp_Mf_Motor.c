 /////////////////////////////////////////////////////////////////////////////////
// ����ͺţ�겿� MF9025 V2
// ���ת�٣�3000 rpm �� 3000*2*pi/60 �� 314.159 rad/s
// ����������16.5A ��Ӧ����ֵ ��2048
//
//
//
/////////////////////////////////////////////////////////////////////////////////

#include "bsp_Mf_Motor.h"
#include <stdio.h>
#include <stdlib.h>
#include "math_calcu.h"
/* Բ���� */
#ifndef PI
#define PI 3.14159265358979323846f
#endif

#ifndef ABS
#define ABS(x)  ( (x)>0? (x): (-(x)) )
#endif

#define  WHEEL_RADIUS 0.120f

/* ������ֵ�˲����� */
uint8_t wspeed_n = 0;
float wspeed[7] = {0};


void encoder_data_receive(moto_mf_t* ptr, uint8_t * CAN_Rx_data)
{
		static float wspeed_rec;
    //�����ֽ�
    ptr->Command_byte = CAN_Rx_data[0];
    //ת��ת��
    ptr->RPM = ((int16_t)(CAN_Rx_data[5] << 8 | CAN_Rx_data[4]))/1.0f;
//    ptr->wspeed     = ((int16_t)(CAN_Rx_data[5] << 8 | CAN_Rx_data[4]))*0.104f/1.0f;
	  wspeed_rec     = ((int16_t)(CAN_Rx_data[5] << 8 | CAN_Rx_data[4]))*0.104f/1.0f;

		ptr->wspeed = GildeAverageValueFilter(wspeed_rec, wspeed);
		
    ptr->vspeed = ptr->wspeed * WHEEL_RADIUS;
    //ת�ص���
    ptr->powercontrol = (int16_t)(CAN_Rx_data[3] << 8 | CAN_Rx_data[2]);
    ptr->torque_current = (ptr->powercontrol / 2048.0f) * 16.5f;
    ptr->torque = ptr->torque_current*0.32f;
    //��е�Ƕ�&�ۼ�Ȧ��&�ۼӽǶ�
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
    //�¶�
    ptr->temperature_real = CAN_Rx_data[1];

    //������λ�ƣ���Ҫ�־���
    ptr->total_position += ptr->RPM * 2 * PI /60 * 0.002f * WHEEL_RADIUS;
}
