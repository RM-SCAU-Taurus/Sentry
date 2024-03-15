#ifndef __DJI_MOTOR_H__
#define __DJI_MOTOR_H__

#include "stdint.h"

typedef struct
{
    uint16_t ecd;
    uint16_t last_ecd;

    int16_t  speed_rpm;
    int16_t  given_current;

    int32_t  round_cnt;
    int32_t  total_ecd;

    uint16_t offset_ecd;
    uint32_t msg_cnt;
} moto_measure_t;



void get_moto_offset(moto_measure_t* ptr, uint8_t* CAN_Rx_data);
void encoder_data_handler(moto_measure_t* ptr, uint8_t* CAN_Rx_data);

#endif
