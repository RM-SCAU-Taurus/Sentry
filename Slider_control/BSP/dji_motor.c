#include "dji_motor.h"

/**
  * @brief     get motor initialize offset value
  * @param     ptr: Pointer to a moto_measure_t structure
  * @retval    None
  * @attention this function should be called after system can init
  */
void get_moto_offset(moto_measure_t* ptr, uint8_t* CAN_Rx_data)
{
    ptr->ecd        = (uint16_t)(CAN_Rx_data[0] << 8 | CAN_Rx_data[1]);
    ptr->offset_ecd = ptr->ecd;
}

/**
  * @brief     get motor rpm and calculate motor round_count/total_encoder/total_angle
  * @param     ptr: Pointer to a moto_measure_t structure
  * @attention this function should be called after get_moto_offset() function
  */
void encoder_data_handler(moto_measure_t* ptr, uint8_t* CAN_Rx_data)
{
    //转子转速
    ptr->speed_rpm     = (int16_t)(CAN_Rx_data[2] << 8 | CAN_Rx_data[3]);
    ptr->given_current = (int16_t)(CAN_Rx_data[4] << 8 | CAN_Rx_data[5]);

    //机械角度
    ptr->last_ecd = ptr->ecd;
    ptr->ecd      = (uint16_t)(CAN_Rx_data[0] << 8 | CAN_Rx_data[1]);

    //相对开机后的角度
    if (ptr->ecd - ptr->last_ecd > 4096)
        ptr->round_cnt--;
    else if (ptr->ecd - ptr->last_ecd < -4096)
        ptr->round_cnt++;

    ptr->total_ecd = ptr->round_cnt * 8192 + ptr->ecd - ptr->offset_ecd;
}
