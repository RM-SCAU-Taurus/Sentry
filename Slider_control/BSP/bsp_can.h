#ifndef __BSP_CAN_H__
#define __BSP_CAN_H__

#include "stdint.h"
#include "can.h"

void canx_init(CAN_HandleTypeDef * hcan, uint32_t  * id, void (*pFunc)(uint32_t,uint8_t*));

void can1_send_message(int16_t TX_ID, int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4);
void can2_send_message(int16_t TX_ID, int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4);
void can1_send_balance(int16_t TX_ID, int16_t iq1, int16_t iq2, uint8_t supercap_mode, float supercap_current);

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan);

#endif
