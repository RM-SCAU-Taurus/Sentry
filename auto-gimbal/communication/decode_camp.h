#ifndef REFEREE_H
#define REFEREE_H

//裁判信息解读与透传数据帧封装程序
//#include "bsp_remote_control.h"

#include "struct_typedef.h"

//USB接收FIFO初始化
void usb_fifo_init(void);
extern uint16_t referee_data_solve(uint8_t *frame);

#endif
