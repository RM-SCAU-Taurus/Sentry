#ifndef __SYS_INIT_H__
#define __SYS_INIT_H__

#include "bsp_usart.h"
#include "fifo.h"

void sys_init_task(void const *argu);
	
extern fifo_s_t DBUS_fifo;						 // DBUS FIFO���ƽṹ��
extern uint8_t DBUS_fifo_buf[3 * DMA_DBUS_LEN]; // DBUS FIFO���λ�����
extern fifo_s_t JUDGE_fifo;						 // JUDGE FIFO���ƽṹ��
extern uint8_t JUDGE_fifo_buf[2 * DMA_JUDGE_LEN];; // JUDGE FIFO���λ�����

#endif
