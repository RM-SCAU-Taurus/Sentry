/**
 *  @file   
 *	@author          
 *	@brief           
 *	@version         
 *	@date            
 *	@param          
 *	@return          
 *	@exception      
 *	@warning         
 *	@remarks         
 *	@note           
 */



#ifndef __UART_DECODE_TASK_H__
#define __UART_DECODE_TASK_H__

#include "bsp_usart.h"
#include "fifo.h"

#define been_written 1
#define Data_processing_completed 0
void uart_decode_task(void const *argu);

extern volatile int uartDecodeSignal;
extern fifo_s_t DBUS_fifo;
<<<<<<< HEAD
extern fifo_s_t JUDGE_fifo;
extern uint8_t DBUS_fifo_buf[3*DMA_DBUS_LEN];
=======
extern uint8_t DBUS_fifo_buf[4*DMA_DBUS_LEN];
>>>>>>> parent of 51913b3 (又改了一版dma)
#endif
