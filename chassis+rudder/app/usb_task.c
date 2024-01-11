/*
 * @Author: your name
 * @Date: 2021-12-19 14:37:59
 * @LastEditTime: 2022-01-01 23:59:52
 * @LastEditors: your name
 * @Description: 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 * @FilePath: \MDK-ARMd:\RM\auto-Infantry\rm-Infantry-20211026\app\usb_task.c
 */
/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       usb_task.c/h
  * @brief      usb outputs the error message.usb输出错误信息
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Nov-11-2019     RM              1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
#include "usb_task.h"

#include "cmsis_os.h"

#include "usb_device.h"
#include "usbd_cdc_if.h"
#include <stdio.h>
#include <stdarg.h>
#include "string.h"

#include "CRC8_CRC16.h"

#include "fifo.h"


//static void usb_printf(const char *fmt,...);

static uint8_t usb_buf[128];
//const error_t *error_list_usb_local;

extern QueueHandle_t CDC_send_queue;

extern void rm_dequeue_send_data(void* buf,uint16_t len);

void usb_task(void const * argument)
{
    MX_USB_DEVICE_Init();
    //error_list_usb_local = get_error_list_point();
    while(1)
    {
    if(xQueueReceive( CDC_send_queue, usb_buf, 10 ) == pdTRUE)
        {
            
					rm_dequeue_send_data(usb_buf,128);
					//CDC_Transmit_FS(usb_buf,128 );
        }
					osDelay(2);
    }

}

//static void usb_printf(const char *fmt,...)
//{
//    static va_list ap;
//    uint16_t len = 0;

//    va_start(ap, fmt);

//    len = vsprintf((char *)usb_buf, fmt, ap);

//    va_end(ap);


//    CDC_Transmit_FS(usb_buf, len);
//}
