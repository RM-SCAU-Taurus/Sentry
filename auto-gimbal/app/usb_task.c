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
#include "gimbal_task.h"
#include "CRC8_CRC16.h"
#include "protocol_camp.h"
#include "fifo.h"


uint8_t usb_buf[128];
extern QueueHandle_t CDC_send_queue;
extern vision_ctrl_info_t  vision_ctrl;//自动步兵控制
extern void rm_dequeue_send_data(void* buf,uint16_t len);
extern void rm_queue_data(uint16_t cmd_id,void* buf,uint16_t len );
void usb_task(void const * argument)
{
    MX_USB_DEVICE_Init();
    while(1)
    {
    if(xQueueReceive( CDC_send_queue, usb_buf, 10 ) == pdTRUE)
        {
           
					rm_dequeue_send_data(usb_buf,128);
					//memset(usb_buf,0,sizeof(usb_buf));
        }
					osDelay(2);//2
    }

}
