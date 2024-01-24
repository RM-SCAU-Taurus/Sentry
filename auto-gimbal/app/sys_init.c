
/**********C��*************************/

/**********Ӳ�������******************/

/**********�����**********************/
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"
#include "sys_init.h"
#include "chassis_task.h"
#include "gimbal_task.h"
#include "status_task.h"
#include "shoot_task.h"
#include "vision_predict.h"
#include "uart_decode.h"
#include "decode_camp.h"
/**********��ѧ��**********************/

/**********Ӳ�������******************/
#include "cmsis_os.h"
#include "can.h"
#include "crc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"
/**********���ݴ����******************/

/**********���Ͷ����******************/

/**********�弶֧�ֿ�******************/
#include "bsp_powerlimit.h"
#include "bsp_usart.h"
#include "bsp_can.h"
/**********�ⲿ��������****************/

/**********�ⲿ��������****************/

/**********��̬��������****************/

/**********�궨������******************/

/**********�ṹ�嶨��******************/

/**********��������********************/
fifo_s_t DBUS_fifo;						 // DBUS FIFO���ƽṹ��
uint8_t DBUS_fifo_buf[3 * DMA_DBUS_LEN]; // DBUS FIFO���λ�����

fifo_s_t JUDGE_fifo;						 // JUDGE FIFO���ƽṹ��
uint8_t JUDGE_fifo_buf[2 * DMA_JUDGE_LEN]; // JUDGE FIFO���λ�����
/**********���Ա�������****************/
unsigned portBASE_TYPE uxHighWaterMark_sys_init;


void sys_init_task(void const *argu)
{
	taskENTER_CRITICAL();
	can_device_init();
	chassis_init();
	gimbal_param_init();
	PowerControl_Init();
	USER_UART_Init();
	vsn_init();
	MX_USB_DEVICE_Init();
	usb_fifo_init();
	UART_fifo_init(&DBUS_fifo, DBUS_fifo_buf, 3 * DMA_DBUS_LEN);
	UART_fifo_init(&JUDGE_fifo, JUDGE_fifo_buf, 2 * DMA_JUDGE_LEN);
	HAL_Delay(1000);
	uxHighWaterMark_sys_init = uxTaskGetStackHighWaterMark(NULL);
	vTaskDelete(NULL);
	
	taskEXIT_CRITICAL();
}
