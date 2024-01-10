#include "status_task.h"
#include "tim.h"
#include "DataScope_DP.h"
#include "usart.h"
#include "bsp_T_imu.h"
#include "stdbool.h"
#include "remote_msg.h"
#include "modeswitch_task.h"
#include "iwdg.h"
#include "bsp_can.h"
status_t status;
int led_task_size;
static void status_deinit(void);
static void status_restore(void);

/**
  * @brief status_task
  * @param
  * @attention
	* @note
  */
void status_task(void const *argu)
{
    for(;;)
    {
        static uint16_t cnt = 0;
        static uint8_t led_status = 0;
        
        taskENTER_CRITICAL();
        //HAL_IWDG_Refresh(&hiwdg);
	      led_task_size = xPortGetFreeHeapSize();
        rc.init_status = rc_FSM(status.rc_status);  //更新遥控器的初始状态
        
        /* 遥控器通信状态检查 */
        //遥控通信周期大约14ms，此任务，每100ms检查并清除一次中断标志
        cnt++;
			    if( gimbal_status)
        {
            status.rc_status = 1;  //系统状态标志置1，供系统模式切换任务中检查
            rc_normal_flag = 0;  //清除遥控器串口中断标志
            
            /* LED 状态显示 */
            if( cnt == 3 )
            {
             if( status.chassis_status[0])
                    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, (GPIO_PinState)led_status);
                else
                   HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_SET);
						 if(  status.chassis_status[1]  )
                    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, (GPIO_PinState)led_status);
                else
                    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_SET);
						if(status.chassis_status[2])
                    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, (GPIO_PinState)led_status);
                else
                    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, GPIO_PIN_SET);
						if( status.chassis_status[3] )
                    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, (GPIO_PinState)led_status);
                else
                    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, GPIO_PIN_SET);
						}
        }
        else  //云台can1失联
        {
            status.rc_status = 0;  //遥控状态标志清0
            if(cnt >= 2)
            {
                led_status = !led_status;
                cnt = 0;
                
                if( led_status )
                    status_init();  //LED全部熄灭
                else
                    status_deinit();  //全部亮起
            }
        }
        /* 复位 */
        status_restore();
        taskEXIT_CRITICAL();
        osDelay(100);
    }
}

void status_init(void)  //灭灯
{
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8,  GPIO_PIN_SET);  //LED_D
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9,  GPIO_PIN_SET);  //LED_C
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_SET);  //LED_B
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_SET);  //LED_A
}

static void status_deinit(void)  //亮灯
{
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8,  GPIO_PIN_RESET);  //LED_D
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9,  GPIO_PIN_RESET);  //LED_C
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_RESET);  //LED_B
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_RESET);  //LED_A
}

static void status_restore(void)  //清除标志位
{
    status.power_control = 0;
    for(int i = 0; i<2; i++)
        status.gyro_status[i] = 0;
    for(int i = 0; i<4; i++)
        status.chassis_status[i] = 0;
    for(int i = 0; i<3; i++)
        status.gimbal_status[i] = 0;
		gimbal_status=0;
}
