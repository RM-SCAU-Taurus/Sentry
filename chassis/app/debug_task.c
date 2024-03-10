#include "debug_task.h"
#include "cmsis_os.h"
#include "usart.h"
#include "data_scope.h"
#include "status_task.h"

/* 待添加头文件 */
#include "chassis_task.h"
#include "math_calcu.h"
#include "bsp_powerlimit.h"

extern chassis_t chassis;

/* 用以选择打印的数据类型 */
uint8_t debug_wave = 3;

/* __weak void DataWavePkg(void)的实例：打包待发送数据 */
void DataWavePkg(void) {
    switch (debug_wave) {
        case 0: {
            DataScope_Get_Channel_Data(ABS(chassis.wheel_spd_fdb[0]));
            DataScope_Get_Channel_Data(ABS(chassis.wheel_spd_fdb[1]));
            DataScope_Get_Channel_Data(ABS(chassis.wheel_spd_fdb[2]));
            DataScope_Get_Channel_Data(ABS(chassis.wheel_spd_fdb[3]));
            break;
        }
        case 1: {
            DataScope_Get_Channel_Data(ABS(chassis.rudder_ecd_fdb[0]));
            DataScope_Get_Channel_Data(ABS(chassis.rudder_ecd_ref[0])); 
            break;
        }
		case 3:{
			DataScope_Get_Channel_Data(powercontrol.chassis_power);
			DataScope_Get_Channel_Data(powercontrol.power_buffer);
			DataScope_Get_Channel_Data(powercontrol.chassis_3508_power);			
			DataScope_Get_Channel_Data(supercap.volage);
			
            DataScope_Get_Channel_Data(ABS(chassis.wheel_spd_fdb[0]));
            DataScope_Get_Channel_Data(ABS(chassis.wheel_spd_fdb[1]));
            DataScope_Get_Channel_Data(ABS(chassis.wheel_spd_fdb[2]));
            DataScope_Get_Channel_Data(ABS(chassis.wheel_spd_fdb[3]));
		}
        default: {
            break;
        }
    }
}

/* 串口上位机数据发送任务 */
void debug_task(void const *argu) {
    uint32_t thread_wake_time = osKernelSysTick();
    for (;;) {
        taskENTER_CRITICAL();
        DataWave(&huart6);
        wdg_user_set_bit(WDG_BIT_TASK_DEBUG);
        taskEXIT_CRITICAL();
        osDelayUntil(&thread_wake_time, 2);
    }
}

