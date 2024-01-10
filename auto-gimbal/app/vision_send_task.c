#include "gimbal_task.h"
#include "chassis_task.h"
#include "status_task.h"
#include "comm_task.h"
#include "modeswitch_task.h"
#include "remote_msg.h"
#include "pid.h"
#include "bsp_can.h"
#include "control_def.h"
#include "bsp_TriggerMotor.h"
#include "remote_msg.h"
#include "math_calcu.h"
#include "bsp_T_imu.h"
#include "KalmanFilter.h"
#include "usart.h"
#include "func_generator.h"
#include "usb_task.h"
#include "protocol_camp.h"
#include "DataScope_DP.h"
#include "usart.h"
#include "bsp_can.h"
#include "bsp_JY901.h"
#include "bsp_T_imu.h"
#include "ubf.h"
#include "us_tim.h"
#include "vision_predict.h"
#include "vision_send_task.h"
extern chassis_odom_info_t chassis_odom;

void vision_send_task(void const *argu) {
    uint32_t thread_wake_time = osKernelSysTick();
    for(;;) {
        taskENTER_CRITICAL();
        
			/*采样陀螺仪数据*/
		     gim_msg_t temp_gim_msg = {
            .pit = moto_pit.ecd, //电控侧PIT角度相位匹配要用编码值
            .yaw = imu_data.yaw,
            .wy  = imu_data.wy,
            .wz  = imu_data.wz,
        };
        ubf_push(gim_msg_ubf, &temp_gim_msg);//相位匹配入列
				/*视觉云台信息*/
				chassis_odom.gimbal_pitch_imu    = imu_data.pitch;           //单位：° 视觉侧PIT角度重力补偿要用IMU
        chassis_odom.gimbal_yaw_imu     = imu_data.yaw;             //单位：°
        chassis_odom.gimbal_pitch_rate_imu  = -imu_data.wy / 16.3835f;  //单位：°/s
        chassis_odom.gimbal_yaw_rate_imu = imu_data.wz / 16.3835f;   //单位：°/s
        taskEXIT_CRITICAL();
        
        /* 绝对1ms发送周期 */
        osDelayUntil(&thread_wake_time, 1); 
    }
}
