/**
	* @file bsp_T_imu.c
	* @version 1.0
	* @date 2020.1.4
  *
  * @brief  Taurus陀螺仪内测版
  *
  *	@author YY
  *
  */

#include "bsp_T_imu.h"
#include "bsp_can.h"
#include "gimbal_task.h"
#include "KalmanFilter.h"
#include "usart.h"
#include "status_task.h"
#include "math_calcu.h"

float 	palstance_buffer[2];
float 	angle_buffer[2];
float 	imu_9025_buffer[2];
float imu9025_array[5] = {0};
float clean_wz;
Taurus_imu_data_t   imu_data;

Taurus_imu_data_t   imu_9025;
void T_imu_calcu(uint32_t can_id,uint8_t * CAN_Rx_data)
{
    switch(can_id)
    {
    case TIMU_PALSTANCE_ID:	//角速度
    {
        memcpy(palstance_buffer,CAN_Rx_data,8);
        imu_data.wy = palstance_buffer[0];  //当陀螺仪水平旋转180度时需要加-号
        imu_data.wz = palstance_buffer[1];
    }
    break;
    case TIMU_ANGLE_ID:			//角度
    {
        memcpy(angle_buffer,CAN_Rx_data,8);
        imu_data.pitch = angle_buffer[0];  //当陀螺仪水平旋转180度时需要加-号
        imu_data.yaw   = angle_buffer[1];
    }
    break;
		    case TIMU_9025_ID:			//角度
    {
        memcpy(imu_9025_buffer,CAN_Rx_data,8);
        imu_9025.wz = imu_9025_buffer[0];  //当陀螺仪水平旋转180度时需要加-号
        imu_9025.yaw   = imu_9025_buffer[1];
				clean_wz = GildeAverageValueFilter(imu_9025.wz, imu9025_array);
			
    }
    break;
    }
}
