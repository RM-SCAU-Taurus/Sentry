#include "vision_task.h"
#include "bsp_can.h"
#include "string.h"
#include "gimbal_task.h"
#include "math.h"
#include "bsp_T_imu.h"

#define ABS(x)		((x>0)? (x): (-x))

vision_msg_t vision;
void vision_read_data(uint8_t * Vision_Data)	//串口读取视觉数据
{
    float vision_yaw_temp;
    float vision_pit_temp;
    static uint32_t mode_wake_time,last_mode_wake_time;
    /*------------------------接受视觉信息------------------------*/
    memcpy(&vision_yaw_temp, Vision_Data,4);
    vision.yaw.angle_error[4]=vision_yaw_temp;
    memcpy(&vision_pit_temp,(Vision_Data+4),4);
    vision.pit.angle_error[4]=vision_pit_temp - VISION_PIT_OFFSET;	//负数向上
    memcpy(&vision.distance,(Vision_Data+8),4);
    memcpy(&vision.cnt,(Vision_Data+12),1);
    memcpy(&vision.data_frame,(Vision_Data+13),1);

    mode_wake_time = HAL_GetTick();
    vision.period = mode_wake_time - last_mode_wake_time;
    last_mode_wake_time = mode_wake_time;			//获取视觉周期

    /*------------------------pit目标速度获取-------------------*/
    if(vision.pit.angle_error[4] && vision.pit.angle_error[3])
    {
        vision.pit.aim_speed = (vision.pit.angle_error[4] - vision.pit.angle_error[3]) / vision.period * 1000.0f;
    }
    else
    {
        vision.pit.aim_speed = 0 ;
    }

    /*------------------------yaw目标速度获取------------------------*/
    if(vision.yaw.angle_error[4] != 0 && vision.yaw.angle_error[3] != 0)
    {
        vision.yaw.aim_speed = (vision.yaw.angle_error[4] - vision.yaw.angle_error[3]) / vision.period * 1000.0f;
        if(ABS(vision.yaw.aim_speed) <= 60)
        {
            vision.yaw.kal.aim_speed_output = Kalman1Filter_calc(&kalman_aim_speed, vision.yaw.aim_speed);
            vision.yaw.abs_speed = imu_data.wz/29.375f + vision.yaw.kal.aim_speed_output;
        }
    }
    else
    {
        vision.yaw.aim_speed = 0;
        vision.yaw.kal.aim_speed_output=0;
        vision.yaw.abs_speed = 0;;
    }

    /*------------------------替换历史信息------------------------*/
    for(uint8_t i=0; i<4; i++)
    {
        vision.pit.angle_error[i] = vision.pit.angle_error[i+1];
        vision.yaw.angle_error[i] = vision.yaw.angle_error[i+1];
    }
}
