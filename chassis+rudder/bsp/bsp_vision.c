/**
  * @file bsp_vision.c
  * @version 1.0
  * @date Feb,23th 2021
	*
  * @brief  视觉信息解算
	*
  *	@author
  *
  */
#include "bsp_vision.h"
#include "control_def.h"
#include "remote_msg.h"
#include "bsp_can.h"
#include "string.h"
#include "math.h"
#include "remote_msg.h"
#include "bsp_T_imu.h"
#include "KalmanFilter.h"
#include "math_calcu.h"
#include "modeswitch_task.h"
#include "math.h"
#include "comm_task.h"

vision_msg_t vision;


float test_abs_speed_kp = 12.8f;
float test_angle_error = 0;
/**
  * @brief 视觉信息解算
  * @param
  * @attention
	* @note
  */
void vision_data_handler(uint8_t *Vision_Data)
{
    /* 局部全局变量 */
    static uint32_t vision_wake_time, last_vision_wake_time;
    
    /* 接收视觉信息 */
    memcpy(&vision.yaw.angle_error[NOW], Vision_Data, 4);
    memcpy(&vision.pit.angle_error[NOW],(Vision_Data+4), 4);
    memcpy(&vision.distance, (Vision_Data+8), 4);
    memcpy(&vision.tof[NOW], (Vision_Data+12), 4);
    memcpy(&vision.cnt, (Vision_Data+16), 1);
    memcpy(&vision.eof, (Vision_Data+17), 1);

    /* 符号统一：俯视逆时针为负 */
    vision.yaw.angle_error[NOW] = -vision.yaw.angle_error[NOW];

    /* 获取视觉运算周期 */
    vision_wake_time = HAL_GetTick();
    vision.period = vision_wake_time - last_vision_wake_time;
    last_vision_wake_time = vision_wake_time;
    
    if( FLAG_VISION_sENERGY != KEY_RUN && FLAG_VISION_bENERGY != KEY_RUN )
    {
        /* 数据处理 */
        //飞行时间
        if( vision.tof[NOW] >= 2.0f || vision.tof[NOW] <= -2.0f )
            vision.tof[NOW] = vision.tof[LAST];
        vision.kal_tof = Kalman1Filter_calc(&kalman_bullet_time, vision.tof[NOW]);
        
        //PIT轴角度
        if( vision.pit.angle_error[NOW] >= 40 || vision.pit.angle_error[NOW] <= -40)
            vision.pit.angle_error[NOW] = vision.pit.angle_error[LAST];
        
        //YAW轴角度
        if( vision.yaw.angle_error[NOW] >= 28 || vision.yaw.angle_error[NOW] <= -28)
            vision.yaw.angle_error[NOW] = vision.yaw.angle_error[LAST];
    }
    /* YAW轴敌方绝对速度 */
    if( vision.yaw.angle_error[LAST] && vision.yaw.angle_error[NOW] && vision.distance )  //前后两帧都识别到目标
    {
        vision.yaw.kal.imu_speed = Kalman1Filter_calc(&kalman_yaw_imu_speed, imu_data.wz);
        
        test_angle_error = vision.yaw.angle_error[NOW] - vision.yaw.angle_error[LAST];
        vision.yaw.aim_speed[NOW] = (vision.yaw.angle_error[NOW] - vision.yaw.angle_error[LAST]) / vision.period * 1000.0f;
        vision.yaw.kal.aim_speed[NOW] = Kalman1Filter_calc(&kalman_yaw_aim_speed, vision.yaw.aim_speed[NOW]);
        
        vision.yaw.abs_speed = vision.yaw.kal.aim_speed[NOW] + (vision.yaw.kal.imu_speed / test_abs_speed_kp);  //确保目标静止时，绝对速度逼近0
        vision.yaw.kal.abs_speed[NOW] = Kalman1Filter_calc(&kalman_yaw_abs_speed, vision.yaw.abs_speed);

        /* 视觉信息二阶导数 */
        vision.yaw.aim_acc = (vision.yaw.kal.abs_speed[NOW] - vision.yaw.kal.abs_speed[LAST]) / vision.period * 1000.0f;
//        if( ABS(vision.yaw.aim_acc) >= 500.0f )
//            vision.yaw.kal.abs_speed[NOW] = vision.yaw.kal.abs_speed[LAST];
//        vision.yaw.aim_acc = (vision.yaw.kal.aim_speed[NOW] - vision.yaw.kal.aim_speed[LAST]) / vision.period * 1000.0f;
    }
    else
    {
        vision.yaw.aim_speed[NOW] = 0;
        vision.yaw.kal.imu_speed = 0;
        vision.yaw.kal.aim_speed[NOW] = 0;
        vision.yaw.abs_speed = 0;
        vision.yaw.kal.abs_speed[NOW] = 0;
    }
    
    /* 替换历史信息 */
    vision.pit.angle_error[LAST] = vision.pit.angle_error[NOW];
    vision.yaw.angle_error[LAST] = vision.yaw.angle_error[NOW];
    
    vision.pit.aim_speed[LAST] = vision.pit.aim_speed[NOW];
    vision.yaw.aim_speed[LAST] = vision.yaw.aim_speed[NOW];
    
    vision.pit.kal.aim_speed[LAST] = vision.pit.kal.aim_speed[NOW];
    vision.yaw.kal.aim_speed[LAST] = vision.yaw.kal.aim_speed[NOW];
    
    vision.yaw.kal.abs_speed[LAST] = vision.yaw.kal.abs_speed[NOW];
    
    vision.tof[LAST] = vision.tof[NOW];
}
