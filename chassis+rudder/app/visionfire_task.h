/**
  * @file     visionfire_task.h
  * @version  v2.0
  * @date     July,6th 2019
	*
  * @brief    视觉通讯任务结构体定义
	*
  *	@author   Fatmouse
  *
  */
#ifndef __VISION_FIRE_TASK_H__
#define __VISION_FIRE_TASK_H__

#include "stm32f4xx_hal.h"

#define VISION_BUFLEN  8
#define VISION_MAX_LEN 8

#define DEBUG_BUFLEN  8
#define DEBUG_MAX_LEN 8

typedef enum
{
    lock_target_status = 0,
    lost_target_status,
} ctrl_vision_status;

typedef enum
{
    aim_mode = 0,
    little_energy_mode,
    macro_energy_mode,
} ctrl_vision_mode;

volatile typedef struct
{
    uint8_t vision_send_msg[8];
    int8_t vision_head;
    int16_t vision_pitch;
    int16_t vision_yaw;
    int16_t vision_distance;
    ctrl_vision_status vision_status;
    ctrl_vision_mode vision_mode;
    int8_t vision_check_last_last_last_last;
    int8_t vision_check_last_last_last;
    int8_t vision_check_last_last;
    int8_t vision_check_last;
    int8_t vision_check;
    float  imu_yaw;
    float  last_imu_yaw;
    float imu_yaw_err;
    uint32_t receive_space;
    uint8_t aim_flag;
    uint32_t aim_count;
} vision_message_t;

typedef struct
{
    float x;
    float y;
    float z;
    int time;
    int num;
} vis_data_typedef;

typedef struct
{
    float R;
    float Q;
    float x_last;
    float x_mid;
    float x_now;
    float p_last;
    float p_mid ;
    float p_now;
    float kg;
    float maxout;
} kalman_typedef;

extern vision_message_t vision_msg;
extern kalman_typedef kal_yaw;
extern kalman_typedef kal_wz ;
extern vis_data_typedef kalman_filter;//卡尔曼滤波输出
extern float kal_predict;
extern float kal_predict1;

void vision_fire_task(void const *argu);
void vision_message(uint8_t *pbuff);
float vis_yaw_filter(void);
void vision_mode_change(void);
void offset_table(int16_t distance);
float KalmanFilter(const float ResrcData,float ProcessNiose_Q,float MeasureNoise_R);
float KalmanFilter1(const float ResrcData,float ProcessNiose_Q,float MeasureNoise_R);
void kal_init(void);
#endif
