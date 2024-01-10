#ifndef __VISION_TASK_H__
#define __VISION_TASK_H__

#include "stdint.h"
#include "KalmanFilter.h"

#define VISION_YAW_OFFSET		0.78f		//正数向右偏移
#define VISION_PIT_OFFSET		30.0f		//正数向下偏移

typedef struct
{
    float angle_input;
    float angle_output;
    float speed_input;
    float speed_output;
    float aim_speed_output;
    float abs_speed_output;
} kal_t;

typedef struct
{
    float angle_error[3];
    float aim_speed;
    float abs_speed;
    kal_t kal;
} vision_gimbal_t;

typedef struct
{
    vision_gimbal_t  yaw;
    vision_gimbal_t  pit;

    float distance;		//视觉回传敌方距离

    char 	lost_cnt;		//丢失目标自加位，判断丢帧
    int  	period;
    char  fire_flag;  //击打标志位
    char  cnt;				//自加位，保证视觉信息在持续变化
    char  data_frame;	//帧尾位
} vision_msg_t;


void vision_data_handler(uint32_t can_id,uint8_t * vision_data);
void vision_read_data(uint8_t * Vision_Data);

extern Kalman1_param_t kalman_pit;
extern vision_msg_t vision;
#endif
