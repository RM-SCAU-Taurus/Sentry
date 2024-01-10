/**
  * @file bsp_vision.h
  * @version 1.0
  * @date Feb,23th 2021
	*
  * @brief  �Ӿ���Ϣ����
	*
  *	@author
  *
  */
#ifndef	__BSP_VISION_H__
#define __BSP_VISION_H__

#include "stdint.h"

typedef struct
{
    float angle_error[2];	//�Ƕ����
    float aim_speed[2];			//�з�����ٶ�
    float aim_acc;              //�з����ٶ�
    float abs_speed;			//�з������ٶ�
    float predict;				//����Ԥ����
    struct
    {
        float angle_error;
        float aim_speed[2];
        float abs_speed[2];
        float imu_speed;
    } kal;								//�������˲����
} vision_gimbal_t;

typedef struct
{
    vision_gimbal_t  pit;   //PIT������Ӿ���Ϣ
    vision_gimbal_t  yaw;	//YAW������Ӿ���Ϣ

    float  		distance;   //�з�����
    float       tof[2];     //�ӵ�����ʱ��
    float       kal_tof;
    uint16_t	period;     //�Ӿ���������
    char  		cnt;        //�Լ�λ(��֤�Ӿ���Ϣ�ڳ����仯)
    char  		eof;        //֡β
    uint8_t     aim_flag;   //���б�־λ ����Ŀ����1
} vision_msg_t;

typedef enum
{
    NOW = 0,
    LAST = 1,
    LLAST = 2
} time_rank_e;

extern vision_msg_t vision;

void vision_data_handler(uint8_t *Vision_Data);

#endif

