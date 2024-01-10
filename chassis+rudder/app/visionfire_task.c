#include "visionfire_task.h"
#include "cmsis_os.h"
#include "stdlib.h"
#include "pid.h"
#include "bsp_FricMotor.h"
#include "bsp_T_imu.h"
#include "remote_msg.h"
#include "usart.h"

#define VISION_HUART  huart3				//暂时定义，要改can传输
uint8_t vision_buf[10];							//暂时定义

vision_message_t vision_msg;
kalman_typedef kal_yaw ;
kalman_typedef kal_wz ;
float kal_predict;
float kal_predict1;
uint8_t vision_send_msg[8];
uint8_t C_key_up = 1;
uint8_t X_key_up = 1;
uint8_t Z_key_up = 1;
/**
  * @brief vision_fire_task
  * @param
  * @attention
  * @note
  */
void vision_fire_task(void const *argu)
{
    for(;;)
    {
        vision_message(vision_buf);
        //HAL_GPIO_TogglePin(GPIOD, LED_B_Pin);

        vision_mode_change();
        osDelay(10);
    }

}

void vision_message(uint8_t *pbuff)
{
    if(vision_msg.vision_check_last != pbuff[7])
    {
        if((pbuff[0] == 0xcc && vision_msg.vision_mode == aim_mode)||(pbuff[0] == 0xec && vision_msg.vision_mode == little_energy_mode)||(pbuff[0] == 0xdc && vision_msg.vision_mode == macro_energy_mode))
        {
            vision_msg.vision_head = pbuff[0];
            if((pbuff[1]&0x80)!=0)
            {
                vision_msg.vision_pitch =-((pbuff[1]&0x7f)*256+pbuff[2]);
            }
            else
            {
                vision_msg.vision_pitch =pbuff[1]*256+pbuff[2];
            }
            if((pbuff[3]&0x80)!=0)
            {
                vision_msg.vision_yaw=-((pbuff[3]&0x7f)*256+pbuff[4]);
            }
            else
            {
                vision_msg.vision_yaw=pbuff[3]*256+pbuff[4];
            }
            if((pbuff[5]&0x80)!=0)
            {
                vision_msg.vision_distance=-((pbuff[5]&0x7f)*256+pbuff[6]);
            }
            else
            {
                vision_msg.vision_distance=pbuff[5]*256+pbuff[6];
            }


            vision_msg.last_imu_yaw = vision_msg.imu_yaw;
            vision_msg.imu_yaw = imu_data.yaw;
            vision_msg.imu_yaw_err = vision_msg.imu_yaw - vision_msg.last_imu_yaw ;

        }
    }
    vision_msg.vision_check_last_last_last_last = vision_msg.vision_check_last_last_last;
    vision_msg.vision_check_last_last_last = vision_msg.vision_check_last_last;
    vision_msg.vision_check_last_last = vision_msg.vision_check_last;
    vision_msg.vision_check_last = vision_msg.vision_check;
    vision_msg.vision_check = pbuff[7];

    if(vision_msg.vision_check_last_last_last_last == vision_msg.vision_check)
    {
        vision_msg.vision_status = lost_target_status;
        vision_msg.vision_pitch = 0;
        vision_msg.vision_yaw = 0;
    }
    else
    {
        vision_msg.vision_status = lock_target_status;
    }
}

void vision_mode_change(void)
{
    vision_send_msg[0] = 0x66;
    if(rc.kb.bit.C && C_key_up)
    {
        C_key_up = 0;
        vision_msg.vision_mode = aim_mode;
        vision_send_msg[1] = 0x01;
        HAL_UART_Transmit_IT(&VISION_HUART,vision_send_msg,8);
    }
    else if(!rc.kb.bit.C)C_key_up = 1;
    if(rc.kb.bit.X && X_key_up)
    {
        X_key_up = 0;
        vision_msg.vision_mode = little_energy_mode;
        vision_send_msg[1] = 0x02;
        HAL_UART_Transmit_IT(&VISION_HUART,vision_send_msg,8);
    }
    else if(!rc.kb.bit.X)X_key_up = 1;
    if(rc.kb.bit.Z && Z_key_up)
    {
        Z_key_up = 0;
        vision_msg.vision_mode = macro_energy_mode;
        vision_send_msg[1] = 0x03;
        HAL_UART_Transmit_IT(&VISION_HUART,vision_send_msg,8);
    }
    else if(!rc.kb.bit.Z)Z_key_up = 1;
//			judge_send_mesg.show_in_client_data.data2 = vision_msg.vision_mode;
}

//	switch (vision_msg.vision_mode)
//	{
//    case aim_mode:
//		{
//			vision_send_msg[1] = 0x01;
//		}
//		break;
//		case little_energy_mode:
//		{
//			vision_send_msg[1] = 0x02;
//		}
//		break;
//		case macro_energy_mode:
//		{
//			vision_send_msg[1] = 0x03;
//		}
//		break;
//		default:
//		break;
//	}
//	HAL_UART_Transmit_IT(&VISION_HUART,vision_send_msg,8);


void offset_table(int16_t distance)
{

    if(distance > 60 && distance <= 120)
    {
        //pid_yaw_aim.p = 150;
        kal_yaw.maxout = 0.0f;
    }
    else if(distance > 120 && distance <= 180)
        kal_yaw.maxout = 1.0f;
    else if(distance > 180 && distance <= 240)
        kal_yaw.maxout = 3.0f;
    else if(distance > 240 && distance <= 300)
        kal_yaw.maxout = 3.5f;
    else if(distance > 300 && distance <= 360)
        kal_yaw.maxout = 4.0;
    else kal_yaw.maxout = 5.0;

}


/*-------------------------------------------------------------------------------------------------------------*/
/*
        Q:过程噪声，Q增大，动态响应变快，收敛稳定性变坏
        R:测量噪声，R增大，动态响应变慢，收敛稳定性变好
*/

/* 卡尔曼滤波处理 */

float KalmanFilter(const float ResrcData,float ProcessNiose_Q,float MeasureNoise_R)
{
    kal_yaw.x_mid=kal_yaw.x_last;                      				  //x_last=x(k-1|k-1),x_mid=x(k|k-1)
    kal_yaw.p_mid=kal_yaw.p_last+kal_yaw.Q;                     //p_mid=p(k|k-1),p_last=p(k-1|k-1),Q=噪声
    /*
     *  卡尔曼滤波的五个重要公式
     */
    kal_yaw.kg=kal_yaw.p_mid/(kal_yaw.p_mid+kal_yaw.R);                 //kg为kalman filter，R 为噪声
    kal_yaw.x_now=kal_yaw.x_mid+kal_yaw.kg*(ResrcData-kal_yaw.x_mid);   //估计出的最优值
    kal_yaw.p_now=(1-kal_yaw.kg)*kal_yaw.p_mid;													//最优值对应的covariance
    kal_yaw.p_last = kal_yaw.p_now;                     								//更新covariance 值
    kal_yaw.x_last = kal_yaw.x_now;                 										//更新系统状态值
    return kal_yaw.x_now;
}

float KalmanFilter1(const float ResrcData,float ProcessNiose_Q,float MeasureNoise_R)
{
    kal_wz.x_mid=kal_wz.x_last;                       				//x_last=x(k-1|k-1),x_mid=x(k|k-1)
    kal_wz.p_mid=kal_wz.p_last+kal_wz.Q;                      //p_mid=p(k|k-1),p_last=p(k-1|k-1),Q=噪声
    /*
     *  卡尔曼滤波的五个重要公式
     */
    kal_wz.kg=kal_wz.p_mid/(kal_wz.p_mid+kal_wz.R);                 //kg为kalman filter，R 为噪声
    kal_wz.x_now=kal_wz.x_mid+kal_wz.kg*(ResrcData-kal_wz.x_mid);   //估计出的最优值
    kal_wz.p_now=(1-kal_wz.kg)*kal_wz.p_mid;                 //最优值对应的covariance
    kal_wz.p_last = kal_wz.p_now;                     //更新covariance 值
    kal_wz.x_last = kal_wz.x_now;                 		//更新系统状态值
    return kal_wz.x_now;
}

void kal_init(void)
{
    vision_msg.vision_mode = aim_mode;//初始化默认自瞄模式；
    //vision_msg.vision_mode = little_energy_mode;

    kal_yaw.kg=1.0f;
    kal_yaw.p_last =0.0f;
    kal_yaw.p_mid = 0.0f;
    kal_yaw.p_now = 0.0f;
    kal_yaw.x_last = 0.0f;
    kal_yaw.x_mid = 0.0f;
    kal_yaw.x_now = 0.0f;
    kal_yaw.x_last = 0.0f;
    kal_yaw.Q = 1.0f;
    kal_yaw.R = 2000.0f;

    kal_wz.kg=1.0f;
    kal_wz.p_last =0.02f;
    kal_wz.p_mid = 0.0f;
    kal_wz.p_now = 0.0f;
    kal_wz.x_last = 0.0f;
    kal_wz.x_mid = 0.0f;
    kal_wz.x_now = 0.0f;
    kal_wz.x_last = 0.2f;
    kal_wz.Q = 0.001f;
    kal_wz.R = 0.542f;
}

