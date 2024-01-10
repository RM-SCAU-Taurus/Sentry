#define __GIMBAL_TASK_GLOBALS
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
extern TaskHandle_t can_msg_send_task_t;

gimbal_t gimbal;
extern vision_ctrl_info_t  vision_ctrl;//自动步兵控制
extern chassis_odom_info_t chassis_odom;
 extern Game_Status_t Game_Status;
 ubf_t gim_msg_ubf;   /* 云台姿态历史数据 */
extern void rm_queue_data(uint16_t cmd_id,void* buf,uint16_t len );
static void gimbal_pid_calcu(void);
void gimbal_param_init(void)
{
    memset(&gimbal, 0, sizeof(gimbal_t));
    /* pit 轴 */
    PID_struct_init(&pid_pit_ecd, POSITION_PID, 8000, 0,
                    pid_pit_ecd_P, pid_pit_ecd_I, pid_pit_ecd_D);
    PID_struct_init(&pid_pit_spd, POSITION_PID, 28000,10000,
                   pid_pit_spd_P ,pid_pit_spd_I, pid_pit_spd_D);
    /* YAW 轴 */
    PID_struct_init(&pid_yaw_angle, POSITION_PID, 8000, 0,
                   15 0.0f, 0.0f, 0.0f);
    PID_struct_init(&pid_yaw_spd, POSITION_PID, 28000, 20000,
                    12.0f, 0.1f, 0.0f);

    /* 测试用 YAW PID参数 */
    PID_struct_init(&pid_yaw_mecd, POSITION_PID, 5000, 0,
                    pid_yaw_mecd_P, pid_yaw_mecd_I, pid_yaw_mecd_D);
    PID_struct_init(&pid_yaw_mspd, POSITION_PID, 28000, 20000,
                    pid_yaw_mspd_P,pid_yaw_mspd_I, pid_yaw_mspd_D);
    
    scale.ch1 = RC_CH1_SCALE;
    scale.ch2 = RC_CH2_SCALE;
}

/* ================================== TEST PARAM ================================== */
/* sin信号发生器 */
// 调试视觉绝对速度时用
// 注释底盘，调成位置环YAW
// 效果：自动摇头
FGT_sin_t test_s = 
{
    .Td = 1,
    .time = 0,
    .max = GIMBAL_YAW_CENTER_OFFSET + 800,
    .min = GIMBAL_YAW_CENTER_OFFSET - 800,
    .dc = GIMBAL_YAW_CENTER_OFFSET,
    .T = 800,
    .A = 250,
    .phi = 0,
    .out = 0
};
/* ================================== TEST PARAM ================================== */

extern vision_tx_msg_t vision_tx_msg;
/**
  * @brief gimbal_task
  */
int8_t choose_pid_flag=0, goback_flag=0;
void gimbal_task(void const *argu)
{
    uint32_t mode_wake_time = osKernelSysTick();
    for(;;)
    {
        taskENTER_CRITICAL();
        
        switch( ctrl_mode )
        {
            case PROTECT_MODE:
            {
                gimbal.pid.pit_ecd_ref   = GIMBAL_PIT_CENTER_OFFSET;  //云台默认俯仰水平
                gimbal.pid.yaw_angle_ref = imu_data.yaw;  //云台默认当前水平朝向
//							  goback_flag = 0;
							vision_ctrl.yaw = imu_data.yaw;
                for( uint8_t i=0; i<2; i++ )	gimbal.current[i] = 0;
            }    
            break;
            case REMOTER_MODE:
            {
                gimbal.pid.pit_ecd_ref   += rc.ch2 * scale.ch2;
                gimbal.pid.yaw_angle_ref += rc.ch1 * scale.ch1;
//							  goback_flag = 0;
							vision_ctrl.yaw = imu_data.yaw;
							  vision.status = vFIRST_LOST;
            }
            break;
            case KEYBOARD_MODE:
            {
                /* 由于云台与系统状态几乎重叠，仅多一个补给，故暂时不单设置云台状态机 */
                if( chassis.mode == CHASSIS_MODE_KEYBOARD_SUPPLY )
                {   
                    gimbal.pid.pit_ecd_ref = GIMBAL_PIT_CENTER_OFFSET;  //补给模式，头保持水平
                    gimbal.pid.yaw_angle_ref += rc.mouse.x *  KEYBOARD_SCALE_YAW_SUPPLY;
                }
                else
                {
                    gimbal.pid.pit_ecd_ref   += rc.mouse.y *  KEYBOARD_SCALE_PIT;
                    gimbal.pid.yaw_angle_ref += rc.mouse.x *  KEYBOARD_SCALE_YAW;
                }
            }
             case AUTO_MODE:
            {
//							  if(goback_flag == 0)
//								{
//									choose_pid_flag = 1;
//									gimbal.pid.yaw_mecd_ref = GIMBAL_YAW_CENTER_OFFSET;
//								}
//								if(fabs(moto_yaw.ecd - gimbal.pid.yaw_mecd_ref) < 5 && goback_flag == 0)
//								{
//									vision_ctrl.yaw = imu_data.yaw;
//									goback_flag = 1;
//									choose_pid_flag = 0;
//								}
							vsn_gimbal_ref_calc();	
							vsn_calc();			
                    
							
            }
            break;
            default:break;

        }
        /*发送云台状态数据给导航控制*/
			if(imu_data.pitch>180)
		 {
		     chassis_odom.gimbal_pitch_fdb =(imu_data.pitch-360)/57.3f;
		 }
		 else 
		 {
		    chassis_odom.gimbal_pitch_fdb= imu_data.pitch/57.3f;
		 }
		 chassis_odom.gimbal_yaw_fdb  =  moto_yaw.ecd*360/8191/57.3f; 
		 		 if(chassis_odom.gimbal_yaw_fdb>3.14f)
		 {
			     chassis_odom.gimbal_yaw_fdb  -=6.28f;  
		 }
		 else 
		 {
			     chassis_odom.gimbal_yaw_fdb = chassis_odom.gimbal_yaw_fdb;		    
		 }    

        /* 云台串级PID */
        gimbal_pid_calcu();
        memcpy(motor_cur.gimbal_cur, gimbal.current, sizeof(gimbal.current));
        osSignalSet(can_msg_send_task_t, GIMBAL_MOTOR_MSG_SEND);
//				DataWave(&huart3);
				taskEXIT_CRITICAL();
        osDelayUntil(&mode_wake_time, GIMBAL_PERIOD);
    }
}



/* 云台串级PID控制 */
void gimbal_pid_calcu(void)
{
    /*------------------------pit轴串级pid计算------------------------*/
    //位置反馈：编码器位置
    //速度反馈：陀螺仪速度
    gimbal.pid.pit_ecd_ref = data_limit(gimbal.pid.pit_ecd_ref,  31,-18);	//目标值限幅
    gimbal.pid.pit_ecd_fdb = imu_data.pitch;
    gimbal.pid.pit_ecd_err = gimbal.pid.pit_ecd_ref-gimbal.pid.pit_ecd_fdb;
    pid_calc(&pid_pit_ecd, gimbal.pid.pit_ecd_fdb, gimbal.pid.pit_ecd_fdb + gimbal.pid.pit_ecd_err);

    gimbal.pid.pit_spd_ref = pid_pit_ecd.pos_out;   //PID外环目标值
    gimbal.pid.pit_spd_fdb = imu_data.wy;			 //pit角速度反馈传进PID结构体
    pid_calc(&pid_pit_spd, gimbal.pid.pit_spd_fdb, gimbal.pid.pit_spd_ref);

    gimbal.current[1] = -pid_pit_spd.pos_out;
    
    
//	  if(choose_pid_flag == 0)
//		{
			/*------------------------yaw轴串级pid计算------------------------*/
      //位置反馈：陀螺仪角度
      //速度反馈：陀螺仪WZ
			if(gimbal.pid.yaw_angle_ref<0)            gimbal.pid.yaw_angle_ref += 360;
			else if(gimbal.pid.yaw_angle_ref>360)     gimbal.pid.yaw_angle_ref -= 360;	//目标值限幅
				 
			
			gimbal.pid.yaw_angle_fdb = imu_data.yaw;  //陀螺仪角度反馈
			gimbal.pid.yaw_angle_err = circle_error(gimbal.pid.yaw_angle_ref, gimbal.pid.yaw_angle_fdb, 360);
			pid_calc(&pid_yaw_angle, gimbal.pid.yaw_angle_fdb, gimbal.pid.yaw_angle_fdb + gimbal.pid.yaw_angle_err);

			gimbal.pid.yaw_spd_ref = pid_yaw_angle.pos_out;
			gimbal.pid.yaw_spd_fdb = imu_data.wz;  //陀螺仪速度反馈
			pid_calc(&pid_yaw_spd, gimbal.pid.yaw_spd_fdb, gimbal.pid.yaw_spd_ref);

			gimbal.current[0] = -pid_yaw_spd.pos_out;
//		}
//		else
//		{
//			//位置反馈：编码器位置
//			//速度反馈：陀螺仪WZ
//			//注意：测试发射器时用。使用时，需要注释掉底盘，保证编码值绝对，下方电流来源切换
//			if( gimbal.pid.yaw_mecd_ref >8191 )
//					gimbal.pid.yaw_mecd_ref -= 8191;
//			else if( gimbal.pid.yaw_mecd_ref < 0 )
//					gimbal.pid.yaw_mecd_ref += 8191;
//			gimbal.pid.yaw_mecd_fdb = moto_yaw.ecd;
//			gimbal.pid.yaw_mecd_err = circle_error(gimbal.pid.yaw_mecd_ref,gimbal.pid.yaw_mecd_fdb,8191);
//			pid_calc(&pid_yaw_mecd, gimbal.pid.yaw_mecd_fdb, gimbal.pid.yaw_mecd_fdb + gimbal.pid.yaw_mecd_err);

//			gimbal.pid.yaw_mspd_ref = pid_yaw_mecd.pos_out;
//			gimbal.pid.yaw_mspd_fdb = moto_yaw.speed_rpm;  //陀螺仪速度反馈
//			pid_calc(&pid_yaw_mspd, gimbal.pid.yaw_mspd_fdb, gimbal.pid.yaw_mspd_ref);
//			gimbal.current[0] = pid_yaw_mspd.pos_out;
//	  }
}

