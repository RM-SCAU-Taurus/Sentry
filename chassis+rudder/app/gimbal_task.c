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
#include "bsp_vision.h"
#include "usb_task.h"
#include "protocol_camp.h"
extern TaskHandle_t can_msg_send_task_t;

gimbal_t gimbal;
extern vision_ctrl_info_t  vision_ctrl;//自动步兵控制
extern game_status_info_t game_state;
extern chassis_odom_info_t chassis_odom;
extern void rm_queue_data(uint16_t cmd_id,void* buf,uint16_t len );
static void gimbal_pid_calcu(void);
static void vision_autoaiming_calcu(void);
static void vision_energy_calcu(void);
static void vision_antirotate_calu(void);
static void vision_sentry_calcu(void);

void gimbal_param_init(void)
{
    memset(&gimbal, 0, sizeof(gimbal_t));
    /* pit 轴 */
    PID_struct_init(&pid_pit_ecd, POSITION_PID, 5500, 0,
                    pid_pit_ecd_P, pid_pit_ecd_I, pid_pit_ecd_D);
    PID_struct_init(&pid_pit_spd, POSITION_PID, 28000,20000,
                    pid_pit_spd_P, pid_pit_spd_I, pid_pit_spd_D);
    /* YAW 轴 */
    PID_struct_init(&pid_yaw_angle, POSITION_PID, 5000, 0,
                    pid_yaw_angle_P, pid_yaw_angle_I, pid_yaw_angle_D);
    PID_struct_init(&pid_yaw_spd, POSITION_PID, 28000, 20000,
                    pid_yaw_spd_P, pid_yaw_spd_I, pid_yaw_spd_D);

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
void gimbal_task(void const *argu)
{
    uint32_t mode_wake_time = osKernelSysTick();
    int  a;
    for(;;)
    {
        taskENTER_CRITICAL();

        switch( ctrl_mode )
        {
            case PROTECT_MODE:
            {
                gimbal.pid.pit_ecd_ref   = GIMBAL_PIT_CENTER_OFFSET;  //云台默认俯仰水平
                gimbal.pid.yaw_angle_ref = imu_data.yaw;  //云台默认当前水平朝向
                gimbal.pid.yaw_mecd_ref  = GIMBAL_YAW_CENTER_OFFSET;  //发射器测试时使用
                for( uint8_t i=0; i<2; i++ )	gimbal.current[i] = 0;
            }
            break;
            case REMOTER_MODE:
            {
//                if( ABS(vision.distance) > 1e-6f )
//                {
//                    vision_autoaiming_calcu();  //自瞄测试（注释掉下面三行代码即可）
//                }
//                else
//                {
//                    gimbal.pid.pit_ecd_ref   += rc.ch2 * scale.ch2;
//                    gimbal.pid.yaw_angle_ref += rc.ch1 * scale.ch1;
//                    gimbal.pid.yaw_mecd_ref  += rc.ch1 * (-0.008f);  //发射器测试时用
//                }
//                FGT_sin_cal(&test_s);//发射器测试时用
//                gimbal.pid.yaw_mecd_ref  = test_s.out;  //发射器测试时用
                
                gimbal.pid.pit_ecd_ref   += rc.ch2 * scale.ch2;
                gimbal.pid.yaw_angle_ref += rc.ch1 * scale.ch1;
                gimbal.pid.yaw_mecd_ref  += rc.ch1 * (-0.008f);  //发射器测试时用
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
            case VISION_MODE:
            {
                if( vision.distance )  //视觉模式下捕获到目标
                {
                    vision.aim_flag = 1;  //有目标，标志置1
                    
                    switch( vision_mode )
                    {
                        case VISION_MODE_AUTO:
                            vision_autoaiming_calcu();  //自瞄
                        break;
                        case VISION_MODE_bENERGY:
                        case VISION_MODE_sENERGY:
                            vision_energy_calcu();  //击打能量
                        break;
                        case VISION_MODE_ANTIROTATE:
                            vision_antirotate_calu();  //反小陀螺
                        break;
                        case VISION_MODE_SENTRY:
                            vision_sentry_calcu();  //打哨兵
                        break;   
                        default:break;
                    }
                }
                else if( vision.aim_flag == 1 )  //丢失目标后的第一帧  重置目标值 防止云台疯
                {
                    gimbal.pid.pit_ecd_ref   = gimbal.pid.pit_ecd_fdb;
                    gimbal.pid.yaw_angle_ref = gimbal.pid.yaw_angle_fdb;
                    vision.aim_flag = 0;
                }
                else
                {
                    gimbal.pid.pit_ecd_ref   += rc.mouse.y * KEYBOARD_SCALE_PIT;
                    gimbal.pid.yaw_angle_ref += rc.mouse.x * KEYBOARD_SCALE_YAW;
                }
            }break;
             case AUTO_MODE:
            {
                gimbal.pid.pit_ecd_ref   = vision_ctrl.gimbal_pitch_cmd;
                gimbal.pid.yaw_angle_ref = vision_ctrl.gimbal_yaw_cmd; 
            }
            break;
            default:break;

        }
        /*发送云台状态数据*/
        // rm_queue_data(GAME_STATUS_FDB_ID,&game_state,sizeof(game_state));
         
          chassis_odom.gimbal_yaw_fdb = imu_data.yaw;
          chassis_odom.gimbal_pitch_fdb = gimbal.pid.pit_ecd_fdb;
         
        /* 云台串级PID */
        gimbal_pid_calcu();
        memcpy(motor_cur.gimbal_cur, gimbal.current, sizeof(gimbal.current));
        osSignalSet(can_msg_send_task_t, GIMBAL_MOTOR_MSG_SEND);

        taskEXIT_CRITICAL();

        osDelayUntil(&mode_wake_time, GIMBAL_PERIOD);
    }
}

/* 自瞄 */
/* ================================== TEST PARAM ================================== */
float test_vision_predict_kp1 = 1.0f;//0.8f
float test_vision_predict_kp2 = 0.2f;
float test_vision_predict_error_kp = 1.7f;
float test_vision_angle_error_pit_kp = 1.3f;
float test_vision_angle_error_yaw_kp = 1.5f;
/* ================================== TEST PARAM ================================== */
void vision_autoaiming_calcu(void)
{
//    /* ------------------------- pit轴视觉目标值计算 ------------------------- */
//    if( ABS(vision.pit.angle_error[NOW]) <= 30 )  //重力下坠有解时才预测
//    {
//        vision.pit.kal.angle_error = Kalman1Filter_calc(&kalman_pit_angle_error, vision.pit.angle_error[NOW] * 22.75f);
//        gimbal.pid.pit_ecd_ref = gimbal.pid.pit_ecd_fdb + vision.pit.kal.angle_error;  //正负号注意pit电机安装正方向
//    }
//    else  
//        gimbal.pid.pit_ecd_ref += rc.mouse.y * KEYBOARD_SCALE_PIT;  //操作手控制pit轴
//    
//    /* ------------------------- YAW轴视觉目标值计算 ------------------------- */
//    /* YAW 目标角度计算 */
//    vision.yaw.predict = 0.4f * vision.yaw.kal.abs_speed[NOW] * vision.tof[NOW];
//    vision.yaw.kal.angle_error = Kalman1Filter_calc(
//                                &kalman_yaw_angle_error,
//                                vision.yaw.angle_error[1]/1.3f + vision.yaw.predict);
//    
//    /* 预测量限幅 */
//    if( vision.yaw.predict >= 10.0f )		    vision.yaw.predict = 10.0f;
//    else if( vision.yaw.predict <= -10.0f )		vision.yaw.predict = -10.0f;
//    
//    gimbal.pid.yaw_angle_ref = gimbal.pid.yaw_angle_fdb + vision.yaw.kal.angle_error;

//测试
    /* ------------------------- pit轴视觉目标值计算 ------------------------- */
    gimbal.pid.pit_ecd_ref += rc.mouse.y * KEYBOARD_SCALE_PIT;  //操作手控制pit轴
    
    /* ------------------------- YAW轴视觉目标值计算 ------------------------- */
    /* YAW 目标角度计算 */
    vision.yaw.kal.angle_error = Kalman1Filter_calc(&kalman_yaw_angle_error,vision.yaw.angle_error[1]/1.3f);
    gimbal.pid.yaw_angle_ref = gimbal.pid.yaw_angle_fdb + vision.yaw.kal.angle_error;
}

/* 能量机关 */
/* ================================== TEST PARAM ================================== */
float test_energy_th1 = 45.0f;
float test_energy_k1  = 0.30f;
float test_energy_k2  = 0.25f;

float test_energy_th2 = 45.0f;
float test_energy_k3  = 0.014f;
float test_energy_k4  = 0.010f;
/* ================================== TEST PARAM ================================== */
static void vision_energy_calcu(void)
{
    /*------------------------pit轴视觉目标值计算------------------------*/
    if( ABS(vision.pit.angle_error[NOW]) >= test_energy_th1)
        vision.pit.kal.angle_error = Kalman1Filter_calc(&kalman_pit_energy, test_energy_k1 * vision.pit.angle_error[NOW]);
    else if( ABS(vision.pit.angle_error[NOW]) < test_energy_th1 )
        vision.pit.kal.angle_error = Kalman1Filter_calc(&kalman_pit_energy, test_energy_k2 * vision.pit.angle_error[NOW]);
    gimbal.pid.pit_ecd_ref = gimbal.pid.pit_ecd_fdb - vision.pit.kal.angle_error;

    /*------------------------yaw轴视觉目标值计算------------------------*/
    if(ABS(vision.yaw.angle_error[NOW]) >= test_energy_th2)
        vision.yaw.kal.angle_error = Kalman1Filter_calc(&kalman_yaw_energy, test_energy_k3 * vision.yaw.angle_error[NOW]);
    else if(ABS(vision.yaw.angle_error[NOW]) < test_energy_th2)
        vision.yaw.kal.angle_error = Kalman1Filter_calc(&kalman_yaw_energy, test_energy_k4 * vision.yaw.angle_error[NOW]);
    gimbal.pid.yaw_angle_ref = gimbal.pid.yaw_angle_fdb + vision.yaw.kal.angle_error;
}

/* 反小陀螺 */
static void vision_antirotate_calu(void)
{
    /* ------------------------- pit轴视觉目标值计算 ------------------------- */
    if( ABS(vision.pit.angle_error[NOW]) <= 30 )  //重力下坠有解时才预测
    {
        vision.pit.kal.angle_error = Kalman1Filter_calc(&kalman_pit_angle_error, vision.pit.angle_error[NOW] * 22.75f);
        gimbal.pid.pit_ecd_ref = gimbal.pid.pit_ecd_fdb + vision.pit.kal.angle_error / 1.7f;  //正负号注意pit电机安装正方向
    }
    else  
        gimbal.pid.pit_ecd_ref += rc.mouse.y * KEYBOARD_SCALE_PIT;  //操作手控制pit轴
    
    /* ------------------------- YAW轴视觉目标值计算 ------------------------- */
    /* YAW 预测角度计算 */
    vision.yaw.predict = 0.1f * vision.yaw.kal.abs_speed[NOW] * vision.tof[NOW];
    vision.yaw.kal.angle_error = Kalman1Filter_calc(
                                &kalman_yaw_angle_error,
                                vision.yaw.angle_error[1]/test_vision_angle_error_yaw_kp + vision.yaw.predict);

    /* 预测量限幅 */
    if( vision.yaw.predict >= 10.0f )		    vision.yaw.predict = 10.0f;
    else if( vision.yaw.predict <= -10.0f )		vision.yaw.predict = -10.0f;
    
    gimbal.pid.yaw_angle_ref = gimbal.pid.yaw_angle_fdb + vision.yaw.kal.angle_error;
}

/* 哨兵模式 */
static void vision_sentry_calcu(void)
{
    /* ------------------------- pit轴视觉目标值计算 ------------------------- */
    if( ABS(vision.pit.angle_error[NOW]) <= 30 )  //重力下坠有解时才预测
    {
        vision.pit.kal.angle_error = Kalman1Filter_calc(&kalman_pit_angle_error, vision.pit.angle_error[NOW] * 22.75f);
        gimbal.pid.pit_ecd_ref = gimbal.pid.pit_ecd_fdb + vision.pit.kal.angle_error;  //正负号注意pit电机安装正方向
    }
    else  
        gimbal.pid.pit_ecd_ref += rc.mouse.y * KEYBOARD_SCALE_PIT;  //操作手控制pit轴
    
    /* ------------------------- YAW轴视觉目标值计算 ------------------------- */
    /* YAW 预测角度计算 */
    vision.yaw.predict = test_vision_predict_kp1 * vision.yaw.kal.abs_speed[NOW] * vision.tof[NOW];
    vision.yaw.kal.angle_error = Kalman1Filter_calc(
                                &kalman_yaw_angle_error,
                                vision.yaw.angle_error[1]/test_vision_angle_error_yaw_kp + vision.yaw.predict);  // + vision.yaw.predict

    /* 预测量限幅 */
    if( vision.yaw.predict >= 10.0f )		    vision.yaw.predict = 10.0f;
    else if( vision.yaw.predict <= -10.0f )		vision.yaw.predict = -10.0f;
    
    gimbal.pid.yaw_angle_ref = gimbal.pid.yaw_angle_fdb + vision.yaw.kal.angle_error;
}

/* 云台串级PID控制 */
void gimbal_pid_calcu(void)
{
    /*------------------------pit轴串级pid计算------------------------*/
    //位置反馈：编码器位置
    //速度反馈：陀螺仪速度
    gimbal.pid.pit_ecd_ref = data_limit(gimbal.pid.pit_ecd_ref, GIMBAL_PIT_MAX, GIMBAL_PIT_MIN);	//目标值限幅
    gimbal.pid.pit_ecd_fdb = moto_pit.ecd;
    gimbal.pid.pit_ecd_err = circle_error(gimbal.pid.pit_ecd_ref, gimbal.pid.pit_ecd_fdb, 8191);
    pid_calc(&pid_pit_ecd, gimbal.pid.pit_ecd_fdb, gimbal.pid.pit_ecd_fdb + gimbal.pid.pit_ecd_err);

    gimbal.pid.pit_spd_ref = -pid_pit_ecd.pos_out;   //PID外环目标值
    gimbal.pid.pit_spd_fdb = imu_data.wy;			 //pit角速度反馈传进PID结构体
    pid_calc(&pid_pit_spd, gimbal.pid.pit_spd_fdb, gimbal.pid.pit_spd_ref);

    gimbal.current[1] = -1.0f * pid_pit_spd.pos_out;

    
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

    gimbal.current[0] = pid_yaw_spd.pos_out;

    //位置反馈：编码器位置
    //速度反馈：陀螺仪WZ
    //注意：测试发射器时用。使用时，需要注释掉底盘，保证编码值绝对，下方电流来源切换
//    if( gimbal.pid.yaw_mecd_ref >8191 )
//        gimbal.pid.yaw_mecd_ref -= 8191;
//    else if( gimbal.pid.yaw_mecd_ref < 0 )
//        gimbal.pid.yaw_mecd_ref += 8191;
//    gimbal.pid.yaw_mecd_fdb = moto_yaw.ecd;
//    gimbal.pid.yaw_mecd_err = circle_error(gimbal.pid.yaw_mecd_ref,gimbal.pid.yaw_mecd_fdb,8191);
//    pid_calc(&pid_yaw_mecd, gimbal.pid.yaw_mecd_fdb, gimbal.pid.yaw_mecd_fdb + gimbal.pid.yaw_mecd_err);

//    gimbal.pid.yaw_mspd_ref = pid_yaw_mecd.pos_out;
//    gimbal.pid.yaw_mspd_fdb = imu_data.wz;  //陀螺仪速度反馈
//    pid_calc(&pid_yaw_mspd, gimbal.pid.yaw_mspd_fdb, gimbal.pid.yaw_mspd_ref);
//    gimbal.current[0] = pid_yaw_mspd.pos_out;
}
