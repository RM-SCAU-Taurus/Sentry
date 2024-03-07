/**********C库****************/

/**********硬件外设库*********/
#include "usart.h"
/**********任务库*************/
#include "gimbal_task.h"
#include "chassis_task.h"
#include "status_task.h"
#include "comm_task.h"
#include "usb_task.h"
#include "modeswitch_task.h"
/**********数学库*************/
#include "us_tim.h"
#include "pid.h"
#include "KalmanFilter.h"
#include "math_calcu.h"
#include "func_generator.h"
/**********数据处理库**********/
#include "vision_predict.h"
#include "ubf.h"
#include "DataScope_DP.h"
#include "remote_msg.h"
/**********类型定义库**********/
#include "protocol_camp.h"
#include "control_def.h"
/**********板级支持库**********/
#include "bsp_T_imu.h"
#include "bsp_can.h"
#include "bsp_TriggerMotor.h"
#include "bsp_Mf_Motor.h"
/**********外部变量声明********/
extern TaskHandle_t can_msg_send_task_t;
extern vision_ctrl_info_t vision_ctrl; // 自动步兵控制
extern chassis_odom_info_t chassis_odom;
extern Game_Status_t Game_Status;
/**********外部函数声明********/
extern void rm_queue_data(uint16_t cmd_id, void *buf, uint16_t len);
extern vision_tx_msg_t vision_tx_msg;
extern moto_mf_t YAW_9025;
extern uint8_t flag_out_ROTATE;
extern uint8_t flag_in_ROTATE;
/**********静态函数声明********/
static void gimbal_pid_calcu(void);
static void Gimbal_MODE_PROTECT_callback(void);
static void Gimbal_MODE_AUTO_callback(void);
static void Gimbal_MODE_REMOTER_callback(void);
static void Gimbal_data_calc(void);
static Gimbal_Base *Gimbal_mode_check(void);
/**********宏定义声明**********/
#define __GIMBAL_TASK_GLOBALS
#define __9025ready
#define __6020_Yaw_off 5000
/**********结构体定义**********/
gimbal_t gimbal;
ubf_t gim_msg_ubf; /* 云台姿态历史数据 */

static Gimbal_Derived Drv_PROTECT;

static Gimbal_Derived Drv_REMOTER;

static Gimbal_Derived Drv_AUTO;
/**********测试变量声明********/
int8_t choose_pid_flag = 0, goback_flag = 0;
uint16_t cnt_9025=0;
float test_i =0.0003;
void gimbal_param_init(void)
{
    memset(&gimbal, 0, sizeof(gimbal_t));
    /* pit 轴 */
    PID_struct_init(&pid_pit_angle, POSITION_PID, 8000, 0,
                    pid_pit_angle_P, pid_pit_angle_I, pid_pit_angle_D);
    PID_struct_init(&pid_pit_spd, POSITION_PID, 28000, 10000,
                    pid_pit_spd_P, pid_pit_spd_I, pid_pit_spd_D);
    /* YAW 轴 */
    PID_struct_init(&pid_yaw_angle_6020, POSITION_PID, 8000, 0,
                    pid_yaw_angle_6020_P, pid_yaw_angle_6020_I, pid_yaw_angle_6020_D);
    PID_struct_init(&pid_yaw_spd_6020, POSITION_PID, 28000, 20000,
                    pid_yaw_spd_6020_P, pid_yaw_spd_6020_I, pid_yaw_spd_6020_D);

    PID_struct_init(&pid_yaw_angle_9025, POSITION_PID, 8000, 0,
                    pid_yaw_angle_9025_P, pid_yaw_angle_9025_I, pid_yaw_angle_9025_D);
    PID_struct_init(&pid_yaw_spd_9025, POSITION_PID, 1024, 512,
                    pid_yaw_spd_9025_P, pid_yaw_spd_9025_I, pid_yaw_spd_9025_D);

      PID_struct_init(&pid_yaw_angle_9025, POSITION_PID, 150, 50,

                    -0.04f, -0.00005f, 0.0f);
		
		
		    PID_struct_init(&pid_9025_i, POSITION_PID, 50, 0,
                    0.0f, 0.0f, 0.0f);
										
    scale.ch1 = RC_CH1_SCALE;
    scale.ch2 = RC_CH2_SCALE;

        Drv_PROTECT.Base.Control_Fun = Gimbal_MODE_PROTECT_callback;

    Drv_REMOTER.Base.Control_Fun = Gimbal_MODE_REMOTER_callback;

    Drv_AUTO.Base.Control_Fun = Gimbal_MODE_AUTO_callback;
}

/* ================================== TEST PARAM ================================== */
/* sin信号发生器 */
// 调试视觉绝对速度时用
// 注释底盘，调成位置环YAW
// 效果：自动摇头
// FGT_sin_t test_s =
//    {
//        .Td = 1,
//        .time = 0,
//        .max = GIMBAL_YAW_CENTER_OFFSET + 800,
//        .min = GIMBAL_YAW_CENTER_OFFSET - 800,
//        .dc = GIMBAL_YAW_CENTER_OFFSET,
//        .T = 800,
//        .A = 250,
//        .phi = 0,
//        .out = 0};
/* ================================== TEST PARAM ================================== */

/**
 * @brief gimbal_task
 */

void gimbal_task(void const *argu)
{
    uint32_t mode_wake_time = osKernelSysTick();
    static Gimbal_Base *Action_ptr = NULL;
    for (;;)
    {
        taskENTER_CRITICAL();
         Action_ptr = Gimbal_mode_check();

          Action_ptr->Control_Fun();

        Gimbal_data_calc();
        /* 云台串级PID */
        gimbal_pid_calcu();
        osSignalSet(can_msg_send_task_t, GIMBAL_MOTOR_MSG_SEND);
//        DataWave(&huart3);
        taskEXIT_CRITICAL();
        osDelayUntil(&mode_wake_time, GIMBAL_PERIOD);
    }
}


static Gimbal_Base *Gimbal_mode_check(void)
{


    static Gimbal_Base *p_return = NULL;



    switch (ctrl_mode)

    {

    case PROTECT_MODE:

    {

        p_return = (Gimbal_Base *)&Drv_PROTECT;

    }

    break;

    case REMOTER_MODE:

    {

        p_return = (Gimbal_Base *)&Drv_REMOTER;

    }

    break;

    case AUTO_MODE:

    {

        p_return = (Gimbal_Base *)&Drv_AUTO;

    }

    break;

    default:

        break;

    }


    return p_return;

}

static void Gimbal_MODE_PROTECT_callback(void)
{
    gimbal.pid.pit_angle_ref = GIMBAL_PIT_CENTER_OFFSET; // 云台默认俯仰水平
    gimbal.pid.yaw_angle_6020_ref = imu_data.yaw;        // 云台默认当前水平朝向
    vision_ctrl.yaw = imu_data.yaw;
    for (uint8_t i = 0; i < 2; i++)
        gimbal.current[i] = 0;
	pid_yaw_angle_9025.iout=0;	
}


static void Gimbal_MODE_REMOTER_callback(void)
{
    gimbal.pid.pit_angle_ref += rc.ch2 * scale.ch2;
    gimbal.pid.yaw_angle_6020_ref += rc.ch1 * scale.ch1;
		
    vision_ctrl.yaw = imu_data.yaw;
    vision.status = vFIRST_LOST;
}

static void Gimbal_MODE_AUTO_callback(void)
{
    vsn_gimbal_ref_calc();
    vsn_calc();
}

static void Gimbal_data_calc(void)
{
    /*发送云台状态数据给导航控制*/
    if (imu_data.pitch > 180)
    {
        chassis_odom.gimbal_pitch_fdb = (imu_data.pitch - 360) / 57.3f;
    }
    else
    {
        chassis_odom.gimbal_pitch_fdb = imu_data.pitch / 57.3f;
    }
    chassis_odom.gimbal_yaw_fdb = moto_yaw.ecd * 360 / 8191 / 57.3f;
    if (chassis_odom.gimbal_yaw_fdb > 3.14f)
    {
        chassis_odom.gimbal_yaw_fdb -= 6.28f;
    }
    else
    {
        chassis_odom.gimbal_yaw_fdb = chassis_odom.gimbal_yaw_fdb;
    }
}

/* 云台串级PID控制 */
void gimbal_pid_calcu(void)
{
    /*------------------------pit轴串级pid计算------------------------*/
    // 位置反馈：编码器位置
    // 速度反馈：陀螺仪速度
    gimbal.pid.pit_angle_ref = data_limit(gimbal.pid.pit_angle_ref, 31, -18); // 目标值限幅
    gimbal.pid.pit_angle_fdb = imu_data.pitch;
    gimbal.pid.pit_angle_err = gimbal.pid.pit_angle_ref - gimbal.pid.pit_angle_fdb;
    pid_calc(&pid_pit_angle, gimbal.pid.pit_angle_fdb, gimbal.pid.pit_angle_fdb + gimbal.pid.pit_angle_err);

    gimbal.pid.pit_spd_ref = pid_pit_angle.pos_out; // PID外环目标值
    gimbal.pid.pit_spd_fdb = imu_data.wy;           // pit角速度反馈传进PID结构体
    pid_calc(&pid_pit_spd, gimbal.pid.pit_spd_fdb, gimbal.pid.pit_spd_ref);

    gimbal.current[1] = -pid_pit_spd.pos_out;
    /*------------------------yaw轴串级pid计算------------------------*/
    // 位置反馈：陀螺仪角度
    // 速度反馈：陀螺仪WZ

    /*------------------------下yaw轴串级pid计算------------------------*/
	if (vision_ctrl.speed_mode == 1 && ctrl_mode ==AUTO_MODE)
    {
        gimbal.pid.yaw_spd_6020_fdb = imu_data.wz; // 陀螺仪速度反馈
        pid_calc(&pid_yaw_spd_6020, gimbal.pid.yaw_spd_6020_fdb, gimbal.pid.yaw_spd_6020_ref);
        gimbal.current[0] = -pid_yaw_spd_6020.pos_out;
    }
    else 
    {
          if (gimbal.pid.yaw_angle_6020_ref < 0)
            gimbal.pid.yaw_angle_6020_ref += 360;
        else if (gimbal.pid.yaw_angle_6020_ref > 360)
            gimbal.pid.yaw_angle_6020_ref -= 360; // 目标值限幅

        gimbal.pid.yaw_angle_6020_fdb = imu_data.yaw; // 陀螺仪角度反馈
        gimbal.pid.yaw_angle_6020_err = circle_error(gimbal.pid.yaw_angle_6020_ref, gimbal.pid.yaw_angle_6020_fdb, 360);
        pid_calc(&pid_yaw_angle_6020, gimbal.pid.yaw_angle_6020_fdb, gimbal.pid.yaw_angle_6020_fdb + gimbal.pid.yaw_angle_6020_err);

        gimbal.pid.yaw_spd_6020_ref = pid_yaw_angle_6020.pos_out;
        gimbal.pid.yaw_spd_6020_fdb = imu_data.wz; // 陀螺仪速度反馈
        pid_calc(&pid_yaw_spd_6020, gimbal.pid.yaw_spd_6020_fdb, gimbal.pid.yaw_spd_6020_ref);

        gimbal.current[0] = -pid_yaw_spd_6020.pos_out;
    }

    /*------------------------下yaw轴串级pid计算------------------------*/
#ifdef __9025ready


    gimbal.position_ref = GIMBAL_YAW_9025_OFFSET;
    gimbal.position_error = circle_error(gimbal.position_ref, follow_yaw_data, 8191);
    gimbal.angle_error = gimbal.position_error  * (2.0f * PI / 8191.0f);


   pid_calc(&pid_yaw_angle_9025, follow_yaw_data, follow_yaw_data+ gimbal.position_error);
   gimbal.pid.yaw_spd_9025_ref = pid_yaw_angle_9025.pos_out;
    gimbal.pid.yaw_spd_9025_fdb = YAW_9025.wspeed; // 陀螺仪速度反馈
    pid_calc(&pid_yaw_spd_9025, gimbal.pid.yaw_spd_9025_fdb, gimbal.pid.yaw_spd_9025_ref);
		if( flag_out_ROTATE == 1 && cnt_9025++ <250)
		{
			pid_yaw_angle_9025.iout=0;	
			pid_yaw_spd_9025.pos_out =0;
		}
		else{
		cnt_9025 =0;
		flag_out_ROTATE=0;	
		}
		
		if(flag_in_ROTATE == 1 ){

		pid_yaw_angle_9025.i =-test_i;
			
		}
		else{
		
      PID_struct_init(&pid_yaw_angle_9025, POSITION_PID, 150, 50,

                    -0.04f, -0.00005f, 0.0f);
		}
		
		
		

    gimbal.current[2] = pid_yaw_spd_9025.pos_out;
#endif
    memcpy(motor_cur.gimbal_cur, gimbal.current, sizeof(gimbal.current)); // 赋值电流结果进CAN发送缓冲
}
