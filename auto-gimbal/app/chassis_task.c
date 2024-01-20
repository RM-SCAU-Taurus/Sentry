#include "chassis_task.h"
#include "gimbal_task.h"
#include "cmsis_os.h"
#include "comm_task.h"
#include "string.h"
#include "modeswitch_task.h"
#include "remote_msg.h"
#include "pid.h"
#include "bsp_can.h"
#include "math_calcu.h"
#include "math.h"
#include "control_def.h"
#include "bsp_powerlimit.h"
#include "usart.h"
#include "DataScope_DP.h"
#include "protocol_camp.h"
#include "func_generator.h"
#include "usb_task.h"
#include "bsp_can.h"
#include "shoot_task.h"
#include "bsp_T_imu.h"
extern TaskHandle_t can_msg_send_task_t;
int vx_test;
uint8_t imu_offset_flag = 1;
float state_test;
extern chassis_ctrl_info_t chassis_ctrl;
extern chassis_odom_info_t chassis_odom;
extern Game_Status_t Game_Status;
chassis_t chassis;
extern void rm_queue_data(uint16_t cmd_id, void *buf, uint16_t len);
static void chassis_mode_switch(void)
{
    /* 系统历史状态机 */
    static ctrl_mode_e last_ctrl_mode = PROTECT_MODE;

    /* 单次触发使能标志 */
    static uint8_t spin_flag = 0;

    /* 底盘状态机 */
    switch (ctrl_mode)
    {
    case PROTECT_MODE: // 能量模式和保护模式下，底盘行为相同
    {
        chassis.mode = CHASSIS_MODE_PROTECT;
    }
    break;
    case REMOTER_MODE:
    {
        if (last_ctrl_mode != REMOTER_MODE) // 切入遥控模式，初始化底盘模式
            chassis.mode = CHASSIS_MODE_REMOTER_FOLLOW;
        /* 底盘小陀螺模式 */
        if (rc.ch5 == 0)
            spin_flag = 1;
        if (rc.ch5 == 660 && spin_flag)
        {
            spin_flag = 0;
            if (chassis.mode == CHASSIS_MODE_REMOTER_ROTATE)
            {
                chassis.spin_dir = -chassis.spin_dir; // 小陀螺反向
                chassis.mode = CHASSIS_MODE_REMOTER_FOLLOW;
            }
            else if (chassis.mode == CHASSIS_MODE_REMOTER_FOLLOW)
                chassis.mode = CHASSIS_MODE_REMOTER_ROTATE;
        }
    }
    break;
    case AUTO_MODE:
    {
			if(last_ctrl_mode != AUTO_MODE )
			memset(&chassis_ctrl,0,sizeof(chassis_ctrl_info_t));
        chassis.mode = CHASSIS_MODE_AUTO;
    }
    default:
        break;
    }
    /* 系统历史状态更新 */
    last_ctrl_mode = ctrl_mode;
}

/**
 * @brief chassis_task
 * @param
 * @attention
 * @note
 */
void chassis_task(void const *argu)
{
    uint32_t mode_wake_time = osKernelSysTick();
    for (;;)
    {
        chassis_mode_switch();
        switch (chassis.mode)
        {
        case CHASSIS_MODE_PROTECT: // 底盘保护模式
        {

            chassis.position_ref = GIMBAL_YAW_CENTER_OFFSET;
            chassis.position_error = circle_error(chassis.position_ref, moto_yaw.ecd, 8191);
            chassis.angle_error_degree = chassis.position_error * (360.0f / 8191.0f);

            chassis.spd_input.vx = 0;
            chassis.spd_input.vy = 0;
            chassis.spd_input.vw = 0;
        }
        break;
        case CHASSIS_MODE_REMOTER_FOLLOW: // 底盘遥控跟随模式
        case CHASSIS_MODE_REMOTER_ROTATE: // 底盘遥控陀螺模式
        {
            chassis.position_ref = GIMBAL_YAW_CENTER_OFFSET;
            chassis.position_error = circle_error(chassis.position_ref, moto_yaw.ecd, 8191);
            chassis.angle_error = chassis.position_error * (2.0f * PI / 8191.0f);

            chassis.spd_input.vx = 1.0f * (float)(rc.ch4 * scale.ch4 * cos(chassis.angle_error) - (-1.0f) * rc.ch3 * scale.ch3 * sin(chassis.angle_error));
            chassis.spd_input.vy = 1.0f * (float)(-rc.ch4 * scale.ch4 * sin(chassis.angle_error) - (-1.0f) * rc.ch3 * scale.ch3 * cos(chassis.angle_error));
            chassis.spd_input.vy = chassis.spd_input.vy; // 换模式加-

            if (chassis.mode == CHASSIS_MODE_REMOTER_FOLLOW)
            {
                
                chassis.spd_input.vw = 0;
            }

            else if (chassis.mode == CHASSIS_MODE_REMOTER_ROTATE)
            {
                if (chassis.wheel_max <= 8000)
                {
                    chassis.spd_input.vw = chassis.spin_dir * chassis.wheel_max;
                    //										chassis.spd_input.vw = chassis.spin_dir * 5000;
                }
                else
                {
                    chassis.spd_input.vw = chassis.spin_dir * chassis.wheel_max;
                    //												chassis.spd_input.vw = chassis.spin_dir * 5000;
                }
            }
        }
        break;
        case CHASSIS_MODE_AUTO:
        {

            /////
            chassis.position_ref = GIMBAL_YAW_CENTER_OFFSET;
            chassis.position_error = circle_error(chassis.position_ref, moto_yaw.ecd, 8191);
            chassis.angle_error = chassis.position_error * (2.0f * PI / 8191.0f);
            chassis.angle_error_degree = chassis.position_error * (360.0f / 8191.0f);
            gimbal.yaw_imu_offset = imu_data.yaw - chassis.angle_error_degree;
            /////

            chassis.spd_input.vx = chassis_ctrl.vx / 0.375f * 19.0f * 57.3f;
            chassis.spd_input.vy = -chassis_ctrl.vy / 0.375f * 19.0f * 57.3f;
            chassis.spd_input.vw = chassis_ctrl.vw / 0.375f * 19.0f * 57.3f * 0.25967f;
        }
        break;
        default:
            break;
        }
        chassis.odom.x += chassis.spd_fdb.vx * 0.001f;
        chassis.odom.y += chassis.spd_fdb.vy * 0.001f;
        chassis_odom.diff_base_to_gimbal = chassis.angle_error;
        /* 功率及超级电容控制 */
        PowerParam_Update();
        rm_queue_data(CHASSIS_ODOM_FDB_ID, &chassis_odom, sizeof(chassis_odom));
        rm_queue_data(GAME_STATUS_FDB_ID, &Game_Status, sizeof(Game_Status)); // 两个入队列的发送函数要在同一个任务用，原因未知。有入队函数的任务不能有taskEXIT_CRITICAL()保护;
        
				// 裁判系统数据

        chassis_odom.vx_fdb = chassis.spd_fdb.vx;
        chassis_odom.vy_fdb = chassis.spd_fdb.vy;
        chassis_odom.vw_fdb = chassis.spd_fdb.vw;
        chassis_odom.x_fdb = chassis.odom.x;
        chassis_odom.y_fdb = chassis.odom.y;
        if (supercap.volage > SUPERCAP_DISCHAGER_VOLAGE)
            chassis_odom.super_cup_state = 1;
        else
            chassis_odom.super_cup_state = 0;
        osSignalSet(can_msg_send_task_t, CHASSIS_MOTOR_MSG_SEND);
        osDelayUntil(&mode_wake_time, CHASSIS_PERIOD);
    }
}

void chassis_init()
{
    for (uint8_t i = 0; i < 4; i++)
    {
        PID_struct_init(&pid_chassis_spd[i], POSITION_PID, 10000, 5000,
                        7.0f, 0.0f, 0.0f);
    }
    PID_struct_init(&pid_chassis_angle, POSITION_PID, 6000, 0,
                    6.0f, 0.0f, 0.0f);
    scale.ch3 = RC_CH3_SCALE;
    scale.ch4 = RC_CH4_SCALE;
    chassis.mode = CHASSIS_MODE_PROTECT;
    chassis.mode = chassis.mode;
    chassis.fight_dir = 1;
    chassis.spin_dir = 1;
    chassis.msg_handle = can_msg_read;
    chassis.msg_send = can1_send_chassis_message;
    chassis.wheel_max = 8000;
}

/**
 * @brief          扭腰模式计算底盘角度设定值
 * @author
 * @param[in]      void
 * @retval         void
 */
void sparate_move(void)
{
    if (ABS(chassis.position_error) <= 300)
        chassis.position_ref = moto_yaw.ecd;
    if (chassis.position_error > 300)
    {
        chassis.position_ref = gimbal.yaw_center_offset - 300;
        if (chassis.position_ref > 8191)
            chassis.position_ref = chassis.position_ref - 8191;
    }
    if (chassis.position_error < -300)
    {
        chassis.position_ref = gimbal.yaw_center_offset + 300;
        if (chassis.position_ref < 0)
            chassis.position_ref = chassis.position_ref + 8191;
    }
    chassis.position_error = circle_error(chassis.position_ref, moto_yaw.ecd, 8191);
}


void can_msg_read(uint32_t can_id, uint8_t *data)
{
    chassis.spd_fdb.vx = (int16_t)(data[0] << 8 | data[1]);
    chassis.spd_fdb.vx = chassis.spd_fdb.vx * 0.25f * 6.0f * 0.0625f / 19 / 57.3f; // 转/s -》度/s->转弧度->除转速比->乘轮子半径;
    chassis.spd_fdb.vy = (int16_t)(data[2] << 8 | data[3]);
    chassis.spd_fdb.vy = -chassis.spd_fdb.vy * 0.25f * 6.0f * 0.0625f / 19 / 57.3f; // 转/s -》度/s->转弧度->除转速比->乘轮子半径
    chassis.spd_fdb.vw = (int16_t)(data[4] << 8 | data[5]);
    chassis.spd_fdb.vw = chassis.spd_fdb.vw * 0.25f * 0.5f * 0.375f * 5.64f / 19.0f / 57.3f * 10.0f; // 转/s -》度/s->转弧度->除转速比->乘轮子半径->除轮子到车中心半径
}


