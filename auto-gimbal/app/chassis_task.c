/**********C库****************/
#include "string.h"
/**********硬件外设库*********/
#include "usart.h"
/**********任务库*************/
#include "cmsis_os.h"
#include "usb_task.h"
#include "shoot_task.h"
#include "comm_type.h"
#include "chassis_task.h"
#include "gimbal_task.h"
/**********数学库*************/
#include "math_calcu.h"
#include "math.h"
#include "pid.h"
#include "func_generator.h"
/**********数据处理库**********/
#include "remote_msg.h"
#include "DataScope_DP.h"
#include "msg_center.h"
/**********类型定义库**********/
#include "control_def.h"
#include "protocol_camp.h"
#include "comm_type.h"
/**********板级支持库**********/
#include "bsp_T_imu.h"
#include "bsp_powerlimit.h"
#include "bsp_can.h"
#include "bsp_can.h"
/**********外部变量声明********/
extern TaskHandle_t can_msg_send_task_t;
// extern chassis_ctrl_info_t chassis_ctrl;

extern chassis_odom_info_t chassis_odom;
extern Game_Status_t Game_Status;
static chassis_ctrl_info_t chassis_ctrl_sub_msg;
static ctrl_mode_e ctrl_mode_sys;
/**********外部函数声明********/
extern void rm_queue_data(uint16_t cmd_id, void *buf, uint16_t len);
/**********宏定义声明********/

/**********静态函数声明********/
static void ChasisInstance_Create(ChasisInstance_t *_instance, ChasisInstance_mode_e mode_sel, chassis_mode_callback callback);
static void CHASSIS_MODE_PROTECT_callback(void);
static void CHASSIS_MODE_FOLL_ROTA_callback(void);
static void CHASSIS_MODE_AUTO_callback(void);
static void Chassis_odom_calc(void);
static void Chassis_queue_send(void);
static void ROTATE_State_Check(void);
// static void chassis_mode_switch(void);
static Chassis_Base*  chassis_mode_switch(void);
static void Gimbal_to_Chassis_input(Gimbal_to_Chassis_t *str);

/**********静态变量声明********/
static Gimbal_to_Chassis_t Gimbal_to_Chassis;
static Subscriber_t *chassis_ctrl_sub;                   // 用于订阅底盘的控制命令
static Subscriber_t *C_ctrl_mode_sub;                   // 用于订阅控制模式的命令
static Publisher_t  *chassis_ctrl_send_pub;                   // 用于订阅底盘的控制命令
/**********测试变量声明********/
float state_test;
unsigned portBASE_TYPE uxHighWaterMark_chassis;
/**********结构体定义**********/
chassis_t chassis;
ChasisInstance_t Chasis_behavior[3];

Chassis_Derived Drv_PROTECT;
Chassis_Derived Drv_REMOTER;
Chassis_Derived Drv_AUTO;


/**********函数定义************/

/**
 * @brief chassis_task
 * @param
 * @attention
 * @note
 */
void chassis_task(void const *argu)
{
   uint32_t mode_wake_time = osKernelSysTick();
    static Chassis_Base* p= NULL;
    for (;;)
    {
        p = chassis_mode_switch();
        SubGetMessage(chassis_ctrl_sub,&chassis_ctrl_sub_msg);
        
        p->c_Fun();

        /* 里程计 数据计算 */
        Chassis_odom_calc();
        /* 里程计&裁判系统 数据入列 */
        Chassis_queue_send();
        /* Gimbal_to_Chassis 数据入列 */
        Gimbal_to_Chassis_input(&Gimbal_to_Chassis);
        PubPushMessage(chassis_ctrl_send_pub, (void *)&Gimbal_to_Chassis);

        /* 任务通知 */
        osSignalSet(can_msg_send_task_t, CHASSIS_MOTOR_MSG_SEND);
        // uxHighWaterMark_chassis = uxTaskGetStackHighWaterMark(NULL);
        osDelayUntil(&mode_wake_time, CHASSIS_PERIOD);
    }
}










/* --------------------------------------------静态函数定义-------------------------------------------------------------------------------- */

/*  static void chassis_mode_switch(void)
{
  
    static ctrl_mode_e last_ctrl_mode = PROTECT_MODE;
    SubGetMessage(C_ctrl_mode_sub,&ctrl_mode_sys);
    
    
    switch (ctrl_mode_sys)
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
       
        ROTATE_State_Check();
    }
    break;
    case AUTO_MODE:
    {
        if (last_ctrl_mode != AUTO_MODE)
            // memset(&chassis_ctrl, 0, sizeof(chassis_ctrl_info_t));//清除上一帧数据
            memset(&chassis_ctrl_sub_msg, 0, sizeof(chassis_ctrl_info_t));//清除上一帧数据
        chassis.mode = CHASSIS_MODE_AUTO;
    }
    default:
        break;
    }
 
    last_ctrl_mode = ctrl_mode_sys;
}*/



static Chassis_Base*  chassis_mode_switch(void)
{
    /* 系统历史状态机 */
    static ctrl_mode_e last_ctrl_mode = PROTECT_MODE;
    SubGetMessage(C_ctrl_mode_sub,&ctrl_mode_sys);
    static Chassis_Base* p_re= NULL;
    /* 底盘状态机 */
    switch (ctrl_mode_sys)
    {
    case PROTECT_MODE: // 能量模式和保护模式下，底盘行为相同
    {
        p_re = (Chassis_Base *)&Drv_PROTECT;
    }
    break;
    case REMOTER_MODE:
    {
        if (last_ctrl_mode != REMOTER_MODE) // 切入遥控模式，初始化底盘模式
            chassis.mode = CHASSIS_MODE_REMOTER_FOLLOW;
        /* 底盘小陀螺模式 */
        ROTATE_State_Check();
         p_re = (Chassis_Base *)&Drv_REMOTER;
    }
    break;
    case AUTO_MODE:
    {
        if (last_ctrl_mode != AUTO_MODE)
            // memset(&chassis_ctrl, 0, sizeof(chassis_ctrl_info_t));//清除上一帧数据
            memset(&chassis_ctrl_sub_msg, 0, sizeof(chassis_ctrl_info_t));//清除上一帧数据
        chassis.mode = CHASSIS_MODE_AUTO;
         p_re = (Chassis_Base *)&Drv_AUTO;
    }
    default:
        break;
    }
    /* 系统历史状态更新 */
    last_ctrl_mode = ctrl_mode_sys;
    return p_re;
}


static void ChasisInstance_Create(ChasisInstance_t *_instance, ChasisInstance_mode_e mode_sel, chassis_mode_callback callback)
{
    _instance->Chassis_Mode = mode_sel;
    _instance->mode_callback = callback;
}

static void CHASSIS_MODE_PROTECT_callback(void)
{
    chassis.position_ref = GIMBAL_YAW_CENTER_OFFSET;
    chassis.position_error = circle_error(chassis.position_ref, moto_yaw.ecd, 8191);
    chassis.angle_dif_degree = chassis.position_error * (360.0f / 8191.0f);
    chassis.spd_input.vx = 0;
    chassis.spd_input.vy = 0;
    chassis.spd_input.vw = 0;
}

static void CHASSIS_MODE_AUTO_callback(void)
{

    // chassis.position_ref = GIMBAL_YAW_CENTER_OFFSET;
    // chassis.position_error = circle_error(chassis.position_ref, moto_yaw.ecd, 8191);
    // chassis.angle_error = chassis.position_error * (2.0f * PI / 8191.0f);
    // chassis.angle_dif_degree = chassis.position_error * (360.0f / 8191.0f);
    // gimbal.yaw_imu_offset = imu_data.yaw - chassis.angle_dif_degree;
    // chassis.spd_input.vx = chassis_ctrl.vx / 0.375f * 19.0f * 57.3f;
    // chassis.spd_input.vy = -chassis_ctrl.vy / 0.375f * 19.0f * 57.3f;
    // chassis.spd_input.vw = chassis_ctrl.vw / 0.375f * 19.0f * 57.3f * 0.25967f;
    chassis.position_ref = GIMBAL_YAW_CENTER_OFFSET;
    chassis.position_error = circle_error(chassis.position_ref, moto_yaw.ecd, 8191);
    chassis.angle_error = chassis.position_error * (2.0f * PI / 8191.0f);
    chassis.angle_dif_degree = chassis.position_error * (360.0f / 8191.0f);
    gimbal.yaw_imu_offset = imu_data.yaw - chassis.angle_dif_degree;
    chassis.spd_input.vx = chassis_ctrl_sub_msg.vx / 0.375f * 19.0f * 57.3f;
    chassis.spd_input.vy = -chassis_ctrl_sub_msg.vy / 0.375f * 19.0f * 57.3f;
    chassis.spd_input.vw = chassis_ctrl_sub_msg.vw / 0.375f * 19.0f * 57.3f * 0.25967f;

}

static void CHASSIS_MODE_FOLL_ROTA_callback(void)
{
    chassis.position_ref = GIMBAL_YAW_CENTER_OFFSET;
    chassis.position_error = circle_error(chassis.position_ref, moto_yaw.ecd, 8191);
    chassis.angle_error = chassis.position_error * (2.0f * PI / 8191.0f);
    chassis.angle_dif_degree = chassis.position_error * (360.0f / 8191.0f);

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

static void Chassis_odom_calc(void)
{
    chassis.odom.x += chassis.spd_fdb.vx * 0.001f;
    chassis.odom.y += chassis.spd_fdb.vy * 0.001f;
    chassis_odom.diff_base_to_gimbal = chassis.angle_error;
    chassis_odom.vx_fdb = chassis.spd_fdb.vx;
    chassis_odom.vy_fdb = chassis.spd_fdb.vy;
    chassis_odom.vw_fdb = chassis.spd_fdb.vw;
    chassis_odom.x_fdb = chassis.odom.x;
    chassis_odom.y_fdb = chassis.odom.y;
    PowerParam_Update();
    if (supercap.volage > SUPERCAP_DISCHAGER_VOLAGE)
        chassis_odom.super_cup_state = 1;
    else
        chassis_odom.super_cup_state = 0;
}

static void Chassis_queue_send(void)
{
    rm_queue_data(CHASSIS_ODOM_FDB_ID, &chassis_odom, sizeof(chassis_odom));
    rm_queue_data(GAME_STATUS_FDB_ID, &Game_Status, sizeof(Game_Status)); // 两个入队列的发送函数要在同一个任务用，原因未知。有入队函数的任务不能有taskEXIT_CRITICAL()保护;
}

static void Gimbal_to_Chassis_input(Gimbal_to_Chassis_t *str)
{
    str->spd_input.vx = chassis.spd_input.vx ;
    str->spd_input.vy = chassis.spd_input.vy ;
    str->spd_input.vw = chassis.spd_input.vw ;
    str->ctrl_mode_sys= ctrl_mode_sys;
    str->spin_dir     = chassis.spin_dir;
    str->super_cup    = chassis_ctrl_sub_msg.super_cup;
    
}

static void ROTATE_State_Check(void)
{

    /* 单次触发使能标志 */
    static uint8_t spin_flag = 0;
    if (ABS(rc.ch5) <= 60)
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

    chassis_ctrl_send_pub = PubRegister("Chassis_spd_send",sizeof(Gimbal_to_Chassis_t));
    chassis_ctrl_sub = SubRegister("chassis_ctrl",sizeof(chassis_ctrl_info_t));
    C_ctrl_mode_sub    = SubRegister("Mode_Switch",sizeof(ctrl_mode_e));
    /*模式实例赋值*/
  /*  ChasisInstance_Create(&Chasis_behavior[ChasisInstance_MODE_PROTECT], ChasisInstance_MODE_PROTECT, CHASSIS_MODE_PROTECT_callback);
    ChasisInstance_Create(&Chasis_behavior[ChasisInstance_MODE_REMOTER_FOLLOW_ROTATE], ChasisInstance_MODE_REMOTER_FOLLOW_ROTATE, CHASSIS_MODE_FOLL_ROTA_callback);
    ChasisInstance_Create(&Chasis_behavior[ChasisInstance_MODE_AUTO], ChasisInstance_MODE_AUTO, CHASSIS_MODE_AUTO_callback);
    */
   Drv_PROTECT.Base.c_Fun = CHASSIS_MODE_PROTECT_callback;
   Drv_REMOTER.Base.c_Fun = CHASSIS_MODE_FOLL_ROTA_callback;
   Drv_AUTO.Base.c_Fun    = CHASSIS_MODE_AUTO_callback;
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
