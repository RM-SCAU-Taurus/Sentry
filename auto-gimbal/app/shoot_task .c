/**********C库****************/

/**********硬件外设库******/
#include "tim.h"
#include "usart.h"
/**********任务库*************/
#include "cmsis_os.h"
#include "modeswitch_task.h"
#include "comm_task.h"
#include "shoot_task.h"
#include "freertos.h"
/**********数学库*************/

/**********数据处理库**********/
#include "remote_msg.h"
/**********类型定义库**********/
#include "control_def.h"
#include "protocol_camp.h"
/**********板级支持库**********/
#include "bsp_CoverServo.h"
#include "bsp_FricMotor.h"
#include "bsp_TriggerMotor.h"
#include "bsp_judge.h"
/**********外部变量声明********/
extern vision_ctrl_info_t vision_ctrl;
/**********外部函数声明********/

/**********静态函数声明********/
static void ShootParam_Update(void);
static void shoot_mode_sw(void);
/**********宏定义声明**********/
#define __SHOOT_TASK_GLOBALS
/**********结构体定义**********/
fric_t fric;
shoot_t shoot;
extern osThreadId can_msg_send_task_t;
/**********变量声明*************/

/**********测试变量声明********/
int flag = 0;
int cnt = 0;

void shoot_task(void const *argu)
{
    uint32_t mode_wake_time = osKernelSysTick();
    shoot_init();
    for (;;)
    {
        taskENTER_CRITICAL();
        /* 电调初始化 */
        static uint8_t last_fric_enable, fric_enable;
        fric_enable = !!Game_Robot_Status.mains_power_shooter_output;
        if (fric_enable && !last_fric_enable)
        {
            shoot.fric_protect_mode = FRIC_PROTECT;
        }
        last_fric_enable = fric_enable;

        shoot_mode_sw();        /* 发射器模式切换 */
        FricMotor_Control();    /* 摩擦轮电机控制 */
        TriggerMotor_control(); /* 拨弹电机控制 */
        osSignalSet(can_msg_send_task_t, SHOOT_MOTOR_MSG_SEND);
        taskEXIT_CRITICAL();
        osDelayUntil(&mode_wake_time, SHOOT_PERIOD);
    }
}

void shoot_init(void)
{
    /* 发射器底层初始化 */
    FricMotor_init();    // 摩擦轮初始化
    TriggerMotor_init(); // 拨盘初始化

    shoot.firc_mode = FIRC_MODE_STOP;
    shoot.fric_protect_mode = FRIC_PROTECT;
    shoot.stir_mode = STIR_MODE_PROTECT;
    shoot.house_mode = HOUSE_MODE_PROTECT;
    /* 枪管参数初始化 */
    shoot.trigger_period = TRIGGER_PERIOD;
    shoot.barrel.cooling_rate = 25;
    shoot.barrel.heat_max = 240;
    shoot.shoot_speed = 30;
}

static void shoot_mode_sw(void)
{
    /* 系统历史状态机 */
    static ctrl_mode_e last_ctrl_mode = PROTECT_MODE;
    /*发射器电机保护标志位*/
    if (!fric.init_flag)
    {
        fric.protect_flag = FRIC_PROTECT;
    }
    /* 更新裁判系统参数 */
    ShootParam_Update();
    // 让导航控制射频
    if (vision_ctrl.shoot_cmd != 0 && vision_ctrl.shoot_cmd > 0)
    {
        if (vision_ctrl.shoot_cmd > 25) // 最大25hz
        {
            shoot.trigger_period = 40;
        }
        else
        {
            shoot.trigger_period = (1 / vision_ctrl.shoot_cmd) * 1000; // 视觉直接发hz
        }
    }
    else
    {
        shoot.trigger_period = TRIGGER_PERIOD;
    }

    /* 模式切换 */
    switch (ctrl_mode)
    {
    case PROTECT_MODE:
    {
        shoot.firc_mode = FIRC_MODE_STOP;
        shoot.stir_mode = STIR_MODE_PROTECT;
        shoot.house_mode = HOUSE_MODE_PROTECT;
        FricMotor_speed_set(0);
        if (fric.protect_flag != FRIC_PROTECT)
        {
            fric.protect_flag = FRIC_SLOW_TO_PROTECT;
        }
    }
    break;
    case REMOTER_MODE:
    case AUTO_MODE:
    {
        /* 摩擦轮和拨盘模式切换 */
        switch (rc.sw2)
        {
        case RC_UP:
        {
            // LASER_UP;
            LASER_DOWN;
            shoot.firc_mode = FIRC_MODE_STOP;
            shoot.stir_mode = STIR_MODE_STOP;
            FricMotor_speed_set(0);
        }
        break;
        case RC_MI:
        {
            LASER_UP;
            if (fric.init_flag)
            {
                shoot.firc_mode = FIRC_MODE_RUN; // 开启摩擦轮
                FricMotor_speed_set(30);
            }
            fric.protect_flag = FRIC_UNPROTECT;
            shoot.stir_mode = STIR_MODE_STOP;
        }
        break;
        case RC_DN:
        {
            LASER_UP;
            if (fric.init_flag)
            {
                shoot.firc_mode = FIRC_MODE_RUN;
            } // 开启摩擦轮
            fric.protect_flag = FRIC_UNPROTECT;
            shoot.stir_mode = STIR_MODE_SERIES; // 连发
        }
        break;
        default:
            break;
        }
        /* 弹舱盖模式切换 */
        static uint8_t house_switch_enable = 1;
        if (last_ctrl_mode != REMOTER_MODE)
            shoot.house_mode = HOUSE_MODE_CLOSE;
        if (rc.ch5 == 0)
            house_switch_enable = 1;
        if (house_switch_enable && rc.ch5 == -660) // 鍒囨崲寮硅埍寮�鍏崇姸鎬佹爣蹇椾綅
        {
            house_switch_enable = 0;
            shoot.house_mode = (shoot_house_mode_e)(!(uint8_t)shoot.house_mode); // 寮�鍏冲脊鑸辩洊
        }
    }
    break;
    default:
        break;
    }
    /* 历史状态更新 */
    last_ctrl_mode = ctrl_mode;
}

/* 发射器裁判系统数据更新 */
static void ShootParam_Update(void)
{
    /* 更新裁判系统数据 */
    if (Game_Robot_Status.shooter_id1_17mm_speed_limit != 0)
    {
        shoot.shoot_speed = Game_Robot_Status.shooter_id1_17mm_speed_limit;          //
        shoot.barrel.heat_max = Game_Robot_Status.shooter_id1_17mm_cooling_limit;    //
        shoot.barrel.cooling_rate = Game_Robot_Status.shooter_id1_17mm_cooling_rate; //
    }
    /* 更新 模拟裁判系统 数据 */
    shoot.barrel.heat -= shoot.barrel.cooling_rate * SHOOT_PERIOD * 0.001f; //
    if (shoot.barrel.heat < 0)
        shoot.barrel.heat = 0;
    shoot.barrel.heat_remain = shoot.barrel.heat_max - shoot.barrel.heat; //
    //		shoot.barrel.heat_remain = shoot.barrel.heat_max ;  //无限热量测试用
}
