#define __SHOOT_TASK_GLOBALS

#include "shoot_task.h"
#include "control_def.h"
#include "cmsis_os.h"
#include "usart.h"
#include "tim.h"
#include "bsp_CoverServo.h"
#include "bsp_FricMotor.h"
#include "bsp_TriggerMotor.h"
#include "bsp_judge.h"
#include "bsp_vision.h"
#include "remote_msg.h"
#include "comm_task.h"
#include "modeswitch_task.h"

static void ShootParam_Update(void);
static void shoot_mode_sw(void);
static void house_init(void);
static void house_control(void);

void shoot_task(void const *argu)
{
    uint32_t mode_wake_time = osKernelSysTick();
    for(;;)
    {
        shoot_mode_sw();  //射击模式切换
        FricMotor_Control();	//摩擦轮电机控制
        TriggerMotor_control();	//拨弹电机控制
        house_control();        //弹舱盖控制

        osDelayUntil(&mode_wake_time, SHOOT_PERIOD);
    }
}

void shoot_init(void)
{
    /* 发射器底层初始化 */
    FricMotor_init();   //摩擦轮初始化
    TriggerMotor_init();//拨盘初始化
    house_init();       //弹舱初始化
    /* 发射器模式初始化 */
    shoot.firc_mode     = FIRC_MODE_STOP;
    shoot.stir_mode     = STIR_MODE_PROTECT;
    shoot.house_mode    = HOUSE_MODE_PROTECT;  //上电保护模式，弹舱盖无力
    /* 枪管参数初始化 */
    shoot.trigger_period = TRIGGER_PERIOD;
    shoot.barrel.cooling_rate   = 10;
    shoot.barrel.heat_max       = 50;
    shoot.shoot_speed           = 15;
}

static void shoot_mode_sw(void)
{
    /* 系统历史状态机 */
    static ctrl_mode_e last_ctrl_mode = PROTECT_MODE;
    
    /* 更新裁判系统参数 */
    ShootParam_Update();
    
    /* 视觉射速档位处理 */
    if( shoot.shoot_speed == 15 )			shoot.shoot_speed_vision = 1;
    else if( shoot.shoot_speed == 18 )		shoot.shoot_speed_vision = 2;
    else if( shoot.shoot_speed == 30 )		shoot.shoot_speed_vision = 3;

    /* 模式切换 */
    switch( ctrl_mode )
    {
        case PROTECT_MODE:
        {
            shoot.firc_mode = FIRC_MODE_STOP;
            shoot.stir_mode = STIR_MODE_PROTECT;
            shoot.house_mode= HOUSE_MODE_PROTECT;
        }
        break;
        case REMOTER_MODE:
        {
            /* 摩擦轮和拨盘模式切换 */
            switch( rc.sw2 )
            {
                case RC_UP:
                {
                    LASER_UP;
                    //LASER_DOWN;
                    shoot.firc_mode = FIRC_MODE_STOP;
                    shoot.stir_mode = STIR_MODE_STOP;
                }
                break;
                case RC_MI:
                {
                    LASER_UP;
                    shoot.firc_mode = FIRC_MODE_RUN;  //开启摩擦轮
                    shoot.stir_mode = STIR_MODE_STOP;
                }
                break;
                case RC_DN:
                {
                    LASER_UP;
                    shoot.firc_mode = FIRC_MODE_RUN;  //保持开启摩擦轮
                    if( shoot.barrel.heat_max == 120 )
                        shoot.stir_mode = STIR_MODE_SERIES;  //拨盘自动单发（热量控制）
                    else if( shoot.barrel.heat_max == 180 )
                        shoot.stir_mode = STIR_MODE_SINGLE;  //拨盘手动单发
                    else
                        shoot.stir_mode = STIR_MODE_SERIES;  //拨盘自动连发
                }
                break;
                default: break;
            }
            /* 弹舱盖模式切换 */
            static uint8_t house_switch_enable = 1;
            if( last_ctrl_mode != REMOTER_MODE )
                shoot.house_mode = HOUSE_MODE_CLOSE;
            if( rc.ch5 == 0 )   house_switch_enable = 1;
            if( house_switch_enable && rc.ch5 == -660 )  //切换弹舱开关状态标志位
            {
                house_switch_enable = 0;
                shoot.house_mode = (shoot_house_mode_e) (!(uint8_t)shoot.house_mode);  //开关弹舱盖
            }
        }
        break;
        case VISION_MODE:
        case KEYBOARD_MODE:
        {
            /* 摩擦轮模式切换 */
            if( Game_Robot_Status.mains_power_shooter_output )  //发射机构得到供电
            {
                if( key_scan_clear(KEY_SHOOT_FRIC) )
                    shoot.firc_mode = (shoot_firc_mode_e)(!(uint8_t)shoot.firc_mode);  //开关摩擦轮
            }
            else
            {
                shoot.firc_mode = FIRC_MODE_STOP;  //摩擦轮断电，软件保护，禁用摩擦轮
            }
            /* 拨盘模式切换 */
            if( shoot.firc_mode == FIRC_MODE_RUN )  //开摩擦轮后
            {
                LASER_UP;  //打开激光
                if( vision_mode == VISION_MODE_bENERGY || vision_mode == VISION_MODE_sENERGY )  //能量机关，单发模式
                {
                    shoot.stir_mode = STIR_MODE_SINGLE;
                }
                else  //其他模式下，连发模式
                {
                    shoot.stir_mode = STIR_MODE_SERIES;
                }
            }
            else
            {
                LASER_DOWN;
                shoot.stir_mode = STIR_MODE_STOP;  //摩擦轮没开，不开拨盘
            }
            /* 弹舱盖模式切换 */
            keyboard_scan(KEY_SHOOT_HOUSE);
            if( last_ctrl_mode != KEYBOARD_MODE )  //首次进入键盘模式，关闭弹舱盖
            {
                shoot.house_mode = HOUSE_MODE_CLOSE;
                key_status_clear(KEY_SHOOT_HOUSE);
            }
            if( FLAG_SHOOT_HOUSE == KEY_RUN )  
                shoot.house_mode = HOUSE_MODE_OPEN;  //按过奇数次辅助按键，打开弹舱盖
            else if( FLAG_SHOOT_HOUSE == KEY_END )
                shoot.house_mode = HOUSE_MODE_CLOSE;  //按过偶数次辅助按键，关闭弹舱盖
        }
        break;
        default: break;
    }
    /* 历史状态更新 */
    last_ctrl_mode = ctrl_mode;
}

/* 发射器裁判系统数据更新 */
static void ShootParam_Update(void)
{
    /* 更新裁判系统数据 */
    if( Game_Robot_Status.shooter_id1_17mm_speed_limit != 0 )
    {
        shoot.shoot_speed = Game_Robot_Status.shooter_id1_17mm_speed_limit;  //射速上限
        shoot.barrel.heat_max = Game_Robot_Status.shooter_id1_17mm_cooling_limit;  //枪管热量上限
        shoot.barrel.cooling_rate = Game_Robot_Status.shooter_id1_17mm_cooling_rate;  //枪管冷却速率
    }
    /* 更新 模拟裁判系统 数据 */
    shoot.barrel.heat -= shoot.barrel.cooling_rate * SHOOT_PERIOD * 0.001f;  //当前枪管（理论）热量
    if( shoot.barrel.heat < 0 )	  shoot.barrel.heat = 0;
    shoot.barrel.heat_remain = shoot.barrel.heat_max - shoot.barrel.heat;  //当前枪管（理论）剩余热量
}

static void house_init(void)
{
    //HAL_TIM_PWM_Start(Magazine_Time_CH);  //开始系统为保护模式，不给舵机PWM，无力
    Magazine_PWM = COVER_PWM_CLOSE;
}

//uint16_t test_pwm_open = 600;
static void house_control(void)
{
    switch( shoot.house_mode )
    {
        case HOUSE_MODE_OPEN:  //打开弹舱盖
        {
            HAL_TIM_PWM_Start(Magazine_Time_CH);
            Magazine_PWM = COVER_PWM_OPEN;//COVER_PWM_OPEN
        }
        break;
        case HOUSE_MODE_CLOSE:  //关闭弹舱盖
        {
            HAL_TIM_PWM_Start(Magazine_Time_CH);
            Magazine_PWM = COVER_PWM_CLOSE;
        }            
        break;
        case HOUSE_MODE_PROTECT:  //弹舱盖无力
        {
            HAL_TIM_PWM_Stop(Magazine_Time_CH);
            Magazine_PWM = COVER_PWM_CLOSE;
        }
        default: break;
    }
}
