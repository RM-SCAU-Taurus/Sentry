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
#include "remote_msg.h"
#include "comm_task.h"
#include "modeswitch_task.h"
#include "protocol_camp.h"
extern vision_ctrl_info_t  vision_ctrl;
static void ShootParam_Update(void);
static void shoot_mode_sw(void);
static void house_init(void);
static void house_control(void);
fric_t fric;
void shoot_task(void const *argu)
{
    uint32_t mode_wake_time = osKernelSysTick();
		shoot_init();
    for(;;)
    {
			taskENTER_CRITICAL();
			/* 电调初始化 */
				  static uint8_t last_fric_enable, fric_enable;
        fric_enable = !!Game_Robot_Status.mains_power_shooter_output;
        if (fric_enable && !last_fric_enable) {
            fric.init_cnt = 0;
            fric.init_flag = 0;
						fric.acc_flag =0;
        }
				last_fric_enable = fric_enable;
				 FricMotor_init();   
        shoot_mode_sw();  /* 发射器模式切换 */
        FricMotor_Control();	/* 摩擦轮电机控制 */
        TriggerMotor_control();	/* 拨弹电机控制 */
        house_control();         /* 弹舱盖控制 */
				taskEXIT_CRITICAL();
        osDelayUntil(&mode_wake_time, SHOOT_PERIOD);
    }
}

int flag=0;
int cnt=0;
void shoot_init(void)
{
    /* 发射器底层初始化 */
    FricMotor_init();  //摩擦轮初始化 在任务中控制初始化
    TriggerMotor_init();//拨盘初始化
		house_init();       ////弹舱初始化
	
    shoot.firc_mode     = FIRC_MODE_STOP;
    shoot.stir_mode     = STIR_MODE_PROTECT;
    shoot.house_mode    = HOUSE_MODE_PROTECT;  
	/* 枪管参数初始化 */
    shoot.trigger_period = TRIGGER_PERIOD;
	 shoot.barrel.cooling_rate   = 25;
    shoot.barrel.heat_max       = 240;
    shoot.shoot_speed           = 30;
}

static void shoot_mode_sw(void)
{
    /* 系统历史状态机 */
    static ctrl_mode_e last_ctrl_mode = PROTECT_MODE;
    
   /* 更新裁判系统参数 */
	 
    ShootParam_Update();
	//让导航控制射频
		if(vision_ctrl.shoot_cmd!=0&&vision_ctrl.shoot_cmd>0)
		{			
				if(vision_ctrl.shoot_cmd>25)//最大25hz
				{
					shoot.trigger_period=40;
				}
				else
				{
				shoot.trigger_period=(1/vision_ctrl.shoot_cmd)*1000;//视觉直接发hz
				}
		}
		else 
		{
				shoot.trigger_period=TRIGGER_PERIOD;	
		}


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
				case AUTO_MODE:
        {
           /* 摩擦轮和拨盘模式切换 */
            switch( rc.sw2 )
            {
                case RC_UP:
                {
                    //LASER_UP;
                    LASER_DOWN;
                    shoot.firc_mode = FIRC_MODE_STOP;
                    shoot.stir_mode = STIR_MODE_STOP;
                }
                break;
                case RC_MI:
                {
                    LASER_UP;
									   if (fric.init_flag) {
                        shoot.firc_mode= FIRC_MODE_RUN;  //开启摩擦轮
                    }
                    shoot.stir_mode = STIR_MODE_STOP;
										//shoot.stir_mode = STIR_MODE_SINGLE;  //单发（测弹道用）
                }
                break;
                case RC_DN:
                {
                    LASER_UP;
										if (fric.init_flag) {
                        shoot.firc_mode = FIRC_MODE_RUN;  // //开启摩擦轮
                    }
										 shoot.stir_mode = STIR_MODE_SERIES; //连发
										  if (!fric.acc_flag) {
                        shoot.stir_mode = STIR_MODE_STOP;//未完成加速不开播盘
                    }
                }
                break;
                default: break;
            }
             /* 弹舱盖模式切换 */
            static uint8_t house_switch_enable = 1;
            if( last_ctrl_mode != REMOTER_MODE )
                shoot.house_mode = HOUSE_MODE_CLOSE;
            if( rc.ch5 == 0 )   house_switch_enable = 1;
            if( house_switch_enable && rc.ch5 == -660 )  //鍒囨崲寮硅埍寮�鍏崇姸鎬佹爣蹇椾綅
            {
                house_switch_enable = 0;
                shoot.house_mode = (shoot_house_mode_e) (!(uint8_t)shoot.house_mode);  //寮�鍏冲脊鑸辩洊
            }
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
        shoot.shoot_speed = Game_Robot_Status.shooter_id1_17mm_speed_limit;  //
			shoot.barrel.heat_max = Game_Robot_Status.shooter_id1_17mm_cooling_limit;  //
        shoot.barrel.cooling_rate = Game_Robot_Status.shooter_id1_17mm_cooling_rate;  //
    }
     /* 更新 模拟裁判系统 数据 */
    shoot.barrel.heat -= shoot.barrel.cooling_rate * SHOOT_PERIOD * 0.001f;  //
    if( shoot.barrel.heat < 0 )	  shoot.barrel.heat = 0;
    shoot.barrel.heat_remain = shoot.barrel.heat_max - shoot.barrel.heat;  //
//		shoot.barrel.heat_remain = shoot.barrel.heat_max ;  //无限热量测试用
}

static void house_init(void)
{
    //HAL_TIM_PWM_Start(Magazine_Time_CH);
	Magazine_PWM = COVER_PWM_CLOSE;
}

//uint16_t test_pwm_open = 600;
static void house_control(void)
{
    switch( shoot.house_mode )
    {
        case HOUSE_MODE_OPEN: //打开弹舱盖       
					{
            HAL_TIM_PWM_Start(Magazine_Time_CH);
            Magazine_PWM = COVER_PWM_OPEN;
        }
        break;
        case HOUSE_MODE_CLOSE:  //使能pwm
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
