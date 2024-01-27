
/**********C库****************/

/**********硬件外设库*********/
#include "usart.h"
/**********任务库*************/
#include "cmsis_os.h"
#include "chassis_task.h"
#include "gimbal_task.h"
#include "shoot_task.h"
#include "status_task.h"
#include "modeswitch_task.h"
/**********数学库*************/

/**********数据处理库**********/
#include "vision_predict.h"
#include "remote_msg.h"
#include "msg_center.h"
/**********类型定义库**********/
#include "control_def.h"
/**********板级支持库**********/
#include "bsp_can.h"
#include "bsp_TriggerMotor.h"
/**********外部变量声明********/

/**********外部函数声明********/

/**********宏定义声明**********/

/**********测试变量声明********/

/**********变量声明********/
uint8_t lock_flag = 0;/* 解锁标志 */
static Publisher_t *mode_switch_pub;                   // 用于订阅底盘的控制命令
ctrl_mode_e ctrl_mode; /* 系统状态机 */
/**********结构体定义**********/

/**********静态函数声明********/
static uint8_t rc_normal_check(void);
static void unlock_init(void);
static void sw1_mode_handler(void);
static void rc_abnormal_proess(void);


/* 系统模式切换任务函数 */
 void mode_switch_task(void const *argu)
{
    mode_switch_pub = PubRegister("Mode_Switch",sizeof(ctrl_mode_e));
	vision.mode = vMODE_AUTO;
    for(;;)
    {
        if( !lock_flag )
        {
            unlock_init();  //解锁操作
        }
        else if( rc_normal_check() )
        {
            sw1_mode_handler();  //根据左拨杆切换系统模式
        }
        else
        {
            rc_abnormal_proess();  //遥控器失联处理
        }
    
    PubPushMessage(mode_switch_pub, (void *)&ctrl_mode);
        osDelay(5);
    }
}

/* 遥控器连接状态检查函数 */
static uint8_t rc_normal_check(void)
{
    if( status.rc_status == 0 ) return 0;
    else                        return 1;
}

/* 遥控器失联处理函数 */
static void rc_abnormal_proess(void)
{
    lock_flag = 0;  //需要重新解锁
    ctrl_mode = PROTECT_MODE;
}

/* 系统模式切换函数 */
static void sw1_mode_handler(void)  //由拨杆1决定系统模式切换，主要是云台、底盘和发射器
{
    switch( rc.sw1 )
    {
        case RC_UP:
        {
            ctrl_mode = REMOTER_MODE;
        }
        break;
        case RC_MI:
        {
            ctrl_mode = PROTECT_MODE;
        }
        break;
        case RC_DN:
        {
          ctrl_mode = AUTO_MODE;

        }
        break;
        default: break;
    }
}
/* 解锁函数 */
 static void unlock_init(void)
{
    if( rc.sw1 == RC_MI && rc.sw2 == RC_UP )//左拨杆居中，右拨杆置上
    {
        if( rc.ch4 == -660 && rc.ch3 == 660 )
        {
            lock_flag = 1;  //左控制杆拨至右下
        }
    }
}
