#include "modeswitch_task.h"
#include "control_def.h"
#include "cmsis_os.h"

#include "remote_msg.h"
#include "bsp_can.h"
#include "bsp_vision.h"
#include "bsp_TriggerMotor.h"

#include "chassis_task.h"
#include "gimbal_task.h"
#include "shoot_task.h"
#include "status_task.h"

static void vision_mode_switch(void);
static uint8_t rc_normal_check(void);
static void unlock_init(void);
static void sw1_mode_handler(void);
static void rc_abnormal_proess(void);

/* 系统状态机 */
ctrl_mode_e ctrl_mode;

/* 视觉状态机 */
vision_mode_e vision_mode;

/* 解锁标志 */
//uint8_t lock_flag = 0;
uint8_t lock_flag =1;
/* 系统模式切换任务函数 */
void mode_switch_task(void const *argu)
{
    for(;;)
    {
        if( !lock_flag )
        {
            unlock_init();  //解锁操作
        }
        else if( rc_normal_check() )
        {
            sw1_mode_handler();  //根据左拨杆切换系统模式
            vision_mode_switch();  //根据鼠标右键 与 若干辅助按键 切换视觉模式
        }
        else
        {
            rc_abnormal_proess();  //遥控器失联处理
        }
//        sw1_mode_handler();  //校准小蜜蜂电调时用，未知bug！！
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
           /* if( rc.mouse.r == 1 )  //视觉模式，右键开启
           // {
                ctrl_mode = VISION_MODE;
           // }
            else
           // {
                ctrl_mode = KEYBOARD_MODE;  //键盘模式，非视觉模式下
          //  }*/
          ctrl_mode = AUTO_MODE;

        }
        break;
        default: break;
    }
}

/* 视觉状态切换 */
static void vision_mode_switch(void)
{
    static ctrl_mode_e last_ctrl_mode = PROTECT_MODE;
    
    /* 视觉状态按键扫描 */
    keyboard_scan(KEY_VISION_bENERGY);
    keyboard_scan(KEY_VISION_sENERGY);
    keyboard_scan(KEY_VISION_ANTIROTATE);
    keyboard_scan(KEY_VISION_SENTRY);
    
    /* 视觉状态切换 */
    if( FLAG_VISION_SENTRY )  // 反哨兵
         vision_mode = VISION_MODE_SENTRY;
    else if( FLAG_VISION_bENERGY )  //大能量 
        vision_mode = VISION_MODE_bENERGY;
    else if( FLAG_VISION_sENERGY )  //小能量
        vision_mode = VISION_MODE_sENERGY;
    else if( FLAG_VISION_ANTIROTATE )  //反小陀螺
        vision_mode = VISION_MODE_ANTIROTATE;
    else
        vision_mode = VISION_MODE_AUTO;  //自瞄
    
    /* 空闲位---能量机关方向 */
    if( key_scan_clear(KEY_VISION_ENERGY_DIR) )  //反了就按
        vision_tx_msg.mode_msg.vacancy = !vision_tx_msg.mode_msg.vacancy;  //暂时不用
    
    /* 主动退出视觉模式（松开右键时）*/
    if( last_ctrl_mode == VISION_MODE && ctrl_mode != VISION_MODE )  
    {
        /* 视觉保留位复位 */
        vision_tx_msg.mode_msg.vacancy = 0;  //默认顺时针
        /* 清除视觉标志位 */
        key_status_clear(KEY_VISION_sENERGY);
        key_status_clear(KEY_VISION_bENERGY);
        key_status_clear(KEY_VISION_SENTRY);
        key_status_clear(KEY_VISION_ANTIROTATE);
    }
    
    /* 系统历史状态更新 */
    last_ctrl_mode = ctrl_mode;
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
