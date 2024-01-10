/**
  * @file     remote_msg.c
  * @version  v3.0
  * @date     July,22 2019
  *
  * @brief    遥控器数据解算文件
  *
  *	@author   ZZJ
  *
  */
#include "remote_msg.h"
#include "stdlib.h"
#include "string.h"
#include "gimbal_task.h"
#include "modeswitch_task.h"

/**
  * @brief 遥控器状态编码
  */
/* 遥控左侧拨杆 */
#define RC_LEFT_LU_CH_VALUE  ( rc.ch3 == -660 && rc.ch4 ==  660 ) // 居左上
#define RC_LEFT_RU_CH_VALUE  ( rc.ch3 ==  660 && rc.ch4 ==  660 ) // 居右上
#define RC_LEFT_RD_CH_VALUE  ( rc.ch3 ==  660 && rc.ch4 == -660 ) // 居右下
#define RC_LEFT_LD_CH_VALUE  ( rc.ch3 == -660 && rc.ch4 == -660 ) // 居左下

/* 遥控右侧拨杆 */
#define RC_RIGHT_LU_CH_VALUE ( rc.ch2 ==  660 && rc.ch1 == -660 )   // 居左上
#define RC_RIGHT_RU_CH_VALUE ( rc.ch2 ==  660 && rc.ch1 ==  660 )   // 居右上
#define RC_RIGHT_RD_CH_VALUE ( rc.ch2 == -660 && rc.ch1 ==  660 )   // 居右下
#define RC_RIGHT_LD_CH_VALUE ( rc.ch2 == -660 && rc.ch1 == -660 )   // 居左下

/* 遥控器数据 */
rc_info_t rc;

/* 遥控器数据传输状态标志 */
uint8_t rc_normal_flag = 0;

/* 灵敏度数据 */
scale_t scale;

/* 键盘按键状态标志位 */
uint8_t kb_status[11]= {0};

/**
  * @brief       handle received rc data
  * @param[out]  rc:   structure to save handled rc data
  * @param[in]   buff: the buff which saved raw rc data
  * @retval
  */
void rc_callback_handler(rc_info_t *rc, uint8_t *buff)
{
    /* 遥控器通信检测 */
    rc_normal_flag = 1;
    /* 数据解算 */
    rc->ch1 = (buff[0]      | buff[1]  << 8) & 0x07FF;
    rc->ch1 -= 1024;
    rc->ch2 = (buff[1] >> 3 | buff[2]  << 5) & 0x07FF;
    rc->ch2 -= 1024;
    rc->ch3 = (buff[2] >> 6 | buff[3]  << 2 | buff[4] << 10) & 0x07FF;
    rc->ch3 -= 1024;
    rc->ch4 = (buff[4] >> 1 | buff[5]  << 7) & 0x07FF;
    rc->ch4 -= 1024;
    rc->ch5=  (buff[16]     | buff[17] << 8) & 0x07FF;
    rc->ch5 -= 1024;
    rc->sw1 = ((buff[5] >> 4) & 0x000C) >> 2;
    rc->sw2 = (buff[5] >> 4) & 0x0003;

    if ((abs(rc->ch1) > 660) || \
            (abs(rc->ch2) > 660) || \
            (abs(rc->ch3) > 660) || \
            (abs(rc->ch4) > 660)    )
    {
        memset(rc, 0, sizeof(rc_info_t));
    }
    rc->mouse.x = buff[6] | (buff[7] << 8); // x axis
    rc->mouse.y = buff[8] | (buff[9] << 8);
    rc->mouse.z = buff[10] | (buff[11] << 8);

    rc->mouse.l = buff[12];
    rc->mouse.r = buff[13];

    rc->kb.key_code = buff[14] | buff[15] << 8; // key borad code
}

/**
  * @brief       按键枚举常量映射函数
  * @param[out]  key: 所需按键当前状态
  * @param[in]   key_index: 需要检测的按键
  * @author      ZZJ
  */
static uint16_t key_map(key_index_e key_index)
{
    uint16_t key;
    switch( key_index )
    {
    case KB_Q:
        key = rc.kb.bit.Q;
        break;
    case KB_E:
        key = rc.kb.bit.E;
        break;
    case KB_R:
        key = rc.kb.bit.R;
        break;
    case KB_F:
        key = rc.kb.bit.F;
        break;
    case KB_G:
        key = rc.kb.bit.G;
        break;
    case KB_Z:
        key = rc.kb.bit.Z;
        break;
    case KB_X:
        key = rc.kb.bit.X;
        break;
    case KB_C:
        key = rc.kb.bit.C;
        break;
    case KB_V:
        key = rc.kb.bit.V;
        break;
    case KB_B:
        key = rc.kb.bit.B;
        break;
    case KB_CTRL:
        key = rc.kb.bit.CTRL;
        break;
    default:
        break;
    }
    return key;
}

/**
  * @brief       键盘按键扫描
  * @param[out]  kb_status[key_index]: 按键按下并弹起后，状态标志位取反
  * @param[in]   key_index：	       被检测的按键序号
  * @note        按键按下后，给相应标志位取反,不支持长按
  * @author      ZZJ
  */
void keyboard_scan(key_index_e key_index)
{
    static uint8_t key_press_enable[11] = {0};
    uint16_t key = key_map(key_index);  //获取当前按键状态
    if( key && key_press_enable[key_index] == 0 )  //如果当前按键按下且之前没有按下
    {
        key_press_enable[key_index] = 1;  //记录上一次状态：防止重复进入
        kb_status[key_index] = !kb_status[key_index];  //全局状态标志位取反 0-1
    }
    else if( key == 0 )
    {
        key_press_enable[key_index] = 0;  //允许下一次进入
    }
}

/**
  * @brief       键盘状态清零
  * @param[out]  NONE
  * @param[in]   NONE
  * @author      ZZJ
  */
void key_status_clear(key_index_e key_index)
{
    kb_status[key_index] = KEY_END;
}

/**
  * @brief       键盘按键扫描，并清除标志位
  * @param[out]  NONE
  * @param[in]   NONE
  * @note        按键按下后，给相应标志位取反,不支持长按
  * @author      ZZJ
  */
uint8_t key_scan_clear(key_index_e key_index)
{
    uint8_t res = 0;
    keyboard_scan(key_index);  //如果按下一次按键，则反转标志位
    if( kb_status[key_index] == KEY_RUN )
    {
        kb_status[key_index] = KEY_END;  //清除标志位
        res = 1;  //返回 TRUE
    }
    else if( kb_status[key_index] == KEY_END )
        res = 0;
    return res;
}

/**
  * @brief       遥控器拨杆连接启动状态机
  * @param[out]  state: 遥控器上电时的状态
  * @param[in]   trig_flag: 遥控器连接状态标志
  * @author      ZZJ
  */
uint8_t rc_FSM(uint8_t trig_flag)
{
    static uint8_t last_trig_flag;
    static uint8_t state;
    if( trig_flag == 1 && last_trig_flag == 0 )  //检测到触发信号上升沿
    {
        if( RC_LEFT_LU_CH_VALUE )
        {
            state |= RC_LEFT_LU;
        }
        if( RC_LEFT_RU_CH_VALUE )
        {
            state |= RC_LEFT_RU;
        }
        if( RC_LEFT_RD_CH_VALUE )
        {
            state |= RC_LEFT_RD;
        }
        if( RC_LEFT_LD_CH_VALUE )
        {
            state |= RC_LEFT_LD;
        }
        if( RC_RIGHT_LU_CH_VALUE )
        {
            state |= RC_RIGHT_LU;
        }
        if( RC_RIGHT_RU_CH_VALUE )
        {
            state |= RC_RIGHT_RU;
        }
        if( RC_RIGHT_RD_CH_VALUE )
        {
            state |= RC_RIGHT_RD;
        }
        if( RC_RIGHT_LD_CH_VALUE )
        {
            state |= RC_RIGHT_LD;
        }
    }
    else if( trig_flag == 0 && last_trig_flag == 1 )  //遥控器断开连接
    {
        state = 0x00;
    }
    
    last_trig_flag = trig_flag;
    
    return state;
}

/**
  * @brief       遥控器拨杆状态机检查，查看当前遥控器是否处于目标状态
  * @param[out]  boolres:   0或1
  * @param[in]   trig_flag: 遥控器连接状态标志
  * @author      ZZJ
  */
uint8_t rc_FSM_check(uint8_t target_status)
{
    uint8_t res = 0;
    if( rc.init_status & target_status )
    {
        res = 1;
    }
    return res;
}

