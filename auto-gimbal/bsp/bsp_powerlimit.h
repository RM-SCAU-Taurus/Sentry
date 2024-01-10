#ifndef __BSP_POWERLIMIT_H
#define __BSP_POWERLIMIT_H

#include "stdint.h"
#include "pid.h"

#define POWERLIMIT_CNT			    50

#define SUPERCAP_DISCHAGER_VOLAGE	12.5f		//超级电容最大放电电压 小于该电压 电机不能正常运作

typedef struct
{
    float       judge_power;			//裁判系统反馈回来的底盘实时功率
    uint16_t    judge_power_buffer;     //裁判系统反馈回来的底盘缓冲能量

    float       chassis_power;	        //实际底盘功率 1kHz
    float       supply_power;		    //电源输出功率
    float       max_power;			    //上限功率
    float		power_buffer;		    //当前缓存能量剩余值
    float       limit_kp;				//限制比例
    float       limit_temp;		        //限制倍数
    uint8_t     status;			        //限制标志位，1时限制电流，0时不限制
    uint8_t     cnt;					//功率信息自加位，判断信息的连续性
} powercontrol_t;

typedef struct
{
    uint8_t mode;				//超级电容充电模式位 0为保护 不充电不放电 1为只充不放 2为边充边放  3为只放电
    float volage;				//超级电容电压 反映超级电容容量
    float power;
    uint8_t volume_percent;		//超级电容容量百分比 用于UI显示
    float charge_power_fdb;		//当前超级电容充电功率的返回值 由电源功率-底盘功率得出 并不完全精确
    float charge_power_ref;		//超级电容充电功率目标值
    float charge_current_set;	//超级电容充电电流，发给超级电容板,控制他充电
} supercap_t;

extern powercontrol_t powercontrol;
extern supercap_t supercap;
extern uint8_t supercap_status_flag;

void Power_Control(int16_t * current);
void PowerControl_Init(void);
void PowerParam_Update(void);
void SuperCap_Control(void);	//超级电容状态更新函数
void Power_data_handler(uint32_t can_id,uint8_t * CAN_Rx_data);
void can1_send_supercap(void);
void SuperCap_Mode_Update(void);


#endif
