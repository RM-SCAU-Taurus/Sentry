/*
 * @Author: your name
 * @Date: 2022-01-12 16:36:31
 * @LastEditTime: 2022-01-13 19:41:40
 * @LastEditors: Please set LastEditors
 * @Description: 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 * @FilePath: \MDK-ARMd:\RM\RM-auto-Infatry\auto_Infatrty\chassis\bsp\bsp_powerlimit.h
 */
#ifndef __BSP_POWERLIMIT_H
#define __BSP_POWERLIMIT_H

#include "stdint.h"
#include "pid.h"

#define POWERLIMIT_CNT			    55

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
	 uint8_t     status_35;			        //限制标志位，1时限制电流，0时不限制
    uint8_t     cnt;					//功率信息自加位，判断信息的连续性
    void(*ParamUpdate)();
} powercontrol_t;

typedef enum {
    CAP_PROTECT_MODE    = 0,  //不冲不放模式
    CAP_CHARGE_MODE     = 1,  //只冲不放模式
    CAP_DISCHARGE_MODE  = 2   //边冲边放模式
} cap_mode_t;

typedef struct
{
    cap_mode_t mode;			//超级电容模式
    float volage;				//超级电容电压 反映超级电容容量
    float power;
    uint8_t volume_percent;		//超级电容容量百分比 用于UI显示
    float charge_power_fdb;		//当前超级电容充电功率的返回值 由电源功率-底盘功率得出 并不完全精确
    float charge_power_ref;		//超级电容充电功率目标值
    float charge_current_set;	//超级电容充电电流，发给超级电容板,控制他充电

    float charge_volume_max;
    float discharge_volume_min;
	uint8_t flag;
} supercap_t;
extern supercap_t supercap;
extern powercontrol_t powercontrol;
void supercap_mode_update(void);
void supercap_control(void);
void Power_Control(int16_t * current,int16_t* currrent_6020);
void PowerControl_Init(void);
void PowerParam_Update(void);
void SuperCap_Control(void);	//超级电容状态更新函数
void Power_data_handler(uint32_t can_id,uint8_t * CAN_Rx_data);
void SuperCap_Mode_Update(void);

#endif
