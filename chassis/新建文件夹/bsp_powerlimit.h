#ifndef __BSP_POWERLIMIT_H
#define __BSP_POWERLIMIT_H

#include "stdint.h"
#include "pid.h"

#define  SUPERCAP_MAX_VOLAGE				24.0f	//超级电容最大电压
#define	 SUPERCAP_DISCHAGER_VOLAGE	12.5f	//超级电容最小放电电压(小于该电压电机不能正常运作)
#define  SUPERCAP_LIMIT_VOLAGE			13.5f //超级电容开始限制功率的电压//13.5

typedef struct
{
	float     max_power;					//上限功率(裁判系统反馈)
	float			supply_power;				//电源输出功率(电容控制板反馈)
	float     chassis_power;			//实际底盘功率(功率控制板反馈) 1KHz
	float     chassis_3508_power;
	float     chassis_volage;     //底盘电压
	float     chassis_current;     //底盘电流
	float		  power_buffer;				//当前缓存能量剩余值
	uint8_t   status;							//限制标志位，1时限制电流，0时不限制
	float     limit_kp;						//限制比例
	float   	limit_temp;					//限制倍数
	uint8_t   cnt;								//功率信息自加位，判断信息的连续性
}powercontrol_t;

typedef struct
{
	uint8_t	 mode;								//超级电容充电模式(0为不充不放 1为只充不放 2为边充边放)
	uint8_t  volume_percent;			//超级电容容量百分比 用于UI显示
	float 	 volage;							//超级电容电压
	float 	 charge_power_ref;		//超级电容充电功率目标值
	float 	 charge_power_fdb;		//当前超级电容充电功率的返回值 由电源功率-底盘功率得出 并不完全精确
	float 	 charge_current_set;	//超级电容充电电流，发给超级电容板,控制他充电
}supercap_t;

extern powercontrol_t powercontrol;
extern supercap_t supercap;

void PowerControl_Init(void);
void Power_Data_Update(void);
void Power_Control(int16_t* current_3508,int16_t* current_6020);
void SuperCap_Control(void);
void SuperCap_Mode_Update(void);
void Power_data_handler(uint32_t can_id,uint8_t * CAN_Rx_data);

#endif
