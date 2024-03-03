/**
  ******************************************************************************
  * FileName    : bsp_powerlimit.c
  * Version     : v1.0
  * Author      : BZW
  * Date        : 2021-04-24
  * modification：
  * Functions   :
  * Description :
  *         功率控制逻辑：
  *         目标：比赛全程，底盘实际功率接近当前最大功率，当前剩余缓存能量一直无法恢复（即满功率状态）
  *         分析：
  *               1. 缓存能量可以设置为30J，比较安全，哪怕轮速限制放开，电流限制比例降低，也有比较充足的空间限制住功率
  *               2. 电流限制比例，在能实现功能的前提下，越小越好。实际效果：限功率时行驶比较顺滑
  *               3. 轮速限制：在能实现功能的前提下，越大越好。基本设置为启动和刹车时的最大轮速即可。
  *                            即使限制放开，能达到的速度也是有限的，只要缓存能量足够，加上电流限制，总能限制回来。
  *
  *
  ******************************************************************************
  */
#include "bsp_powerlimit.h"
#include "chassis_task.h"
#include "DataScope_DP.h"
#include "usart.h"
#include "math_calcu.h"
#include "math.h"
#include "remote_msg.h"
#include "string.h"
#include "bsp_can.h"
#include "can.h"
#include "modeswitch_task.h"
#include "pid.h"
#include "bsp_judge.h"
#include "control_def.h"
#include "status_task.h"
/* ----------------------------------------------------------------------------------------------------------------------- */
/* 功率控制要调的参数：最小缓冲能量   最大功率 */
#define 	BUFFER_MIN	   40.0f	//预测缓冲能量小于该值则进行电流分配  30
uint8_t   MAX_POWER_JUDGE = 100;		//裁判系统给的机器人最大功率 该变量用于给初值  接收到裁判系统时会按裁判系统信息更新

/* ----------------------------------------------------------------------------------------------------------------------- */
extern CAN_TxHeaderTypeDef Tx1Message;
extern CAN_RxHeaderTypeDef Rx1Message;

powercontrol_t powercontrol = {0};
supercap_t supercap;
uint8_t supercap_status_flag = 0;

void PowerControl_Init(void)
{
    powercontrol.max_power = MAX_POWER_JUDGE;//自动步兵非裁判系统模式下默认120W
    powercontrol.limit_kp = 0.25f;  //过大会犬吠 0.4
    chassis.keyboard_input = 40.0f;
    powercontrol.power_buffer = 60.0f;
    powercontrol.ParamUpdate = PowerParam_Update;
}

//float test_power = 5800.0f;  //5600 80W
//float test_input = 42.0f;
//float test_kp    = 0.2;
void PowerParam_Update(void)
{
    /* 裁判系统反馈赋值 */
    powercontrol.judge_power = Power_Heat_Data.chassis_power; 
//		powercontrol.judge_power =MAX_POWER_JUDGE;	//更新数据但不用
    powercontrol.judge_power_buffer = Power_Heat_Data.chassis_power_buffer;  //更新裁判系统回传的缓冲能量
	/*裁判系统功率控制数据直接由云台板can1传下来*/
    /* 缓冲能量限幅 */
    if( powercontrol.power_buffer >= MAX_POWER_JUDGE )	 
        powercontrol.power_buffer = MAX_POWER_JUDGE;

    /* 底盘最大轮速刷新 */
    if( rc.kb.bit.SHIFT && supercap.volage > 14.0f )  //超级电容电压比电调最低工作电压高，且有余量供加速放电时
    {
        chassis.wheel_max = SPEED_SUPERCAP;
        chassis.keyboard_input = 50.0f;
    }
    else if( chassis.mode == CHASSIS_MODE_KEYBOARD_SUPPLY )  //补给模式：左右反向，速度上限和灵敏度降低
    {
        chassis.wheel_max = SPEED_SUPPLY;
        chassis.keyboard_input = 10.0f;
    }
    else if(powercontrol.max_power == 45 )
    {
        chassis.wheel_max = SPEED_45W;
        chassis.keyboard_input = 35.0f;
    }
    else if( powercontrol.max_power == 50 )
    {
        chassis.wheel_max = SPEED_50W;
        chassis.keyboard_input = 40.0f;
    }
    else if(powercontrol.max_power == 55 )
    {
        chassis.wheel_max = SPEED_55W;
        chassis.keyboard_input = 40.0f;
    }
    else if( powercontrol.max_power == 60 )
    {
        chassis.wheel_max = SPEED_60W;
        chassis.keyboard_input = 40.0f;
    }
    else if( powercontrol.max_power == 80 )
    {
        chassis.wheel_max = SPEED_80W;
        chassis.keyboard_input = 40.0f;
    }
    else if( powercontrol.max_power == 100 )
    {
        chassis.wheel_max = SPEED_100W;
        chassis.keyboard_input = 40.0f;
    }
    else if( powercontrol.max_power == 120 || powercontrol.max_power >120 )
    {
        chassis.wheel_max = SPEED_120W;
        chassis.keyboard_input = 45.0f;
    }
    else  //功率规则外
    {
        chassis.wheel_max = SPEED_100W ;
        chassis.keyboard_input = 40.0f;
    }
}


/**
  * @name     supercap_mode_update
  * @brief    超级电容模式切换（哨兵定制）
  * @param    None
  * @retval
  * @date     2023-05-12
  * @note     0：保护模式
  *           1：只充电模式
  *           2：充放电模式（电源管理模块全部给电容充电，电容给底盘放电，
  *                         电容电压降低时：电流闭环，力矩下降）
  */
void supercap_mode_update(void) {
    if (!lock_flag  || !status.power_control ) {  /*系统处于保护模式 或电容通信异常 */
        supercap.mode = CAP_PROTECT_MODE;
    }
		else if( ctrl_mode == PROTECT_MODE&&lock_flag &&status.power_control)//未解锁只充不放
		{
			supercap.mode = CAP_CHARGE_MODE;
		}
		else  {  /*<! 哨兵默认使用电容 !>*/
        if (supercap.volage >= 14.0) {  /* 电容能量足够 */
            supercap.mode = CAP_DISCHARGE_MODE;
        } else if (powercontrol.judge_power_buffer <= 30) {  /* 缓存能量较低，功率无剩余 */
            supercap.mode = CAP_PROTECT_MODE;
        } else {  /* 电容能量不足，功率有剩余 */
            supercap.mode = CAP_CHARGE_MODE;
        }
    }
}
/**
  * @name     supercap_control
  * @brief    超级电容充放电控制
  */
void supercap_control(void)
{
    /*反馈实时充电功率 = 电容控制板检测到的电管Chassis输出功率 - 电容控制板检测到的底盘功率*/
    supercap.charge_power_fdb = powercontrol.supply_power - powercontrol.chassis_power;
    
	/* 电容模式行为控制 */
	if (supercap.mode == CAP_PROTECT_MODE) { /* 保护模式：不充不放 */
		supercap.charge_power_ref 	= 0;
		supercap.charge_current_set = 0;
    } else {
        if (supercap.mode == CAP_CHARGE_MODE) { /* 只充电模式：空闲功率给电容充电 (空闲功率) = 最大功率 - 底盘功率 */
            supercap.charge_power_ref = powercontrol.max_power - powercontrol.chassis_power - 10.0f;	/* 充电功率要略低于理想充电 */
        } else if (supercap.mode == CAP_DISCHARGE_MODE) { /* 边充边放模式：电管全部给电容充电，电容给底盘供电 */
					supercap.charge_power_ref = powercontrol.max_power-powercontrol.max_power/2- powercontrol.chassis_power ;//空闲功率给电容充电 (空闲功率) = 最大功率 - 底盘功率 */
				}
        
        /* 充电功率限幅 */
        if (supercap.charge_power_ref < 0) {  
            supercap.charge_power_ref = 0;
        }
        
        /*设置充电电流=空闲功率/超级电容电压(恒功率充电)*/
		supercap.charge_current_set = supercap.charge_power_ref / supercap.volage; 
		if (supercap.charge_current_set < 0) {
			supercap.charge_current_set = 0;
		} else if (supercap.charge_current_set > 10) {
			supercap.charge_current_set = 10;
        }
    }
}

/**
  * @name     Power_Control
  * @brief    功率控制
  * @param    current:底盘电机电流数组首地址
  * @retval
  * @date     2021-04-24
  * @note
  */
void Power_Control(int16_t * current,int16_t* current_6020)
{
		supercap_mode_update(); //超级电容模式切换
    supercap_control();     //超级电容充放电控制
    static uint8_t powerlimit_cnt=0;  //限制计数位	每次缓冲能量低于5后限制100ms 即限制50次功率
		static uint8_t powerlimit_cnt_35=0;
    if ( supercap.mode!=2 || supercap.volage<14.0f )  //超级电容非放电模式下才进行6020功率控制
    {
        if ( powercontrol.power_buffer < BUFFER_MIN )
        {
            powercontrol.status=1;  //缓冲能量小时限制状态位 置1
            powerlimit_cnt = 0;
            powercontrol.limit_temp= powercontrol.limit_kp * (BUFFER_MIN - powercontrol.power_buffer);	//计算限制比例
        }

        if ( powercontrol.status )
        {
            powerlimit_cnt++;
            for(uint8_t i=0; i<4; i++)
            {
								if(powercontrol.power_buffer<= 30)//增加6020功率控制
								{
//									current_6020[i]/=( ABS(powercontrol.limit_temp)*1.3 + 1.0f );
										current_6020[i]/=( 1.0f );
								}
						}
        }

        if ( powerlimit_cnt >= POWERLIMIT_CNT )
        {
            powercontrol.status = 0;  //限制状态位清零
            powerlimit_cnt = 0;  //限制计数位清零
        }
    }
		
		  if ( powercontrol.power_buffer < BUFFER_MIN )//3508功率控制时刻进行
        {
            powercontrol.status_35=1;  //缓冲能量小时限制状态位 置1
            powerlimit_cnt_35 = 0;
            powercontrol.limit_temp= powercontrol.limit_kp * (BUFFER_MIN - powercontrol.power_buffer);	//计算限制比例
        }
				    if ( powercontrol.status_35)
        {
            powerlimit_cnt_35++;
            for(uint8_t i=0; i<4; i++)
            {
//                current[i] /= ( ABS(powercontrol.limit_temp) + 1.0f ) ;
							
						       current[i] /=  ( 1.0f ) ;	
						}
        }
				   if ( powerlimit_cnt_35 >= POWERLIMIT_CNT )
        {
            powercontrol.status_35 = 0;  //限制状态位清零
            powerlimit_cnt_35 = 0;  //限制计数位清零
        }
}






uint32_t test_run_period = 0;
/**
  * @name   Power_data_handler
  * @brief
  * @param  can_id: 数据来源CAN
  *			CAN_Rx_data: CAN接收数据缓存
  * @retval
  * @note   电容控制板测得电容电压与电源管理模块的chassis输出功率
  *         功率控制板测得底盘实际功率
  */
void Power_data_handler(uint32_t can_id,uint8_t * CAN_Rx_data)
{
    uint16_t supercap_voltage_buf = 0;
    uint16_t source_power_buf = 0;
    uint16_t chassis_power_buf = 0;
    
    supercap_status_flag = 1;  //超级电容通信状态正常标志
    static uint32_t run_time,last_run_time = 0;
    run_time = HAL_GetTick();
    test_run_period = run_time - last_run_time;
    
    memcpy(&supercap_voltage_buf, CAN_Rx_data, 2);
    memcpy(&source_power_buf, (CAN_Rx_data+2), 2);
    memcpy(&chassis_power_buf, (CAN_Rx_data+4), 2);

    /* 电容电压信息 */
    supercap.volage = supercap_voltage_buf / 100.0f;
    supercap.volume_percent = (supercap.volage - SUPERCAP_DISCHAGER_VOLAGE) /
                              (SUPERCAP_MAX_VOLAGE - SUPERCAP_DISCHAGER_VOLAGE) * 100;
    /* 电源功率 */
    powercontrol.supply_power = source_power_buf /100.0f;

    /* 底盘实时功率信息 */
    powercontrol.chassis_power = chassis_power_buf / 100.0f;

}

