#include "bsp_powerlimit.h"
#include "chassis_task.h"
#include "scope_ptl.h"
#include "usart.h"
#include "math_calcu.h"
#include "math.h"
#include "string.h"
#include "bsp_can.h"
#include "modeswitch_task.h"
/***********************************/
#define 	BUFFER_MIN				40.0f		//��С�������� С�ڸ�ֵ�Ϳ�ʼ���Ƶ���
#define   BUFFER_MIN_6020   15.0f
#define   POWER_OFFSET			3.0f
/***********************************/

powercontrol_t powercontrol = {0};
supercap_t supercap;
float test;

/**
	* @brief      ���ʿ��Ƴ�ʼ��
	* @author         
	* @param[in]  
	* @retval     
	*/
void PowerControl_Init(void)
{
	powercontrol.max_power = 50;
	chassis.wheel_max = 5500;
	powercontrol.power_buffer = 60.0f;
	powercontrol.limit_kp  = 0.4f;
}

//uint16_t test_power_wheel = 10000;

/**
	* @brief      ���̹�����Ϣ����
	* @author     
	* @param[in]  
	* @retval     
	*/
void Power_Data_Update(void)
{
	/* 60J������������(ģ�����ϵͳ) */
	//powercontrol.power_buffer -= ((powercontrol.supply_power-POWER_OFFSET) - powercontrol.max_power) * CHASSIS_PERIOD *0.001f;
	if(powercontrol.power_buffer > 60.0f)
		powercontrol.power_buffer = 60.0f;
	
		/* ���ݵ��̹������޸���������� */
	/*�������ݷŵ�*/
	if(chassis.kb_C && supercap.volage >= 13.5f&&supercap.flag==1)
		chassis.wheel_max = 8500;
	/*Ѫ������1��*/
	else if(powercontrol.max_power == 45)
		chassis.wheel_max = 4550;
	else if(powercontrol.max_power == 50)
		chassis.wheel_max = 5500;
	else if(powercontrol.max_power == 55)
		chassis.wheel_max = 5800;
	/*��������1��*/
	else if(powercontrol.max_power == 60)
		chassis.wheel_max = 6200;//6700
	else if(powercontrol.max_power == 80)
		chassis.wheel_max = 6500;
	else if(powercontrol.max_power == 100)
		chassis.wheel_max = 7600;
    else if(powercontrol.max_power == 120)
		chassis.wheel_max = 7650;
	else if(powercontrol.max_power==255)
		chassis.wheel_max = 7650;
	/*��ʼ0��(50w)*/
	else
//		chassis.wheel_max = 4650;
////	else
		chassis.wheel_max = 5500;
}

/**
	* @brief       ���̹��ʿ��ƺ���
	* @author         
	* @param[in]  
	* @retval     
	*/
void Power_Control(int16_t* current_3508,int16_t* current_6020)
{
	static uint8_t powerlimit_cnt = 0;

	/* ���ݲ��ŵ����ݵ����������Ƶ�ѹ�ͽ��й��ʿ��� */
	if(supercap.mode != 2 || supercap.volage < SUPERCAP_LIMIT_VOLAGE)
	{
		if(powercontrol.power_buffer < BUFFER_MIN)
		{
			powercontrol.status = 1;
			powerlimit_cnt = 0;
			/* �������Ʊ��� */
			powercontrol.limit_temp = powercontrol.limit_kp * (BUFFER_MIN - powercontrol.power_buffer);
		}

		if(powercontrol.status)
		{
			powerlimit_cnt++;
			for(uint8_t i=0; i<4; i++)
			{
//				current_3508[i] /= (ABS(powercontrol.limit_temp) + 1.0f);
//				if(powercontrol.power_buffer<=BUFFER_MIN_6020)
//				{ current_6020[i]/=(ABS(powercontrol.limit_temp)/10 + 1.0f); test++;}
				current_3508[i] /=1.0f;
			}
		}

		/* ����100ms ��50�ι��� */
		if(powerlimit_cnt >= 50)
		{
			powercontrol.status = 0; //����״̬λ����
			powerlimit_cnt = 0;			 //���Ƽ���λ����
		}
	}
}

/**
	* @brief       �������ݿ��ƺ���
	* @author         
	* @param[in]  
	* @retval     
	*/
void SuperCap_Control(void)
{
	/* ��������ģʽ���� */
	SuperCap_Mode_Update();

	/* �������ݱ���ģʽ */
	if(supercap.mode == 0)
	{
		supercap.charge_power_ref 	= 0;
		supercap.charge_current_set = 0;
	}
	/* ��������ֻ���ģʽ */
	else if(supercap.mode == 1)
	{
		/*��ʵ��繦��=��Դ����-���̹���*/
		supercap.charge_power_fdb = powercontrol.supply_power - powercontrol.chassis_power;
		/*Ŀ���繦��(���й���)=�����-���̹���*/
		supercap.charge_power_ref = powercontrol.max_power - powercontrol.chassis_3508_power - 10.0f;	//��繦��Ҫ�Ե���������
		if(supercap.charge_power_ref < 0)
			supercap.charge_power_ref = 0;
		/*������=���й���/�������ݵ�ѹ(�㹦�ʳ��)*/
		supercap.charge_current_set = supercap.charge_power_ref / supercap.volage;
		if(supercap.charge_current_set < 0)
			supercap.charge_current_set = 0;
		else if(supercap.charge_current_set > 10)
			supercap.charge_current_set = 10;
	}

	/* �������ݱ߳�߷�ģʽ */
	else if(supercap.mode == 2)
	{
		/* ���ó�繦�� */
		supercap.charge_power_ref = powercontrol.max_power -powercontrol.chassis_3508_power-10.0f;
	//	supercap.charge_power_ref = powercontrol.max_power -20.0f;
		if(supercap.charge_power_ref < 0)
			supercap.charge_power_ref = 0;
		/* ���ó����� */
		supercap.charge_current_set = supercap.charge_power_ref / supercap.volage;
		if(supercap.charge_current_set < 0)
			supercap.charge_current_set = 0;
		else if(supercap.charge_current_set > 10)
			supercap.charge_current_set = 10;
	}
}

/**
	* @brief       �������ݳ�ŵ�ģʽ����
	* @author         
	* @param[in]  
	* @retval     
	*/
void SuperCap_Mode_Update(void)
{
	if(ctrl_mode == PROTECT_MODE)
	{
		supercap.mode = 0;
		supercap.flag=0;
	}
	else if(chassis.kb_C)
	{
		/* �����ѹ����13.5V���ɽ��г�ŵ���� */
		if(supercap.volage >= SUPERCAP_LIMIT_VOLAGE&&supercap.flag==1)
			supercap.mode = 2;
		/* ���̹������ƣ�ֹͣ��� */
		else 
			supercap.mode = 0;
	}
	else
	{
		/* ���̹������ƣ�ֹͣ��� */
		if(powercontrol.power_buffer <= BUFFER_MIN)
			supercap.mode = 0;
		/* �ŵ��������͵�ѹС��12.5V���ҵ����õĹ��ʲ���ʱ��� */
		else
			supercap.mode = 1;
	}
	if(supercap.volage<13.5f)
	{
	  supercap.flag=0;
	}
	else if(supercap.volage>=15.0f)
	{
	  supercap.flag=1;
	}
}

void Power_data_handler(uint32_t can_id,uint8_t * CAN_Rx_data)
{
	uint16_t supercap_volage_buf=0;
	uint16_t supply_power_buf=0;
	uint16_t chassis_power_buf=0;
	
	switch(can_id)
  {
		case CAN_POWER_ID:
		{

				memcpy(&supercap_volage_buf,CAN_Rx_data,2);
				memcpy(&supply_power_buf,(CAN_Rx_data+2),2);
				memcpy(&chassis_power_buf,(CAN_Rx_data+4),2);
			
				supercap.volage = supercap_volage_buf / 100.0f;
			  
			 //  powercontrol.chassis_current = supercap_volage_buf/100.0f;
			 //  powercontrol.chassis_volage  = supply_power_buf/100.0f;
				supercap.volume_percent=(supercap.volage-SUPERCAP_DISCHAGER_VOLAGE)/(SUPERCAP_MAX_VOLAGE-SUPERCAP_DISCHAGER_VOLAGE)*100;
				powercontrol.supply_power = supply_power_buf/100.0f;
				powercontrol.chassis_3508_power = chassis_power_buf/100.0f;			
			break;
		}
	}
}
