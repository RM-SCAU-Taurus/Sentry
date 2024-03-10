#ifndef __BSP_POWERLIMIT_H
#define __BSP_POWERLIMIT_H

#include "stdint.h"
#include "pid.h"

#define  SUPERCAP_MAX_VOLAGE				24.0f	//������������ѹ
#define	 SUPERCAP_DISCHAGER_VOLAGE	12.5f	//����������С�ŵ��ѹ(С�ڸõ�ѹ���������������)
#define  SUPERCAP_LIMIT_VOLAGE			13.5f //�������ݿ�ʼ���ƹ��ʵĵ�ѹ//13.5

typedef struct
{
	float     max_power;					//���޹���(����ϵͳ����)
	float			supply_power;				//��Դ�������(���ݿ��ư巴��)
	float     chassis_power;			//ʵ�ʵ��̹���(���ʿ��ư巴��) 1KHz
	float     chassis_3508_power;
	float     chassis_volage;     //���̵�ѹ
	float     chassis_current;     //���̵���
	float		  power_buffer;				//��ǰ��������ʣ��ֵ
	uint8_t   status;							//���Ʊ�־λ��1ʱ���Ƶ�����0ʱ������
	float     limit_kp;						//���Ʊ���
	float   	limit_temp;					//���Ʊ���
	uint8_t   cnt;								//������Ϣ�Լ�λ���ж���Ϣ��������
}powercontrol_t;

typedef struct
{
	uint8_t	 mode;								//�������ݳ��ģʽ(0Ϊ���䲻�� 1Ϊֻ�䲻�� 2Ϊ�߳�߷�)
	uint8_t  volume_percent;			//�������������ٷֱ� ����UI��ʾ
	float 	 volage;							//�������ݵ�ѹ
	float 	 charge_power_ref;		//�������ݳ�繦��Ŀ��ֵ
	float 	 charge_power_fdb;		//��ǰ�������ݳ�繦�ʵķ���ֵ �ɵ�Դ����-���̹��ʵó� ������ȫ��ȷ
	float 	 charge_current_set;	//�������ݳ������������������ݰ�,���������
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
