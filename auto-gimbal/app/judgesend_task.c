/**
  * @file judgesend_task.c
  * @version 1.0
  * @date Mar,26th 2021
	*
  * @brief UI
	*
  *	@author
  *
  */
#include "bsp_judge.h"
#include "string.h"
#include "usart.h"
#include "bsp_usart.h"
#include "cmsis_os.h"
#include "modeswitch_task.h"
#include "bsp_powerlimit.h"
#include "remote_msg.h"
#include "shoot_task.h"
#include "chassis_task.h"
#include "control_def.h"
#include "crc8_crc16.h"
#include "protocol_camp.h"
#include "bsp_can.h"
unsigned char CliendTxBuffer[150];
/*****************发送数据结构体定义**********************/
ext_SendClient_t									Judgesend_Data;
ext_SendClientDelete_t						Delete_data;
ext_client_custom_character_t			Judgesend_strings_Data;
robot_interactive_data_t					Comm_senddata;
self_inf_t self_inf;
extern map_data_t map_data;
/*********************************************************/
/**
	* @brief  发送自定义数据到电脑客户端
  * @param
  * @retval
  * @attention  数据打包,打包完成后通过串口发送到裁判系统,一个周期只允许发送一帧
  */
void judge_send_task(void const *argu)
{
//		Sentry_map.start_position_x=70;
//	Sentry_map.start_position_y=30;
//	int i=0;
//	for(i=0;i<49;i++)
//	{
//	Sentry_map.delta_x[i]=1;
//	Sentry_map.delta_y[i]=1;
//	}	
    for(;;)
    {	
			judge_get_basedata();
			ui_sendmessage();
        osDelay(100);
    }
}




/**
  * @brief  判断自身ID，选择客户端ID及阵营
  * @param  void
  * @attention
  */
void judge_get_basedata(void)
{
	self_inf.robot_ID = Game_Robot_Status.robot_id;//读取当前机器人ID
	if(self_inf.robot_ID > 100)
	{
		self_inf.color = BLUE;
		self_inf.client_ID = 0x0164 + (self_inf.robot_ID-100);//计算客户端ID
	}else
	{
		self_inf.color = RED;
		self_inf.client_ID = 0x0100 + self_inf.robot_ID;
	}
}

/**
	* @brief  发送给云台手小地图
  * @param  
  * @param  
  * @param  
  * @attention
  */
	uint8_t CliendTxBuffer_message[9+sizeof(Sentry_map)];
void ui_sendmessage(void)
{
	//uint8_t CliendTxBuffer_message[9+sizeof(Sentry_map)];
	judgesend_frame_t message_frame;
	//uint8_t index=0;
	message_frame.txFrameHeader.SOF = 0xA5;			//数据帧头
	message_frame.txFrameHeader.DataLength =sizeof(Sentry_map);
	message_frame.txFrameHeader.Seq =message_frame.txFrameHeader.Seq+1;
//	ui_seq++;
	if(Robot_command.commd_keyboard==65)
	{
		Sentry_map.intention=1;
	}
//	Sentry_map.intention=2;
//	  memcpy(&Sentry_map,&map_data,sizeof(map_data));
	  Sentry_map.intention=map_data.intention;
	  Sentry_map.start_position_x=map_data.start_position_x;
	  Sentry_map.start_position_y=map_data.start_position_y;
	  memcpy(Sentry_map.delta_x,map_data.delta_x,sizeof(map_data.delta_x));
	  memcpy(Sentry_map.delta_y,map_data.delta_y,sizeof(map_data.delta_y));
//	Sentry_map.start_position_x=70;
//	Sentry_map.start_position_y=30;
	//message_frame.CmdID = ID_map_output;
	uint16_t cmd_id=0x0307;
	
	/*for(i=0;i<30;i++)
	{
	Sentry_map.delta_x[i]=1;
	Sentry_map.delta_y[i]=1;
	}*/
//	message_frame.data_header.data_cmd_id = CLIENT_INTERACTIVE_CMD_ID_TEST;
//	message_frame.data_header.send_ID = self_inf.robot_ID-1;//发送给空中机器人id为哨兵-1
//	message_frame.data_header.receiver_ID = Robot_Target_ID;//目标机器人ID
	
	//打包写入数据段
	//append_CRC8_check_sum((uint8_t *)CliendTxBuffer_message, sizeof(frame_header));//写入帧头CRC8校验码
	memcpy(CliendTxBuffer_message, (uint8_t *)&message_frame.txFrameHeader, sizeof(frame_header));//写入帧头数据
	append_CRC8_check_sum((uint8_t *)CliendTxBuffer_message, sizeof(frame_header));//写入帧头CRC8校验码
	memcpy(CliendTxBuffer_message + 5, &cmd_id, 2);
	memcpy(CliendTxBuffer_message + 7, (uint8_t *)&Sentry_map, sizeof(Sentry_map));
	append_CRC16_check_sum(CliendTxBuffer_message, 9+sizeof(Sentry_map)); //写入数据段CRC16校验码
	HAL_UART_Transmit_DMA(&JUDGE_HUART, CliendTxBuffer_message, 9+sizeof(Sentry_map));

}



