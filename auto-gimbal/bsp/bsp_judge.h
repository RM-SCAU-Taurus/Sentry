#ifndef __BSP_JUDGE_H__
#define __BSP_JUDGE_H__

#include "stdint.h"
#include "crc.h"

#define    LEN_HEADER    5        //帧头长度(字节)
#define    LEN_CMDID     2        //命令码长度
#define    LEN_TAIL      2	      //帧尾CRC16

/* 协议固定起始字节：0xA5 */
#define   JUDGE_FRAME_HEADER   (0xA5)
/* 整段数据偏移量 */
typedef enum
{
    FRAME_HEADER = 0,
    CMD_ID       = 5,
    DATA         = 7,
} JudgeFrameOffset;

/* 帧头数据偏移量 */
typedef enum
{
    SOF         = 0, //起始位
    DATA_LENGTH = 1, //帧内数据长度,根据这个来获取数据长度
    SEQ         = 3, //包序号
    CRC8        = 4, //CRC8
} FrameHeaderOffset;

/* 命令码ID说明 */
typedef enum
{
    ID_game_state		  =		0x0001,		//比赛状态				1Hz
    ID_game_result		=		0x0002,		//比赛结果				结束时发送
    ID_robot_HP				=		0x0003,		//机器人血量			1Hz
    ID_darts_status		=		0x0004,		//飞镖发射状态		飞镖发射后发送

    ID_event_data			=		0x0101,		//场地事件数据		事件改变后发送
    ID_supply_action	=		0x0102,		//补给站动作标识	动作发生后发送
    ID_judge_warning	=		0x0104,		//裁判警告数据		警告发生后发送
    ID_dart_remaining_time =0x0105,	//飞镖发射倒计时	1Hz

    ID_robot_status		=		0x0201,		//机器人状态数据	10Hz
    ID_robot_power		=		0x0202,		//实时功率热量		50Hz
    ID_robot_position =		0x0203,		//机器人位置数据	10Hz
    ID_robot_buff			=		0x0204,		//机器人增益数据	增益状态改变后发送
    ID_AerialRobotEnergy = 0x0205, 	//无人机能量			10Hz,只发送空中
    ID_robot_hurt			=		0x0206,		//伤害状态数据		伤害发生后发送
    ID_shoot_data			=		0x0207,		//实时射击数据		子弹发射后发送
    ID_bullet_remaining = 0x0208,   //弹丸剩余发射数	1Hz，空中/哨兵
    ID_RFID_status		=		0x0209,		//机器人RFID状态	1Hz
    ID_dart_client    =   0x020A,		//飞镖机器人客户端指令数据 10Hz

    ID_interact				=		0x0301,		//机器人间交互数据	发送方触发发送,上限10Hz
    ID_client_map		  =		0x0303,		//客户端小地图交互数据	触发发送
		ID_map_output		  =		0x0307,		//发给云台手信息
} cmd_ID;

/* 数据段长度 */
typedef enum
{
    LEN_game_state			=		11,
    LEN_game_result			=		1,
    LEN_robot_HP				=		32,
    LEN_darts_status		=		3,

    LEN_event_data			=		4,
    LEN_supply_action		=		4,
    LEN_judge_warning		=		2,
    LEN_darts_remaining_tim = 1,

    LEN_robot_status		=		27,
    LEN_robot_power			=		16,
    LEN_robot_position 	=		16,
    LEN_robot_buff			=   1,
    LEN_AerialRobotEnergy = 2,
    LEN_robot_hurt			=		1,
    LEN_shoot_data			=		7,
    LEN_bullet_remaining=		6,
    LEN_RFID_status			=		4,
    LEN_dart_client			=   12,

    LEN_interact				=		20,		//机器人间交互数据段(自定义,不超过113)
		LEN_Map				=		103,		
} cmd_LEN;


/******************************以下为数据结构体的详细定义******************************/

/* 帧头 */
typedef __packed struct
{
  	uint8_t  SOF;					//起始字节
	uint16_t DataLength;	//数据长度
	uint8_t  Seq;					//包序号
	uint8_t  CRC8;				//crc8校验
} frame_header;

/* 比赛状态数据：0x0001。发送频率：1Hz，发送范围：所有机器人。*/
typedef __packed struct
{
    uint8_t	 game_type : 4;
    uint8_t  game_progress : 4;
    uint16_t stage_remain_time;
    uint64_t SyncTimeStamp;
} ext_game_status_t;

/* 比赛结果数据：0x0002。发送频率：比赛结束后发送，发送范围：所有机器人。*/
typedef __packed struct
{
    uint8_t winner;
} ext_game_result_t;

/* 机器人血量数据：0x0003。发送频率：1Hz，发送范围：所有机器人。*/
typedef __packed struct
{
    uint16_t red_1_robot_HP;
    uint16_t red_2_robot_HP;
    uint16_t red_3_robot_HP;
    uint16_t red_4_robot_HP;
    uint16_t red_5_robot_HP;
    uint16_t red_7_robot_HP;
    uint16_t red_outpost_HP;	//前哨站
    uint16_t red_base_HP; 		//基地

    uint16_t blue_1_robot_HP;
    uint16_t blue_2_robot_HP;
    uint16_t blue_3_robot_HP;
    uint16_t blue_4_robot_HP;
    uint16_t blue_5_robot_HP;
    uint16_t blue_7_robot_HP;
    uint16_t blue_outpost_HP;
    uint16_t blue_base_HP;
} ext_game_robot_HP_t;

/* 飞镖发射状态：0x0004。发送频率：飞镖发射后发送，发送范围：所有机器人。*/
typedef __packed struct
{
    uint8_t  dart_belong;
    uint16_t stage_remaining_time;
} ext_dart_status_t;

/* 场地事件数据：0x0101。发送频率：1Hz 周期发送，发送范围：己方机器人。*/
typedef __packed struct
{
    uint32_t event_type;
} ext_event_data_t;

/* 补给站动作标识：0x0102。发送频率：动作触发后发送，发送范围：己方机器人。*/
typedef __packed struct
{
    uint8_t supply_projectile_id;
    uint8_t supply_robot_id;
    uint8_t supply_projectile_step;
    uint8_t supply_projectile_num;
} ext_supply_projectile_action_t;

/* 裁判警告信息：cmd_id (0x0104)。发送频率：警告发生后发送，发送范围：己方机器人。*/
typedef __packed struct
{
    uint8_t level;
    uint8_t foul_robot_id;
} ext_referee_warning_t;

/* 飞镖发射口倒计时：cmd_id (0x0105)。发送频率：1Hz 周期发送，发送范围：己方机器人*/
typedef __packed struct
{
    uint8_t dart_remaining_time;
} ext_dart_remaining_time_t;

/* 比赛机器人状态：0x0201。发送频率：10Hz，发送范围：单一机器人。*/
typedef __packed struct
{
    uint8_t	 robot_id;
    uint8_t	 robot_level;
    uint16_t remain_HP;
    uint16_t max_HP;

    uint16_t shooter_id1_17mm_cooling_rate;
    uint16_t shooter_id1_17mm_cooling_limit;
    uint16_t shooter_id1_17mm_speed_limit;
    uint16_t shooter_id2_17mm_cooling_rate;
    uint16_t shooter_id2_17mm_cooling_limit;
    uint16_t shooter_id2_17mm_speed_limit;
    uint16_t shooter_id1_42mm_cooling_rate;
    uint16_t shooter_id1_42mm_cooling_limit;
    uint16_t shooter_id1_42mm_speed_limit;

    uint16_t chassis_power_limit;

    uint8_t	 mains_power_gimbal_output : 1;
    uint8_t	 mains_power_chassis_output : 1;
    uint8_t	 mains_power_shooter_output : 1;
} ext_game_robot_status_t;

/* 实时功率热量数据：0x0202。发送频率：50Hz，发送范围：单一机器人。*/
typedef __packed struct
{
    uint16_t chassis_volt;
    uint16_t chassis_current;
    float		 chassis_power;
    uint16_t chassis_power_buffer;
    uint16_t shooter_id1_17mm_cooling_heat;
    uint16_t shooter_id2_17mm_cooling_heat;
    uint16_t shooter_id1_42mm_cooling_heat;
} ext_power_heat_data_t;

/* 机器人位置：0x0203。发送频率：10Hz，发送范围：单一机器人。*/
typedef __packed struct
{
    float x;
    float y;
    float z;
    float yaw;
} ext_game_robot_pos_t;

/* 机器人增益：0x0204。发送频率：1Hz 周期发送，发送范围：单一机器人。*/
typedef __packed struct
{
    uint8_t power_rune_buff;
} ext_buff_t;

/* 空中机器人能量状态：0x0205。发送频率：10Hz，发送范围：单一机器人。*/
typedef __packed struct
{
    uint8_t	attack_time;
} ext_aerial_robot_energy_t;

/* 伤害状态：0x0206。发送频率：伤害发生后发送，发送范围：单一机器人。*/
typedef __packed struct
{
    uint8_t armor_id  : 4;
    uint8_t hurt_type : 4;
} ext_robot_hurt_t;

/* 实时射击信息：0x0207。发送频率：射击后发送，发送范围：单一机器人。*/
typedef __packed struct
{
    uint8_t bullet_type;
    uint8_t shooter_id;
    uint8_t bullet_freq;
    float 	bullet_speed;
} ext_shoot_data_t;

/* 子弹剩余发射数：0x0208。发送频率：10Hz，发送范围：所有机器人。*/
typedef __packed struct
{
    uint16_t bullet_remaining_num_17mm;
    uint16_t bullet_remaining_num_42mm;
    uint16_t coin_remaining_num;
} ext_bullet_remaining_t;

/* 机器人RFID状态：0x0209。发送频率：1Hz，发送范围：单一机器人。*/
typedef __packed struct
{
    uint32_t rfid_status;
} ext_rfid_status_t;

/* 飞镖机器人客户端指令数据：0x020A。发送频率：10Hz，发送范围：单一机器人。*/
typedef __packed struct
{
    uint8_t  dart_launch_opening_status;
    uint8_t  dart_attack_target;
    uint16_t target_change_time;
    uint8_t  first_dart_speed;
    uint8_t  second_dart_speed;
    uint8_t  third_dart_speed;
    uint8_t  fourth_dart_speed;
    uint16_t last_dart_launch_time;
    uint16_t operate_launch_cmd_time;
} ext_dart_client_cmd_t;


/**************************************************************************/
/*
	交互数据接收信息：0x0301。
	包括一个统一的数据段头结构，
	包含了内容 ID，发送者以及接受者的 ID 和内容数据段，
	整个交互数据的包总共长最大为 128 个字节，
	减去 frame_header,cmd_id,frame_tail 以及数据段头结构的 6 个字节，
	故而发送的内容数据段最大为 113。
	每个机器人交互数据与自定义控制器数据上下行合计带宽不超过5000 Byte。
	上下行发送频率分别不超过30Hz。

	机器人 ID：
	1,英雄(红)；
	2,工程(红)；
	3/4/5,步兵(红)；
	6,空中(红)；
	7,哨兵(红)；
	9,雷达站(红);
	101,英雄(蓝)；
	102,工程(蓝)；
	103/104/105,步兵(蓝)；
	106,空中(蓝)；
	107,哨兵(蓝)
	109,雷达站(蓝)。
	客户端 ID：
	0x0101 为英雄操作手客户端(红) ；
	0x0102 ，工程操作手客户端 (红)；
	0x0103/0x0104/0x0105，步兵操作手客户端(红)；
	0x0106，空中操作手客户端(红)；
	0x0165，英雄操作手客户端(蓝)；
	0x0166，工程操作手客户端(蓝)；
	0x0167/0x0168/0x0169，步兵操作手客户端(蓝)；
	0x016A，空中操作手客户端(蓝)。
*/
/* 自定义数据发送总长度 */
typedef enum
{
    DRAW_IMAGINE1=30,
    DRAW_IMAGINE2=45,
    DRAW_IMAGINE5=90,
    DRAW_IMAGINE7=120,
    PRINTF_CODE	=60,
    JUDGE_DELETE=17,
    ROBOT_COMM=(15+LEN_interact),
} Judgedatalength;

/* 数据段的内容ID */
typedef enum
{
    CLIENT_INTERACTIVE_CMD_ID_TEST    = 0x0200,  //交互数据，可在0x0200~0x02ff选取，具体ID含义由参赛队自定义
    CLIENT_DELETE_GRAPH_CMD_ID     		= 0x0100,  //客户端删除图形
    CLIENT_DRAW_1_GRAPHS_CMD_ID     	= 0x0101,  //客户端绘制1个图形
    CLIENT_DRAW_2_GRAPHS_CMD_ID    		= 0x0102,  //客户端绘制2个图形
    CLIENT_DRAW_5_GRAPHS_CMD_ID    		= 0x0103,  //客户端绘制5个图形
    CLIENT_DRAW_7_GRAPHS_CMD_ID    		= 0x0104,  //客户端绘制7个图形
    CLIENT_WRITE_STRINGS_CMD_ID    		= 0x0110,  //客户端绘制字符串图形
} client_data_cmd_e;

/* 交互数据接收信息帧头 */
typedef __packed struct
{
    client_data_cmd_e data_cmd_id;
    uint16_t 					send_ID;
    uint16_t 					receiver_ID;
} ext_judgesend_custom_header_t;

/* 图形删除类型 */
typedef enum
{
    type_delete_nop   = 0,
    type_delete_layer = 1,
    type_delete_all   = 2,
} type_graphic_delete_e;

/* 客户端删除图形 */
typedef __packed struct
{
    type_graphic_delete_e operate_tpye;
    uint8_t layer;
} ext_SendClientDelete_t;

/* 图形数据 */
typedef __packed struct
{
    uint8_t graphic_name[3];
    uint32_t operate_tpye:3;
    uint32_t graphic_tpye:3;
    uint32_t layer:4;
    uint32_t color:4;
    uint32_t start_angle:9;
    uint32_t end_angle:9;
    uint32_t width:10;
    uint32_t start_x:11;
    uint32_t start_y:11;
    uint32_t radius:10;
    uint32_t end_x:11;
    uint32_t end_y:11;
} graphic_data_struct_t;

/* 客户端绘制字符 */
typedef __packed struct
{
    graphic_data_struct_t grapic_data_struct;
    uint8_t data[30];
} ext_client_custom_character_t;

/* 机器人间交互数据：0x0200~0x02ff */
typedef __packed struct
{
    uint8_t data[LEN_interact];
} robot_interactive_data_t;

/*
	小地图交互信息包括一个统一的数据段头结构。
	小地图交互信息标识：0x0303。发送频率：触发时发送。
*/
typedef __packed struct
{
    float target_position_x;		//目标x位置坐标，单位m		当发送目标机器人ID时，该项为0
    float target_position_y;		//目标y位置坐标，单位m 		当发送目标机器人ID时，该项为0
    float target_position_z;		//目标z位置坐标，单位m 		当发送目标机器人ID时，该项为0
    uint8_t commd_keyboard;			//发送指令时，云台手按下的键盘信息 		无按键按下则为0
    uint16_t target_robot_ID;		//要作用的目标机器人ID 		当发送位置信息时，该项为0
} ext_robot_command_t;
//0x0307 哨兵模式以及路径小地图显示
typedef __packed struct 
{ 
 uint8_t intention; 
 uint16_t start_position_x; 
 uint16_t start_position_y; 
 int8_t delta_x[49]; 
 int8_t delta_y[49]; 
 }map_sentry_data_t; //103个字节
/* 数据段 */
typedef __packed struct
{
    ext_SendClientDelete_t 					delete_data;		//(2)
    graphic_data_struct_t						draw1_data;			//客户端绘制1个图形(15)
    graphic_data_struct_t						draw2_data[2];	//客户端绘制2个图形(30)
    graphic_data_struct_t						draw5_data[5];	//客户端绘制5个图形(75)
    graphic_data_struct_t						draw7_data[7];	//客户端绘制7个图形(105)
    ext_client_custom_character_t		code_data;			//客户端绘制字符(45)
    robot_interactive_data_t				comm_senddata;	//机器人间交互数据(LEN_comm_data)
    ext_robot_command_t							Client_map;			//小地图交互信息(15)
} ext_clientdata_t;

/* 上传客户端 */
typedef __packed struct
{
    frame_header   									txFrameHeader;//帧头5
    cmd_ID		 											CmdID;//命令码2
    ext_judgesend_custom_header_t		FrameHeader;//数据段头结构6
    ext_clientdata_t  							clientdata;//数据段
    uint16_t		 										FrameTail;//帧尾2
} ext_SendClient_t;

typedef __packed struct
{		
		uint16_t data_cmd_id;
		uint16_t sender_ID;
		uint16_t receiver_ID;
    float blue_x;
		float blue_y;
		float blue_confiden;
		float red_x;
		float red_y;
		float red_confiden;
} radar_data;
typedef __packed struct
{		
		uint16_t data_cmd_id;
		uint16_t sender_ID;
		uint16_t receiver_ID;
    uint16_t position_control;
		uint16_t tuoluo_control;
		uint16_t shoot_control;
		uint16_t other_control;
	
} sentry_control;
typedef struct 
{
    /* data */
ext_game_status_t					Game_State_data;
ext_game_result_t					Game_Result_data;
ext_game_robot_HP_t                 RobotHP_data;
ext_dart_status_t					Dart_Status_data;
ext_event_data_t					Event_Data_data;
ext_supply_projectile_action_t		Supply_Projectile_Action_data;
ext_referee_warning_t				Referee_Warning_data;
ext_dart_remaining_time_t			Dart_Remaining_Time_data;
ext_game_robot_status_t				Game_Robot_Status_data;
ext_power_heat_data_t				Power_Heat_Data_data;
ext_game_robot_pos_t				Robot_Position_data;
ext_buff_t							Buff_data;
ext_aerial_robot_energy_t			Aerial_Robot_Energy_data;
ext_robot_hurt_t					Robot_Hurt_data;
ext_shoot_data_t					Shoot_Data_data;
ext_bullet_remaining_t				Bullet_Remaining_data;
ext_rfid_status_t					RFID_Status_data;
ext_dart_client_cmd_t				Dart_Client_data;
	radar_data 									Radar_Data;
	ext_robot_command_t  				Robot_command_Data;
}robot_judge_msg_t;



/* 交互数据接收信息帧头 */
typedef __packed struct 
{ 
	client_data_cmd_e data_cmd_id;    
	uint16_t 					send_ID;    
	uint16_t 					receiver_ID; 
} judgesend_custom_header_t;
typedef __packed struct
{
	frame_header   							txFrameHeader;//帧头5
	cmd_ID		 									CmdID;//命令码2;
	judgesend_custom_header_t		data_header;//数据段帧头 6
	//此处为各种数据段
	uint16_t		 								FrameTail;//帧尾 2
}judgesend_frame_t;
typedef enum 
{
	BLUE,
	RED
}color_e;
typedef __packed struct
{
	color_e	 color;				//阵营
	uint16_t robot_ID;		//自身id
	uint16_t client_ID;		//对应客户端id
	uint16_t self_HP;			//自身血量
}self_inf_t;
extern robot_judge_msg_t robot_judge_msg;
extern ext_game_robot_status_t Game_Robot_Status;
extern ext_power_heat_data_t	 Power_Heat_Data;
extern ext_game_status_t			 Game_State;
extern map_sentry_data_t 					Sentry_map;
extern ext_robot_command_t        	Robot_command;//云台手指令数据
extern ext_shoot_data_t					Shoot_Data;
/* 读取裁判系统反馈信息 */
int judge_data_handler(uint8_t *ReadFromUsart);

/* UI绘制任务 */
void judge_send_task(void const *argu);
/* 机器人阵营/ID判断 */
int  determine_red_blue(void);
void determine_ID(void);
void judge_get_basedata(void);
void ui_sendmessage(void);
/* UI绘图底层函数 */
void data_pack_imagine(Judgedatalength data_length, uint8_t* data_locate, client_data_cmd_e date_type);
void data_pack_code(ext_client_custom_character_t code_date, uint8_t* CODE);
void data_pack_delete(type_graphic_delete_e type, uint8_t layer);
void data_pack_comm_data(uint16_t Robot_Target_ID, client_data_cmd_e CLIENT_INTERACTIVE_CMD_ID, uint8_t* comm_date);
void data_pack_map_position(float target_position_x, float target_position_y, float target_position_z, uint8_t commd_keyboard, uint16_t target_robot_ID);

/* UI功能图形绘制 */
void ui_init(void);
void ui_data_pack(void);
void judge_send_supercap(uint8_t supercap_percent);
void judge_send_shoot_mode(uint8_t up, uint8_t down);
void judge_send_code_display(uint8_t gimbal, uint8_t chassis);
void judge_send_light_display(uint8_t light);
void robot_judge_msg_copy(void);
#endif

