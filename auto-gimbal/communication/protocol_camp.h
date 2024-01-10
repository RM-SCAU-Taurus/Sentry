#ifndef ROBOMASTER_PROTOCOL_H
#define ROBOMASTER_PROTOCOL_H

#include "struct_typedef.h"

#define HEADER_SOF 0xA5
#define REF_PROTOCOL_FRAME_MAX_SIZE        256	//128
#define END1_SOF 0x0D
#define END2_SOF 0x0A

#define REF_PROTOCOL_HEADER_SIZE            sizeof(frame_header_struct_t)
#define REF_PROTOCOL_CMD_SIZE               2
#define REF_PROTOCOL_CRC16_SIZE             2
#define REF_HEADER_CRC_LEN                  (REF_PROTOCOL_HEADER_SIZE + REF_PROTOCOL_CRC16_SIZE)
#define REF_HEADER_CRC_CMDID_LEN            (REF_PROTOCOL_HEADER_SIZE + REF_PROTOCOL_CRC16_SIZE + sizeof(uint16_t))
#define REF_HEADER_CMDID_LEN                (REF_PROTOCOL_HEADER_SIZE + sizeof(uint16_t))

#pragma pack(push, 1)

//RM协议内置命令码   //发送的ID号
typedef enum
{
  GAME_STATUS_FDB_ID = 0x0001,
  CHASSIS_ODOM_FDB_ID = 0x0101,
  CHASSIS_CTRL_CMD_ID = 0x0102,
	VISION_CMD_ID = 0x0103,
	MAP_FDD_ID=0x0104,
} referee_data_cmd_id_tpye;

//RM协议帧头结构体
typedef  struct
{
  uint8_t SOF;
  uint16_t data_length;
  uint8_t seq;
  uint8_t CRC8;
} frame_header_struct_t;

//RM协议反序列化步骤枚举
typedef enum
{
  STEP_HEADER_SOF  = 0,
  STEP_LENGTH_LOW  = 1,
  STEP_LENGTH_HIGH = 2,
  STEP_FRAME_SEQ   = 3,
  STEP_HEADER_CRC8 = 4,
  STEP_DATA_CRC16  = 5,
} unpack_step_e;

//RM协议反序列化结构体
typedef struct
{
  frame_header_struct_t *p_header;
  uint16_t       data_len;
  uint8_t        protocol_packet[REF_PROTOCOL_FRAME_MAX_SIZE];
  unpack_step_e  unpack_step;
  uint16_t       index;
} unpack_data_t;

typedef struct
{
	uint8_t end1;
	uint8_t end2;
} taurus_end_info ;


// chassis control message
typedef struct
{
  float vx;
  float vy;
  float vw;
	uint8_t super_cup;
	
}  chassis_ctrl_info_t;

// chassis odom feedback message
typedef struct
{
  
	float x_fdb;
  float y_fdb;
  float vx_fdb;
  float vy_fdb;
  float vw_fdb;
	float gimbal_yaw_fdb;
	float gimbal_pitch_fdb;
	float gimbal_yaw_imu;
  float gimbal_yaw_rate_imu;
	float gimbal_pitch_imu;
	float gimbal_pitch_rate_imu;
	float super_cup_state;
	float diff_base_to_gimbal;
	
}chassis_odom_info_t;

typedef struct
{
    float game_type;				
    float game_progress;				
    float shooter_id1_17mm_cooling_rate;
    float shooter_id1_17mm_speed_limit;		
    float shooter_id1_17mm_cooling_limit;	
    float robot_id;		
    float remain_HP;	
	  float max_HP;		
    float armor_id;	
    float hurt_type;		
    float bullet_freq;
		float bullet_speed;
		float bullet_remaining_num_17mm;
		float commd_keyboard;
		float target_robot_ID;
		float target_position_x;
		float target_position_y;
		float target_position_z;
		float game_time;
		float Enemy_Sentry_HP;//敌方哨兵血量
		float Enemy_outpost_HP;
		float Self_outpost_HP;
		float red_y;
		float red_confiden;
} Game_Status_t;
// vision control message
typedef struct
{
	float gimbal_yaw_cmd;
	float gimbal_pitch_cmd;
	float shoot_cmd;//3个挡位
	float friction_cmd;	
	float aiming_flag;
	float pit;
	float yaw;
	float dis;
	float tof;
	float connect_status;
	float process_status;
	float cmd_mode;
	float spinning_mode;
	float navigation_status;
	float location_status;
	float dfend_mode;
	float auto_aim;
	float robot_x;
	float robot_y;
	float robot_vx;
	float robot_vy;
	float robot_vw;
	float robot_yaw;
}vision_ctrl_info_t;

typedef struct
{
	uint8_t intention;
	uint8_t start_position_x;
	uint8_t start_position_y;
  int8_t delta_x[49]; 
  int8_t delta_y[49];
}map_data_t;

// game status feedback message
//typedef struct
//{
//	uint8_t area;
//	float shoot_number;
//	float health_point;
//	uint8_t state;//比赛状态
//	float heat_rest;
//}  game_status_info_t;



//信息数据
typedef struct
{
	float x;
  float y;
  float yaw;
  float vx;
  float vy;
  float vw;
	float gyro_z;
	float gyro_yaw;
	
} message_info_t;



#pragma pack(pop)

#endif //ROBOMASTER_PROTOCOL_H
