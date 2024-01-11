#ifndef ROBOMASTER_PROTOCOL_H
#define ROBOMASTER_PROTOCOL_H

#include "struct_typedef.h"

#define HEADER_SOF 0xA5
#define REF_PROTOCOL_FRAME_MAX_SIZE         128
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
  uint8_t super_cup_state;
	
}  chassis_odom_info_t;

// vision control message
typedef struct
{
	float gimbal_yaw_cmd;
	float gimbal_pitch_cmd;
	uint8_t shoot_cmd;
	uint8_t friction_cmd;
	
}  vision_ctrl_info_t;

// game status feedback message
typedef struct
{
	uint8_t area;
	float shoot_number;
	float health_point;
	
}  game_status_info_t;



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
