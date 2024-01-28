

#ifndef __COMM_TYPE_H__
#define __COMM_TYPE_H__

#include "comm_type.h"
#include "stm32f4xx_hal.h"

typedef enum
{
    PROTECT_MODE,   //保护模式
    REMOTER_MODE,   //遥控模式
    AUTO_MODE,    //自动模式
    KEYBOARD_MODE,  //键盘模式
    VISION_MODE    //视觉模式(鼠标右键开启)
    
} ctrl_mode_e;

typedef struct
{
    float vx;
    float vy;
    float vw;
    int8_t    spin_dir;      //小陀螺方向
} spd_comm_t;



typedef struct
{
  ctrl_mode_e ctrl_mode_sys;
  spd_comm_t     spd_input;
  int8_t    spin_dir;      //小陀螺方向
}Gimbal_to_Chassis_t;

#endif
