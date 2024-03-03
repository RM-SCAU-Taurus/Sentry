#ifdef  __GIMBAL_TASK_GLOBALS
#define __GIMBAL_TASK_EXT
#else
#define __GIMBAL_TASK_EXT extern
#endif

#ifndef __GIMBAL_TASK_H__
#define __GIMBAL_TASK_H__

#include "stm32f4xx_hal.h"
#include "pid.h"
#include "ubf.h"

/*******************OOP*********************************/

// 云台模式回调函数,用于解析协议
typedef void (*gimbal_mode_callback)();


//父类
typedef struct
{
    gimbal_mode_callback c_Fun;
}Gimbal_Base;

//子类
typedef struct 
{
    Gimbal_Base Base;
}Gimbal_Derived;

/*******************OOP*********************************/

typedef void (*gimbal_mode_callback)();

typedef struct
{
    /* ------------------------------- PIT ------------------------------- */
    /* ------------- position ------------- */
	  /* yaw angle pid param */
		float pit_angle_ref;
    float pit_angle_fdb;
    float pit_angle_err;
	
    /* pit ecd pid param */
    float pit_ecd_ref;
    float pit_ecd_fdb;
    float pit_ecd_err;
    /* -------------   speed  ------------- */
    /* pit spd pid param */
    float pit_spd_ref;
    float pit_spd_fdb;

    /* ------------------------------- YAW ------------------------------- */
    /* ------------- position ------------- */
    /* yaw angle pid param */
    float yaw_angle_6020_ref;
    float yaw_angle_6020_fdb;
    float yaw_angle_6020_err;
		
    float yaw_angle_9025_ref;
    float yaw_angle_9025_fdb;
    float yaw_angle_9025_err;			
    /* yaw motor ecd pid param */
    float yaw_mecd_ref;
    float yaw_mecd_fdb;
    float yaw_mecd_err;
    /* -------------   speed  ------------- */
    /* yaw speed pid param */
    float yaw_spd_6020_ref;
    float yaw_spd_6020_fdb;
		
		float yaw_spd_9025_ref;
    float yaw_spd_9025_fdb;	
    /* yaw motor ecd pid param */
    float yaw_mspd_ref;
    float yaw_mspd_fdb;
} gim_pid_t;


typedef enum 
{
    GimbalInstance_MODE_PROTECT,
    GimbalInstance_MODE_REMOTER,
    GimbalInstance_MODE_AUTO,  
} GimbalInstance_mode_e;

typedef struct
{
	GimbalInstance_mode_e Gimbal_Mode;
  gimbal_mode_callback mode_callback; // 瑙ｆ瀽鏀跺埌鐨勬暟鎹?鐨勫洖璋冨嚱鏁?
}GimbalInstance_t;

typedef struct
{
    /* gimbal ctrl parameter */
    gim_pid_t     pid;

    /* read from flash */
    int32_t       pit_center_offset;
    int32_t       yaw_center_offset;
		int16_t       current[3];  //yaw_6020 0  pit 1  yaw_9025 2
		float         yaw_imu_offset;//锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷头锟斤拷

    /*9025跟随*/
    int16_t   position_ref;
    float 	  position_error;
    float     angle_error;
    float     angle_dif_degree;
} gimbal_t;
typedef struct {
    float pit;
    float wy;
    float yaw;
    float wz;
} gim_msg_t;

extern gimbal_t gimbal;
extern  ubf_t gim_msg_ubf;   /* 锟斤拷台锟斤拷态锟斤拷史锟斤拷锟斤拷 */
void gimbal_task(void const *argu);
void gimbal_param_init(void);

#endif
