#ifdef  __GIMBAL_TASK_GLOBALS
#define __GIMBAL_TASK_EXT
#else
#define __GIMBAL_TASK_EXT extern
#endif

#ifndef __GIMBAL_TASK_H__
#define __GIMBAL_TASK_H__

#include "stm32f4xx_hal.h"
#include "pid.h"

#define GIMBAL_PERIOD	1

typedef struct
{
  float pit_ecd_ref;
  float pit_ecd_fdb;
	float pit_ecd_error;
	float pit_spd_ref;
  float pit_spd_fdb;
	
  float yaw_angle_ref;
  float yaw_angle_fdb;
	float yaw_angle_error;
  float yaw_spd_ref;
	float yaw_spd_fdb;

  float pit_relative_ecd;	
  float yaw_relative_angle;

} gim_pid_t;


typedef struct
{
  /* gimbal ctrl parameter */
  gim_pid_t     pid;
	
  /* read from flash */
  int32_t       pit_center_offset;
  int32_t       yaw_center_offset;
  int16_t       current[2];  //yaw 0  pit  1 

} gimbal_t;

extern gimbal_t gimbal;

void gimbal_task(void const *argu);
void vision_calcu(void);
void gimbal_param_init(void);
void gimbal_pid_calcu(void);
#endif
