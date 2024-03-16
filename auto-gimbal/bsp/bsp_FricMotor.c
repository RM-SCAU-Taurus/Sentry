#include "bsp_FricMotor.h"
#include "tim.h"
#include "pid.h"
#include "math_calcu.h"
#include "remote_msg.h"
#include "shoot_task.h"
#include "modeswitch_task.h"
#include "control_def.h"
#include "status_task.h"
#include "bsp_judge.h"
#include "bsp_can.h"
ramp_function_source_t Fric_Speed_Target_Ramp;

#define test 100
/*3508摩擦轮电机电机*/
moto_measure_t fric_motor_l;
moto_measure_t fric_motor_r;
pid_t l_pid_spd;
pid_t r_pid_spd;

uint8_t FricMotor_init(void)
{
	if (!fric.init_flag)
	{
		PID_struct_init(&l_pid_spd, POSITION_PID, 16000, 0, 10, 0, 0);
		PID_struct_init(&r_pid_spd, POSITION_PID, 16000, 0, 10, 0, 0);
		fric.init_flag = 1;
	}
	else
	{
		fric.init_flag = 0;
	}
	return fric.init_flag;
}

void FricMotor_speed_set(uint16_t speed)
{
	if (speed == 30)
	{
		ramp_calc(&Fric_Speed_Target_Ramp, SHOOT_PERIOD, 10.0f, 7500, 0);
	}
	else if(speed == test )
	{
		ramp_calc(&Fric_Speed_Target_Ramp, SHOOT_PERIOD, 10.0f, 1500, 0);
	}
	else if (speed == 0)
	{
		Fric_Speed_Target_Ramp.out = 0;
	}

	fric.set_speed = Fric_Speed_Target_Ramp.out;

	fric.Fric_Pid_Set[0].fric_spd_ref = +fric.set_speed;
	fric.Fric_Pid_Set[1].fric_spd_ref = -fric.set_speed;
}

void FricMotor_Control(void)
{
	// fric.Fric_Pid_Set[0].fric_spd_fdb = fric_motor_l.speed_rpm;
//	// fric.Fric_Pid_Set[1].fric_spd_fdb = fric_motor_r.speed_rpm;
//	fric.Fric_Pid_Set[0].fric_spd_fdb = moto_fric[0].speed_rpm;
//	fric.Fric_Pid_Set[1].fric_spd_fdb = moto_fric[1].speed_rpm;

//	switch (shoot.firc_mode)
//	{
//	case FIRC_MODE_STOP:
//	{
//		FricMotor_speed_set(0);
//		pid_calc(&l_pid_spd, fric.Fric_Pid_Set[0].fric_spd_fdb, fric.Fric_Pid_Set[0].fric_spd_ref);
//		pid_calc(&r_pid_spd, fric.Fric_Pid_Set[1].fric_spd_fdb, fric.Fric_Pid_Set[1].fric_spd_ref);
//		break;
//	}
//	case FIRC_MODE_RUN:
//	{
//		FricMotor_speed_set(30);
//		pid_calc(&l_pid_spd, fric.Fric_Pid_Set[0].fric_spd_fdb, fric.Fric_Pid_Set[0].fric_spd_ref);
//		pid_calc(&r_pid_spd, fric.Fric_Pid_Set[1].fric_spd_fdb, fric.Fric_Pid_Set[1].fric_spd_ref);
//		break;
//	}
//	default:
//		break;
//	}
//	motor_cur.fric_cur[0] = l_pid_spd.pos_out;
//	motor_cur.fric_cur[1] = r_pid_spd.pos_out;
}

void Fric_protect(void)
{

	fric.Fric_Pid_Set[0].fric_spd_fdb = moto_fric[0].speed_rpm;
	fric.Fric_Pid_Set[1].fric_spd_fdb = moto_fric[1].speed_rpm;
	FricMotor_speed_set(0);
	pid_calc(&l_pid_spd, fric.Fric_Pid_Set[0].fric_spd_fdb, fric.Fric_Pid_Set[0].fric_spd_ref);
	pid_calc(&r_pid_spd, fric.Fric_Pid_Set[1].fric_spd_fdb, fric.Fric_Pid_Set[1].fric_spd_ref);
	motor_cur.fric_cur[0] = l_pid_spd.pos_out;
	motor_cur.fric_cur[1] = r_pid_spd.pos_out;
}

void Fric_unprotect(void)
{

	fric.Fric_Pid_Set[0].fric_spd_fdb = moto_fric[0].speed_rpm;
	fric.Fric_Pid_Set[1].fric_spd_fdb = moto_fric[1].speed_rpm;
	FricMotor_speed_set(test);
	pid_calc(&l_pid_spd, fric.Fric_Pid_Set[0].fric_spd_fdb, fric.Fric_Pid_Set[0].fric_spd_ref);
	pid_calc(&r_pid_spd, fric.Fric_Pid_Set[1].fric_spd_fdb, fric.Fric_Pid_Set[1].fric_spd_ref);
		motor_cur.fric_cur[0] = l_pid_spd.pos_out;
	motor_cur.fric_cur[1] = r_pid_spd.pos_out;
}
