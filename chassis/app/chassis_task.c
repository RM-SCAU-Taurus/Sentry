#include "chassis_task.h"
#include "cmsis_os.h"
#include "comm_task.h"
#include "string.h"
#include "pid.h"
#include "stdlib.h"
#include "bsp_can.h"
#include "bsp_powerlimit.h"
#include "math_calcu.h"
#include "math.h"
#include "remote_msg.h"
#include "modeswitch_task.h"
#include "status_task.h"
#include "control_def.h"

float P1 = 0.5f;
float P2 = 168.0f;
float I2 = 1.0f;
extern TaskHandle_t can_msg_send_task_t;

chassis_t chassis;
chassis_control_msg_t chassis_control_msg= {0};
/**
	* @brief      底盘任务初始化
	* @author
	* @param[in]
	* @retval
	*/

void chassis_param_init(void)
{
    PowerControl_Init();
    for(int i=0; i<4; i++)
    {
        PID_struct_init(&pid_chassis_3508_spd[i], POSITION_PID, 13000, 0,
                        7.0f, 0.0f, 0.0f);
        PID_struct_init(&pid_chassis_6020_ecd[i], POSITION_PID, 400, 100,
                        0.5f, 0.0f, 0.0f);
        PID_struct_init(&pid_chassis_6020_spd[i], POSITION_PID, 28000, 25000,
                        168.0f, 1.0f, 0.0f);
            
    }
		chassis.rudder_ecd_offset[0] = 5050;//100
		chassis.rudder_ecd_offset[1] = 3597;//5300
		chassis.rudder_ecd_offset[2] = 6253;//6350
		chassis.rudder_ecd_offset[3] = 6668;//6500
    
		chassis.spd_distribution = chassis_spd_distribution;
		chassis.wheel_max = 6300;  /* 关掉功率控制时需要初始化 */
}

float chassis_power_user;
float chassis_power_judge;
float test_power_kp_3508 = 24 * 20 / 16384.0f;
float test_power_kp_6020 = 24 * 20 / 16384.0f;
float test_power_static_power = 6.0f;

void chassis_power_calcu(void)
{
    
    chassis_power_user = 0;
    for(int i = 0; i < 4; ++i) {       
        chassis_power_user += test_power_kp_3508 * ABS(moto_msg.chassis_3508[i].given_current);
                 chassis_power_user += test_power_kp_6020 * ABS(moto_msg.chassis_6020[i].given_current);
    }
    chassis_power_user += test_power_static_power;
    chassis_power_judge = powercontrol.chassis_power;
}

/**
  * @brief 底盘姿态任务
  * @param
  * @attention
	* @note
  */
void chassis_task(void const *argu)
{
    uint32_t mode_wake_time = osKernelSysTick();
    for(;;)
    {
        
        taskENTER_CRITICAL();
        switch(ctrl_mode)
        {
        case PROTECT_MODE:
        {
            memset(chassis.current_3508, 0, sizeof(chassis.current_3508));
            memset(chassis.current_6020, 0, sizeof(chassis.current_6020));
            for(uint8_t i=0; i<4; i++)
            {
                pid_chassis_6020_spd[i].iout=0;
            }
            supercap.mode = 0;
            supercap.charge_current_set = 0;
            break;
        }
        case REMOTER_MODE:
				case AUTO_MODE:
        {
                 chassis.spd_input.vx = chassis.spd_input.vx;
                chassis.spd_input.vy = chassis.spd_input.vy;
                chassis.spd_input.vw = chassis.spd_input.vw;

//            /* 转舵保护 */
//            for(uint8_t i=0; i<4; i++)
//            {
//                if(ABS(chassis.rudder_ecd_error[i])>2048)
//                {
//                    memset(chassis.current_3508, 0, sizeof(chassis.current_3508));
//                }
//            }
//            break;
        }
        default:
        {

            break;
        }
        }
				/* 底盘速度分配 */
					 chassis.spd_distribution();
				
            /* PID计算电机控制电流 */
            chassis_pid_calcu();
            /* 最大轮速控制 */
            Power_Data_Update(); 
			/* 电容控制 */			
            SuperCap_Control();
			/* 功率控制 */
            Power_Control(chassis.current_3508, chassis.current_6020);
            
            chassis_power_calcu();
            
            /* 转舵保护 */
            for(uint8_t i=0; i<4; i++)
            {
                if(ABS(chassis.rudder_ecd_error[i])>2048)
                {
                    memset(chassis.current_3508, 0, sizeof(chassis.current_3508));
                }
            }

        memcpy(motor_cur.chassis_cur_3508, chassis.current_3508, sizeof(chassis.current_3508));
        memcpy(motor_cur.chassis_cur_6020, chassis.current_6020, sizeof(chassis.current_6020));

        osSignalSet(can_msg_send_task_t, CHASSIS_MOTOR_MSG_SEND);

        taskEXIT_CRITICAL();

        osDelayUntil(&mode_wake_time, CHASSIS_PERIOD);
    }
}


/**
  * @brief 麦轮解算函数
  * @param input : ?=+vx(mm/s)  ?=+vy(mm/s)  ccw=+vw(deg/s)
  *        output: every wheel speed(rpm)
  * @note  1=FL 2=FR 3=BL 4=BR
  */
void mecanum_calc(float vx, float vy, float vw, float speed[])
{
    float wheel_rpm[4];

    wheel_rpm[0] =    vx + vy - vw;
    wheel_rpm[1] = 	 -vx + vy - vw;
    wheel_rpm[2] =    vx - vy - vw;
    wheel_rpm[3] =   -vx - vy - vw;

    memcpy(speed, wheel_rpm, 4*sizeof(float));
}



/**
  * @brief 舵轮解算函数
  * @param input : ?=+vx(mm/s)  ?=+vy(mm/s)  ccw=+vw(deg/s)
  *        output: every wheel speed(rpm) and angle(°)
  * @note  0=FL 1=BL 2=FR 3=BR
  */
void steering_calc(float vx, float vy, float vw, float angle[], float speed[])
{
	int16_t wheel_rpm[4];
	int16_t	rudder_angle[4];
	const float rudder_deadband = 200;

	/* 轮速解算 */
//    wheel_rpm[0] = -sqrt(pow(vx,2) + pow(vy,2) + pow(vw,2) - 2*vw*(vx+vy));
//    wheel_rpm[1] = sqrt(pow(vx,2) + pow(vy,2) + pow(vw,2) - 2*vw*(vx-vy));
//    wheel_rpm[2] = sqrt(pow(vx,2) + pow(vy,2) + pow(vw,2) + 2*vw*(vx-vy));
//    wheel_rpm[3] = -sqrt(pow(vx,2) + pow(vy,2) + pow(vw,2) + 2*vw*(vx+vy));
		
	wheel_rpm[0] = sqrt(pow(vx - vw * 0.707107f,2) + pow(vy - vw * 0.707107f,2));
	wheel_rpm[1] = -sqrt(pow(vx - vw * 0.707107f,2) + pow(vy + vw * 0.707107f,2));
	wheel_rpm[2] = -sqrt(pow(vx + vw * 0.707107f,2) + pow(vy - vw * 0.707107f,2));
	wheel_rpm[3] = sqrt(pow(vx + vw * 0.707107f,2) + pow(vy + vw * 0.707107f,2));
	/* 舵角解算 */
    rudder_angle[0] = atan2((vy-vw*0.707107f), (vx+vw*0.707107f)) * 57.3f; 
    rudder_angle[1] = atan2((vy+vw*0.707107f), (vx-vw*0.707107f)) * 57.3f; 
    rudder_angle[2] = atan2((vy-vw*0.707107f), (vx-vw*0.707107f)) * 57.3f;
    rudder_angle[3] = atan2((vy+vw*0.707107f), (vx+vw*0.707107f)) * 57.3f;
	
//					for(uint8_t i = 0;i<4; i++)
//				{

//				chassis.rudder_ecd_fdb[i] = (moto_chassis_6020[i].ecd - chassis.rudder_ecd_offset[i])*0.04395f;
//				if(circle_error(rudder_angle[i],chassis.rudder_ecd_fdb[i],360)>90.0f)
//				{
//				rudder_angle[i] -= 180;
//				wheel_rpm[i] = -wheel_rpm[i];
//				}
//				else if(circle_error(rudder_angle[i],chassis.rudder_ecd_fdb[i],360)<(-90.0f))
//				{
//				rudder_angle[i] += 180;
//				wheel_rpm[i] = -wheel_rpm[i];
//				}
//				if(vx == 0 && vy == 0 && vw == 0)
//				{
//				rudder_angle[i] = 0;
//				}
//				}


	      if(ABS(chassis.spd_input.vx)>rudder_deadband || ABS(chassis.spd_input.vy)>rudder_deadband || ABS(chassis.spd_input.vw)>rudder_deadband)
        {
            memcpy(speed, wheel_rpm, 4*sizeof(int16_t));
            memcpy(angle, rudder_angle, 4*sizeof(int16_t));
        }
        else
        {
					
            memset(speed, 0, 4*sizeof(int16_t));
           memset(angle, 0, 4*sizeof(int16_t));
        }		
}

/**
	* @brief	底盘速度分配函数，输入总速度超过总最大轮速时，将输入的速度按比例重分配
	* @author
	* @param[in]
	* @retval
	*/
void chassis_spd_distribution(void)
{
    float  wheel_spd_input_buf[4];
    float  wheel_spd_total = 0;  //总轮速
    float  distribution_temp = 1.0f;	//限制比例
		
	steering_calc(chassis.spd_input.vx,chassis.spd_input.vy, chassis.spd_input.vw,chassis.rudder_angle_ref, chassis.wheel_spd_input);//舵轮
	
		    /* 计算总速度 */
    for(uint8_t i = 0; i < 4; ++i) {
        wheel_spd_input_buf[i] = ABS(chassis.wheel_spd_input[i]);
			wheel_spd_total += wheel_spd_input_buf[i];
    }
		    /* 速度重分配 并 限幅 */
    for(uint8_t j=0; j<4; j++)
    {
        chassis.wheel_spd_ref[j] = chassis.wheel_spd_input[j] / distribution_temp ;
        chassis.wheel_spd_ref[j] = data_limit(chassis.wheel_spd_ref[j], 8900, -8900);  //电机转速最高到8900
    }
		    /* 计算速度减小比例系数 */
    if(wheel_spd_total > (chassis.wheel_max * 4.0f))  //判断最大速度是否超额
    {
        distribution_temp = wheel_spd_total / (chassis.wheel_max * 4.0f); //超额速度除最大速度得分配比例
    }
    /* 速度重分配 并 限幅 */
    for(uint8_t j=0; j<4; j++)
    {
        chassis.wheel_spd_ref[j] = chassis.wheel_spd_input[j] / distribution_temp ;
        chassis.wheel_spd_ref[j] = data_limit(chassis.wheel_spd_ref[j], 8900, -8900);  //电机转速最高到8900
    }
}

/**
  * @brief      底盘PID
  * @author
  * @param[in]
  * @retval
  */
void chassis_pid_calcu(void)
{
    for(uint8_t i=0; i<4; i++)
    {
        /* 底盘电机速度环 */
        chassis.wheel_spd_fdb[i] = moto_msg.chassis_3508[i].speed_rpm;
        chassis.current_3508[i]  = (int16_t)pid_calc(&pid_chassis_3508_spd[i], chassis.wheel_spd_fdb[i], chassis.wheel_spd_ref[i]);

        /* 舵6020位置串级闭环 */
        chassis.rudder_ecd_ref[i] = (chassis.rudder_angle_ref[i]*22.753f) + chassis.rudder_ecd_offset[i];
        if(chassis.rudder_ecd_ref[i] >= 8191)
            chassis.rudder_ecd_ref[i] -= 8191;
        else if(chassis.rudder_ecd_ref[i] < 0)
            chassis.rudder_ecd_ref[i] += 8191;

        /* 舵角斜坡输入 */
        //chassis.rudder_ecd_ref_ramp[i] =
        chassis.rudder_ecd_fdb[i]   = moto_msg.chassis_6020[i].ecd;
        chassis.rudder_ecd_error[i] = circle_error(&chassis.rudder_ecd_ref[i], &chassis.rudder_ecd_fdb[i], 8191);
        /*位置环*/
        pid_calc(&pid_chassis_6020_ecd[i], chassis.rudder_ecd_fdb[i], (chassis.rudder_ecd_fdb[i]+chassis.rudder_ecd_error[i]));

        chassis.rudder_spd_ref[i] = pid_chassis_6020_ecd[i].pos_out;
        chassis.rudder_spd_fdb[i] = moto_msg.chassis_6020[i].speed_rpm;
        /*速度环*/
        chassis.current_6020[i] = (int16_t)pid_calc(&pid_chassis_6020_spd[i], chassis.rudder_spd_fdb[i], chassis.rudder_spd_ref[i]);
        
        
        
        
        
//        if(chassis.rudder_ecd_error[i] > 8191/3 && chassis.wheel_spd_fdb[i] < 1000) 
        
//        if(chassis.rudder_ecd_error[i] > 8191/5 ) 
//        {
//            chassis.current_3508[i] = 0;
//            chassis.current_6020[i] = 0;
//            if(chassis.current_3508[i] < 10) chassis.current_6020[i] = (int16_t)pid_calc(&pid_chassis_6020_spd[i], chassis.rudder_spd_fdb[i], chassis.rudder_spd_ref[i]);
//        }    
    }
}

