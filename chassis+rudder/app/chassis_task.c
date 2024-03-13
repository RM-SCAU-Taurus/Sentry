#include "chassis_task.h"
#include "cmsis_os.h"
#include "comm_task.h"
#include "string.h"
#include "modeswitch_task.h"
#include "remote_msg.h"
#include "pid.h"
#include "bsp_can.h"
#include "math_calcu.h"
#include "math.h"
#include "control_def.h"
#include "bsp_powerlimit.h"
#include "usart.h"
#include "DataScope_DP.h"
#include "bsp_vision.h"
#include "protocol_camp.h"
#include "func_generator.h"
#include "usb_task.h"
#include "DataScope_DP.h"
#include "iwdg.h"
#include "FreeRTOS.h"
#include "status_task.h"
extern TaskHandle_t can_msg_send_task_t;
float test[4];
float vwcos[4];
float vwsin[4];
//int32_t size;
extern chassis_ctrl_info_t chassis_ctrl;
extern game_status_info_t game_state;
extern chassis_odom_info_t chassis_odom;
chassis_t chassis;

extern void rm_queue_data(uint16_t cmd_id,void* buf,uint16_t len );
void chassis_init()
{
    for(uint8_t i=0; i<4; i++)
    {
        PID_struct_init(&pid_chassis_spd[i], POSITION_PID, 10000, 0,
                        7.0f, 0.0f, 0.0f);
				PID_struct_init(&pid_chassis_6020_ecd[i], POSITION_PID, 400, 100,
										0.5f, 0.0f, 0.0f);
				PID_struct_init(&pid_chassis_6020_spd[i], POSITION_PID, 28000, 15000,
										150.0f, 0.5f,  0.0f);//168 0.5
		}
//    PID_struct_init(&pid_chassis_angle, POSITION_PID, 6000, 0,
//                    7.0f, 0.0f, 0.0f);
//		
//		chassis.rudder_ecd_offset[0] = 2153;//100
//		chassis.rudder_ecd_offset[1] = 5290;//5300
//		chassis.rudder_ecd_offset[2] = 6350;//6350
//		chassis.rudder_ecd_offset[3] = 3880;//6500
		
				
		chassis.rudder_ecd_offset[0] = 5050;//100
		chassis.rudder_ecd_offset[1] = 3597;//5300
		chassis.rudder_ecd_offset[2] = 7657;//6350
		chassis.rudder_ecd_offset[3] = 6668;//6500
		
		
		
    scale.ch3 = RC_CH3_SCALE;
    scale.ch4 = RC_CH4_SCALE;
    chassis.mode = CHASSIS_MODE_PROTECT;
    chassis.mode = chassis.mode;
    chassis.fight_dir = 1;
    chassis.spin_dir  = 1;
    chassis.msg_handle = can_msg_read;
    chassis.msg_send = can1_send_chassis_message;
   // chassis.power_control = Power_Control;
    chassis.spd_distribution = chassis_spd_distribution;
    chassis.Float_to_uint8 = Float2Byte;
    chassis.game_msg_send = can1_send_game_msg;
    chassis.spd_msg_send  = can1_send_spd_msg;
    chassis.wheel_max = 9000;  /* 关掉功率控制时需要初始化 */
		
}
static void chassis_mode_switch(void)
{
    /* 系统历史状态机 */
    static ctrl_mode_e last_ctrl_mode = PROTECT_MODE;
    
    /* 单次触发使能标志 */
    static uint8_t spin_flag = 0;
    
    /* 底盘状态机 */
    switch( ctrl_mode )
    {
        case PROTECT_MODE:  //能量模式和保护模式下，底盘行为相同
        {
            chassis.mode = CHASSIS_MODE_PROTECT;
        }
        break;
        case REMOTER_MODE:
        {
            if( last_ctrl_mode != REMOTER_MODE )  //切入遥控模式，初始化底盘模式
                chassis.mode = CHASSIS_MODE_REMOTER_FOLLOW;
            /* 底盘小陀螺模式 */
            if( rc.ch5 == 0 )   spin_flag = 1;
            if( rc.ch5 == 660 && spin_flag )
            {
                spin_flag = 0;
                if( chassis.mode == CHASSIS_MODE_REMOTER_ROTATE )
                {
                    chassis.spin_dir = -chassis.spin_dir;  //小陀螺反向
                    chassis.mode = CHASSIS_MODE_REMOTER_FOLLOW;
                }
                else if( chassis.mode == CHASSIS_MODE_REMOTER_FOLLOW )
                    chassis.mode = CHASSIS_MODE_REMOTER_ROTATE;
            }
        }
        break;
        case AUTO_MODE:
        {
            chassis.mode = CHASSIS_MODE_AUTO;

        }
				break;
        default: break;
    }
    /* 系统历史状态更新 */
    last_ctrl_mode = ctrl_mode;
}


/**
  * @brief chassis_task
  * @param
  * @attention
	* @note
  */
float test_speed1,test_speed2,test_speed3;
void chassis_task(void const *argu)
{
    uint32_t mode_wake_time = osKernelSysTick();
    for(;;)
    {
        taskENTER_CRITICAL();
		//	HAL_IWDG_Refresh(&hiwdg);//看门狗
        chassis_mode_switch();
        switch( chassis.mode )
        {
            case CHASSIS_MODE_PROTECT:  //底盘保护模式
            {
                chassis.spd_input.vx = 0;
                chassis.spd_input.vy = 0;
                chassis.spd_input.vw = 0;
							 rudder_odom_cal();//里程计计算
							memset(motor_cur.chassis_cur_3508,0, sizeof(chassis.current)); 
							memset(motor_cur.chassis_cur_6020,0, sizeof(chassis.current_6020)); 
							  for(uint8_t i=0; i<4; i++)
            {
                pid_chassis_6020_spd[i].iout=0;
            }
            } 
            break;
            case CHASSIS_MODE_REMOTER_FOLLOW:  //底盘遥控跟随模式
            case CHASSIS_MODE_REMOTER_ROTATE:  //底盘遥控陀螺模式
            case CHASSIS_MODE_AUTO:
            {
                chassis.spd_input.vx = chassis.spd_input.vx;
                chassis.spd_input.vy = chassis.spd_input.vy;
                chassis.spd_input.vw = chassis.spd_input.vw;
							/*遥控控制*/
//								chassis.spd_input.vx=rc.ch4*6.0f*2.0f;
//								chassis.spd_input.vy=rc.ch3*6.0f*2.0f;
//								chassis.spd_input.vw= -1.0f * rc.ch1 * 12.0f*1.5f;
//							chassis.spd_input.vx = test_speed1;
//                chassis.spd_input.vy = test_speed2;
//                chassis.spd_input.vw = test_speed3;
						}
            break;
            default: break;
        }
							/* 底盘速度分配 */
					 chassis.spd_distribution();
						chassis_pid_calcu();//舵轮pid计算
						/* 功率控制 */
						powercontrol.ParamUpdate();
						Power_Control(chassis.current,chassis.current_6020);
						     for(uint8_t i=0; i<4; i++)//转舵保护
		        {
		               if(ABS(chassis.rudder_ecd_error[i])>1000)
		              {
		                    memset(motor_cur.chassis_cur_3508, 0, sizeof(chassis.current));
		               }
		      }
						        /* 发送电流，控制电机 */
					memcpy(motor_cur.chassis_cur_3508,chassis.current, sizeof(chassis.current));  
					 memcpy(motor_cur.chassis_cur_6020, chassis.current_6020, sizeof(chassis.current_6020));
        /* 波形显示 */			
//        static uint8_t debug_i;
//        debug_i++;
//        if( debug_i == 15 && ctrl_mode != PROTECT_MODE )  //15
//        {
//            FGT_sin_cal(&test_sin);
//            DataWave(&huart3);
//            debug_i=0;
//        }
        /*读取底盘状态数据*/
        chassis.Float_to_uint8(&chassis.spd_fdb.vx,chassis.spd_tx_data,0);
        chassis.Float_to_uint8(&chassis.spd_fdb.vy,chassis.spd_tx_data,4);
        chassis.Float_to_uint8(&chassis.spd_fdb.vw,chassis.spd_tx_data,8);
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
void mecanum_calc(float vx, float vy, float vw, int16_t speed[])
{
    int16_t wheel_rpm[4];
    wheel_rpm[0] =  vx + vy - vw;
    wheel_rpm[1] = -vx + vy - vw;
    wheel_rpm[2] =  vx - vy - vw;
    wheel_rpm[3] = -vx - vy - vw;    
    memcpy(speed, wheel_rpm, 4 * sizeof(int16_t));
}

/**
  * @brief 舵轮解算函数 Vw = sqrt（2）Vw；
  * @param input : ?=+vx(mm/s)  ?=+vy(mm/s)  ccw=+vw(deg/s)
  *        output: every wheel speed(rpm) and angle(°)
  * @note  0=FL 1=BL 2=FR 3=BR改-> 	2 0
																		1 3
  */
void steering_calc(float vx, float vy, float vw, int16_t angle[], int16_t speed[])
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
	
					for(uint8_t i = 0;i<4; i++)
				{

				chassis.rudder_ecd_fdb[i] = (moto_chassis_6020[i].ecd - chassis.rudder_ecd_offset[i])*0.04395f;
				if(circle_error(rudder_angle[i],chassis.rudder_ecd_fdb[i],360)>90.0f)
				{
				rudder_angle[i] -= 180;
				wheel_rpm[i] = -wheel_rpm[i];
				}
				else if(circle_error(rudder_angle[i],chassis.rudder_ecd_fdb[i],360)<(-90.0f))
				{
				rudder_angle[i] += 180;
				wheel_rpm[i] = -wheel_rpm[i];
				}
				if(vx == 0 && vy == 0 && vw == 0)
				{
				rudder_angle[i] = 0;
				}
				}


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
	* @brief          底盘速度分配函数，输入速度超过最大轮速时，将输入的速度按比例分配到三个轴向上
	* @author
	* @param[in]      void
	* @retval         void
	*/
void chassis_spd_distribution(void)
{
    float  wheel_spd_input_buf[4];
    float  wheel_spd_total = 0;  //总轮速
    float  distribution_temp = 1.0f;	//限制比例

    /* 麦轮逆运动学原设定速度解算 */
    //mecanum_calc(chassis.spd_input.vx,chassis.spd_input.vy, chassis.spd_input.vw, chassis.wheel_spd_input);
		steering_calc(chassis.spd_input.vx,chassis.spd_input.vy, chassis.spd_input.vw,chassis.rudder_angle_ref, chassis.wheel_spd_input);//舵轮
    /* 计算总速度 */
    for(int i=0; i<4; i++)
    {
        wheel_spd_input_buf[i]=ABS(chassis.wheel_spd_input[i]);
        wheel_spd_total += wheel_spd_input_buf[i];
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
//    /* 麦轮正运动学处理后设定速度解算 */
//    chassis.spd_ref.vx = (+chassis.wheel_spd_ref[0] - chassis.wheel_spd_ref[1] + chassis.wheel_spd_ref[2] - chassis.wheel_spd_ref[3] )* 0.25f;
//    chassis.spd_ref.vy = (+chassis.wheel_spd_ref[0] + chassis.wheel_spd_ref[1] - chassis.wheel_spd_ref[2] - chassis.wheel_spd_ref[3] )* 0.25f;
//    chassis.spd_ref.vw = (-chassis.wheel_spd_ref[0] - chassis.wheel_spd_ref[1] - chassis.wheel_spd_ref[2] - chassis.wheel_spd_ref[3] )* 0.25f;
//   
//		
//    chassis.spd_error = chassis.spd_ref.vx + chassis.spd_ref.vy + chassis.spd_ref.vw
//                        - chassis.spd_fdb.vx - chassis.spd_fdb.vy - chassis.spd_fdb.vw;

//    /* 麦轮正运动学反馈速度解算 */
//    chassis.spd_fdb.vx = (+moto_chassis[0].speed_rpm - moto_chassis[1].speed_rpm + moto_chassis[2].speed_rpm - moto_chassis[3].speed_rpm )* 0.25f;
//    chassis.spd_fdb.vy = (+moto_chassis[0].speed_rpm + moto_chassis[1].speed_rpm - moto_chassis[2].speed_rpm - moto_chassis[3].speed_rpm )* 0.25f;
//    chassis.spd_fdb.vw = (-moto_chassis[0].speed_rpm - moto_chassis[1].speed_rpm - moto_chassis[2].speed_rpm - moto_chassis[3].speed_rpm )* 0.25f;
    
					if(!status.chassis_status[0]&&!status.chassis_status[1]&&!status.chassis_status[2]&&!status.chassis_status[3])//4个全掉才发零
					{
					chassis.spd_fdb.vx=0;
					chassis.spd_fdb.vy=0;
					chassis.spd_fdb.vw=0;
					}
					else
					{
												//		/*舵轮逆运动速度解算*/调每个轮子的正负和轮子的r
//						chassis.spd_fdb.vx= (-moto_chassis[0].speed_rpm*cosf((chassis.rudder_ecd_fdb[0]-chassis.rudder_ecd_offset[0])/22.753f/57.3f)//-
//											+moto_chassis[1].speed_rpm*cosf((chassis.rudder_ecd_fdb[1]-chassis.rudder_ecd_offset[1])/22.753f/57.3f)
//											+moto_chassis[2].speed_rpm*cosf((chassis.rudder_ecd_fdb[2]-chassis.rudder_ecd_offset[2])/22.753f/57.3f)
//											-moto_chassis[3].speed_rpm*cosf((chassis.rudder_ecd_fdb[3]-chassis.rudder_ecd_offset[3])/22.753f/57.3f));//-
//											
//											//* 0.25f*6.0f*0.0625f/19/57.3f;// 转/s -》度/s->转弧度->除转速比->乘轮子半径
//						chassis.spd_fdb.vy= (-moto_chassis[0].speed_rpm*sinf((chassis.rudder_ecd_fdb[0]-chassis.rudder_ecd_offset[0])/22.753f/57.3f)
//											+moto_chassis[1].speed_rpm*sinf((chassis.rudder_ecd_fdb[1]-chassis.rudder_ecd_offset[1])/22.753f/57.3f)
//											+moto_chassis[2].speed_rpm*sinf((chassis.rudder_ecd_fdb[2]-chassis.rudder_ecd_offset[2])/22.753f/57.3f)
//											-moto_chassis[3].speed_rpm*sinf((chassis.rudder_ecd_fdb[3]-chassis.rudder_ecd_offset[3])/22.753f/57.3f));
//											//* 0.25f*6.0f*0.0625f/19/57.3f;// 转/s -》度/s->转弧度->除转速比->乘轮子半径
//						chassis.spd_fdb.vw= ((-moto_chassis[0].speed_rpm*cosf((chassis.rudder_ecd_fdb[0]-chassis.rudder_ecd_offset[0]-45*22.753f)/22.753f/57.3f)
//											-moto_chassis[1].speed_rpm*cosf((chassis.rudder_ecd_fdb[1]-chassis.rudder_ecd_offset[1]-45*22.753f)/22.753f/57.3f)
//											-moto_chassis[2].speed_rpm*cosf((chassis.rudder_ecd_fdb[2]-chassis.rudder_ecd_offset[2]+45*22.753f)/22.753f/57.3f)
//											-moto_chassis[3].speed_rpm*cosf((chassis.rudder_ecd_fdb[3]-chassis.rudder_ecd_offset[3]+45*22.753f)/22.753f/57.3f))+
//											(moto_chassis[0].speed_rpm*sinf((chassis.rudder_ecd_fdb[0]-chassis.rudder_ecd_offset[0]+45*22.753f)/22.753f/57.3f)
//											+moto_chassis[1].speed_rpm*sinf((chassis.rudder_ecd_fdb[1]-chassis.rudder_ecd_offset[1]+45*22.753f)/22.753f/57.3f)
//											-moto_chassis[2].speed_rpm*sinf((chassis.rudder_ecd_fdb[2]-chassis.rudder_ecd_offset[2]-45*22.753f)/22.753f/57.3f)
//											-moto_chassis[3].speed_rpm*sinf((chassis.rudder_ecd_fdb[3]-chassis.rudder_ecd_offset[3]-45*22.753f)/22.753f/57.3f)))*1000;
//											//*0.25f*0.5f*0.375f/19.0f/57.3f*5.64f;// 转/s -》度/s->转弧度->除转速比->乘轮子半径->除轮子到车中心半径

				//		/*舵轮逆运动速度解算*/调每个轮子的正负和轮子的r
						chassis.spd_fdb.vx= (+moto_chassis[0].speed_rpm*cosf((chassis.rudder_ecd_fdb[0]-chassis.rudder_ecd_offset[0])/22.753f/57.3f)//-
											-moto_chassis[1].speed_rpm*cosf((chassis.rudder_ecd_fdb[1]-chassis.rudder_ecd_offset[1])/22.753f/57.3f)
											-moto_chassis[2].speed_rpm*cosf((chassis.rudder_ecd_fdb[2]-chassis.rudder_ecd_offset[2])/22.753f/57.3f)
											+moto_chassis[3].speed_rpm*cosf((chassis.rudder_ecd_fdb[3]-chassis.rudder_ecd_offset[3])/22.753f/57.3f));//-
											
											//* 0.25f*6.0f*0.0625f/19/57.3f;// 转/s -》度/s->转弧度->除转速比->乘轮子半径
						chassis.spd_fdb.vy= (+moto_chassis[0].speed_rpm*sinf((chassis.rudder_ecd_fdb[0]-chassis.rudder_ecd_offset[0])/22.753f/57.3f)
											-moto_chassis[1].speed_rpm*sinf((chassis.rudder_ecd_fdb[1]-chassis.rudder_ecd_offset[1])/22.753f/57.3f)
											-moto_chassis[2].speed_rpm*sinf((chassis.rudder_ecd_fdb[2]-chassis.rudder_ecd_offset[2])/22.753f/57.3f)
											+moto_chassis[3].speed_rpm*sinf((chassis.rudder_ecd_fdb[3]-chassis.rudder_ecd_offset[3])/22.753f/57.3f));
											//* 0.25f*6.0f*0.0625f/19/57.3f;// 转/s -》度/s->转弧度->除转速比->乘轮子半径
						chassis.spd_fdb.vw= ((+moto_chassis[0].speed_rpm*cosf((chassis.rudder_ecd_fdb[0]-chassis.rudder_ecd_offset[0]+45*22.753f)/22.753f/57.3f)
											+moto_chassis[1].speed_rpm*cosf((chassis.rudder_ecd_fdb[1]-chassis.rudder_ecd_offset[1]+45*22.753f)/22.753f/57.3f)
											+moto_chassis[2].speed_rpm*cosf((chassis.rudder_ecd_fdb[2]-chassis.rudder_ecd_offset[2]-45*22.753f)/22.753f/57.3f)
											+moto_chassis[3].speed_rpm*cosf((chassis.rudder_ecd_fdb[3]-chassis.rudder_ecd_offset[3]-45*22.753f)/22.753f/57.3f))+
											(moto_chassis[0].speed_rpm*sinf((chassis.rudder_ecd_fdb[0]-chassis.rudder_ecd_offset[0]-45*22.753f)/22.753f/57.3f)
											+moto_chassis[1].speed_rpm*sinf((chassis.rudder_ecd_fdb[1]-chassis.rudder_ecd_offset[1]-45*22.753f)/22.753f/57.3f)
											-moto_chassis[2].speed_rpm*sinf((chassis.rudder_ecd_fdb[2]-chassis.rudder_ecd_offset[2]+45*22.753f)/22.753f/57.3f)
											-moto_chassis[3].speed_rpm*sinf((chassis.rudder_ecd_fdb[3]-chassis.rudder_ecd_offset[3]+45*22.753f)/22.753f/57.3f)))*1000;
//											//*0.25f*0.5f*0.375f/19.0f/57.3f*5.64f;// 转/s -》度/s->转弧度->除转速比->乘轮子半径->除轮子到车中心半径
//											test[0] =((chassis.rudder_ecd_fdb[0]-chassis.rudder_ecd_offset[0]+0*22.753f)/22.753f/1.0f);
//											test[1] =((chassis.rudder_ecd_fdb[1]-chassis.rudder_ecd_offset[1]+0*22.753f)/22.753f/1.0f);
//											test[2] = ((chassis.rudder_ecd_fdb[2]-chassis.rudder_ecd_offset[2]-0*22.753f)/22.753f/1.0f);
//											test[3] = ((chassis.rudder_ecd_fdb[3]-chassis.rudder_ecd_offset[3]-0*22.753f)/22.753f/1.0f);
//											
//											vwcos[0]=(+moto_chassis[0].speed_rpm*cosf((chassis.rudder_ecd_fdb[0]-chassis.rudder_ecd_offset[0]+45*22.753f)/22.753f/57.3f));
//											vwcos[1]=(+moto_chassis[1].speed_rpm*cosf((chassis.rudder_ecd_fdb[1]-chassis.rudder_ecd_offset[1]+45*22.753f)/22.753f/57.3f));
//											vwcos[2]=(+moto_chassis[2].speed_rpm*cosf((chassis.rudder_ecd_fdb[2]-chassis.rudder_ecd_offset[2]-45*22.753f)/22.753f/57.3f));
//											vwcos[3]=(+moto_chassis[3].speed_rpm*cosf((chassis.rudder_ecd_fdb[3]-chassis.rudder_ecd_offset[3]-45*22.753f)/22.753f/57.3f));
//											
//											vwsin[0]=(+moto_chassis[0].speed_rpm*sinf((chassis.rudder_ecd_fdb[0]-chassis.rudder_ecd_offset[0]-45*22.753f)/22.753f/57.3f));
//											vwsin[1]=(+moto_chassis[1].speed_rpm*sinf((chassis.rudder_ecd_fdb[1]-chassis.rudder_ecd_offset[1]-45*22.753f)/22.753f/57.3f));
//											vwsin[2]=(-moto_chassis[2].speed_rpm*sinf((chassis.rudder_ecd_fdb[2]-chassis.rudder_ecd_offset[2]+45*22.753f)/22.753f/57.3f));
//											vwsin[3]=(-moto_chassis[3].speed_rpm*sinf((chassis.rudder_ecd_fdb[3]-chassis.rudder_ecd_offset[3]+45*22.753f)/22.753f/57.3f));
											
						}
	
		chassis.odom.x += chassis.spd_fdb.vx*0.001f;
    chassis.odom.y += chassis.spd_fdb.vy*0.001f;

}

void chassis_pid_calcu(void)
{
    for(uint8_t i=0; i<4; i++)
    {
			    chassis.wheel_spd_fdb[i] = moto_chassis[i].speed_rpm;
          chassis.current[i] = (int16_t)pid_calc(&pid_chassis_spd[i], chassis.wheel_spd_fdb[i], chassis.wheel_spd_ref[i]);		
        //							/* 舵6020位置串级闭环 */
					chassis.rudder_ecd_ref[i] = (chassis.rudder_angle_ref[i]*22.753f) + chassis.rudder_ecd_offset[i];
					if(chassis.rudder_ecd_ref[i] >= 8191)
							chassis.rudder_ecd_ref[i] -= 8191;
					else if(chassis.rudder_ecd_ref[i] < 0)
							chassis.rudder_ecd_ref[i] += 8191;
					/* 舵角斜坡输入 */
					chassis.rudder_ecd_fdb[i]   = moto_chassis_6020[i].ecd;
					chassis.rudder_ecd_error[i] = circle_error(chassis.rudder_ecd_ref[i], chassis.rudder_ecd_fdb[i], 8191);
					/*位置环*/
					pid_calc(&pid_chassis_6020_ecd[i], chassis.rudder_ecd_fdb[i], (chassis.rudder_ecd_fdb[i]+chassis.rudder_ecd_error[i]));

					chassis.rudder_spd_ref[i] = pid_chassis_6020_ecd[i].pos_out;
					chassis.rudder_spd_fdb[i] = moto_chassis_6020[i].speed_rpm;
					/*速度环*/
					chassis.current_6020[i] = (int16_t)pid_calc(&pid_chassis_6020_spd[i], chassis.rudder_spd_fdb[i], chassis.rudder_spd_ref[i]);
    }
										
}


void rudder_odom_cal(void)
{
			if(!status.chassis_status[0]||!status.chassis_status[1]||!status.chassis_status[2]||!status.chassis_status[3])
					{
					chassis.spd_fdb.vx=0;
					chassis.spd_fdb.vy=0;
					chassis.spd_fdb.vw=0;
					}
					else
					{
												//		/*舵轮逆运动速度解算*/调每个轮子的正负和轮子的r
//						chassis.spd_fdb.vx= (+moto_chassis[0].speed_rpm*cosf((chassis.rudder_ecd_fdb[0]-chassis.rudder_ecd_offset[0])/22.753f/57.3f)//-
//											-moto_chassis[1].speed_rpm*cosf((chassis.rudder_ecd_fdb[1]-chassis.rudder_ecd_offset[1])/22.753f/57.3f)
//											-moto_chassis[2].speed_rpm*cosf((chassis.rudder_ecd_fdb[2]-chassis.rudder_ecd_offset[2])/22.753f/57.3f)
//											+moto_chassis[3].speed_rpm*cosf((chassis.rudder_ecd_fdb[3]-chassis.rudder_ecd_offset[3])/22.753f/57.3f));//-
//											
//											//* 0.25f*6.0f*0.0625f/19/57.3f;// 转/s -》度/s->转弧度->除转速比->乘轮子半径
//						chassis.spd_fdb.vy= (+moto_chassis[0].speed_rpm*sinf((chassis.rudder_ecd_fdb[0]-chassis.rudder_ecd_offset[0])/22.753f/57.3f)
//											-moto_chassis[1].speed_rpm*sinf((chassis.rudder_ecd_fdb[1]-chassis.rudder_ecd_offset[1])/22.753f/57.3f)
//											-moto_chassis[2].speed_rpm*sinf((chassis.rudder_ecd_fdb[2]-chassis.rudder_ecd_offset[2])/22.753f/57.3f)
//											+moto_chassis[3].speed_rpm*sinf((chassis.rudder_ecd_fdb[3]-chassis.rudder_ecd_offset[3])/22.753f/57.3f));
//											//* 0.25f*6.0f*0.0625f/19/57.3f;// 转/s -》度/s->转弧度->除转速比->乘轮子半径
//						chassis.spd_fdb.vw= ((+moto_chassis[0].speed_rpm*cosf((chassis.rudder_ecd_fdb[0]-chassis.rudder_ecd_offset[0]-45*22.753f)/22.753f/57.3f)
//											+moto_chassis[1].speed_rpm*cosf((chassis.rudder_ecd_fdb[1]-chassis.rudder_ecd_offset[1]-45*22.753f)/22.753f/57.3f)
//											+moto_chassis[2].speed_rpm*cosf((chassis.rudder_ecd_fdb[2]-chassis.rudder_ecd_offset[2]+45*22.753f)/22.753f/57.3f)
//											+moto_chassis[3].speed_rpm*cosf((chassis.rudder_ecd_fdb[3]-chassis.rudder_ecd_offset[3]+45*22.753f)/22.753f/57.3f))+
//											(moto_chassis[0].speed_rpm*sinf((chassis.rudder_ecd_fdb[0]-chassis.rudder_ecd_offset[0]+45*22.753f)/22.753f/57.3f)
//											+moto_chassis[1].speed_rpm*sinf((chassis.rudder_ecd_fdb[1]-chassis.rudder_ecd_offset[1]+45*22.753f)/22.753f/57.3f)
//											-moto_chassis[2].speed_rpm*sinf((chassis.rudder_ecd_fdb[2]-chassis.rudder_ecd_offset[2]-45*22.753f)/22.753f/57.3f)
//											-moto_chassis[3].speed_rpm*sinf((chassis.rudder_ecd_fdb[3]-chassis.rudder_ecd_offset[3]-45*22.753f)/22.753f/57.3f)))*1000;
//											//*0.25f*0.5f*0.375f/19.0f/57.3f*5.64f;// 转/s -》度/s->转弧度->除转速比->乘轮子半径->除轮子到车中心半径
//											
															//		/*舵轮逆运动速度解算*/调每个轮子的正负和轮子的r
						chassis.spd_fdb.vx= (+moto_chassis[0].speed_rpm*cosf((chassis.rudder_ecd_fdb[0]-chassis.rudder_ecd_offset[0])/22.753f/57.3f)//-
											-moto_chassis[1].speed_rpm*cosf((chassis.rudder_ecd_fdb[1]-chassis.rudder_ecd_offset[1])/22.753f/57.3f)
											-moto_chassis[2].speed_rpm*cosf((chassis.rudder_ecd_fdb[2]-chassis.rudder_ecd_offset[2])/22.753f/57.3f)
											+moto_chassis[3].speed_rpm*cosf((chassis.rudder_ecd_fdb[3]-chassis.rudder_ecd_offset[3])/22.753f/57.3f));//-
											
											//* 0.25f*6.0f*0.0625f/19/57.3f;// 转/s -》度/s->转弧度->除转速比->乘轮子半径
						chassis.spd_fdb.vy= (+moto_chassis[0].speed_rpm*sinf((chassis.rudder_ecd_fdb[0]-chassis.rudder_ecd_offset[0])/22.753f/57.3f)
											-moto_chassis[1].speed_rpm*sinf((chassis.rudder_ecd_fdb[1]-chassis.rudder_ecd_offset[1])/22.753f/57.3f)
											-moto_chassis[2].speed_rpm*sinf((chassis.rudder_ecd_fdb[2]-chassis.rudder_ecd_offset[2])/22.753f/57.3f)
											+moto_chassis[3].speed_rpm*sinf((chassis.rudder_ecd_fdb[3]-chassis.rudder_ecd_offset[3])/22.753f/57.3f));
											//* 0.25f*6.0f*0.0625f/19/57.3f;// 转/s -》度/s->转弧度->除转速比->乘轮子半径
						chassis.spd_fdb.vw= ((+moto_chassis[0].speed_rpm*cosf((chassis.rudder_ecd_fdb[0]-chassis.rudder_ecd_offset[0]+45*22.753f)/22.753f/57.3f)
											+moto_chassis[1].speed_rpm*cosf((chassis.rudder_ecd_fdb[1]-chassis.rudder_ecd_offset[1]+45*22.753f)/22.753f/57.3f)
											+moto_chassis[2].speed_rpm*cosf((chassis.rudder_ecd_fdb[2]-chassis.rudder_ecd_offset[2]-45*22.753f)/22.753f/57.3f)
											+moto_chassis[3].speed_rpm*cosf((chassis.rudder_ecd_fdb[3]-chassis.rudder_ecd_offset[3]-45*22.753f)/22.753f/57.3f))+
											(moto_chassis[0].speed_rpm*sinf((chassis.rudder_ecd_fdb[0]-chassis.rudder_ecd_offset[0]-45*22.753f)/22.753f/57.3f)
											+moto_chassis[1].speed_rpm*sinf((chassis.rudder_ecd_fdb[1]-chassis.rudder_ecd_offset[1]-45*22.753f)/22.753f/57.3f)
											-moto_chassis[2].speed_rpm*sinf((chassis.rudder_ecd_fdb[2]-chassis.rudder_ecd_offset[2]+45*22.753f)/22.753f/57.3f)
											-moto_chassis[3].speed_rpm*sinf((chassis.rudder_ecd_fdb[3]-chassis.rudder_ecd_offset[3]+45*22.753f)/22.753f/57.3f)))*1000;
//											//*0.25f*0.5f*0.375f/19.0f/57.3f*5.64f;// 转/s -》度/s->转弧度->除转速比->乘轮子半径->除轮子到车中心半径
						}
}
/**
  * @brief          底盘斜坡启动函数，通过斜坡函数映射目标速度
  * @author
  * @param[in]      void
  * @retval         void
  */
void chassis_ramp(void)
{
    if(rc.kb.bit.W)
    {
        ramp_calc(&chassis_x_ramp,1.0f,chassis.keyboard_input, chassis.wheel_max, 0.0f);
    }
    else if(rc.kb.bit.S)
    {
        ramp_calc(&chassis_x_ramp,1.0f,-chassis.keyboard_input, 0.0f, -chassis.wheel_max);
    }
    else
    {
        if(chassis_x_ramp.out > 0)
        {
            ramp_calc(&chassis_x_ramp,1.0f,-chassis.keyboard_input,chassis.wheel_max, 0.0f);
        }
        else if(chassis_x_ramp.out < 0)
        {
            ramp_calc(&chassis_x_ramp,1.0f,chassis.keyboard_input, 0.0f, -chassis.wheel_max);
        }
    }
    if(rc.kb.bit.D)
    {
        ramp_calc(&chassis_y_ramp,1.0f,chassis.keyboard_input, chassis.wheel_max, 0.0f);
    }
    else if(rc.kb.bit.A)
    {
        ramp_calc(&chassis_y_ramp,1.0f,-chassis.keyboard_input, 0.0f, -chassis.wheel_max);
    }
    else
    {
        if(chassis_y_ramp.out > 0)
        {
            ramp_calc(&chassis_y_ramp,1.0f,-chassis.keyboard_input, chassis.wheel_max, 0.0f);
        }
        else if(chassis_y_ramp.out < 0)
        {
            ramp_calc(&chassis_y_ramp,1.0f,chassis.keyboard_input, 0.0f, -chassis.wheel_max);
        }
    }
}

/**
  * @brief          底盘S型启动函数，通过S型函数映射目标速度
  * @author
  * @param[in]      void
  * @retval         void
  */
void chassis_sigmoid(void)
{
    static float x_speed_sigmoid,y_speed_sigmoid;
    static float x_input,y_input;
    static float x_sigmoid,y_sigmoid;
    if(rc.kb.bit.W) 					x_input += SIGMOID_PERIOD;
    else if(rc.kb.bit.S)			x_input -= SIGMOID_PERIOD;
    else
    {
        if(x_input > 0)					x_input -= SIGMOID_PERIOD;
        else if(x_input < 0)		x_input += SIGMOID_PERIOD;
    }
    if(rc.kb.bit.D) 					y_input += SIGMOID_PERIOD;
    else if(rc.kb.bit.A)			y_input -= SIGMOID_PERIOD;
    else
    {
        if(y_input > 0)					y_input -= SIGMOID_PERIOD;
        else if(y_input < 0)		y_input += SIGMOID_PERIOD;
    }

    if(x_input >= (2*SIGMOID_MAX))					x_input=  (2*SIGMOID_MAX);
    else if(x_input <= -(2*SIGMOID_MAX))		x_input= -(2*SIGMOID_MAX);

    if(y_input >= (2*SIGMOID_MAX))					y_input=  (2*SIGMOID_MAX);
    else if(y_input <= -(2*SIGMOID_MAX))		y_input= -(2*SIGMOID_MAX);

    if(x_input <= ABS(SIGMOID_PERIOD) && x_input >= -ABS(SIGMOID_PERIOD))
        x_sigmoid = 0;
    else if(x_input >= ABS(SIGMOID_PERIOD))
        x_sigmoid = Sigmoid_function(x_input);
    else if(x_input <= -ABS(SIGMOID_PERIOD))
        x_sigmoid = -Sigmoid_function(x_input);

    if(y_input <= ABS(SIGMOID_PERIOD) && y_input >= -ABS(SIGMOID_PERIOD))
        y_sigmoid = 0;
    else if(y_input >= ABS(SIGMOID_PERIOD))
        y_sigmoid = Sigmoid_function(y_input);
    else if(y_input <= -ABS(SIGMOID_PERIOD))
        y_sigmoid = -Sigmoid_function(y_input);

    x_speed_sigmoid = x_sigmoid * chassis.wheel_max;
    y_speed_sigmoid = y_sigmoid * chassis.wheel_max;

    chassis.spd_input.vx = (float)(x_speed_sigmoid * cos(chassis.angle_error) + (-1.0f)*y_speed_sigmoid * sin(chassis.angle_error));
    chassis.spd_input.vy = (float)(x_speed_sigmoid * sin(chassis.angle_error) - (-1.0f)*y_speed_sigmoid * cos(chassis.angle_error));
}
void can_msg_read(uint32_t can_id,uint8_t * data)
{
    if(can_id==CHASSIS_MSG_ID)
       memcpy(&chassis.spd_fdb.vx,data,4);
       memcpy(&chassis.spd_fdb.vy,data+4,4);
     if (can_id==GAME_MSG_ID)
    {
        memcpy(&chassis.spd_fdb.vw,data,4);
        /* code */
    }
  /*  if (can_id==CHASSIS_CTRl_MSG_ID)
    {
          chassis.spd_input.vx = data[0]<<data[1];
          chassis.spd_input.vy = data[2]<<data[3];
          chassis.spd_input.vw = data[4]<<data[5];
    }*/
    
    
}


