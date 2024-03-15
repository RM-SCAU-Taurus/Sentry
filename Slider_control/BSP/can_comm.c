#include "can_comm.h"
#include "pid.h"
#include "string.h"
#include "bsp_can.h"
#include "dji_motor.h"
#include "KalmanFilter.h"
#include "iwdg.h"
#define CHASSIS_CAN_TX_ID 	0x200
#define GIMBAL_CAN_TX_ID  	0x1ff
#define Fric_run 1
#define Fric_stop 0
#define Fric_slow 2
moto_measure_t moto_fric[2];
fric_protect_mode_e fric_flag;
int16_t fric_cur[2];

fric_protect_mode_e fric_protect_flag;

fric_t fric;

pid_t pid_spd[2];	

	 static uint8_t stop_flag=0;
	 static uint8_t last_fric_mdoe=0;
	 static uint8_t fric_mode=0;
	 
/* CAN send and receive ID */
typedef enum
{
    /* can1 */
    /* can2 */
	  CAN_Fric_Ctrl_ID   = 	0x777,
		CAN_3508_R_ID = 0x201,
		CAN_3508_L_ID = 0x202,
} can_msg_id_e;
moto_measure_t moto_balance[3];

//static void CHASSIS_SLIPPED_DECT(void);
static void User_can1_callback(uint32_t ID, uint8_t* CAN_RxData);
static void User_can2_callback(uint32_t ID, uint8_t* CAN_RxData);

void can_device_init(void)
{
    uint32_t can_ID1[] = { CAN_Fric_Ctrl_ID,0xFFF};
    uint32_t can_ID2[] = { 0x201, 0x202, 0xFFF};
    canx_init(&hcan1, can_ID1, User_can1_callback);
    canx_init(&hcan2, can_ID2, User_can2_callback);
}

void Fric_msg_handle(uint8_t * CAN_Rx_data)
{
		fric.Fric_Pid_Set[0].fric_spd_ref = (int16_t)(CAN_Rx_data[2] << 8 | CAN_Rx_data[3]);
		fric.Fric_Pid_Set[1].fric_spd_ref = (int16_t)(CAN_Rx_data[4] << 8 | CAN_Rx_data[5]);
}

static void User_can1_callback(uint32_t ID, uint8_t* CAN_RxData)
{
    switch (ID)
    {
    case CAN_Fric_Ctrl_ID:			
    {
        Fric_msg_handle(CAN_RxData);
        break;
    }
    default:
        break;
    }
}

static void User_can2_callback(uint32_t ID, uint8_t* CAN_RxData)
{
    switch (ID)
    {

			case CAN_3508_L_ID:
			case CAN_3508_R_ID:
		  	{
														
            static uint8_t i;
            i = ID- CAN_3508_R_ID;
            encoder_data_handler(&moto_fric[i],CAN_RxData);
            break;		
				}
    default:
        break;
    }
}
void Fric_init(void) {
	 for(uint8_t i = 0; i < 2; i++) {
   PID_struct_init(&pid_spd[i], POSITION_PID, 16000, 0, 0, 0,
            10.1f, 0.0f, 0.0f);
	 }
	 
}

static float ramp_input(float fdb, float ref, float slope) {
    if (ref - fdb > slope) {
        fdb += slope;		//目标角度斜坡改变
    } else if (ref - fdb < -slope) {
        fdb -= slope;
    } else {
        fdb = ref;
    }
    return fdb;
}

float data_limit(float data, float max, float min)
{
    if(data >= max)					return max;
    else if(data <= min)		return min;
    else 										return data;
}



 void Balance_slider_control(void)
{
//    /*------------------------------------平衡块位置控制-----------------------------------------*/
//    /* 平衡块6020的位置反馈零点迁移处理 */
//    if (moto_balance[0].ecd > WEIGHT0_OFFSET_ECD) {
//        balance.weight[0].ecd_fdb = -(moto_balance[0].ecd - WEIGHT0_OFFSET_ECD - 8191);
//    } else {
//        balance.weight[0].ecd_fdb = -(moto_balance[0].ecd - WEIGHT0_OFFSET_ECD);
//    }
//    if (moto_balance[1].ecd < WEIGHT1_OFFSET_ECD) {
//        balance.weight[1].ecd_fdb = (moto_balance[1].ecd - WEIGHT1_OFFSET_ECD + 8191);
//    } else {
//        balance.weight[1].ecd_fdb = (moto_balance[1].ecd - WEIGHT1_OFFSET_ECD);
//    }
//    
//    /* 平衡块6020的速度反馈滤波处理 */
//    balance.weight[0].spd_fdb = Kalman1FilterCalc(&kal_chassis_weight[0],-moto_balance[0].speed_rpm);
//    balance.weight[1].spd_fdb = Kalman1FilterCalc(&kal_chassis_weight[1], moto_balance[1].speed_rpm);

//    balance.weight[0].ecd_ref = ramp_input(balance.weight[0].ecd_ref, weight0_ecd_ref, 60.0f);
//    balance.weight[1].ecd_ref = ramp_input(balance.weight[0].ecd_ref, weight1_ecd_ref, 60.0f);
//    
////		balance.weight[0].ecd_ref = weight0_ecd_ref;
////    balance.weight[1].ecd_ref =  weight1_ecd_ref;
//    /* 平衡块位置速度串级PID控制 */
//    balance.weight[0].ecd_ref = data_limit(balance.weight[0].ecd_ref, WEIGHT_MAX_ECD, WEIGHT_MIN_ECD);
//    balance.weight[1].ecd_ref = data_limit(balance.weight[1].ecd_ref, WEIGHT_MAX_ECD, WEIGHT_MIN_ECD);


//    balance.weight[0].spd_ref =  1.0f * pid_calc(&pid_spd[0], balance.weight[0].ecd_fdb, balance.weight[0].ecd_ref);
//    balance.weight[1].spd_ref =  1.0f * pid_calc(&pid_spd[1], balance.weight[1].ecd_fdb, balance.weight[1].ecd_ref);

//    balance.weight[0].current = -1.0f *pid_calc(&pid_balance_weight_spd[0], balance.weight[0].spd_fdb, balance.weight[0].spd_ref);
//    balance.weight[1].current = 1.0f *pid_calc(&pid_balance_weight_spd[1], balance.weight[1].spd_fdb, balance.weight[1].spd_ref);
//		if(weight0_ecd_ref <=0.0f)
//		can2_send_message(GIMBAL_CAN_TX_ID, 0, 0, 0, 0);
//		else		
//		can2_send_message(GIMBAL_CAN_TX_ID, 0, 0, balance.weight[0].current, balance.weight[1].current);
		
}

void Fric_control(void){

				fric.Fric_Pid_Set[0].fric_spd_fdb = moto_fric[0].speed_rpm;
				fric.Fric_Pid_Set[1].fric_spd_fdb = moto_fric[1].speed_rpm;
				pid_calc(&pid_spd[0],fric.Fric_Pid_Set[0].fric_spd_fdb,fric.Fric_Pid_Set[0].fric_spd_ref);
				pid_calc(&pid_spd[1],fric.Fric_Pid_Set[1].fric_spd_fdb,fric.Fric_Pid_Set[1].fric_spd_ref);
				fric_cur[0] = pid_spd[0].pos_out;
				fric_cur[1] = pid_spd[1].pos_out;
//				HAL_IWDG_Refresh(&hiwdg);
}

void mode_check(void){
	
		static uint8_t Fric_times;
		static uint8_t stopped=0;
	 if((fric.Fric_Pid_Set[0].fric_spd_ref == 0 && fric.Fric_Pid_Set[0].fric_spd_ref == 0) && stopped !=1)
	 {	
		if(last_fric_mdoe != fric_mode)
		fric_mode = Fric_slow;
		else
		{
			Fric_times++;
			if(Fric_times>50)
			{	
				stopped = 1;
				fric_mode = Fric_stop;
				Fric_times=0;
			}
		}

	 }
	 else
	 {
		fric_mode = Fric_run;
		 stopped = 0;
	 }
	 
	 last_fric_mdoe = fric_flag;
	 
}					
 void can_out(void)
 {
		
	 if(fric.Fric_Pid_Set[0].fric_spd_ref == 0 && fric.Fric_Pid_Set[0].fric_spd_ref == 0)
	 {	
			if(fric_mode == Fric_slow)
			{can2_send_message(0x200,fric_cur[0],	fric_cur[1],0,0);}
			else
			{	
				can2_send_message(0x200,fric_cur[0],	fric_cur[1],0,0);

			}
	 }
	 else
	 { 
    	 can2_send_message(0x200,fric_cur[0],	fric_cur[1],0,0);
		 
	 }
	 

//		HAL_IWDG_Refresh(&hiwdg);
 
 }
