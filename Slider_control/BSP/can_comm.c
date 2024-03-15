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
	  CAN_Fric_Ctrl_ID   = 	0x200,
		CAN_3508_R_ID = 0x201,
		CAN_3508_L_ID = 0x202,
} can_msg_id_e;
moto_measure_t moto_balance[3];

//static void CHASSIS_SLIPPED_DECT(void);
static void User_can1_callback(uint32_t ID, uint8_t* CAN_RxData);
static void User_can2_callback(uint32_t ID, uint8_t* CAN_RxData);

void can_device_init(void)
{
    uint32_t can_ID1[] = { 0x201, 0x202, 0xFFF};
		uint32_t can_ID2[] = { CAN_Fric_Ctrl_ID,0xFFF};
    canx_init(&hcan1, can_ID1, User_can1_callback);
    canx_init(&hcan2, can_ID2, User_can2_callback);
}

void Fric_msg_handle(uint8_t * CAN_Rx_data)
{
		fric.Fric_Pid_Set[0].fric_spd_ref = (int16_t)(CAN_Rx_data[0] << 8 | CAN_Rx_data[1]);
		fric.Fric_Pid_Set[1].fric_spd_ref = (int16_t)(CAN_Rx_data[2] << 8 | CAN_Rx_data[3]);
}

static void User_can1_callback(uint32_t ID, uint8_t* CAN_RxData)
{
    switch (ID)
    {

			case CAN_3508_R_ID:
			case CAN_3508_L_ID:
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

static void User_can2_callback(uint32_t ID, uint8_t* CAN_RxData)
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
			{can1_send_message(0x200,fric_cur[0],	fric_cur[1],0,0);}
			else
			{	
				can1_send_message(0x200,fric_cur[0],	fric_cur[1],0,0);

			}
	 }
	 else
	 { 
    	 can1_send_message(0x200,fric_cur[0],	fric_cur[1],0,0);
		 
	 }
	 

//		HAL_IWDG_Refresh(&hiwdg);
 
 }
