#include "test_task.h"
#include "cmsis_os.h"
#include "func_generator.h"
#include "comm_task.h"
#include "bsp_can.h"
#include "bsp_vision.h"
#include "cmsis_os.h"
#include "modeswitch_task.h"
#include "bsp_can.h"
#include "bsp_T_imu.h"
#include "control_def.h"

#include "bsp_usart.h"
#include "remote_msg.h"
#include "bsp_judge.h"
#include "shoot_task.h"
#include "protocol_camp.h"
#define TEST_PERIOD 10

//FGT_sin_t test_sin_func = 
//{
//    .Td = TEST_PERIOD,
//    .time = 0,
//    .max = 0 + 0,
//    .min = 0 - 0,
//    .dc = 0,
//    .T = 800,
//    .A = 250,
//    .phi = 0,
//    .out = 0
//};
void test_task(void const *argu)
{
    uint32_t wake_up_time = osKernelSysTick();
    for(;;)
    {
////        FGT_sin_cal(&test_sin_func);
//        if(tent2<=200000)
//					{
//						chassis.msg_send(CHASSIS_TO_Gimbal_CAN_TX_ID,chassis.spd_fdb.vx,chassis.spd_fdb.vy,chassis.spd_fdb.vw,0x00,0);//CAN1·¢ËÍ
////							tent++;
//						tent2++;
//					}
        osDelayUntil(&wake_up_time, TEST_PERIOD);
    }
}
