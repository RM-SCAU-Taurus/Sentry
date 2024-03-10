#include "comm_task.h"
#include "bsp_can.h"
#include "cmsis_os.h"
#include "pid.h"
#include "bsp_powerlimit.h"
#include "string.h"
#include "usart.h"
#include "chassis_task.h"
#include "modeswitch_task.h"
motor_current_t   motor_cur;
chassis_msg_t     chassis_rec_msg;
float send_time;
float test_current;
/**
  * @brief can_msg_send_task
  * @param
  * @attention
  * @note
  */
void can_msg_send_task(void const *argu)
{
    osEvent event;
    for(;;)
    {
        event = osSignalWait(CHASSIS_MOTOR_MSG_SEND, osWaitForever);
        if (event.status == osEventSignal)
        {
            if(ctrl_mode == PROTECT_MODE)
            {
                for(int i=0; i<4; i++)
                {
                    motor_cur.chassis_cur_3508[i]=0;
                    motor_cur.chassis_cur_6020[i]=0;
                }
                can1_send_supercap(SUPERCAP_CAN_TX_ID);
                can1_send_message(CHASSIS_CAN_TX_ID,0,0,0,0);
                can2_send_message(CHASSIS_CAN_TX_ID,0,0,0,0);
                can2_send_message(GIMBAL_CAN_TX_ID,0,0,0,0);
                //can1_send_message(0x2ff,0,0,0,0);
            }
            else
            {
                if (event.value.signals & CHASSIS_MOTOR_MSG_SEND)
                {
                    send_time++;
                    if(send_time>=3)
                    {
                        can1_send_supercap(SUPERCAP_CAN_TX_ID);
                        send_time=0;
                    }

                    can1_send_message(CHASSIS_CAN_TX_ID,0,0,motor_cur.chassis_cur_3508[2],0);
                    can2_send_message(CHASSIS_CAN_TX_ID,motor_cur.chassis_cur_3508[0],motor_cur.chassis_cur_3508[1],0,motor_cur.chassis_cur_3508[3]);
                    can2_send_message(GIMBAL_CAN_TX_ID,motor_cur.chassis_cur_6020[0],motor_cur.chassis_cur_6020[1],motor_cur.chassis_cur_6020[2],motor_cur.chassis_cur_6020[3]);
                }
            }
        }
    }
}

void can1_send_supercap(int16_t TX_ID)
{
    uint8_t supercap_msg_uint8_t[8]= {0};
    uint8_t FreeTxNum = 0;

    Tx1Message.StdId = TX_ID;
    Tx1Message.IDE 	 = CAN_ID_STD;
    Tx1Message.RTR   = CAN_RTR_DATA;
    Tx1Message.DLC   = 0x08;

    supercap_msg_uint8_t[0] = supercap.mode;		//超级电容模式
    memcpy(supercap_msg_uint8_t+1,&supercap.charge_current_set,sizeof(supercap.charge_current_set));
    //查询发送邮箱是否为空
    FreeTxNum = HAL_CAN_GetTxMailboxesFreeLevel(&hcan1);
    while(FreeTxNum == 0)
    {
        FreeTxNum = HAL_CAN_GetTxMailboxesFreeLevel(&hcan1);
    }

    HAL_CAN_AddTxMessage(&hcan1, &Tx1Message,supercap_msg_uint8_t,(uint32_t*)CAN_TX_MAILBOX1);
}

void get_chassis_msg(CAN_HandleTypeDef* hcan, uint8_t * CAN_Rx_data)
{
    uint16_t chassis_v[3];

    memcpy(&chassis_rec_msg.sign_msg,CAN_Rx_data,1);
    memcpy(&chassis_rec_msg.power_max,CAN_Rx_data+1,1);
    memcpy(&chassis_v,CAN_Rx_data+2,6);
    chassis_rec_msg.spd_vx=chassis_v[0];
    chassis_rec_msg.spd_vy=chassis_v[1];
    chassis_rec_msg.spd_vw=chassis_v[2];

    if(chassis_rec_msg.sign_msg.mode==1||chassis_rec_msg.sign_msg.mode==2)
    {
        ctrl_mode=STANDARD_MODE;
        if(chassis_rec_msg.sign_msg.mode==1)
            chassis.mode=0x01;
        if(chassis_rec_msg.sign_msg.mode==2)
            chassis.mode=0x02;
    }
    else
    {
        chassis.mode=0x00;
        ctrl_mode=PROTECT_MODE;
    }

    powercontrol.max_power=chassis_rec_msg.power_max;

    if(chassis_rec_msg.sign_msg.vx_sign)
        chassis.spd_input.vx=-chassis_rec_msg.spd_vx;
    else
        chassis.spd_input.vx=chassis_rec_msg.spd_vx;

    if(chassis_rec_msg.sign_msg.vy_sign)
        chassis.spd_input.vy=-chassis_rec_msg.spd_vy;
    else
        chassis.spd_input.vy=chassis_rec_msg.spd_vy;

    if(chassis_rec_msg.sign_msg.vw_sign)
        chassis.spd_input.vw=-chassis_rec_msg.spd_vw;
    else
        chassis.spd_input.vw=chassis_rec_msg.spd_vw;

    if(chassis_rec_msg.sign_msg.C)
        chassis.kb_C=1;
    else
        chassis.kb_C=0;
}

void judge_msg_get(CAN_HandleTypeDef* hcan, uint8_t * CAN_Rx_data)
{
    powercontrol.power_buffer = CAN_Rx_data[0];
    memcpy(&powercontrol.chassis_power,CAN_Rx_data+1,sizeof(float));
}
