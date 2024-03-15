/******************************简介*******************************************
采用了16位的列表过滤模式
每个can独立使用一个接受邮箱，即：
CAN1使用FIFO0，CAN2使用FIFO1。
*****************************使用说明*****************************************
                   直接调用canx_init函数即可
参数说明：
CAN_HandleTypeDef * hcan：CAN结构体

uint32_t  * id:需要的ID号数组，要以0xFFF结尾，比如你要读取ID为0x201的电机，则定义如下
uint32_t ID[] = {0x201,0xFFF};        注意要以0xFFF结尾！！！！
如果没有在数组中定义ID，即使外设挂载在总线上也会被过滤器滤除，导致读不到数据

void (*pFunc)(uint32_t ,uint8_t*):回调函数指针，需要自己定义一个回调函数
该回调函数返回类型为void 第一个参数为uint32_t的ID，第二个则是uint8_t的反馈数据
比如 void CAN1_CALLBACK(uint32_t ID,uint8_* CAN_RxData)
{
       ………………;
}
具体内容自定义，可以直接把之前代码复制过来，但是只需要复制数据处理部分就可以
不需要复制有关can的任何内容

使用范例：
uint32_t ID[] = {0x201,0xFFF};
void CAN1_CALLBACK(uint32_t ID,uint8_* CAN_RxData)
{
    switch(ID)
    {
        …………;
    }
}
canx_init(&hcan1,ID,CAN1_CALLBACK);

同时，本模块还针对DJI电机封装了CAN信息发送函数：
void can1_send_message(int16_t TX_ID, int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4);
void can1_send_message(int16_t TX_ID, int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4);
*******************************************************************************/
#include "bsp_can.h"
#include "string.h"
#include "dji_motor.h"
#include "can_comm.h"

extern moto_measure_t moto_balance[2];
struct can_rx_buff
{
    CAN_RxHeaderTypeDef header;
    uint8_t             data[8];
} can_rx_data;

/* 本地变量，CAN中断回调函数指针 */
static void (*pCAN1_RxFifo0CpltCallback)(uint32_t,uint8_t*);
static void (*pCAN2_RxFifo1CpltCallback)(uint32_t,uint8_t*);

static void CAN_Filter_IDList_Config(CAN_HandleTypeDef * hcan, uint32_t  * ID);
static void CAN1_FilterEnd_Config(CAN_HandleTypeDef * hcan,CAN_FilterTypeDef  *Can_Fliter,uint8_t num);
static void CAN2_FilterEnd_Config(CAN_HandleTypeDef * hcan,CAN_FilterTypeDef  *Can_Fliter,uint8_t num);
static void CAN1_IDList_Config(CAN_HandleTypeDef * hcan,CAN_FilterTypeDef  *Can_Fliter,uint32_t  * ID);
static void CAN2_IDList_Config(CAN_HandleTypeDef * hcan,CAN_FilterTypeDef  *Can_Fliter,uint32_t  * ID);

/**
* @brief  Initialize CAN Bus
* @param  hcan: CANx created by CubeMX. id:an array of ListID. (*pFunc):USER_Callback_Func
* @return None.
*/
void canx_init(CAN_HandleTypeDef * hcan, uint32_t  * id, void (*pFunc)(uint32_t,uint8_t*))
{
    CAN_Filter_IDList_Config(hcan,id);
    HAL_CAN_Start(hcan);
    if (CAN1 == hcan->Instance)
    {
        pCAN1_RxFifo0CpltCallback = pFunc;
        HAL_CAN_ActivateNotification(hcan,CAN_IT_RX_FIFO0_MSG_PENDING);
        __HAL_CAN_ENABLE_IT(hcan,CAN_IT_RX_FIFO0_MSG_PENDING);
    }
    else if (CAN2 == hcan->Instance)
    {
        pCAN2_RxFifo1CpltCallback = pFunc;
        HAL_CAN_ActivateNotification(hcan,CAN_IT_RX_FIFO1_MSG_PENDING);
        __HAL_CAN_ENABLE_IT(hcan,CAN_IT_RX_FIFO1_MSG_PENDING);
    }
}

/**
* @brief  Initialize CAN Filter
* @param  hcan: CANx created by CubeMX. id:an array of ListID.
* @return None.
*/
static void CAN_Filter_IDList_Config(CAN_HandleTypeDef * hcan, uint32_t  * ID)
{
    /* can filter config */
    CAN_FilterTypeDef  can_filter;

    can_filter.FilterMode = CAN_FILTERMODE_IDLIST;
    can_filter.FilterScale = CAN_FILTERSCALE_16BIT;
    can_filter.SlaveStartFilterBank = 14;
    //Config IDList
    if (CAN1 == hcan->Instance)
        CAN1_IDList_Config(hcan,&can_filter,ID);
    else if (CAN2 == hcan->Instance)
        CAN2_IDList_Config(hcan,&can_filter,ID);
}

/**
* @brief  Initialize Register about IDlist for CAN1
* @param  hcan: CANx created by CubeMX. Can_Fliter: CAN filter configuration structure.id:an array of ListID.
* @return None.
*/
static void CAN1_IDList_Config(CAN_HandleTypeDef * hcan,CAN_FilterTypeDef  *Can_Fliter,uint32_t  * ID)
{
    uint8_t i = 0;
    while(ID[i]!=0xFFF)
    {
        switch (i%4)
        {
        case 0:
            Can_Fliter->FilterIdHigh = ID[i]<<5;
            break;
        case 1:
            Can_Fliter->FilterIdLow = ID[i]<<5;
            break;
        case 2:
            Can_Fliter->FilterMaskIdHigh = ID[i]<<5;
            break;
        case 3:
        {
            Can_Fliter->FilterMaskIdLow = ID[i]<<5;
            CAN1_FilterEnd_Config(hcan,Can_Fliter,i);
            break;
        }
        }
        if (ID[i+1]==0xFFF)
        {
            CAN1_FilterEnd_Config(hcan,Can_Fliter,i);
        }
        i++;
    }
}

/**
* @brief  Initialize Register about IDlist for CAN2
* @param  hcan: CANx created by CubeMX. Can_Fliter: CAN filter configuration structure.id:an array for ListID.
* @return None.
*/
static void CAN2_IDList_Config(CAN_HandleTypeDef * hcan,CAN_FilterTypeDef  *Can_Fliter,uint32_t  * ID)
{
    uint8_t i = 0;
    while(ID[i]!=0xFFF)
    {
        switch (i%4)
        {
        case 0:
            Can_Fliter->FilterIdHigh = ID[i]<<5;
            break;
        case 1:
            Can_Fliter->FilterIdLow = ID[i]<<5;
            break;
        case 2:
            Can_Fliter->FilterMaskIdHigh = ID[i]<<5;
            break;
        case 3:
        {
            Can_Fliter->FilterMaskIdLow = ID[i]<<5;
            CAN2_FilterEnd_Config(hcan,Can_Fliter,i);
            break;
        }
        }
        if (ID[i+1]==0xFFF)
        {
            CAN2_FilterEnd_Config(hcan,Can_Fliter,i);
        }
        i++;
    }
}

/**
* @brief  Finish initialize register about IDlist for CAN1
* @param  hcan: CANx created by CubeMX. Can_Fliter: CAN filter configuration structure. num: the index of ID array
* @return None.
*/
static void CAN1_FilterEnd_Config(CAN_HandleTypeDef * hcan,CAN_FilterTypeDef  *Can_Fliter,uint8_t num)
{
    Can_Fliter->FilterBank  = num/4;
    Can_Fliter->FilterFIFOAssignment = CAN_FILTER_FIFO0;
    Can_Fliter->FilterActivation = CAN_FILTER_ENABLE;
    HAL_CAN_ConfigFilter(hcan,Can_Fliter);
}

/**
* @brief  Finish initialize register about IDlist for CAN2
* @param  hcan: CANx created by CubeMX. Can_Fliter: CAN filter configuration structure. num: the index of ID array
* @return None.
*/
static void CAN2_FilterEnd_Config(CAN_HandleTypeDef * hcan,CAN_FilterTypeDef  *Can_Fliter,uint8_t num)
{
    Can_Fliter->FilterBank  = num/4+14;
    Can_Fliter->FilterFIFOAssignment = CAN_FILTER_FIFO1;
    Can_Fliter->FilterActivation = CAN_FILTER_ENABLE;
    HAL_CAN_ConfigFilter(hcan,Can_Fliter);
}

/*HAL库FIFO0中断*/
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    if (hcan->Instance ==CAN1)
    {
        if (HAL_CAN_GetRxMessage(hcan,CAN_FILTER_FIFO0,&can_rx_data.header,can_rx_data.data) == HAL_ERROR) {};
        pCAN1_RxFifo0CpltCallback(can_rx_data.header.StdId,can_rx_data.data);
    }
	}


extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

void can_filter_init(void)
{

    CAN_FilterTypeDef can_filter_st;
    can_filter_st.FilterActivation = ENABLE;
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter_st.FilterIdHigh = 0x0000;
    can_filter_st.FilterIdLow = 0x0000;
    can_filter_st.FilterMaskIdHigh = 0x0000;
    can_filter_st.FilterMaskIdLow = 0x0000;
    can_filter_st.FilterBank = 0;
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);


    can_filter_st.SlaveStartFilterBank = 14;
    can_filter_st.FilterBank = 14;
    HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);
    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);



}



/*HAL库FIFO1中断*/
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    if (hcan->Instance == CAN2)
    {
        if (HAL_CAN_GetRxMessage(hcan,CAN_FILTER_FIFO1,&can_rx_data.header,can_rx_data.data)==HAL_ERROR) {};
        pCAN2_RxFifo1CpltCallback(can_rx_data.header.StdId,can_rx_data.data);
    }
}

void can1_send_message(int16_t TX_ID, int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4)
{
    static uint32_t MailBox;

    CAN_TxHeaderTypeDef Tx1Message;
    uint8_t CAN1_Tx_data[8];
    static uint32_t txmailbox;
	  static uint16_t time =0;

    Tx1Message.StdId = TX_ID;
    Tx1Message.IDE 	 = CAN_ID_STD;
    Tx1Message.RTR   = CAN_RTR_DATA;
    Tx1Message.DLC   = 0x08;

    CAN1_Tx_data[0] = iq1 >> 8;
    CAN1_Tx_data[1] = iq1;
    CAN1_Tx_data[2] = iq2 >> 8 ;
    CAN1_Tx_data[3] = iq2;
    CAN1_Tx_data[4] = iq3 >> 8;
    CAN1_Tx_data[5] = iq3;
    CAN1_Tx_data[6] = iq4 >> 8;
    CAN1_Tx_data[7] = iq4;

			while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0)
		{
				time++;
			if(time > 2)
			{
			time=0;
				return;
			}
		
		}  
		
		
	if ((hcan1.Instance->TSR & CAN_TSR_TME0) != RESET)     //如果邮箱0空闲
	{
		txmailbox =CAN_TX_MAILBOX0;
	}

	/* Check Tx Mailbox 1 status */
	else if ((hcan1.Instance->TSR & CAN_TSR_TME1) != RESET)
	{
		txmailbox =CAN_TX_MAILBOX1;
	}

	/* Check Tx Mailbox 2 status */
	else if ((hcan1.Instance->TSR & CAN_TSR_TME2) != RESET)
	{
		txmailbox =CAN_TX_MAILBOX2;
	}
    HAL_CAN_AddTxMessage(&hcan1, &Tx1Message, CAN1_Tx_data, &txmailbox);
}

void can2_send_message(int16_t TX_ID, int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4)
{
  
    CAN_TxHeaderTypeDef Tx2Message;
    uint8_t CAN2_Tx_data[8];
		static uint32_t txmailbox;
	  static uint16_t time =0;
	
    Tx2Message.StdId = TX_ID;
    Tx2Message.IDE 	 = CAN_ID_STD;
    Tx2Message.RTR   = CAN_RTR_DATA;
    Tx2Message.DLC   = 0x08;

    CAN2_Tx_data[0] = iq1 >> 8;
    CAN2_Tx_data[1] = iq1;
    CAN2_Tx_data[2] = iq2 >> 8 ;
    CAN2_Tx_data[3] = iq2;
    CAN2_Tx_data[4] = iq3 >> 8;
    CAN2_Tx_data[5] = iq3;
    CAN2_Tx_data[6] = iq4 >> 8;
    CAN2_Tx_data[7] = iq4;

			while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan2) == 0)
		{
				time++;
			if(time > 2)
			{
			time=0;
				return;
			}
		
		}  
		
		
	if ((hcan2.Instance->TSR & CAN_TSR_TME0) != RESET)     //如果邮箱0空闲
	{
		txmailbox =CAN_TX_MAILBOX0;
	}

	/* Check Tx Mailbox 1 status */
	else if ((hcan2.Instance->TSR & CAN_TSR_TME1) != RESET)
	{
		txmailbox =CAN_TX_MAILBOX1;
	}

	/* Check Tx Mailbox 2 status */
	else if ((hcan2.Instance->TSR & CAN_TSR_TME2) != RESET)
	{
		txmailbox =CAN_TX_MAILBOX2;
	}
    HAL_CAN_AddTxMessage(&hcan2, &Tx2Message, CAN2_Tx_data, &txmailbox);
}
