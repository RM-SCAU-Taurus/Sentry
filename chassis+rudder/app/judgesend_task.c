/**
  * @file judgesend_task.c
  * @version 1.0
  * @date Mar,26th 2021
	*
  * @brief UI
	*
  *	@author
  *
  */
#include "bsp_judge.h"
#include "string.h"
#include "usart.h"
#include "bsp_usart.h"
#include "cmsis_os.h"
#include "modeswitch_task.h"
#include "bsp_powerlimit.h"
#include "remote_msg.h"
#include "shoot_task.h"
#include "chassis_task.h"
#include "bsp_vision.h"
#include "control_def.h"
#include "crc8_crc16.h"

unsigned char CliendTxBuffer[150];
/*****************发送数据结构体定义**********************/
ext_SendClient_t									Judgesend_Data;
ext_SendClientDelete_t						Delete_data;
ext_client_custom_character_t			Judgesend_strings_Data;
robot_interactive_data_t					Comm_senddata;
/*********************************************************/
uint8_t  Robot_Self_ID;   //当前机器人的ID
uint16_t Judge_Client_ID; //发送者机器人对应的客户端ID
uint8_t chassis_reflash_flag=0;

graphic_data_struct_t enr_data;
uint8_t battery_energy[3] = "enc";
graphic_data_struct_t light_data;
uint8_t shoot_light[3] 	= "lig";
graphic_data_struct_t shoot_data[2];
uint8_t shoot_shape1[3] = "sh1";
uint8_t shoot_shape2[3] = "sh2";
graphic_data_struct_t background[7];
uint8_t battery_rectangle[3] = "rec";
uint8_t mode_frame[3] = "inf";
uint8_t separate1[3] 	= "sp1";
uint8_t separate2[3] 	= "sp2";
uint8_t separate3[3] 	= "sp3";
//graphic_data_struct_t background_point[5];
//uint8_t ui_point1[3]="po1";
//uint8_t ui_point2[3]="po2";
//uint8_t ui_point3[3]="po3";
//uint8_t ui_point4[3]="po4";
//uint8_t ui_point5[3]="po5";
graphic_data_struct_t	chassis_Data;
uint8_t chassis_mode[3] = "cha";

ext_client_custom_character_t	gimbal_Data;
uint8_t gimbal_mode[3] 	= "gim";

/**
	* @brief  发送自定义数据到电脑客户端
  * @param
  * @retval
  * @attention  数据打包,打包完成后通过串口发送到裁判系统,一个周期只允许发送一帧
  */
void judge_send_task(void const *argu)
{
//    uint8_t shoot_upper_flag, shoot_under_flag;
    uint8_t gimbal_mode_flag, spinning_flag;
//    uint8_t vision_distance_flag;

    /* 初始化数据打包 */
    ui_data_pack();
    chassis_reflash_flag=1;

    /* 比赛中复活初始化UI界面 */
    if(Game_State.game_progress == 4)
    {
        for(int i=0; i<6; i++)
        {
            data_pack_imagine(DRAW_IMAGINE2,(uint8_t*)&shoot_data,CLIENT_DRAW_2_GRAPHS_CMD_ID);
            osDelay(40);
            data_pack_imagine(DRAW_IMAGINE1,(uint8_t*)&chassis_Data,CLIENT_DRAW_1_GRAPHS_CMD_ID);
            osDelay(40);
            gimbal_Data.grapic_data_struct.operate_tpye=2;
            uint8_t	gimbal_ui[30]="NORMAL";
            gimbal_Data.grapic_data_struct.end_angle=6;
            data_pack_code(gimbal_Data,gimbal_ui);
            osDelay(40);
        }
    }

    for(;;)
    {
        /* 比赛准备阶段绘制图层底层 */
        if( Game_State.game_progress == 1 )
        {
            ui_init();
        }
        /* 比赛开始后才允许改变UI中的参数 */

        else if( Game_State.game_progress )
        {
            /* 发送超级电容容量UI */

            /* 发送发射器模式UI */
//            if(kb_status[KB_V] == KEY_RUN)
//            {
//                //<! 默认是上枪管，即单枪管模式
//                shoot_upper_flag = 1;
//                shoot_under_flag = 0;
//            }
//            else
//            {
//                shoot_upper_flag = 0;
//                shoot_under_flag = 0;
//            }
//            judge_send_shoot_mode(shoot_upper_flag, shoot_under_flag);

            /* 发送云台底盘模式UI */
            /*云台*/
            if( rc.mouse.r == 1 && vision.distance )
            {
                if( FLAG_VISION_sENERGY || FLAG_VISION_bENERGY )
                    gimbal_mode_flag = 2;  //能量模式
                else if( kb_status[KB_R] == KEY_RUN )
                    gimbal_mode_flag = 3;  //返小陀螺模式
                else
                    gimbal_mode_flag = 1;  //自瞄模式
            }
            else
                gimbal_mode_flag = 0;  //普通键鼠模式
            /*底盘*/
            if( chassis.mode == CHASSIS_MODE_KEYBOARD_ROTATE )
                spinning_flag = 1;
            else
                spinning_flag = 0;
            judge_send_code_display(gimbal_mode_flag, spinning_flag);

//            /* 发送视觉距离UI */
//            if(vision.distance == 0)
//                vision_distance_flag = 0;
//            else if( vision.distance <= 3000 )
//                vision_distance_flag = 1;
//            else if( vision.distance <= 5000 )
//                vision_distance_flag = 2;
//            else
//                vision_distance_flag = 3;
//            judge_send_light_display(vision_distance_flag);
        }
        osDelay(10);
    }
}


/**
  * @brief  判断自己红蓝方
  * @param  void
  * @retval RED   BLUE
  * @attention  数据打包,打包完成后通过串口发送到裁判系统
  */
int determine_red_blue(void)
{
    Robot_Self_ID = Game_Robot_Status.robot_id;//读取当前机器人ID

    if(Game_Robot_Status.robot_id > 100)
    {
        return BLUE;
    } else
    {
        return RED;
    }
}

/**
  * @brief  判断自身ID，选择客户端ID
  * @param  void
  * @retval RED   BLUE
  * @attention  数据打包,打包完成后通过串口发送到裁判系统
  */
void determine_ID(void)
{
    int Color;//当前机器人的阵营
    Color = determine_red_blue();
    if(Color == BLUE)
    {
        Judge_Client_ID = 0x0164 + (Robot_Self_ID-100);//计算客户端ID
    } else if(Color == RED)
    {
        Judge_Client_ID = 0x0100 + Robot_Self_ID;//计算客户端ID
    }
}

/**
	* @brief  自定义ui初始化
  * @param  void
  * @retval void
  * @attention  各种数据填充
  */
void ui_data_pack(void)
{
    determine_ID();																//判断发送者ID和其对应的客户端ID
    Judgesend_Data.txFrameHeader.SOF = 0xA5;			//数据帧头
    Judgesend_Data.txFrameHeader.seq = 0;					//数据包序号
    Judgesend_Data.FrameHeader.send_ID 	 = Robot_Self_ID;	//发送者ID

    //电池容量表示
    memcpy(&enr_data.graphic_name, battery_energy, 3); //命名
    enr_data.operate_tpye=1;
    enr_data.graphic_tpye=0;
    enr_data.layer=0;
    enr_data.color=3;
    enr_data.width=40;
    enr_data.start_x=1885;
    enr_data.start_y=785;
    enr_data.end_x=1885;
    enr_data.end_y=890;

    //击打指示灯
    memcpy(&light_data.graphic_name, shoot_light, 3);
    light_data.operate_tpye=1;
    light_data.graphic_tpye=2;
    light_data.layer=1;
    light_data.color=7;
    light_data.width=24;
    light_data.start_x=965;
    light_data.start_y=880;
    light_data.radius=12;

    //云台字符
    memcpy(&gimbal_Data.grapic_data_struct.graphic_name, gimbal_mode, 3);
    gimbal_Data.grapic_data_struct.operate_tpye=1;
    gimbal_Data.grapic_data_struct.graphic_tpye=7;
    gimbal_Data.grapic_data_struct.layer=2;
    gimbal_Data.grapic_data_struct.color=0;
    gimbal_Data.grapic_data_struct.start_angle=15;//字体大小
    gimbal_Data.grapic_data_struct.width=2;
    gimbal_Data.grapic_data_struct.start_x=55;
    gimbal_Data.grapic_data_struct.start_y=875;

    //底盘模式
    memcpy(&chassis_Data.graphic_name, chassis_mode, 3);
    chassis_Data.operate_tpye=3;
    chassis_Data.graphic_tpye=0;
    chassis_Data.layer=3;
    chassis_Data.color=1;
    chassis_Data.width=35;
    chassis_Data.start_x=70;
    chassis_Data.start_y=720;
    chassis_Data.end_x=120;
    chassis_Data.end_y=720;

    //发射模式图形
    memcpy(&shoot_data[0].graphic_name, shoot_shape1, 3);
    shoot_data[0].operate_tpye=1;
    shoot_data[0].graphic_tpye=0;
    shoot_data[0].layer=4;
    shoot_data[0].color=0;
    shoot_data[0].width=40;
    shoot_data[0].start_x=15;
    shoot_data[0].start_y=822;
    shoot_data[0].end_x=175;
    shoot_data[0].end_y=822;

    memcpy(&shoot_data[1].graphic_name, shoot_shape2, 3);
    shoot_data[1].operate_tpye=3;
    shoot_data[1].graphic_tpye=0;
    shoot_data[1].layer=4;
    shoot_data[1].color=0;
    shoot_data[1].width=40;
    shoot_data[1].start_x=15;
    shoot_data[1].start_y=767;
    shoot_data[1].end_x=175;
    shoot_data[1].end_y=767;

    /*********背景**********/
    //电池矩形外框
    memcpy(&background[0].graphic_name, battery_rectangle, 3); //命名
    background[0].operate_tpye=1;
    background[0].graphic_tpye=1;
    background[0].layer=9;
    background[0].color=8;
    background[0].width=5;
    background[0].start_x=1860;
    background[0].start_y=920;
    background[0].end_x=1910;
    background[0].end_y=780;

    //左上信息大框
    memcpy(&background[1].graphic_name, mode_frame, 3);
    background[1].operate_tpye=1;
    background[1].graphic_tpye=1;
    background[1].layer=9;
    background[1].color=8;
    background[1].width=5;
    background[1].start_x=10;
    background[1].start_y=700;
    background[1].end_x=180;
    background[1].end_y=890;

    //信息大框分割线1
    memcpy(&background[2].graphic_name, separate1, 3);
    background[2].operate_tpye=1;
    background[2].graphic_tpye=0;
    background[2].layer=9;
    background[2].color=8;
    background[2].width=5;
    background[2].start_x=10;
    background[2].start_y=850;
    background[2].end_x=180;
    background[2].end_y=850;

    //信息大框分割线2
    memcpy(&background[3].graphic_name, separate2, 3);
    background[3].operate_tpye=1;
    background[3].graphic_tpye=0;
    background[3].layer=9;
    background[3].color=8;
    background[3].width=5;
    background[3].start_x=10;
    background[3].start_y=740;
    background[3].end_x=180;
    background[3].end_y=740;

    //信息大框分割线3
    memcpy(&background[4].graphic_name, separate3, 3);
    background[4].operate_tpye=1;
    background[4].graphic_tpye=0;
    background[4].layer=9;
    background[4].color=8;
    background[4].width=2;
    background[4].start_x=30;
    background[4].start_y=795;
    background[4].end_x=160;
    background[4].end_y=795;

//    //准星
//    if(Robot_Number==4)
//    {
//        background_point[1].start_y=415;
//        background_point[1].end_y=415;
//        background_point[3].start_x=950;
//        background_point[3].start_y=430;
//        background_point[3].end_x=970;
//        background_point[3].end_y=430;
//    }
//    else if(Robot_Number==3)
//    {
//        background_point[1].start_y=425;
//        background_point[1].end_y=425;
//        background_point[3].start_x=945;
//        background_point[3].start_y=442;
//        background_point[3].end_x=975;
//        background_point[3].end_y=442;
//    }

//    memcpy(&background_point[0].graphic_name, ui_point1, 3);
//    background_point[0].operate_tpye=1;
//    background_point[0].graphic_tpye=0;
//    background_point[0].layer=9;
//    background_point[0].color=8;
//    background_point[0].width=5;
//    background_point[0].start_x=960;
//    background_point[0].start_y=395;
//    background_point[0].end_x=960;
//    background_point[0].end_y=470;

//    memcpy(&background_point[1].graphic_name, ui_point2, 3);
//    background_point[1].operate_tpye=1;
//    background_point[1].graphic_tpye=0;
//    background_point[1].layer=9;
//    background_point[1].color=8;
//    background_point[1].width=5;
//    background_point[1].start_x=920;
//    background_point[1].end_x=1000;

//    memcpy(&background_point[2].graphic_name, ui_point3, 3);
//    background_point[2].operate_tpye=1;
//    background_point[2].graphic_tpye=0;
//    background_point[2].layer=9;
//    background_point[2].color=8;
//    background_point[2].width=5;
//    background_point[2].start_x=985;
//    background_point[2].start_y=455;
//    background_point[2].end_x=935;
//    background_point[2].end_y=455;

//    memcpy(&background_point[3].graphic_name, ui_point4, 3);
//    background_point[3].operate_tpye=1;
//    background_point[3].graphic_tpye=0;
//    background_point[3].layer=9;
//    background_point[3].color=8;
//    background_point[3].width=5;
}

/**
	* @brief  自定义ui初始化
  * @param  void
  * @retval void
	* @attention  初始图形发送
  */
void ui_init(void)
{
    static int ui_init_flag = 1;
    if(ui_init_flag)
    {
        for(int i = 0; i < 6; i++)
        {
            data_pack_delete(type_delete_all, 0);
            osDelay(100);
        }
        ui_init_flag=0;
    }

    data_pack_imagine(DRAW_IMAGINE1,(uint8_t*)&chassis_Data,CLIENT_DRAW_1_GRAPHS_CMD_ID);
    osDelay(100);
//    data_pack_imagine(DRAW_IMAGINE5,(uint8_t*)background_point,CLIENT_DRAW_5_GRAPHS_CMD_ID);//准星
//    osDelay(100);
    data_pack_imagine(DRAW_IMAGINE7,(uint8_t*)background,CLIENT_DRAW_7_GRAPHS_CMD_ID);//背景
    osDelay(100);
    enr_data.operate_tpye = 1;
    data_pack_imagine(DRAW_IMAGINE1,(uint8_t*)&enr_data,CLIENT_DRAW_1_GRAPHS_CMD_ID);//电池电量
    osDelay(100);
    uint8_t	gimbal_ui[30]="NORMAL";
    gimbal_Data.grapic_data_struct.end_angle=6;
    data_pack_code(gimbal_Data,gimbal_ui);
    osDelay(100);
    data_pack_imagine(DRAW_IMAGINE2,(uint8_t*)&shoot_data,CLIENT_DRAW_2_GRAPHS_CMD_ID);
    osDelay(100);
}

/**
	* @brief  自定义数据打包函数
  * @param  Judgedatalength data_length
	*					uint8_t* data_locate  数据段指针
	*					client_data_cmd_e date_type
  * @retval void
  * @attention  数据打包,打包完成后通过串口发送到裁判系统
  */
void data_pack_imagine(Judgedatalength data_length,uint8_t* data_locate,client_data_cmd_e date_type)
{
    Judgesend_Data.txFrameHeader.data_length = data_length-9;
    memcpy(CliendTxBuffer, &Judgesend_Data.txFrameHeader, sizeof(frame_header));//写入帧头数据
    append_CRC8_check_sum((uint8_t *)CliendTxBuffer, sizeof(frame_header));//写入帧头CRC8校验码
    Judgesend_Data.CmdID = ID_interact;
    Judgesend_Data.FrameHeader.receiver_ID = Judge_Client_ID;//客户端的ID，只能为发送者机器人对应的客户端
    Judgesend_Data.FrameHeader.data_cmd_id=date_type;

    //打包写入数据段
    memcpy(CliendTxBuffer + 5, (uint8_t*)&Judgesend_Data.CmdID, 2);
    memcpy(CliendTxBuffer + 7, (uint8_t*)&Judgesend_Data.FrameHeader, 6);
    memcpy(CliendTxBuffer + 13, (uint8_t*)data_locate, data_length-15);
    append_CRC16_check_sum(CliendTxBuffer,data_length);//写入数据段CRC16校验码
    HAL_UART_Transmit_DMA(&JUDGE_HUART,CliendTxBuffer,data_length);
}

/**
  * @brief  客户端删除图形
  * @param  类型选择：0: 空操作；
	*										1: 删除图层；
	*										2: 删除所有；
  * @param  图层选择：0~9
  * @attention
  */
void data_pack_delete(type_graphic_delete_e type,uint8_t layer)
{
    Judgesend_Data.txFrameHeader.data_length = sizeof(Judgesend_Data.FrameHeader)+sizeof(Judgesend_Data.clientdata.delete_data);
    memcpy(CliendTxBuffer, &Judgesend_Data.txFrameHeader, sizeof(frame_header));//写入帧头数据
    append_CRC8_check_sum((uint8_t *)CliendTxBuffer, sizeof(frame_header));//写入帧头CRC8校验码
    Judgesend_Data.CmdID = ID_interact;
    Judgesend_Data.FrameHeader.receiver_ID = Judge_Client_ID;//客户端的ID，只能为发送者机器人对应的客户端
    Judgesend_Data.FrameHeader.data_cmd_id = CLIENT_DELETE_GRAPH_CMD_ID;
    Judgesend_Data.clientdata.delete_data.operate_tpye = type;
    Judgesend_Data.clientdata.delete_data.layer = layer;

    memcpy(CliendTxBuffer + 5, (uint8_t*)&Judgesend_Data.CmdID, 8);
    memcpy(CliendTxBuffer + 13, (uint8_t*)&Judgesend_Data.clientdata.delete_data, 2);
    append_CRC16_check_sum(CliendTxBuffer,JUDGE_DELETE);//写入数据段CRC16校验码
    HAL_UART_Transmit_DMA(&JUDGE_HUART,CliendTxBuffer,JUDGE_DELETE);
}

/**
  * @brief  客户端绘制字符
  * @param  字体大小
  * @param  字符长度
  * @param  线条宽度
  * @param  起点x坐标
  * @param  起点y坐标
  * @param  字符
  * @attention
  */
void data_pack_code(ext_client_custom_character_t code_date,uint8_t* CODE)
{
    Judgesend_Data.txFrameHeader.data_length = sizeof(Judgesend_Data.FrameHeader)+sizeof(Judgesend_Data.clientdata.code_data);
    memcpy(CliendTxBuffer, &Judgesend_Data.txFrameHeader, sizeof(frame_header));//写入帧头数据
    append_CRC8_check_sum((uint8_t *)CliendTxBuffer, sizeof(frame_header));//写入帧头CRC8校验码
    Judgesend_Data.CmdID = ID_interact;
    Judgesend_Data.FrameHeader.receiver_ID = Judge_Client_ID;//客户端的ID，只能为发送者机器人对应的客户端
    Judgesend_Data.FrameHeader.data_cmd_id = CLIENT_WRITE_STRINGS_CMD_ID;

    //打包写入数据段
    memcpy(CliendTxBuffer + 5, (uint8_t*)&Judgesend_Data.CmdID, 8);
    memcpy(CliendTxBuffer + 13, (uint8_t*)&code_date.grapic_data_struct, 15);
    memcpy(CliendTxBuffer + 28, (uint8_t*)CODE, 30);
    append_CRC16_check_sum(CliendTxBuffer,PRINTF_CODE);//写入数据段CRC16校验码
    HAL_UART_Transmit_DMA(&JUDGE_HUART,CliendTxBuffer,PRINTF_CODE);
}

/**
  * @brief  机器人交互数据
  * @param  目标机器人id
  * @param  交互数据id，可在0x0200~0x02ff选取，具体ID含义由参赛队自定义
  * @param  交互数据?
  * @attention
  */
void data_pack_comm_data(uint16_t Robot_Target_ID,client_data_cmd_e CLIENT_INTERACTIVE_CMD_ID,uint8_t* comm_date)
{
    Judgesend_Data.txFrameHeader.data_length = sizeof(Judgesend_Data.FrameHeader)+sizeof(Judgesend_Data.clientdata.comm_senddata);
    memcpy(CliendTxBuffer, &Judgesend_Data.txFrameHeader, sizeof(frame_header));//写入帧头数据
    append_CRC8_check_sum((uint8_t *)CliendTxBuffer, sizeof(frame_header));//写入帧头CRC8校验码
    Judgesend_Data.CmdID = ID_interact;
    Judgesend_Data.FrameHeader.receiver_ID = Robot_Target_ID;//目标机器人的ID
    Judgesend_Data.FrameHeader.data_cmd_id = CLIENT_INTERACTIVE_CMD_ID;

    //打包写入数据段
    memcpy(CliendTxBuffer + 5, (uint8_t*)&Judgesend_Data.CmdID, 8);
    memcpy(CliendTxBuffer + 13, (uint8_t*)comm_date,LEN_interact);
    append_CRC16_check_sum(CliendTxBuffer,ROBOT_COMM);//写入数据段CRC16校验码
    HAL_UART_Transmit_DMA(&JUDGE_HUART,CliendTxBuffer,ROBOT_COMM);
}

/**
  * @brief  小地图互动数据
  * @param	机器人xyz坐标
  * @param	云台手所按下按键
  * @param  目标机器人id
  * @attention
  */
void data_pack_map_position(float target_position_x,float target_position_y,float target_position_z,uint8_t commd_keyboard,uint16_t target_robot_ID)
{
    Judgesend_Data.txFrameHeader.data_length = sizeof(Judgesend_Data.clientdata.Client_map);
    memcpy(CliendTxBuffer, &Judgesend_Data.txFrameHeader, sizeof(frame_header));//写入帧头数据
    append_CRC8_check_sum((uint8_t *)CliendTxBuffer, sizeof(frame_header));//写入帧头CRC8校验码
    Judgesend_Data.CmdID = ID_client_map;
    Judgesend_Data.clientdata.Client_map.target_position_x	= target_position_x;
    Judgesend_Data.clientdata.Client_map.target_position_y	= target_position_y;
    Judgesend_Data.clientdata.Client_map.target_position_z	= target_position_z;
    Judgesend_Data.clientdata.Client_map.commd_keyboard = commd_keyboard;
    Judgesend_Data.clientdata.Client_map.target_robot_ID = target_robot_ID;

    //打包写入数据段
    memcpy(CliendTxBuffer + 5, (uint8_t*)&Judgesend_Data.CmdID, 2);
    memcpy(CliendTxBuffer + 7, (uint8_t*)&Judgesend_Data.clientdata.Client_map,15);
    append_CRC16_check_sum(CliendTxBuffer,ROBOT_COMM);//写入数据段CRC16校验码
    HAL_UART_Transmit_DMA(&JUDGE_HUART,CliendTxBuffer,29);
}

/**
	* @brief  发送超级电容电量UI
  * @param
  * @retval
  * @attention
  */
void judge_send_supercap(uint8_t supercap_percent)
{
    static uint8_t percent_status;
    static uint8_t percent_status_last;
    percent_status=(supercap_percent/10);

    if(percent_status_last!=percent_status)
    {
        for(int i=0; i<6; i++)
        {
            switch(percent_status)
            {
            case 10:
            case 9:
            {
                enr_data.operate_tpye=2;
                enr_data.end_y=915;
                enr_data.color=2;
                data_pack_imagine(DRAW_IMAGINE1,(uint8_t*)&enr_data,CLIENT_DRAW_1_GRAPHS_CMD_ID);
            }
            break;

            case 8:
            {
                enr_data.operate_tpye=2;
                enr_data.end_y=902;
                enr_data.color=2;
                data_pack_imagine(DRAW_IMAGINE1,(uint8_t*)&enr_data,CLIENT_DRAW_1_GRAPHS_CMD_ID);
            }
            break;

            case 7:
            {
                enr_data.operate_tpye=2;
                enr_data.end_y=889;
                enr_data.color=2;
                data_pack_imagine(DRAW_IMAGINE1,(uint8_t*)&enr_data,CLIENT_DRAW_1_GRAPHS_CMD_ID);
            }
            break;

            case 6:
            {
                enr_data.operate_tpye=2;
                enr_data.end_y=876;
                enr_data.color=3;
                data_pack_imagine(DRAW_IMAGINE1,(uint8_t*)&enr_data,CLIENT_DRAW_1_GRAPHS_CMD_ID);
            }
            break;

            case 5:
            {
                enr_data.operate_tpye=2;
                enr_data.end_y=863;
                enr_data.color=3;
                data_pack_imagine(DRAW_IMAGINE1,(uint8_t*)&enr_data,CLIENT_DRAW_1_GRAPHS_CMD_ID);
            }
            break;

            case 4:
            {
                enr_data.operate_tpye=2;
                enr_data.end_y=850;
                enr_data.color=3;
                data_pack_imagine(DRAW_IMAGINE1,(uint8_t*)&enr_data,CLIENT_DRAW_1_GRAPHS_CMD_ID);
            }
            break;

            case 3:
            {
                enr_data.operate_tpye=2;
                enr_data.end_y=837;
                enr_data.color=3;
                data_pack_imagine(DRAW_IMAGINE1,(uint8_t*)&enr_data,CLIENT_DRAW_1_GRAPHS_CMD_ID);
            }
            break;

            case 2:
            {
                enr_data.operate_tpye=2;
                enr_data.end_y=824;
                enr_data.color=5;
                data_pack_imagine(DRAW_IMAGINE1,(uint8_t*)&enr_data,CLIENT_DRAW_1_GRAPHS_CMD_ID);
            }
            break;

            case 1:
            {
                enr_data.operate_tpye=2;
                enr_data.end_y=811;
                enr_data.color=5;
                data_pack_imagine(DRAW_IMAGINE1,(uint8_t*)&enr_data,CLIENT_DRAW_1_GRAPHS_CMD_ID);
            }
            break;

            case 0:
            {
                enr_data.operate_tpye=2;
                enr_data.end_y=790;
                enr_data.color=5;
                data_pack_imagine(DRAW_IMAGINE1,(uint8_t*)&enr_data,CLIENT_DRAW_1_GRAPHS_CMD_ID);
            }
            break;

            default:
                break;
            }
            osDelay(50);
        }
    }
    percent_status_last=percent_status;
}

/**
	* @brief  发送发射器模式UI
  * @param
  * @retval
  * @attention
  */
//void judge_send_shoot_mode(uint8_t up,uint8_t down)
//{
//    static uint8_t up_last;
//    static uint8_t down_last;

//    if(up!=up_last)
//    {
//        for(int i=0; i<6; i++)
//        {
//            if(up)
//            {
//                shoot_data[0].operate_tpye=1;
//                data_pack_imagine(DRAW_IMAGINE2,(uint8_t*)&shoot_data,CLIENT_DRAW_2_GRAPHS_CMD_ID);
//            } else if(!up)
//            {
//                shoot_data[0].operate_tpye=3;
//                data_pack_imagine(DRAW_IMAGINE2,(uint8_t*)&shoot_data,CLIENT_DRAW_2_GRAPHS_CMD_ID);
//            }
//            osDelay(50);
//        }
//    }
//    if(down!=down_last)
//    {
//        for(int i=0; i<6; i++)
//        {
//            if(down)
//            {
//                shoot_data[1].operate_tpye=1;
//                data_pack_imagine(DRAW_IMAGINE2,(uint8_t*)&shoot_data,CLIENT_DRAW_2_GRAPHS_CMD_ID);
//            } else if(!down)
//            {
//                shoot_data[1].operate_tpye=3;
//                data_pack_imagine(DRAW_IMAGINE2,(uint8_t*)&shoot_data,CLIENT_DRAW_2_GRAPHS_CMD_ID);
//            }
//            osDelay(50);
//        }
//    }
//    up_last=up;
//    down_last=down;
//}

/**
	* @brief  发送云台/底盘模式UI
  * @param
  * @retval
  * @attention
  */
void judge_send_code_display(uint8_t gimbal,uint8_t chassis)
{
    static uint8_t gimbal_last;
    static uint8_t chassis_last;

    if( gimbal != gimbal_last )
    {
        for(int i=0; i<6; i++)
        {
            switch( gimbal )
            {
            case 0 :
            {
                gimbal_Data.grapic_data_struct.operate_tpye=2;
                uint8_t	gimbal_ui[30]="NORMAL";
                gimbal_Data.grapic_data_struct.end_angle=6;
                data_pack_code(gimbal_Data,gimbal_ui);
                break;
            }
            case 1 :
            {
                gimbal_Data.grapic_data_struct.operate_tpye=2;
                uint8_t	gimbal_ui[30]="AUTO";
                gimbal_Data.grapic_data_struct.end_angle=6;
                data_pack_code(gimbal_Data,gimbal_ui);
                break;
            }
            case 2 :
            {
                gimbal_Data.grapic_data_struct.operate_tpye=2;
                uint8_t	gimbal_ui[30]="ENERGY";
                gimbal_Data.grapic_data_struct.end_angle=6;
                data_pack_code(gimbal_Data,gimbal_ui);
                break;
            }
            case 3 :
            {
                gimbal_Data.grapic_data_struct.operate_tpye=2;
                uint8_t	gimbal_ui[30]="ANTISP";
                gimbal_Data.grapic_data_struct.end_angle=6;
                data_pack_code(gimbal_Data,gimbal_ui);
                break;
            }
            default:
            {
                break;
            }
            }
            osDelay(50);
        }
    }

    if(chassis!=chassis_last||chassis_reflash_flag)
    {
        for(int i=0; i<6; i++)
        {
            if(!chassis)
            {
                chassis_Data.operate_tpye=3;
                data_pack_imagine(DRAW_IMAGINE1,(uint8_t*)&chassis_Data,CLIENT_DRAW_1_GRAPHS_CMD_ID);
            } else if(chassis)
            {
                chassis_Data.operate_tpye=1;
                data_pack_imagine(DRAW_IMAGINE1,(uint8_t*)&chassis_Data,CLIENT_DRAW_1_GRAPHS_CMD_ID);
            }
            osDelay(50);
        }
        chassis_reflash_flag =0;
    }
    gimbal_last=gimbal;
    chassis_last=chassis;
}

/**
	* @brief  发送视觉距离UI
  * @param
  * @retval
  * @attention
  */
void judge_send_light_display(uint8_t light)
{
    static uint8_t light_last;

    if(light_last!=light)
    {
        for(int i=0; i<6; i++)
        {
            switch(light)
            {
            case 0:
            {
                light_data.operate_tpye=2;
                light_data.color=7;
                data_pack_imagine(DRAW_IMAGINE1,(uint8_t*)&light_data,CLIENT_DRAW_1_GRAPHS_CMD_ID);
            }
            break;

            case 1:
            {
                light_data.operate_tpye=2;
                light_data.color=2;
                data_pack_imagine(DRAW_IMAGINE1,(uint8_t*)&light_data,CLIENT_DRAW_1_GRAPHS_CMD_ID);
            }
            break;

            case 2:
            {
                light_data.operate_tpye=2;
                light_data.color=3;
                data_pack_imagine(DRAW_IMAGINE1,(uint8_t*)&light_data,CLIENT_DRAW_1_GRAPHS_CMD_ID);
            }
            break;

            case 3:
            {
                light_data.operate_tpye=2;
                light_data.color=5;
                data_pack_imagine(DRAW_IMAGINE1,(uint8_t*)&light_data,CLIENT_DRAW_1_GRAPHS_CMD_ID);
            }
            break;

            default:
                break;
            }
            osDelay(50);
        }
    }
    light_last=light;
}
