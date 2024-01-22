/**********C库****************/
#include "stdbool.h"
/**********硬件外设库*********/
#include "tim.h"
#include "usart.h"
/**********任务库*************/
#include "modeswitch_task.h"
#include "status_task.h"
/**********数学库*************/

/**********数据处理库**********/
#include "DataScope_DP.h"
#include "remote_msg.h"
/**********类型定义库**********/
#include "protocol_camp.h"
/**********板级支持库**********/
#include "bsp_T_imu.h"
/**********外部变量声明********/
extern vision_ctrl_info_t vision_ctrl; // 自动步兵控制
extern Game_Status_t Game_Status;
extern chassis_odom_info_t chassis_odom;
/**********外部函数声明********/
extern void rm_queue_data(uint16_t cmd_id, void *buf, uint16_t len);
/**********静态函数声明********/

/**********宏定义声明**********/

/**********结构体定义**********/
status_t status;
/**********变量声明************/

/**********测试变量声明********/

/*********************************************************************************************************************************/

/**
 * @brief status_task
 * @param
 * @attention
 * @note
 */
void status_task(void const *argu)
{
  for (;;)
  {
    // rc.init_status = rc_FSM(status.rc_status);  //更新遥控器的初始状态

    /* 遥控器通信状态检查 */
    // 遥控通信周期大约14ms，此任务，每100ms检查并清除一次中断标志
    if (rc_normal_flag)
    {
      status.rc_status = 1; // 系统状态标志置1，供modeswitch_task中检查
      rc_normal_flag = 0;   // 清除遥控器串口中断标志

      /* LED 状态显示 */
    }
    else // 遥控器或陀螺仪失联
    {
      status.rc_status = 0; // 遥控状态标志清0
    }

    osDelay(100);
  }
}
