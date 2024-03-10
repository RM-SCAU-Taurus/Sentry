#include "status_task.h"
#include "string.h"
#include "cmsis_os.h"
#include "gpio.h"
#include "modeswitch_task.h"
//#include "iwdg.h"
float lost_flag;
/* 状态合并 */
#define WDG_BITS_TASK           ( WDG_BIT_TASK_MODES          | WDG_BIT_TASK_CHASSIS        \
                                | WDG_BIT_TASK_GIMBAL         | WDG_BIT_TASK_SHOOT          \
                                | WDG_BIT_TASK_VISION         ) // | WDG_BIT_TASK_JUDGE 
#define WDG_BITS_BSP_CHASSIS    ( WDG_BIT_BSP_CHASSIS_MOTO1   | WDG_BIT_BSP_CHASSIS_MOTO2   \
                                | WDG_BIT_BSP_CHASSIS_MOTO3   | WDG_BIT_BSP_CHASSIS_MOTO4   )
#define WDG_BITS_BSP_GIMBAL     ( WDG_BIT_BSP_GIMBAL_YAW      | WDG_BIT_BSP_GIMBAL_PIT      \
                                | WDG_BIT_BSP_TRIGGER         )
#define WDG_BITS_BSP_IMU        ( WDG_BIT_BSP_IMU_ANGLE       | WDG_BIT_BSP_IMU_SPEED       )

/* 状态显示 R头朝右时：LED从左到右 */
#define LED_STATUS_1 status.chassis
#define LED_STATUS_2 status.gimbal
#define LED_STATUS_3 status.vision
#define LED_STATUS_4 status.supercap

__packed union { /* 系统状态标志位 */
    uint32_t wdg_event_group;
    __packed struct { /* 需要与.h文件中的宏一一对应 */
        uint32_t mode_task      :1; //1
        uint32_t chassis_task   :1; //2
        uint32_t gimbal_task    :1; //3
        uint32_t shoot_task     :1; //4
        uint32_t vision_task    :1; //5
        uint32_t judge_task     :1; //6
        uint32_t debug_task     :1; //7
        uint32_t task_rsvd      :1; //8

        uint32_t chassis_moto1  :1; //9
        uint32_t chassis_moto2  :1; //10
        uint32_t chassis_moto3  :1; //11
        uint32_t chassis_moto4  :1; //12

        uint32_t gimbal_yaw     :1; //13
        uint32_t gimbal_pit     :1; //14
        uint32_t imu_angle      :1; //15
        uint32_t imu_speed      :1; //16

        uint32_t trigger_moto   :1; //17
        uint32_t remote         :1; //18
        uint32_t vision         :1; //19
        uint32_t supercap       :1; //20
        uint32_t judge          :1; //21
        uint32_t else_rsvd      :11;//22-32
    } bit;
} s;

status_t status;
extern uint8_t lock_flag;
void wdg_user_set_bit(uint32_t status_bit) { /* 待检测量执行标记 */
    s.wdg_event_group |= status_bit;
}

static void bit_clear(void) { /* 标记清空 */
    s.wdg_event_group &= 0;
}

static uint8_t bit_check(uint32_t status_bit) { /* 待检测量执行检查 */
    return !((s.wdg_event_group & status_bit) ^ status_bit);
}

static void status_init(void) {
    bit_clear();
    memset(&status, 0, sizeof(status_t));

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8,  GPIO_PIN_SET);  //LED_D
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9,  GPIO_PIN_SET);  //LED_C
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_SET);  //LED_B
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_SET);  //LED_A
}

static void status_deinit(void) { /* 亮灯 */
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8,  GPIO_PIN_RESET);  //LED_D
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9,  GPIO_PIN_RESET);  //LED_C
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_RESET);  //LED_B
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_RESET);  //LED_A
}

void status_task(void const *argu) {
    status_init();
    for (;;) {
        static uint16_t time_cnt;
        static uint8_t led_status;

        taskENTER_CRITICAL();

        ++time_cnt;

        /* 通信状态检测与输出 */
        status.remote   = bit_check(WDG_BIT_BSP_REMOTE);
        status.chassis  = bit_check(WDG_BITS_BSP_CHASSIS);
        status.gimbal   = bit_check(WDG_BITS_BSP_GIMBAL);
        status.imu      = bit_check(WDG_BITS_BSP_IMU);
        status.vision   = bit_check(WDG_BIT_BSP_VISION);
        status.supercap = bit_check(WDG_BIT_BSP_SUPERCAP);

        /* 任务状态检测 */
        if (bit_check(WDG_BITS_TASK)) {
//            HAL_IWDG_Refresh(&hiwdg); /* 喂狗 */
        }
        bit_clear(); /* 状态清空，等待下一次检测 */
        if(bit_check(WDG_BIT_BSP_CHASSIS_MOTO3))
        {
            lost_flag++;
            //lock_flag=1;
            //ctrl_mode=PROTECT_MODE;
        }
        bit_clear(); /* 状态清空，等待下一次检测 */
        /* 状态显示 */
        if (!status.remote) { /* 遥控器失联,灯全灭 遥控通信周期大约14ms */
            status_init();
        } else if (!status.imu) {/* 陀螺仪失联,四灯闪烁 */
            if (time_cnt >= 5) {
                led_status = !led_status;
                time_cnt = 0;

                if (led_status) {
                    status_init(); /* LED全部熄灭 */
                } else {
                    status_deinit(); /* 全部亮起 */
                }
            }
        } else { /* 遥控器和陀螺仪都正常时 */
            /* LED 状态显示 */
            if (time_cnt == 4) {
                if (LED_STATUS_1) { /* 底盘状态 LED4 */
                    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, (GPIO_PinState)led_status);
                } else {
                    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_SET);
                }
            } else if (time_cnt == 8) {
                if (LED_STATUS_2) { /* 云台状态 LED3 */
                    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, (GPIO_PinState)led_status);
                } else {
                    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_SET);
                }
            } else if (time_cnt == 12) {
                if(LED_STATUS_3) {
                    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, (GPIO_PinState)led_status);
                } else {
                    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, GPIO_PIN_SET);
                }
            } else if (time_cnt == 16) {
                if (LED_STATUS_4) { /* status.vision_status */
                    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, (GPIO_PinState)led_status);
                } else {
                    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, GPIO_PIN_SET);
                }
                led_status = !led_status; /* 翻转亮灭状态 */
                time_cnt = 0; /* 清空计数位 开始下一个循环周期 */
            }
        }
        taskEXIT_CRITICAL();
        osDelay(30); /* 决定LED time_cnt 单位，比任务最长的周期长两到三倍即可 */
    }
}
