#include "status_task.h"
#include "string.h"
#include "cmsis_os.h"
#include "gpio.h"
#include "modeswitch_task.h"
//#include "iwdg.h"
float lost_flag;
/* ״̬�ϲ� */
#define WDG_BITS_TASK           ( WDG_BIT_TASK_MODES          | WDG_BIT_TASK_CHASSIS        \
                                | WDG_BIT_TASK_GIMBAL         | WDG_BIT_TASK_SHOOT          \
                                | WDG_BIT_TASK_VISION         ) // | WDG_BIT_TASK_JUDGE 
#define WDG_BITS_BSP_CHASSIS    ( WDG_BIT_BSP_CHASSIS_MOTO1   | WDG_BIT_BSP_CHASSIS_MOTO2   \
                                | WDG_BIT_BSP_CHASSIS_MOTO3   | WDG_BIT_BSP_CHASSIS_MOTO4   )
#define WDG_BITS_BSP_GIMBAL     ( WDG_BIT_BSP_GIMBAL_YAW      | WDG_BIT_BSP_GIMBAL_PIT      \
                                | WDG_BIT_BSP_TRIGGER         )
#define WDG_BITS_BSP_IMU        ( WDG_BIT_BSP_IMU_ANGLE       | WDG_BIT_BSP_IMU_SPEED       )

/* ״̬��ʾ Rͷ����ʱ��LED������ */
#define LED_STATUS_1 status.chassis
#define LED_STATUS_2 status.gimbal
#define LED_STATUS_3 status.vision
#define LED_STATUS_4 status.supercap

__packed union { /* ϵͳ״̬��־λ */
    uint32_t wdg_event_group;
    __packed struct { /* ��Ҫ��.h�ļ��еĺ�һһ��Ӧ */
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
void wdg_user_set_bit(uint32_t status_bit) { /* �������ִ�б�� */
    s.wdg_event_group |= status_bit;
}

static void bit_clear(void) { /* ������ */
    s.wdg_event_group &= 0;
}

static uint8_t bit_check(uint32_t status_bit) { /* �������ִ�м�� */
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

static void status_deinit(void) { /* ���� */
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

        /* ͨ��״̬�������� */
        status.remote   = bit_check(WDG_BIT_BSP_REMOTE);
        status.chassis  = bit_check(WDG_BITS_BSP_CHASSIS);
        status.gimbal   = bit_check(WDG_BITS_BSP_GIMBAL);
        status.imu      = bit_check(WDG_BITS_BSP_IMU);
        status.vision   = bit_check(WDG_BIT_BSP_VISION);
        status.supercap = bit_check(WDG_BIT_BSP_SUPERCAP);

        /* ����״̬��� */
        if (bit_check(WDG_BITS_TASK)) {
//            HAL_IWDG_Refresh(&hiwdg); /* ι�� */
        }
        bit_clear(); /* ״̬��գ��ȴ���һ�μ�� */
        if(bit_check(WDG_BIT_BSP_CHASSIS_MOTO3))
        {
            lost_flag++;
            //lock_flag=1;
            //ctrl_mode=PROTECT_MODE;
        }
        bit_clear(); /* ״̬��գ��ȴ���һ�μ�� */
        /* ״̬��ʾ */
        if (!status.remote) { /* ң����ʧ��,��ȫ�� ң��ͨ�����ڴ�Լ14ms */
            status_init();
        } else if (!status.imu) {/* ������ʧ��,�ĵ���˸ */
            if (time_cnt >= 5) {
                led_status = !led_status;
                time_cnt = 0;

                if (led_status) {
                    status_init(); /* LEDȫ��Ϩ�� */
                } else {
                    status_deinit(); /* ȫ������ */
                }
            }
        } else { /* ң�����������Ƕ�����ʱ */
            /* LED ״̬��ʾ */
            if (time_cnt == 4) {
                if (LED_STATUS_1) { /* ����״̬ LED4 */
                    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, (GPIO_PinState)led_status);
                } else {
                    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_SET);
                }
            } else if (time_cnt == 8) {
                if (LED_STATUS_2) { /* ��̨״̬ LED3 */
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
                led_status = !led_status; /* ��ת����״̬ */
                time_cnt = 0; /* ��ռ���λ ��ʼ��һ��ѭ������ */
            }
        }
        taskEXIT_CRITICAL();
        osDelay(30); /* ����LED time_cnt ��λ��������������ڳ������������� */
    }
}
