#ifndef _STATUS_TASK_H_
#define _STATUS_TASK_H_

#include "stdint.h"

/* 任务状态位 */
#define WDG_BIT_TASK_MODES          (uint32_t)(1 << 0)
#define WDG_BIT_TASK_CHASSIS        (uint32_t)(1 << 1)
#define WDG_BIT_TASK_GIMBAL         (uint32_t)(1 << 2)
#define WDG_BIT_TASK_SHOOT          (uint32_t)(1 << 3)

#define WDG_BIT_TASK_VISION         (uint32_t)(1 << 4)
#define WDG_BIT_TASK_JUDGE          (uint32_t)(1 << 5)
#define WDG_BIT_TASK_DEBUG          (uint32_t)(1 << 6)
#define WDG_BIT_TASK_RSVD2          (uint32_t)(1 << 7)

/* 硬件通信状态位 */
#define WDG_BIT_BSP_CHASSIS_MOTO1   (uint32_t)(1 <<  8)
#define WDG_BIT_BSP_CHASSIS_MOTO2   (uint32_t)(1 <<  9)
#define WDG_BIT_BSP_CHASSIS_MOTO3   (uint32_t)(1 << 10)
#define WDG_BIT_BSP_CHASSIS_MOTO4   (uint32_t)(1 << 11)

#define WDG_BIT_BSP_GIMBAL_YAW      (uint32_t)(1 << 12)
#define WDG_BIT_BSP_GIMBAL_PIT      (uint32_t)(1 << 13)
#define WDG_BIT_BSP_IMU_ANGLE       (uint32_t)(1 << 14)
#define WDG_BIT_BSP_IMU_SPEED       (uint32_t)(1 << 15)

#define WDG_BIT_BSP_TRIGGER         (uint32_t)(1 << 16)
#define WDG_BIT_BSP_REMOTE          (uint32_t)(1 << 17)
#define WDG_BIT_BSP_VISION          (uint32_t)(1 << 18)
#define WDG_BIT_BSP_SUPERCAP        (uint32_t)(1 << 19)

#define WDG_BIT_BSP_JUDGE           (uint32_t)(1 << 20)
#define WDG_BIT_BSP_RSVD2           (uint32_t)(1 << 21)
#define WDG_BIT_BSP_RSVD3           (uint32_t)(1 << 22)
#define WDG_BIT_BSP_RSVD4           (uint32_t)(1 << 23)

#define WDG_BIT_BSP_RSVD5           (uint32_t)(1 << 24)
#define WDG_BIT_BSP_RSVD6           (uint32_t)(1 << 25)
#define WDG_BIT_BSP_RSVD7           (uint32_t)(1 << 26)
#define WDG_BIT_BSP_RSVD8           (uint32_t)(1 << 27)

#define WDG_BIT_BSP_RSVD9           (uint32_t)(1 << 28)
#define WDG_BIT_BSP_RSVD10          (uint32_t)(1 << 29)
#define WDG_BIT_BSP_RSVD11          (uint32_t)(1 << 30)
#define WDG_BIT_BSP_RSVD12          (uint32_t)(1 << 31)

typedef __packed struct {
    uint16_t remote     :1;
    uint16_t chassis    :1;
    uint16_t gimbal     :1;
    uint16_t imu        :1;
    uint16_t supercap   :1;
    uint16_t vision     :1;
    uint16_t judge      :1;
    uint16_t rsvd       :9;
} status_t;

extern status_t status;

extern void wdg_user_set_bit(uint32_t status_bit);
extern void status_task(void const *argu);

#endif
