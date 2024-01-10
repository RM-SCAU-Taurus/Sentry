#ifndef __VISION_PREDICT_H__
#define __VISION_PREDICT_H__

#include "stm32f4xx_hal.h"
#include "pid.h"
/* 滞环死区控制 */
typedef struct {
    float max;
    float min;
    float now;
    float last;
    int8_t out;  //发生跳变的方向
} delay_loop_t;
typedef enum {
    vMODE_AUTO        = 0,    /* 自瞄 */
    vMODE_bENERGY     = 1,    /* 大能量机关 */
    vMODE_sENERGY     = 2,    /* 小能量机关 */
    vMODE_ANTISPIN    = 4,    /* 中远距离反小陀螺模式 */
} vision_mode_e;
/* 视觉状态 */
typedef enum {
    vUNAIMING   = 0,    /* 没有识别到目标 */
    vAIMING     = 1,    /* 识别到目标 */
    vFIRST_LOST = 2     /* 首次丢失目标 */
} vsn_output_status_e;

/* 视觉数据包单元 */
typedef struct {
    float now;
    float kal;
} vsn_data_pkg2_t;

/* 视觉数据包单元 */
typedef struct {
    float now;
    float last;
    float kal;
} vsn_data_pkg3_t;

/* YAW轴视觉算法中间变量 */
typedef struct {
    vsn_data_pkg3_t vsn_agl_err;
    vsn_data_pkg2_t vsn_angular_speed;
    vsn_data_pkg3_t imu_angular_speed;
    vsn_data_pkg2_t angular_speed;
    vsn_data_pkg2_t tangent_speed;
    vsn_data_pkg2_t radial_speed;
    vsn_data_pkg2_t real_speed;
    vsn_data_pkg2_t real_speed_angle;
    vsn_data_pkg2_t predict_distance;
    vsn_data_pkg2_t opposite_line;
    vsn_data_pkg3_t predict_angle;
} vsn_perdict_data_t;
typedef struct {
    vsn_perdict_data_t    yaw;
    vsn_perdict_data_t    pit;
    vsn_data_pkg3_t   pos;      /* 装甲板姿态角 */
    vsn_data_pkg3_t   dis;
    vsn_data_pkg3_t   tof;      /* 飞行时间 */

    float       period;         /* 视觉周期 */
    float       test_period;    /* 实测视觉周期 */
    uint8_t     valid_cnt;      /* 连续收到有效数据计数位 */
} vsn_data_t;
/* 视觉模块接口 */
typedef struct {
    vision_mode_e mode;  /* 视觉模式 */
    int rotation_direction; //旋转方向
    volatile vsn_output_status_e status;    /* 视觉输出状态 */
    volatile uint8_t new_frame_flag;        /* 新一帧数据标志 */
    volatile uint8_t shoot_enable;          /* 发弹允许标志 */
    
    float gimbal_pit_ecd; /* 时间轴同步时云台pit角度 */
    float pit_angle_error;
    float pit_predict_angle;
    
    float gimbal_yaw_angle; /* 时间轴同步时云台yaw角度 */
    float yaw_angle_error;
    float yaw_predict_angle;
} vsn_output_t;
/* 视觉驱动状态枚举类型 */
typedef enum {
    AIMING = 0,
    UNAIMING = 1,
    TAIL_ERROR = 2,    
    REPEAT_ERROR = 3
} vsn_rx_status_e;

/* 视觉驱动结构体类型 */
typedef struct {
    uint32_t repeat_cnt;    /* 收到重复帧计数 */
    vsn_rx_status_e status; /* 视觉接收状态 */
    union {
        uint8_t buff[23];/* 视觉缓存区 */
        __packed struct {           /* 视觉数据处理共用体 */    
            float yaw;              /* 视觉相对yaw角度 */
            float pit;              /* 视觉相对pit角度 */
            float dis;              /* 视觉相对距离 */
            float tof;              /* 弹丸飞行时间 */
            float pos;              /* 装甲板姿态角 */
            uint8_t shoot_flag;     /*英雄击打位*/ 
            uint8_t cnt     :6;     /* 视觉侧发送自加位 */
            uint8_t ist_flag:1;     /* 视觉侧开始补帧 */
            uint8_t aim_flag:1;     /* 视觉识别到目标 */
            uint8_t eof;            /* 视觉发送帧帧尾 */
        } data;                     /* 视觉解码 */
    } rx[2];
} vsn_drive_t;
extern vsn_output_t  vision; /* 视觉模块对外接口 */
extern vsn_data_t    vd;     /* 视觉原始数据与计算中间变量(调试) */
static void vsn_deinit(void);
static void vsn_auto_aiming_calc(void);
static void vsn_predict_angle_calc(void);
static void vsn_shoot_enable(void) ;
static float vsn_gradual_deduction(float array[6]) ;
void vsn_init(void);
int rotation_direction_judge(void);
void vsn_gimbal_ref_calc(void);
void vsn_calc(void);
#endif
