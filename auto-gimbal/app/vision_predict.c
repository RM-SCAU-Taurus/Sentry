#include "gimbal_task.h"
#include "chassis_task.h"
#include "status_task.h"
#include "comm_task.h"
#include "modeswitch_task.h"
#include "remote_msg.h"
#include "pid.h"
#include "bsp_can.h"
#include "control_def.h"
#include "bsp_TriggerMotor.h"
#include "remote_msg.h"
#include "math_calcu.h"
#include "bsp_T_imu.h"
#include "KalmanFilter.h"
#include "usart.h"
#include "func_generator.h"
#include "usb_task.h"
#include "protocol_camp.h"
#include "DataScope_DP.h"
#include "usart.h"
#include "bsp_can.h"
#include "bsp_JY901.h"
#include "bsp_T_imu.h"
#include "ubf.h"
#include "us_tim.h"
#include "vision_predict.h"
#include "shoot_task.h"
/* 圆周率 */
#ifndef PI
#define PI 3.14159265358979323846f
#endif

/* 绝对值 */
#define ABSv(x) ( (x>0)? (x): (-(x)) )

/* 符号 */
#ifndef SIGN
#define SIGN(x) ( (x)>0? (1): ( (x)<0? (-1): 0 ) )
#endif

/* 输出限幅 */
#ifndef OUTPUT_LIMIT
#define OUTPUT_LIMIT(output,max,min) \
    ( (output) <= (max) && (output)>=(min)? output: \
    ( (output) > (max)? (output = max):(output = min)) )
#endif
extern vision_ctrl_info_t  vision_ctrl;//自动步兵控制
vsn_drive_t vsn_drive;
vsn_output_t  vision;
void vsn_calc(void) {
    /* 数据量纲统一、方向统一 */
    vd.pit.vsn_agl_err.now   = vision_ctrl.pit;            /* 视觉PIT相对角度（°） */
//    vd.yaw.vsn_agl_err.now   = vision_ctrl.yaw;            /* 视觉YAW相对角度（°） */
    vd.yaw.vsn_agl_err.now   =  gimbal.yaw_imu_offset;            /* 视觉YAW相对角度（°） 做了修改，为了打导航头回正*/
    vd.dis.now               = vision_ctrl.dis * 1.0e-3f;  /* 距离（m） */
    vd.tof.now               = vision_ctrl.tof;    /* 去除视觉侧加的40ms的通信延时(s) */
//    vd.pos.now               = imu_data.yaw
//                             + vision_ctrl.yaw;            /* 子弹击打角 */
	    vd.pos.now               = imu_data.yaw
                             + gimbal.yaw_imu_offset;            /* 子弹击打角 做了修改，为了打导航头回正*/
	
    
	    if (vision_ctrl.aiming_flag) {
          vsn_drive.status = AIMING;
      } else {
           vsn_drive.status = UNAIMING;
        }
    /* 获取视觉运算周期 */
    vd.period = 7.5f * 1.0e-3f;  /* 固定视觉周期（s） */
    /* 根据视觉模式修改参数 */
   // vsn_mode_switch_init();
    
    /* 视觉预测流程 */
    if (vd.valid_cnt && vsn_drive.status != AIMING) {  /* 首次丢帧或异常重力下坠无解 */
        vd.valid_cnt = 0;
        vsn_deinit();                 /* 视觉历史数据清空 */
        vision.status = vFIRST_LOST;  /* 输出为首次丢帧状态 */
    } else if (vsn_drive.status == AIMING) {  /* 识别到目标 */
        ++vd.valid_cnt;  /* 有效数据计数 */
        vsn_auto_aiming_calc();  /*纯自瞄计算输出*/
        vsn_predict_angle_calc();  /*预测角度计算*/
        vsn_shoot_enable();  /*开枪时机射频控制*/
        vision.new_frame_flag = 1;  /* 标注为新有效数据*/
        vision.status = vAIMING;  /* 输出为识别到目标状态 */
//        if (ABS(vd.pit.vsn_agl_err.now - 45.0f) < 1e-5f) {  /* 重力下坠无解 */
//            vision.status = vUNAIMING;  /* 此时也当作未识别 */
//        }
    }
		else {  /* 未识别到目标 */
        vision.status = vUNAIMING;  /* 输出为丢失目标模式 */
			vsn_auto_aiming_calc();  /*纯自瞄计算输出*/
			vision.shoot_enable=0;
    }
		    /* 数据方差测试 */
//    float* pvsn_data = NULL;
//    switch (vsn_std_switch) {
//        /* 视觉原始数据 */
//        case 0: pvsn_data = &vd.pit.vsn_agl_err.now; break;
//        case 1: pvsn_data = &vd.yaw.vsn_agl_err.now; break;
//        case 2: pvsn_data = &vd.dis.now; break;
//        case 3: pvsn_data = &vd.tof.now; break;
//        case 4: pvsn_data = &vd.pos.now; break;
//        /* 中间数据 */
//        case 5: pvsn_data = &vd.yaw.angular_speed.now; break;
//        case 6: pvsn_data = &vd.yaw.radial_speed.kal; break;
//        default: break;
//    }
//    ubf_push(ubf_var, pvsn_data);
//    float vsn_data_array[100] = {0}, var;
//    uint32_t real_num = ubf_pop_into_array_new2old(ubf_var, vsn_data_array, 0, 100);
//    arm_var_f32(vsn_data_array, real_num, &var);  /* 计算数据方差 */
//    std = sqrtf(var);

    /* 直接交给视觉控制 */
//    vision.new_frame_flag   = 1;                                         /* 标注为新有效数据 */
//    vision.pit_angle_error  = vd.pit.now;                                /* PIT角度偏差 */
//    vision.yaw_angle_error  = vd.yaw.vsn_agl_err.now;                    /* YAW轴角度偏差 */
//    vision.yaw_predict_angle= 0;                                         /* YAW轴角度偏差预测 */
//    vision.gimbal_pit_ecd = ((gim_msg_t*)ubf_pop(gim_msg_ubf, delay_imu_wz))->pit;
//    vision.gimbal_yaw_angle = ((gim_msg_t*)ubf_pop(gim_msg_ubf, delay_imu_wz))->yaw;
}



/* 自瞄 */
/*视觉滤波*/
/* 距离数据滤波与相位匹配 (63 - 30)(ms) */
float   kal_R_dis_tof       = 4.00f;    /* 目标距离卡尔曼 R */

/* 视觉滤波器收敛时间 */
uint8_t stable_time_flt     = 7;        /* 视觉滤波器收敛时间（ms） */

/* 预测输出滤波 */
float   kal_R_predict_angle = 1.00f;    /* 预测角卡尔曼 R */

/* 经验参数细调 */
float   w_start_deadzone    = 0.00f;    /* 绝对角速度死区 */
float predict_agl_limit     = 15.0f;    /* 预测角度限幅 */
/* ================================== TEST PARAM ================================== */
float test_vision_predict_kp1 = 1.0f;//0.8f
float test_vision_predict_kp2 = 0.2f;
float test_vision_predict_error_kp = 1.7f;
float test_vision_angle_error_pit_kp = 1.3f;
float test_vision_angle_error_yaw_kp = 1.5f;
static ubf_t ubf_vd_pit;
static ubf_t ubf_vd_yaw;
static ubf_t ubf_vd_dis;
static ubf_t ubf_vd_tof;
/* ----------------------------------------------------------------- */
/*                        视觉卡尔曼滤波器                           */
/* ----------------------------------------------------------------- */
static Kalman1_param_t kal_pit_agl_err; /* 视觉PIT相对角度 */
static Kalman1_param_t kal_yaw_agl_err; /* 视觉YAW相对角度 */
static Kalman1_param_t kal_yaw_imu_spd; /* 陀螺仪加速度 */
static Kalman1_param_t kal_yaw_vsn_spd; /* 装甲板绝对角速度 */
static Kalman1_param_t kal_pit_imu_spd; /* 陀螺仪加速度 */
static Kalman1_param_t kal_pit_vsn_spd; /* 装甲板绝对角速度 */
static Kalman1_param_t kal_tof;         /* 视觉侧算得子弹飞行时间 */
static Kalman1_param_t kal_dis;         /* 视觉相对距离 */
static Kalman1_param_t kal_yaw_predict_agl; /* YAW预测角度 */
static Kalman1_param_t kal_pit_predict_agl; /* PIT预测角度 */

/* ----------------------------------------------------------------- */
/*                           视觉超参                                */
/* ----------------------------------------------------------------- */
/* 角度相位匹配（视觉处理以及通信延时、幅值匹配在视觉侧完成） */
uint32_t delay_auto_pit_ecd   = 35;     /* 自瞄角度相位匹配时间 */ 
uint32_t delay_auto_yaw_imu   = 30;     /* 自瞄角度相位匹配时间 */ //24
uint32_t delay_energy_pit_ecd = 80;     /* 能量机关角度相位匹配时间 */
uint32_t delay_energy_yaw_imu = 80;     /* 能量机关角度相位匹配时间 */

/* 角速度幅相匹配 */
float   kal_R_yaw_agl       = 2.00f;    /* 视觉角度卡尔曼 R */
float   kal_R_yaw_aim_spd   = 2.00f;    /* 视觉YAW相对角速度卡尔曼 R */
float   kal_R_yaw_imu_spd   = 3.20f;    /* IMU YAW角速度卡尔曼 R */
uint8_t delay_imu_wz        = 30;       /* 角速度相位匹配时间 */

float   kal_R_pit_agl       = 3.00f;    /* 视觉角度卡尔曼 R */
float   kal_R_pit_aim_spd   = 10.00f;   /* 视觉PIT相对角速度卡尔曼 R */
float   kal_R_pit_imu_spd   = 10.00f;   /* IMU PIT角速度卡尔曼 R */
uint8_t delay_imu_wy        = 41;       /* 角速度相位匹配时间 */
 vsn_data_t   vd;        /* 视觉数据结构体 */
 vsn_output_t vision_predict;    /* 视觉计算结果输出结构体 */
 /* 电控侧预测三档系统 */
float kp1 = 1.6f;
float kp2 = 0.0f;
float kp3 = 0.7f;//0 0 1.8
float shoot_enable_test = 0.5f;

delay_loop_t dlp = {
    .max = 1.8f,
    .min = 1.3f
};
 
 /* ----------------------------------------------------------------- */
/*                           局部全局变量                            */
/* ----------------------------------------------------------------- */
/* 预测量系数 */
float   predict_kp_yaw;     /* 1.8f  0.7f */
float   predict_kp_pit;
/* 自瞄角度匹配时间 */
uint8_t match_time_pit=35;   
uint8_t match_time_yaw=30;
/* ================================== TEST PARAM ================================== */
void vsn_init(void) {
 vision_mode_e last_vision_mode = vision.mode;
    memset(&vision,     0, sizeof(vsn_output_t));
    memset(&vsn_drive,  0, sizeof(vsn_drive_t));
    memset(&vd,         0, sizeof(vsn_data_t));
    vision.mode = last_vision_mode;
    
    /* 一阶卡尔曼滤波器初始化 */
    Kalman1FilterCreate(&kal_pit_agl_err,   1, kal_R_pit_agl);
    Kalman1FilterCreate(&kal_yaw_agl_err,   1, kal_R_pit_agl);
    
    Kalman1FilterCreate(&kal_yaw_imu_spd,   1, kal_R_yaw_imu_spd);
    Kalman1FilterCreate(&kal_yaw_vsn_spd,   1, kal_R_yaw_aim_spd);

    Kalman1FilterCreate(&kal_pit_imu_spd,   1, kal_R_pit_imu_spd);
    Kalman1FilterCreate(&kal_pit_vsn_spd,   1, kal_R_pit_aim_spd);
    
    Kalman1FilterCreate(&kal_tof,           1, kal_R_dis_tof);
    Kalman1FilterCreate(&kal_dis,           1, kal_R_dis_tof);
    
    Kalman1FilterCreate(&kal_yaw_predict_agl,   1, kal_R_predict_angle);
    Kalman1FilterCreate(&kal_pit_predict_agl,   1, kal_R_predict_angle);

    /* 数据缓存器 */
    gim_msg_ubf= ubf_create(100, sizeof(gim_msg_t));  /* 云台历史姿态信息 */
    ubf_vd_pit = ubf_create(10,  sizeof(float));    /* 视觉滤波后历史数据 */
    ubf_vd_yaw = ubf_create(10,  sizeof(float));    /* 视觉滤波后历史数据 */
    ubf_vd_dis = ubf_create(10,  sizeof(float));    /* 视觉滤波后历史数据 */
    ubf_vd_tof = ubf_create(10,  sizeof(float));    /* 视觉滤波后历史数据 */
}
static void vsn_deinit(void) {
		
	    /* 清空数据 */
    vision_mode_e last_vision_mode = vision.mode;
    //memset(&vision_ctrl, 0, sizeof(vision_ctrl_info_t));
    memset(&vd,          0, sizeof(vsn_data_t));
    memset(&vision,      0, sizeof(vsn_output_t));
    vision.mode = last_vision_mode;
    /* 清空视觉历史数据缓存 */
    ubf_clear(ubf_vd_pit);
    ubf_clear(ubf_vd_yaw);
		ubf_clear(ubf_vd_dis);
    ubf_clear(ubf_vd_tof);
	    /* 初始化卡尔曼滤波器 */
    Kalman1FilterDeinit(&kal_pit_agl_err);
    Kalman1FilterDeinit(&kal_yaw_agl_err);
    
    Kalman1FilterDeinit(&kal_yaw_imu_spd);
    Kalman1FilterDeinit(&kal_yaw_vsn_spd);
    
    Kalman1FilterDeinit(&kal_pit_imu_spd);
    Kalman1FilterDeinit(&kal_pit_vsn_spd);
    
    Kalman1FilterDeinit(&kal_tof);
    Kalman1FilterDeinit(&kal_dis);
    
    Kalman1FilterDeinit(&kal_yaw_predict_agl);
    Kalman1FilterDeinit(&kal_pit_predict_agl);
}
static void vsn_kal_reinit(void) {//动态调整卡尔曼参数
    Kalman1FilterReinit(&kal_pit_agl_err, 1, kal_R_pit_agl);
    Kalman1FilterReinit(&kal_yaw_agl_err, 1, kal_R_yaw_agl);
	
		Kalman1FilterReinit(&kal_yaw_imu_spd, 1, kal_R_yaw_imu_spd);
    Kalman1FilterReinit(&kal_yaw_vsn_spd, 1, kal_R_yaw_aim_spd);
    
    Kalman1FilterReinit(&kal_pit_imu_spd, 1, kal_R_pit_imu_spd);
    Kalman1FilterReinit(&kal_pit_vsn_spd, 1, kal_R_pit_aim_spd);

    Kalman1FilterReinit(&kal_tof,         1, kal_R_dis_tof);
    Kalman1FilterReinit(&kal_dis,         1, kal_R_dis_tof);
    
    Kalman1FilterReinit(&kal_yaw_predict_agl, 1, kal_R_predict_angle);
    Kalman1FilterReinit(&kal_pit_predict_agl, 1, kal_R_predict_angle);
}
static void vsn_auto_aiming_calc(void) {
    /* 输出不含预测的自瞄数据 */
    vision.pit_angle_error  = vd.pit.vsn_agl_err.now;  /* PIT视觉相对角度偏差 */
    vision.yaw_angle_error  = vd.yaw.vsn_agl_err.now;  /* YAW视觉相对角度偏差 */
    vision.gimbal_pit_ecd   = ((gim_msg_t*)ubf_pop(gim_msg_ubf, match_time_pit))->pit;  /* 相位匹配后的云台PIT角度 */
    vision.gimbal_yaw_angle = ((gim_msg_t*)ubf_pop(gim_msg_ubf, match_time_yaw))->yaw;  /* 相位匹配后的云台YAW角度 */
}
static void vsn_predict_angle_calc(void) {
 /* -------------------------- 视觉原始数据滤波与保存 -------------------------- */
    vsn_kal_reinit();  /*<! 调整卡尔曼的R，调试用 */

    vd.pit.vsn_agl_err.kal = Kalman1FilterCalc(&kal_pit_agl_err, vd.pit.vsn_agl_err.now);/* 视觉PIT轴角度（°）变化幅值 要与IMU的YAW匹配 */
    vd.yaw.vsn_agl_err.kal = Kalman1FilterCalc(&kal_yaw_agl_err, vd.yaw.vsn_agl_err.now);/* 视觉YAW轴角度（°）变化幅值 要与IMU的YAW匹配 */
    vd.dis.kal = Kalman1FilterCalc(&kal_dis, vd.dis.now);  /* 视觉直线距离（m） */
    vd.tof.kal = Kalman1FilterCalc(&kal_tof, vd.tof.now);  /* 弹丸飞行时间（s） */
    
    ubf_push(ubf_vd_pit, &vd.pit.vsn_agl_err.kal);
    ubf_push(ubf_vd_yaw, &vd.yaw.vsn_agl_err.kal);
    ubf_push(ubf_vd_dis, &vd.dis.kal);
    ubf_push(ubf_vd_tof, &vd.tof.kal);
    
    /* -------------------------- 逐差与开始预测 -------------------------- */
    if (vd.valid_cnt > stable_time_flt) {  /* 等待滤波器收敛*/ 
        /* 视觉相对角速度（°/s） */
        float vsn_yaw2diff[6] = {0};
        ubf_pop_into_array_new2old(ubf_vd_yaw, vsn_yaw2diff, 0, 6);
        vd.yaw.vsn_angular_speed.now = vsn_gradual_deduction(vsn_yaw2diff);
        vd.yaw.vsn_angular_speed.kal = Kalman1FilterCalc(&kal_yaw_vsn_spd, vd.yaw.vsn_angular_speed.now);
        
        float vsn_pit2diff[6] = {0};
        ubf_pop_into_array_new2old(ubf_vd_pit, vsn_pit2diff, 0, 6);
        vd.pit.vsn_angular_speed.now = vsn_gradual_deduction(vsn_pit2diff);
        vd.pit.vsn_angular_speed.kal = Kalman1FilterCalc(&kal_pit_vsn_spd, vd.pit.vsn_angular_speed.now);
        
        /* 陀螺仪相对角速度 LSB->(°/s) 与视觉相对角速度相位匹配*/ //0
        vd.yaw.imu_angular_speed.now = ((gim_msg_t*)ubf_pop(gim_msg_ubf, 0))->wz/16.3835f;  /* 更新数据但不用 */
        vd.yaw.imu_angular_speed.last= ((gim_msg_t*)ubf_pop(gim_msg_ubf, delay_imu_wz))->wz/16.3835f;  /* 角速度相位匹配 */
        vd.yaw.imu_angular_speed.kal = Kalman1FilterCalc(&kal_yaw_imu_spd, vd.yaw.imu_angular_speed.last);
        
        vd.pit.imu_angular_speed.now = ((gim_msg_t*)ubf_pop(gim_msg_ubf, 0))->wy/16.3835f; /* 更新数据但不用 */
        vd.pit.imu_angular_speed.last= ((gim_msg_t*)ubf_pop(gim_msg_ubf, delay_imu_wy))->wy/16.3835f;  /* 角速度相位匹配 */
        vd.pit.imu_angular_speed.kal = Kalman1FilterCalc(&kal_pit_imu_spd, vd.pit.imu_angular_speed.last);
        
        /* 绝对角速度（°/s）不用滤波  */
        vd.yaw.angular_speed.now = vd.yaw.vsn_angular_speed.kal + vd.yaw.imu_angular_speed.kal;    
        vd.pit.angular_speed.now = vd.pit.vsn_angular_speed.kal + vd.pit.imu_angular_speed.kal;
        
        /* 绝对角速度死区控制 */
        if (ABS(vd.yaw.angular_speed.now) < w_start_deadzone) {
            vd.yaw.angular_speed.now = 0;
        }
        
        /* 视觉预测计算 */
        vd.pit.predict_angle.now = vd.pit.angular_speed.now * vd.tof.kal * predict_kp_pit;
        vd.yaw.predict_angle.now = vd.yaw.angular_speed.now * vd.tof.kal * predict_kp_yaw;
                vd.yaw.predict_angle.now = vd.yaw.angular_speed.now * vd.tof.kal * predict_kp_yaw;
        vd.yaw.predict_angle.kal = Kalman1FilterCalc(&kal_yaw_predict_agl, vd.yaw.predict_angle.now);
        vd.pit.predict_angle.kal = Kalman1FilterCalc(&kal_pit_predict_agl, vd.pit.predict_angle.now);
        
        /* 输出预测角度数据 */
        OUTPUT_LIMIT(vd.yaw.predict_angle.kal, predict_agl_limit, -predict_agl_limit);
        OUTPUT_LIMIT(vd.pit.predict_angle.kal, predict_agl_limit, -predict_agl_limit);
        
        vision.yaw_predict_angle = vd.yaw.predict_angle.kal;
        vision.pit_predict_angle = vd.pit.predict_angle.kal;
        
        /* 目标运动方向判断 */
        vision.rotation_direction = rotation_direction_judge();  
    }
}

/** 
 * @brief 视觉云台设定值计算函数
 */
void vsn_gimbal_ref_calc(void) {
    switch (vision.status) {
        case vAIMING: {  /* 识别到目标 */
            if (vision.new_frame_flag) { /* 接收到新的数据 */
                /* 清除标志，防止重复处理同一帧 */
                vision.new_frame_flag = 0;
                /* 修改云台设定值 */
//                 gimbal.pid.pit_ecd_ref = vision.pit_angle_error+gimbal.pid.pit_ecd_fdb;
//                gimbal.pid.yaw_angle_ref = vision.gimbal_yaw_angle+vision.yaw_angle_error+vision.yaw_predict_angle;
                gimbal.pid.yaw_angle_ref= vision.yaw_angle_error;//视觉绝对角度
                gimbal.pid.pit_ecd_ref  = vision.pit_angle_error;
                
//                gimbal.pid.pit_ecd_ref = vision.gimbal_pit_ecd + vision.pit_angle_error * 8191.0f / 360;
//                gimbal.pid.yaw_angle_ref = vision.gimbal_yaw_angle + vision.yaw_angle_error;
                
							// gimbal.pid.pit_ecd_ref  = vision.gimbal_pit_ecd + ((vision.pit_angle_error + vision.pit_predict_angle) * 8191.0f / 360.0f);
            }
            break;
        }
        case vFIRST_LOST: {  /* 首次丢失目标 */
            vision.status = vUNAIMING;
          //  gimbal.pid.pit_ecd_ref   = gimbal.pid.pit_ecd_fdb;
             gimbal.pid.pit_ecd_ref = gimbal.pid.pit_ecd_fdb;
            gimbal.pid.yaw_angle_ref = gimbal.pid.yaw_angle_fdb;
            break;
        }
        case vUNAIMING: {  /* 未识别到目标 */
            if (ctrl_mode == REMOTER_MODE) {
               // gimbal.pid.pit_ecd_ref   += rc.ch2 * scale.ch2;
                  gimbal.pid.pit_ecd_ref  +=rc.ch2 * 0.0008f;
                gimbal.pid.yaw_angle_ref += rc.ch1 * scale.ch1;
            } else {
                gimbal.pid.yaw_angle_ref= vision.yaw_angle_error;//由导航控制
                gimbal.pid.pit_ecd_ref  = vision.pit_angle_error;
            }
            break;
        }
        default: {break;}
    }
}

int cha_pitch;
int cha_yaw;
/* 开枪时机与射频控制 */
static void vsn_shoot_enable(void) {
		switch (vision.mode) {
        case vMODE_AUTO:
        case vMODE_ANTISPIN: {
            if (ABSv(gimbal.pid.yaw_angle_ref - gimbal.pid.yaw_angle_fdb) <10.0f && 
                ABSv(gimbal.pid.pit_ecd_ref - gimbal.pid.pit_ecd_fdb) < 5.0f) {
                vision.shoot_enable = 1;
            } else {
                vision.shoot_enable = 0;
            }
						 cha_pitch=ABSv(gimbal.pid.yaw_angle_ref - gimbal.pid.yaw_angle_fdb);
					 	cha_yaw=ABSv(gimbal.pid.pit_ecd_ref - gimbal.pid.pit_ecd_fdb);
					//vision.shoot_enable = 1;
//						if(vd.dis.now>6)
//						{
//							shoot.trigger_period=300;//大于6m采用一秒300ms一发
//						}
//						else
//						{
//							shoot.trigger_period=125;
//						}
						
            break;
        }
        case vMODE_bENERGY:
        case vMODE_sENERGY: {
            if (ABSv(gimbal.pid.yaw_angle_ref - gimbal.pid.yaw_angle_fdb) < 1.0f && 
                ABSv(gimbal.pid.pit_ecd_ref - gimbal.pid.pit_ecd_fdb) < 1.0f) {
                vision.shoot_enable = 1;
            } else {
                vision.shoot_enable = 0;
            }
            break;
        }
    }
}

/** 
 * @brief 视觉加权逐差法
 */
static float vsn_gradual_deduction(float array[6]) {
    /* 数据稳定性细调（基本不用调） */
    float diff_k1 = 0.90f;    /* 逐差法系数1 */ 
    float diff_k2 = 0.06f;    /* 逐差法系数2 */
    float diff_k3 = 0.04f;    /* 逐差法系数3 */
    float error[3];
    for(int i=0; i < 3; i++) {
        error[i] = array[i] - array[3+i];
        if(error[i] > 2.0f ) return 0;
    }  
    return (
        ( diff_k1 * error[0]
        + diff_k2 * error[1]
        + diff_k3 * error[2] ) / (3 * vd.period)
    );
}
float test_threshold = 1.0f;
uint32_t test_cnt = 20;
int rotation_direction_judge(void) {
    static uint8_t valid_cnt, invalid_cnt;
    static int8_t dir;
    if(ABS(vd.yaw.angular_speed.now) > test_threshold) {  /* 目标有移动 */
        invalid_cnt = 0;
        valid_cnt = 1;
        if (vd.yaw.angular_speed.now > 0) {
            dir = 1;
        } else {
            dir = -1;
        }
    } else {
        if ((valid_cnt && invalid_cnt++ > test_cnt) || valid_cnt == 0) {
            valid_cnt = 0;
            dir = 0;
        }
    }
    return dir;
}
