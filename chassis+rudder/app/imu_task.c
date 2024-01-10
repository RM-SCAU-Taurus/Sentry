
#include "imu_task.h"
#include "math.h"
#include "string.h"
#include "bsp_JY901.h"
#include "gimbal_task.h"
rm_imu_data_t rm_imu_data;
extern uint8_t imu_can_data[8];


void imudata_decoding(uint32_t StdId,uint32_t dlc)
{
    switch(StdId)
    {
    case RM_IMU_PARAM_ID:
    {
        rm_imu_data.accel_rangle = imu_can_data[0] &0x0F;
        rm_imu_data.gyro_rangle = (imu_can_data[0] &0xF0) >> 4;
        rm_imu_data.sensor_control_temperature = imu_can_data[2];
        rm_imu_data.imu_sensor_rotation = imu_can_data[3] & 0x1F;
        rm_imu_data.ahrs_rotation_sequence = (imu_can_data[3] & 0xE0) >> 5;
        rm_imu_data.quat_euler = imu_can_data[4] & 0x01;
        switch(rm_imu_data.gyro_rangle)
        {
        case 0:
            rm_imu_data.gyro_sen = GYRO_2000_SEN;
            break;
        case 1:
            rm_imu_data.gyro_sen = GYRO_1000_SEN;
            break;
        case 2:
            rm_imu_data.gyro_sen = GYRO_500_SEN;
            break;
        case 3:
            rm_imu_data.gyro_sen = GYRO_250_SEN;
            break;
        case 4:
            rm_imu_data.gyro_sen = GYRO_125_SEN;
            break;
        }
        switch(rm_imu_data.accel_rangle)
        {
        case 0:
            rm_imu_data.accel_sen = ACCEL_3G_SEN;
            break;
        case 1:
            rm_imu_data.accel_sen = ACCEL_6G_SEN;
            break;
        case 2:
            rm_imu_data.accel_sen = ACCEL_12G_SEN;
            break;
        case 3:
            rm_imu_data.accel_sen = ACCEL_24G_SEN;
            break;
        }
        break;
    }
    case RM_IMU_QUAT_ID:
    {
        if(rm_imu_data.quat_euler && dlc == 6)
        {
            memcpy(rm_imu_data.euler_angle, imu_can_data, dlc);
            rm_imu_data.euler_angle_fp32[0] = rm_imu_data.euler_angle[0] * 0.0001f;
            rm_imu_data.euler_angle_fp32[1] = rm_imu_data.euler_angle[1] * 0.0001f;
            rm_imu_data.euler_angle_fp32[2] = rm_imu_data.euler_angle[2] * 0.0001f;
        }
        else if(rm_imu_data.quat_euler == 0 && dlc == 8)
        {
            memcpy(rm_imu_data.quat, imu_can_data, dlc);
            rm_imu_data.quat_fp32[0] = rm_imu_data.quat[0] * 0.0001f;
            rm_imu_data.quat_fp32[1] = rm_imu_data.quat[1] * 0.0001f;
            rm_imu_data.quat_fp32[2] = rm_imu_data.quat[2] * 0.0001f;
            rm_imu_data.quat_fp32[3] = rm_imu_data.quat[3] * 0.0001f;
        }
        break;
    }
    case RM_IMU_GYRO_ID:
    {
        memcpy(rm_imu_data.gyro_int16, imu_can_data,6);
        rm_imu_data.gyro_fp32[0] = rm_imu_data.gyro_int16[0] * rm_imu_data.gyro_sen;
        rm_imu_data.gyro_fp32[1] = rm_imu_data.gyro_int16[1] * rm_imu_data.gyro_sen;
        rm_imu_data.gyro_fp32[2] = rm_imu_data.gyro_int16[2] * rm_imu_data.gyro_sen;
        rm_imu_data.sensor_temperature = (int16_t)((imu_can_data[6] << 3) | (imu_can_data[7] >> 5));
        if (rm_imu_data.sensor_temperature > 1023)
        {
            rm_imu_data.sensor_temperature -= 2048;
        }
        break;
    }
    case RM_IMU_ACCEL_ID:
    {
        memcpy(rm_imu_data.accel_int16, imu_can_data,6);
        rm_imu_data.accel_fp32[0] = rm_imu_data.accel_int16[0] * rm_imu_data.accel_sen;
        rm_imu_data.accel_fp32[1] = rm_imu_data.accel_int16[1] * rm_imu_data.accel_sen;
        rm_imu_data.accel_fp32[2] = rm_imu_data.accel_int16[2] * rm_imu_data.accel_sen;
        memcpy(&rm_imu_data.sensor_time, (imu_can_data + 6), 2);
        break;
    }
    case RM_IMU_MAG_ID:
    {
        memcpy(rm_imu_data.mag_int16, imu_can_data,6);
        break;
    }
    }

    imu_gimbal.yaw= (atan2f(rm_imu_data.quat_fp32[0] * rm_imu_data.quat_fp32[3] + rm_imu_data.quat_fp32[1] * rm_imu_data.quat_fp32[2],\
                            rm_imu_data.quat_fp32[0] * rm_imu_data.quat_fp32[0] + rm_imu_data.quat_fp32[1] * rm_imu_data.quat_fp32[1] - 0.5f)/PI)*180 +180;

    imu_gimbal.pitch = (asinf(2 * (rm_imu_data.quat_fp32[0] * rm_imu_data.quat_fp32[2] - rm_imu_data.quat_fp32[1] * rm_imu_data.quat_fp32[3]))/PI)*180;

    imu_gimbal.roll = (atan2f(rm_imu_data.quat_fp32[0] * rm_imu_data.quat_fp32[1] + rm_imu_data.quat_fp32[2] * rm_imu_data.quat_fp32[3],\
                              rm_imu_data.quat_fp32[0] * rm_imu_data.quat_fp32[0] + rm_imu_data.quat_fp32[3] * rm_imu_data.quat_fp32[3] - 0.5f)/PI)*180;


    imu_gimbal.wx = rm_imu_data.gyro_fp32[0];
    imu_gimbal.wy = rm_imu_data.gyro_fp32[1];
    imu_gimbal.wz = rm_imu_data.gyro_fp32[2];

    imu_gimbal.ax = rm_imu_data.accel_fp32[0];
    imu_gimbal.ay = rm_imu_data.accel_fp32[1];
    imu_gimbal.az = rm_imu_data.accel_fp32[2];
    gimbal.sensor.pit_palstance = imu_gimbal.wx;
    gimbal.sensor.yaw_palstance = imu_gimbal.wz;


}

