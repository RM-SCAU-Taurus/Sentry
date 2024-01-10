/** 
  * @file bsp_JY901.c
  * @version 2.0
  * @date 2020.1.2
	*
  * @brief  JY901数据读取
	*
  *	@author YY
  *
  */

#include "bsp_JY901.h"
#include "bsp_T_imu.h"
imu_typedef JY901_org_data;
//imu_typedef imu_data;

/**
  * @brief JY901解算
  * @param 
  * @attention  
  * @note  
  */
void JY901_original_data_read(uint8_t imu_buf[])
{
	if(imu_buf[0]==0x55 && imu_buf[1]==0x52)
	{
		JY901_org_data.wx  = ((short)(imu_buf[3]<<8)|imu_buf[2])/32768.0f*2000.0f;
		imu_data.wy = ((short)(imu_buf[5]<<8)|imu_buf[4])/32768.0f*2000.0f;
		imu_data.wz  = ((short)(imu_buf[7]<<8)|imu_buf[6])/32768.0f*2000.0f;
		if(imu_buf[11]==0x55&&imu_buf[12]==0x53)
		{
		  JY901_org_data.roll = ((short)(imu_buf[14]<<8)|imu_buf[13])/32768.0f*180.0f;
		  JY901_org_data.pitch = ((short)(imu_buf[16]<<8)|imu_buf[15])/32768.0f*180.0f;
		  JY901_org_data.yaw  = ((short)(imu_buf[18]<<8)|imu_buf[17])/32768.0f*180.0f;
		}
	//	imu_data.roll = JY901_org_data.roll;
//		 if(JY901_org_data.pitch>180)
//		 {
//		     imu_data.pitch =JY901_org_data.pitch-360;
//		 }
//		 else 
//		 {
		     imu_data.pitch = JY901_org_data.pitch;
//		 }
//		 		 if(JY901_org_data.yaw>180)
//		 {
//		     imu_data.yaw =JY901_org_data.yaw-360;
//		 }
//		 else 
//		 {
		    // imu_data.yaw = JY901_org_data.yaw;
		        if(JY901_org_data.yaw<0)
						{
						   imu_data.yaw = JY901_org_data.yaw+180;
						}
						else
						{
						   imu_data.yaw=JY901_org_data.yaw+180;
						}
//		 }
	//	imu_data.pitch = JY901_org_data.pitch;
	//	imu_data.yaw = JY901_org_data.yaw;
	}
}
