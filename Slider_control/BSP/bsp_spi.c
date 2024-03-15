#include "bsp_spi.h"
//#include "main.h"

////#include "icm42688.h"
////#if defined(ICM_USE_HARD_SPI)
////#include "spi.h"
////#include "IO_SPI.h"
////#elif defined(ICM_USE_HARD_I2C)
////#include "bsp_cpu_i2c2.h"
////#endif
////#include "DWT.h"
////#include "stdio.h"

//static float accSensitivity   = 0.244f;   //加速度的最小分辨率 mg/LSB
//static float gyroSensitivity  = 32.8f;    //陀螺仪的最小分辨率


///*ICM42688使用的ms级延时函数，须由用户提供。*/
//#define ICM42688DelayMs(_nms)  bsp_DelayMS(_nms)

//#if defined(ICM_USE_HARD_SPI)
//#define ICM_RCC_SPIX_CS()    __HAL_RCC_GPIOB_CLK_ENABLE()
//#define ICM_PORT_SPIX_CS		 GPIOB
//#define ICM_PIN_SPIX_CS	     GPIO_PIN_12
//#define ICM_SPI_CS_LOW()     HAL_GPIO_WritePin(ICM_PORT_SPIX_CS, ICM_PIN_SPIX_CS, GPIO_PIN_RESET)
//#define ICM_SPI_CS_HIGH()    HAL_GPIO_WritePin(ICM_PORT_SPIX_CS, ICM_PIN_SPIX_CS, GPIO_PIN_SET)




///*******************************************************************************
//* 名    称： Icm_Spi_ReadWriteNbytes
//* 功    能： 使用SPI读写n个字节
//* 入口参数： pBuffer: 写入的数组  len:写入数组的长度
//* 出口参数： 无
//* 作　　者： Baxiange
//* 创建日期： 2022-07-25
//* 修    改：
//* 修改日期：
//* 备    注：
//*******************************************************************************/
//static void Icm_Spi_ReadWriteNbytes(uint8_t* pBuffer, uint8_t len)
//{
//    uint8_t i = 0;

//#if defined(ICM_USE_HARD_SPI)
//    for(i = 0; i < len; i ++)
//    {
////        *pBuffer = hal_Spi2_ReadWriteByte(*pBuffer);
//			  *pBuffer = SPI_WriteReadByte(*pBuffer);
//        pBuffer++;
//    }
//#endif

//}
//#endif

///*******************************************************************************
//* 名    称： icm42688_read_reg
//* 功    能： 读取单个寄存器的值
//* 入口参数： reg: 寄存器地址
//* 出口参数： 当前寄存器地址的值
//* 作　　者： Baxiange
//* 创建日期： 2022-07-25
//* 修    改：
//* 修改日期：
//* 备    注： 使用SPI读取寄存器时要注意:最高位为读写位，详见datasheet page51.
//*******************************************************************************/
//static uint8_t icm42688_read_reg(uint8_t reg)
//{
//    uint8_t regval = 0;

//#if defined(ICM_USE_HARD_SPI)
//    ICM_SPI_CS_LOW();
////		printf("ICM_SPI_CS_LOW\n");
//    reg |= 0x80;
//    /* 写入要读的寄存器地址 */
//    Icm_Spi_ReadWriteNbytes(&reg, 1);
//    /* 读取寄存器数据 */
//    Icm_Spi_ReadWriteNbytes(&regval, 1);
//    ICM_SPI_CS_HIGH();
////		printf("ICM_SPI_CS_HIGH\n");
//#elif defined(ICM_USE_HARD_I2C)

//#endif

//    return regval;
//}

///*******************************************************************************
//* 名    称： icm42688_read_regs
//* 功    能： 连续读取多个寄存器的值
//* 入口参数： reg: 起始寄存器地址 *buf数据指针,uint16_t len长度
//* 出口参数： 无
//* 作　　者： Baxiange
//* 创建日期： 2022-07-25
//* 修    改：
//* 修改日期：
//* 备    注： 使用SPI读取寄存器时要注意:最高位为读写位，详见datasheet page50.
//*******************************************************************************/
//static void icm42688_read_regs(uint8_t reg, uint8_t* buf, uint16_t len)
//{
//#if defined(ICM_USE_HARD_SPI)
//    reg |= 0x80;
//    ICM_SPI_CS_LOW();
//    /* 写入要读的寄存器地址 */
//    Icm_Spi_ReadWriteNbytes(&reg, 1);
//    /* 读取寄存器数据 */
//    Icm_Spi_ReadWriteNbytes(buf, len);
//    ICM_SPI_CS_HIGH();
//#elif defined(ICM_USE_HARD_I2C)
//#endif
//}


///*******************************************************************************
//* 名    称： icm42688_write_reg
//* 功    能： 向单个寄存器写数据
//* 入口参数： reg: 寄存器地址 value:数据
//* 出口参数： 0
//* 作　　者： Baxiange
//* 创建日期： 2022-07-25
//* 修    改：
//* 修改日期：
//* 备    注： 使用SPI读取寄存器时要注意:最高位为读写位，详见datasheet page50.
//*******************************************************************************/
//static uint8_t icm42688_write_reg(uint8_t reg, uint8_t value)
//{
//#if defined(ICM_USE_HARD_SPI)
//    ICM_SPI_CS_LOW();
//    /* 写入要读的寄存器地址 */
//    Icm_Spi_ReadWriteNbytes(&reg, 1);
//    /* 读取寄存器数据 */
//    Icm_Spi_ReadWriteNbytes(&value, 1);
//    ICM_SPI_CS_HIGH();
//#elif defined(ICM_USE_HARD_I2C)
//#endif
//    return 0;
//}



//float bsp_Icm42688GetAres(uint8_t Ascale)
//{
//    switch(Ascale)
//    {
//    // Possible accelerometer scales (and their register bit settings) are:
//    // 2 Gs (11), 4 Gs (10), 8 Gs (01), and 16 Gs  (00).
//    case AFS_2G:
//        accSensitivity = 2000 / 32768.0f;
//        break;
//    case AFS_4G:
//        accSensitivity = 4000 / 32768.0f;
//        break;
//    case AFS_8G:
//        accSensitivity = 8000 / 32768.0f;
//        break;
//    case AFS_16G:
//        accSensitivity = 16000 / 32768.0f;
//        break;
//    }

//    return accSensitivity;
//}

//float bsp_Icm42688GetGres(uint8_t Gscale)
//{
//    switch(Gscale)
//    {
//    case GFS_15_125DPS:
//        gyroSensitivity = 15.125f / 32768.0f;
//        break;
//    case GFS_31_25DPS:
//        gyroSensitivity = 31.25f / 32768.0f;
//        break;
//    case GFS_62_5DPS:
//        gyroSensitivity = 62.5f / 32768.0f;
//        break;
//    case GFS_125DPS:
//        gyroSensitivity = 125.0f / 32768.0f;
//        break;
//    case GFS_250DPS:
//        gyroSensitivity = 250.0f / 32768.0f;
//        break;
//    case GFS_500DPS:
//        gyroSensitivity = 500.0f / 32768.0f;
//        break;
//    case GFS_1000DPS:
//        gyroSensitivity = 1000.0f / 32768.0f;
//        break;
//    case GFS_2000DPS:
//        gyroSensitivity = 2000.0f / 32768.0f;
//        break;
//    }
//    return gyroSensitivity;
//}

///*******************************************************************************
//* 名    称： bsp_Icm42688RegCfg
//* 功    能： Icm42688 寄存器配置
//* 入口参数： 无
//* 出口参数： 无
//* 作　　者： Baxiange
//* 创建日期： 2022-07-25
//* 修    改：
//* 修改日期：
//* 备    注：
//*******************************************************************************/
//int8_t bsp_Icm42688RegCfg(void)
//{
//    uint8_t reg_val = 0;
//    /* 读取 who am i 寄存器 */
//    reg_val = icm42688_read_reg(ICM42688_WHO_AM_I);
//		printf("reg_val:%d\n",reg_val);
//    icm42688_write_reg(ICM42688_REG_BANK_SEL, 0); //设置bank 0区域寄存器
//    icm42688_write_reg(ICM42688_REG_BANK_SEL, 0x01); //软复位传感器
//    ICM42688DelayMs(100);


//    if(reg_val == ICM42688_ID)
//    {
//        icm42688_write_reg(ICM42688_REG_BANK_SEL, 1); //设置bank 1区域寄存器
//        icm42688_write_reg(ICM42688_INTF_CONFIG4, 0x02); //设置为4线SPI通信

//        icm42688_write_reg(ICM42688_REG_BANK_SEL, 0); //设置bank 0区域寄存器
//        icm42688_write_reg(ICM42688_FIFO_CONFIG, 0x40); //Stream-to-FIFO Mode(page63)


//        reg_val = icm42688_read_reg(ICM42688_INT_SOURCE0);
//        icm42688_write_reg(ICM42688_INT_SOURCE0, 0x00);
//        icm42688_write_reg(ICM42688_FIFO_CONFIG2, 0x00); // watermark
//        icm42688_write_reg(ICM42688_FIFO_CONFIG3, 0x02); // watermark
//        icm42688_write_reg(ICM42688_INT_SOURCE0, reg_val);
//        icm42688_write_reg(ICM42688_FIFO_CONFIG1, 0x63); // Enable the accel and gyro to the FIFO

//        icm42688_write_reg(ICM42688_REG_BANK_SEL, 0x00);
//        icm42688_write_reg(ICM42688_INT_CONFIG, 0x36);

//        icm42688_write_reg(ICM42688_REG_BANK_SEL, 0x00);
//        reg_val = icm42688_read_reg(ICM42688_INT_SOURCE0);
//        reg_val |= (1 << 2); //FIFO_THS_INT1_ENABLE
//        icm42688_write_reg(ICM42688_INT_SOURCE0, reg_val);

//        bsp_Icm42688GetAres(AFS_8G);
//        icm42688_write_reg(ICM42688_REG_BANK_SEL, 0x00);
//        reg_val = icm42688_read_reg(ICM42688_ACCEL_CONFIG0);//page74
//        reg_val |= (AFS_8G << 5);   //量程 ±8g
//        reg_val |= (AODR_50Hz);     //输出速率 50HZ
//        icm42688_write_reg(ICM42688_ACCEL_CONFIG0, reg_val);

//        bsp_Icm42688GetGres(GFS_1000DPS);
//        icm42688_write_reg(ICM42688_REG_BANK_SEL, 0x00);
//        reg_val = icm42688_read_reg(ICM42688_GYRO_CONFIG0);//page73
//        reg_val |= (GFS_1000DPS << 5);   //量程 ±1000dps
//        reg_val |= (GODR_50Hz);     //输出速率 50HZ
//        icm42688_write_reg(ICM42688_GYRO_CONFIG0, reg_val);

//        icm42688_write_reg(ICM42688_REG_BANK_SEL, 0x00);
//        reg_val = icm42688_read_reg(ICM42688_PWR_MGMT0); //读取PWR—MGMT0当前寄存器的值(page72)
//        reg_val &= ~(1 << 5);//使能温度测量
//        reg_val |= ((3) << 2);//设置GYRO_MODE  0:关闭 1:待机 2:预留 3:低噪声
//        reg_val |= (3);//设置ACCEL_MODE 0:关闭 1:关闭 2:低功耗 3:低噪声
//        icm42688_write_reg(ICM42688_PWR_MGMT0, reg_val);
//        ICM42688DelayMs(1); //操作完PWR—MGMT0寄存器后 200us内不能有任何读写寄存器的操作

//        return 0;
//    }
//    return -1;
//}
///*******************************************************************************
//* 名    称： bsp_Icm42688Init
//* 功    能： Icm42688 传感器初始化
//* 入口参数： 无
//* 出口参数： 0: 初始化成功  其他值: 初始化失败
//* 作　　者： Baxiange
//* 创建日期： 2022-07-25
//* 修    改：
//* 修改日期：
//* 备    注：
//*******************************************************************************/
//int8_t bsp_Icm42688Init(void)
//{
////    bsp_IcmSpixCsInit();
//		 SPI_Init();

//    return(bsp_Icm42688RegCfg());

//}

///*******************************************************************************
//* 名    称： bsp_IcmGetTemperature
//* 功    能： 读取Icm42688 内部传感器温度
//* 入口参数： 无
//* 出口参数： 无
//* 作　　者： Baxiange
//* 创建日期： 2022-07-25
//* 修    改：
//* 修改日期：
//* 备    注： datasheet page62
//*******************************************************************************/
//int8_t bsp_IcmGetTemperature(int16_t* pTemp)
//{
//    uint8_t buffer[2] = {0};

//    icm42688_read_regs(ICM42688_TEMP_DATA1, buffer, 2);

//    *pTemp = (int16_t)(((int16_t)((buffer[0] << 8) | buffer[1])) / 132.48 + 25);
//    return 0;
//}

///*******************************************************************************
//* 名    称： bsp_IcmGetAccelerometer
//* 功    能： 读取Icm42688 加速度的值
//* 入口参数： 三轴加速度的值
//* 出口参数： 无
//* 作　　者： Baxiange
//* 创建日期： 2022-07-25
//* 修    改：
//* 修改日期：
//* 备    注： datasheet page62
//*******************************************************************************/
//int8_t bsp_IcmGetAccelerometer(icm42688RawData_t* accData)
//{
//    uint8_t buffer[6] = {0};

//    icm42688_read_regs(ICM42688_ACCEL_DATA_X1, buffer, 6);

//    accData->x = ((uint16_t)buffer[0] << 8) | buffer[1];
//    accData->y = ((uint16_t)buffer[2] << 8) | buffer[3];
//    accData->z = ((uint16_t)buffer[4] << 8) | buffer[5];

//    accData->x = (int16_t)(accData->x * accSensitivity);
//    accData->y = (int16_t)(accData->y * accSensitivity);
//    accData->z = (int16_t)(accData->z * accSensitivity);

//    return 0;
//}

///*******************************************************************************
//* 名    称： bsp_IcmGetGyroscope
//* 功    能： 读取Icm42688 陀螺仪的值
//* 入口参数： 三轴陀螺仪的值
//* 出口参数： 无
//* 作　　者： Baxiange
//* 创建日期： 2022-07-25
//* 修    改：
//* 修改日期：
//* 备    注： datasheet page63
//*******************************************************************************/
//int8_t bsp_IcmGetGyroscope(icm42688RawData_t* GyroData)
//{
//    uint8_t buffer[6] = {0};

//    icm42688_read_regs(ICM42688_GYRO_DATA_X1, buffer, 6);

//    GyroData->x = ((uint16_t)buffer[0] << 8) | buffer[1];
//    GyroData->y = ((uint16_t)buffer[2] << 8) | buffer[3];
//    GyroData->z = ((uint16_t)buffer[4] << 8) | buffer[5];

//    GyroData->x = (int16_t)(GyroData->x * gyroSensitivity);
//    GyroData->y = (int16_t)(GyroData->y * gyroSensitivity);
//    GyroData->z = (int16_t)(GyroData->z * gyroSensitivity);
//    return 0;
//}

///*******************************************************************************
//* 名    称： bsp_IcmGetRawData
//* 功    能： 读取Icm42688加速度陀螺仪数据
//* 入口参数： 六轴
//* 出口参数： 无
//* 作　　者： Baxiange
//* 创建日期： 2022-07-25
//* 修    改：
//* 修改日期：
//* 备    注： datasheet page62,63
//*******************************************************************************/
//int8_t bsp_IcmGetRawData(icm42688RawData_t* accData, icm42688RawData_t* GyroData)
//{
//    uint8_t buffer[12] = {0};

//    icm42688_read_regs(ICM42688_ACCEL_DATA_X1, buffer, 12);

//    accData->x  = ((uint16_t)buffer[0] << 8)  | buffer[1];
//    accData->y  = ((uint16_t)buffer[2] << 8)  | buffer[3];
//    accData->z  = ((uint16_t)buffer[4] << 8)  | buffer[5];
//    GyroData->x = ((uint16_t)buffer[6] << 8)  | buffer[7];
//    GyroData->y = ((uint16_t)buffer[8] << 8)  | buffer[9];
//    GyroData->z = ((uint16_t)buffer[10] << 8) | buffer[11];


//    accData->x = (int16_t)(accData->x * accSensitivity);
//    accData->y = (int16_t)(accData->y * accSensitivity);
//    accData->z = (int16_t)(accData->z * accSensitivity);

//    GyroData->x = (int16_t)(GyroData->x * gyroSensitivity);
//    GyroData->y = (int16_t)(GyroData->y * gyroSensitivity);
//    GyroData->z = (int16_t)(GyroData->z * gyroSensitivity);

//    return 0;
//}

/**
  * @brief  ICM426XX设备初始化
  *         软复位、配置外部时钟和RTC、配置信号链参数、
  *         加载预准数据、配置中断、配置FIFO
  * @note   需要预先配置好处理器的SPI接口、中断引脚、RTC时钟输出
  * @param  None
  * @retval None
  */
//void icm426xx_deviceInit(void)
//{
//	#ifndef _RAWSIGNAL_ANALYSIS_ICM426XX_
//	// 陀螺仪抗混叠滤波器（AAF）带宽，更改配置需查手册中的参数表
//	uint16_t gyro_aaf_delta = 2;		/* 84 Hz（3dB） */
//	uint16_t gyro_aaf_deltasqr = gyro_aaf_delta*gyro_aaf_delta;
//	uint8_t gyro_aaf_bitshift = 13;
//	// 加速度计抗混叠滤波器（AAF）带宽
//	uint16_t accel_aaf_delta = 2;		/* 84 Hz（3dB） */
//	uint16_t accel_aaf_deltasqr = gyro_aaf_delta*gyro_aaf_delta;
//	uint8_t accel_aaf_bitshift = 13;
//	#endif
//	
//	imuStatus = ICM426XX_STATUS_NOREADY;	// 标志设备还没准备好
//	
//	imu.quaternion_body2Earth[0] = 1;	// 初始化四元数
//	imu.quaternion_body2Earth[1] = 0;
//	imu.quaternion_body2Earth[2] = 0;
//	imu.quaternion_body2Earth[3] = 0;
//	
//	// - 等待设备上电安全完成
//	HAL_Delay(3);	// 3ms
//	
//	// - 复位设备--------------------------------------------------------------------------
//	icm426xx_spi_master_write_register(MPUREG_REG_BANK_SEL, 0);	// 切换至寄存器BANK 0
//	icm426xx_spi_master_write_register(MPUREG_DEVICE_CONFIG, ICM426XX_DEVICE_CONFIG_RESET_EN);	// 软件复位
//	HAL_Delay(2);	// 等待内部复位操作完成，最少1ms
//	
//	// - 设置外部时钟源（32.768kHz）和RTC时间戳---------------------------------------------
//	
//	icm426xx_spi_master_write_register(MPUREG_REG_BANK_SEL,1);	// 切换至寄存器BANK 1
//	icm426xx_spi_master_write_register(MPUREG_INTF_CONFIG5_B1,
//	                                   ICM426XX_INTF_CONFIG5_PIN9_asCLKIN
//	                                   ); // 配置9脚为 外部时钟输入模式
//	
//	icm426xx_spi_master_write_register(MPUREG_REG_BANK_SEL,0);	// 切换至寄存器BANK 0
//	// 低功耗模式下加速度计使用内部RC震荡器；使能PLL 和  RTC
//	icm426xx_spi_master_write_register(MPUREG_INTF_CONFIG1,
//	                                   (ICM426XX_INTF_CONFIG1_ACCEL_LP_CLK_RCOSC | \
//																		 ICM426XX_INTF_CONFIG1_RTC_MODE_EN | \
//																		 0x01)
//	                                   ); 
//	// 时间戳设置：时间戳导入可读寄存器、精度1个标准RTC周期、帧同步使能、使能时间戳
//	icm426xx_spi_master_write_register(MPUREG_TMST_CONFIG,
//	                                   (ICM426XX_TMST_CONFIG_TMST_TO_REGS_EN | \
//																		  ICM426XX_TMST_CONFIG_RESOL_16us       | \
//																			ICM426XX_TMST_CONFIG_TMST_FSYNC_EN   | \
//																			ICM426XX_TMST_CONFIG_TMST_EN
//																			)
//																		);
//	
//	// - 设置陀螺仪的信号链参数------------------------------------------------------------
//	
//	// -- 陀螺仪的陷波器（NotchFilter）参数配置，根据实际情况选用
//	//icm426xx_Gyro_ConfigNotchFilter(2,ICM426XX_GYRO_NF_BW_680Hz);
//	
//	// -- 陀螺仪的抗混叠滤波器（Anti-Alias Filter）配置
//	icm426xx_spi_master_write_register(MPUREG_REG_BANK_SEL,1);	// 切换至寄存器BANK 1
//	
//	#ifndef _RAWSIGNAL_ANALYSIS_ICM426XX_
//		icm426xx_spi_master_write_register(MPUREG_GYRO_CONFIG_STATIC3_B1,
//																			 (gyro_aaf_delta & 0x3f)
//																			);
//		icm426xx_spi_master_write_register(MPUREG_GYRO_CONFIG_STATIC4_B1,
//																			 (gyro_aaf_deltasqr & 0xff)
//																			);
//		icm426xx_spi_master_write_register(MPUREG_GYRO_CONFIG_STATIC5_B1,
//																			 ( ((gyro_aaf_deltasqr>>8) & 0x0f) | (gyro_aaf_bitshift<<4) )
//																			);
//																		
//		#ifdef _DONT_USE_NOTCHFILTER_ICM426XX_
//			// ---- 使能AAF，不使用NF
//			icm426xx_spi_master_write_register(MPUREG_GYRO_CONFIG_STATIC2_B1,
//																				 (ICM426XX_GYRO_AAF_EN | ICM426XX_GYRO_NF_DIS)
//																				);
//		#else
//			// ---- 使能AAF和NF
//			icm426xx_spi_master_write_register(MPUREG_GYRO_CONFIG_STATIC2_B1,
//																				 (ICM426XX_GYRO_AAF_EN | ICM426XX_GYRO_NF_EN)
//																				);
//		#endif
//	#else
//		// ---- 关闭AAF和NF，从而收集原始数据
//		icm426xx_spi_master_write_register(MPUREG_GYRO_CONFIG_STATIC2_B1,
//																			 (ICM426XX_GYRO_AAF_DIS | ICM426XX_GYRO_NF_DIS)
//																			);
//	#endif
//																		
//	// -- 设置用户接口滤波器（UI Filter）和量程（FSR）
//	icm426xx_spi_master_write_register(MPUREG_REG_BANK_SEL,  0);	// 切换至寄存器BANK 0
//	// 3阶，200Hz 数据率（ODR），带宽104.8Hz（群延时3.8ms）；量程500dps
//	icm426xx_spi_master_write_register(MPUREG_GYRO_CONFIG0,
//	                                   (_USER_GYRO_FULLSCALE_ | ICM426XX_GYRO_CONFIG0_ODR_200_HZ)
//																		 ); // FSR 和 ODR
//	icm426xx_spi_master_write_register(MPUREG_GYRO_CONFIG1,
//	                                   (ICM426XX_GYRO_CONFIG_GYRO_UI_FILT_ORD_3RD_ORDER | (2<<1))
//																		 );	// 阶数
//	icm426xx_spi_master_write_register(MPUREG_ACCEL_GYRO_CONFIG0,
//	                                   (ICM426XX_GYRO_ACCEL_CONFIG0_ACCEL_FILT_BW_14 | ICM426XX_GYRO_ACCEL_CONFIG0_GYRO_FILT_BW_14)
//																		 ); // 带宽（加速度计和陀螺仪）
//				 
//	
//	// - 设置加速度计的信号链--------------------------------------------------------------
//	
//	icm426xx_spi_master_write_register(MPUREG_REG_BANK_SEL,2);	// 切换至寄存器BANK2
//	// -- 加速度计的抗混叠滤波器（Anti-Alias Filter）配置
//	#ifndef _RAWSIGNAL_ANALYSIS_ICM426XX_
//		icm426xx_spi_master_write_register(MPUREG_ACCEL_CONFIG_STATIC2_B2,
//																			 ((accel_aaf_delta << BIT_ACCEL_AAF_DELT_POS) | \
//																			  ICM426XX_ACCEL_AAF_EN)
//																			);	// 这里顺带使能了AAF
//		icm426xx_spi_master_write_register(MPUREG_ACCEL_CONFIG_STATIC3_B2,
//																			 (accel_aaf_deltasqr & 0xff)
//																			);
//		icm426xx_spi_master_write_register(MPUREG_ACCEL_CONFIG_STATIC4_B2,
//																			 ( ((accel_aaf_deltasqr>>8) & 0x0f) | (accel_aaf_bitshift<<4) )
//																			);
//	#else
//		// 关闭AAF，从而收集原始数据
//		icm426xx_spi_master_write_register(MPUREG_ACCEL_CONFIG_STATIC2_B2,
//																			 ICM426XX_ACCEL_AAF_DIS);
//	#endif
//																		
//	// -- 设置用户接口滤波器（UI Filter）和量程（FSR）
//	icm426xx_spi_master_write_register(MPUREG_REG_BANK_SEL,  0);	// 切换至寄存器BANK 0

//	// 3阶，200Hz 数据率（ODR），带宽104.8Hz（群延时3.8ms）；量程500dps	
//	icm426xx_spi_master_write_register(MPUREG_ACCEL_CONFIG0,
//	                                   (_USER_ACCL_FULLSCALE_ | ICM426XX_ACCEL_CONFIG0_ODR_200_HZ)
//																		 ); // FSR 和 ODR
//	icm426xx_spi_master_write_register(MPUREG_ACCEL_CONFIG1,
//	                                   (ICM426XX_ACCEL_CONFIG_ACCEL_UI_FILT_ORD_3RD_ORDER | (2<<1))
//																		 );	// 阶数
//																		 // NOTE: 这里不操作带宽，因为在陀螺仪配置时一起配了
//	
//	// - 加载预先校准好的6轴偏置数据-------------------------------------------------------
//		// 外部加载！
//	
//	// - 配置中断 并启动中断输出-----------------------------------------------------------
//	
//	icm426xx_spi_master_write_register(MPUREG_REG_BANK_SEL,  0);	// 切换至寄存器BANK 0
//	// 配置INT1的输出模式：推挽（Push Pull），高电平有效，脉冲型中断输出
//	icm426xx_spi_master_write_register(MPUREG_INT_CONFIG,
//	                                    (ICM426XX_INT_CONFIG_INT1_DRIVE_CIRCUIT_PP | \
//																			 ICM426XX_INT_CONFIG_INT1_POLARITY_HIGH)
//																		);
//	// 配置中断源
//	#ifdef _USER_USE_ICM426XX_FIFO_
//	// 可触发的中断源：FIFO阈值、FIFO满
//	icm426xx_spi_master_write_register(MPUREG_INT_SOURCE0,
//	                                   (BIT_INT_SOURCE0_FIFO_THS_INT1_EN  | \
//																		BIT_INT_SOURCE0_FIFO_FULL_INT1_EN
//																			 )
//																		);
//	#else
//	// 可触发的中断源：UI滤波器 数据可读
//	icm426xx_spi_master_write_register(MPUREG_INT_SOURCE0,
//	                                   BIT_INT_SOURCE0_UI_DRDY_INT1_EN
//																		);
//	#endif
//	// 配置清除中断方式
//	icm426xx_spi_master_write_register(MPUREG_INT_CONFIG0,
//	                                    (ICM426XX_UI_DRDY_INT_CLEAR_ON_STATUS_READ | \
//																			 ICM426XX_FIFO_THS_INT_CLEAR_ON_STATUS_READ | \
//																			 ICM426XX_FIFO_FULL_INT_CLEAR_ON_STATUS_READ
//																			)
//																		);
//	// 配置中断信号的脉冲宽度：100us宽度脉冲，使能中断有效抑制；最后异步复位中断脚
//	icm426xx_spi_master_write_register(MPUREG_INT_CONFIG1,
//	                                    (ICM426XX_INT_TPULSE_DURATION_100_US | \
//																			 ICM426XX_INT_TDEASSERT_ENABLED | \
//																			 ICM426XX_INT_CONFIG1_ASY_RST_ENABLED)
//																		);
//																			 
//	// - 配置FIFO的数据格式 并启动FIFO-----------------------------------------------------
//	
//	icm426xx_spi_master_write_register(MPUREG_REG_BANK_SEL,  0);	// 切换至寄存器BANK 0
//	
//	//复位FIFO
//	icm426xx_spi_master_write_register(MPUREG_FIFO_CONFIG,
//	                                   ICM426XX_FIFO_CONFIG_MODE_BYPASS
//																		);
//	
//	#ifdef _USER_USE_ICM426XX_FIFO_
//	//配置FIFO的无效数据报告方式、计数方式、数据字端
//	//// -32768或-32767表示无效数据； 按包计数； FIFO计数器和传感器数据包按“小字端”组织
//	icm426xx_spi_master_write_register(MPUREG_INTF_CONFIG0,
//	                                    (ICM426XX_INTF_CONFIG0_FIFO_SREG_INVALID_IND_EN | \
//																			 ICM426XX_INTF_CONFIG0_FIFO_COUNT_REC_RECORD | \
//																			 ICM426XX_INTF_CONFIG0_FIFO_COUNT_LITTLE_ENDIAN | \
//																			 ICM426XX_INTF_CONFIG0_DATA_LITTLE_ENDIAN
//																			 )
//																		);
//	
//	//配置FIFO包格式为 “Packet 3”：Header（1Byte）+Accel（6Byte）+Gyro（6Byte）+Temp（1Byte）+Time（2Byte）
//	/* 允许断包读取、使能包数量过多中断、不用高精度扩展、使能时间戳、使能温度数据、使能陀螺仪、使能加速度计*/
//	icm426xx_spi_master_write_register(MPUREG_FIFO_CONFIG1,
//	                                    (ICM426XX_FIFO_CONFIG1_RESUME_PARTIAL_RD_EN | \
//																			 ICM426XX_FIFO_CONFIG1_WM_GT_TH_EN           | \
//																			 ICM426XX_FIFO_CONFIG1_HIRES_DIS             | \
//																			 ICM426XX_FIFO_CONFIG1_TMST_FSYNC_EN         | \
//																			 ICM426XX_FIFO_CONFIG1_TEMP_EN               | \
//																			 ICM426XX_FIFO_CONFIG1_GYRO_EN               | \
//																			 ICM426XX_FIFO_CONFIG1_ACCEL_EN) 
//																		 );
//	// FIFO计数报警值（WaterMark）为4个 包！
//	icm426xx_spi_master_write_register(MPUREG_FIFO_CONFIG2,   4);
//	icm426xx_spi_master_write_register(MPUREG_FIFO_CONFIG2+1, 0);
//	
//	//配置FIFO为连续数据流模式，（满时新数据会覆盖旧数据）
//	icm426xx_spi_master_write_register(MPUREG_FIFO_CONFIG,
//	                                   ICM426XX_FIFO_CONFIG_MODE_STOP_ON_FULL
//																		);
//	#endif
//	
//	// - 电源管理配置（给陀螺仪和加速度计上电）--------------------------------------------
//	
//	icm426xx_spi_master_write_register(MPUREG_REG_BANK_SEL,  0);	// 切换至寄存器BANK 0
//	icm426xx_spi_master_write_register(MPUREG_PWR_MGMT_0,
//	                                    (ICM426XX_PWR_MGMT_0_TEMP_EN | \
//																			 ICM426XX_PWR_MGMT_0_IDLE_EN | \
//																			 ICM426XX_PWR_MGMT_0_GYRO_MODE_LN | \
//																			 ICM426XX_PWR_MGMT_0_ACCEL_MODE_LN
//																			 )
//																		);
//	HAL_Delay(50);	// 官方要求陀螺仪切换模式后，必须等最少45ms，这里保险起见50ms
//	
//	
//	// - 配置完成------------------------------------------------------------------------

//	icm426xx_spi_master_write_register(MPUREG_REG_BANK_SEL,  0);	// 切换至寄存器BANK 0
//	imuStatus = ICM426XX_STATUS_NORMAL;
//	return;
//	 
//}