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

//static float accSensitivity   = 0.244f;   //���ٶȵ���С�ֱ��� mg/LSB
//static float gyroSensitivity  = 32.8f;    //�����ǵ���С�ֱ���


///*ICM42688ʹ�õ�ms����ʱ�����������û��ṩ��*/
//#define ICM42688DelayMs(_nms)  bsp_DelayMS(_nms)

//#if defined(ICM_USE_HARD_SPI)
//#define ICM_RCC_SPIX_CS()    __HAL_RCC_GPIOB_CLK_ENABLE()
//#define ICM_PORT_SPIX_CS		 GPIOB
//#define ICM_PIN_SPIX_CS	     GPIO_PIN_12
//#define ICM_SPI_CS_LOW()     HAL_GPIO_WritePin(ICM_PORT_SPIX_CS, ICM_PIN_SPIX_CS, GPIO_PIN_RESET)
//#define ICM_SPI_CS_HIGH()    HAL_GPIO_WritePin(ICM_PORT_SPIX_CS, ICM_PIN_SPIX_CS, GPIO_PIN_SET)




///*******************************************************************************
//* ��    �ƣ� Icm_Spi_ReadWriteNbytes
//* ��    �ܣ� ʹ��SPI��дn���ֽ�
//* ��ڲ����� pBuffer: д�������  len:д������ĳ���
//* ���ڲ����� ��
//* �������ߣ� Baxiange
//* �������ڣ� 2022-07-25
//* ��    �ģ�
//* �޸����ڣ�
//* ��    ע��
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
//* ��    �ƣ� icm42688_read_reg
//* ��    �ܣ� ��ȡ�����Ĵ�����ֵ
//* ��ڲ����� reg: �Ĵ�����ַ
//* ���ڲ����� ��ǰ�Ĵ�����ַ��ֵ
//* �������ߣ� Baxiange
//* �������ڣ� 2022-07-25
//* ��    �ģ�
//* �޸����ڣ�
//* ��    ע�� ʹ��SPI��ȡ�Ĵ���ʱҪע��:���λΪ��дλ�����datasheet page51.
//*******************************************************************************/
//static uint8_t icm42688_read_reg(uint8_t reg)
//{
//    uint8_t regval = 0;

//#if defined(ICM_USE_HARD_SPI)
//    ICM_SPI_CS_LOW();
////		printf("ICM_SPI_CS_LOW\n");
//    reg |= 0x80;
//    /* д��Ҫ���ļĴ�����ַ */
//    Icm_Spi_ReadWriteNbytes(&reg, 1);
//    /* ��ȡ�Ĵ������� */
//    Icm_Spi_ReadWriteNbytes(&regval, 1);
//    ICM_SPI_CS_HIGH();
////		printf("ICM_SPI_CS_HIGH\n");
//#elif defined(ICM_USE_HARD_I2C)

//#endif

//    return regval;
//}

///*******************************************************************************
//* ��    �ƣ� icm42688_read_regs
//* ��    �ܣ� ������ȡ����Ĵ�����ֵ
//* ��ڲ����� reg: ��ʼ�Ĵ�����ַ *buf����ָ��,uint16_t len����
//* ���ڲ����� ��
//* �������ߣ� Baxiange
//* �������ڣ� 2022-07-25
//* ��    �ģ�
//* �޸����ڣ�
//* ��    ע�� ʹ��SPI��ȡ�Ĵ���ʱҪע��:���λΪ��дλ�����datasheet page50.
//*******************************************************************************/
//static void icm42688_read_regs(uint8_t reg, uint8_t* buf, uint16_t len)
//{
//#if defined(ICM_USE_HARD_SPI)
//    reg |= 0x80;
//    ICM_SPI_CS_LOW();
//    /* д��Ҫ���ļĴ�����ַ */
//    Icm_Spi_ReadWriteNbytes(&reg, 1);
//    /* ��ȡ�Ĵ������� */
//    Icm_Spi_ReadWriteNbytes(buf, len);
//    ICM_SPI_CS_HIGH();
//#elif defined(ICM_USE_HARD_I2C)
//#endif
//}


///*******************************************************************************
//* ��    �ƣ� icm42688_write_reg
//* ��    �ܣ� �򵥸��Ĵ���д����
//* ��ڲ����� reg: �Ĵ�����ַ value:����
//* ���ڲ����� 0
//* �������ߣ� Baxiange
//* �������ڣ� 2022-07-25
//* ��    �ģ�
//* �޸����ڣ�
//* ��    ע�� ʹ��SPI��ȡ�Ĵ���ʱҪע��:���λΪ��дλ�����datasheet page50.
//*******************************************************************************/
//static uint8_t icm42688_write_reg(uint8_t reg, uint8_t value)
//{
//#if defined(ICM_USE_HARD_SPI)
//    ICM_SPI_CS_LOW();
//    /* д��Ҫ���ļĴ�����ַ */
//    Icm_Spi_ReadWriteNbytes(&reg, 1);
//    /* ��ȡ�Ĵ������� */
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
//* ��    �ƣ� bsp_Icm42688RegCfg
//* ��    �ܣ� Icm42688 �Ĵ�������
//* ��ڲ����� ��
//* ���ڲ����� ��
//* �������ߣ� Baxiange
//* �������ڣ� 2022-07-25
//* ��    �ģ�
//* �޸����ڣ�
//* ��    ע��
//*******************************************************************************/
//int8_t bsp_Icm42688RegCfg(void)
//{
//    uint8_t reg_val = 0;
//    /* ��ȡ who am i �Ĵ��� */
//    reg_val = icm42688_read_reg(ICM42688_WHO_AM_I);
//		printf("reg_val:%d\n",reg_val);
//    icm42688_write_reg(ICM42688_REG_BANK_SEL, 0); //����bank 0����Ĵ���
//    icm42688_write_reg(ICM42688_REG_BANK_SEL, 0x01); //��λ������
//    ICM42688DelayMs(100);


//    if(reg_val == ICM42688_ID)
//    {
//        icm42688_write_reg(ICM42688_REG_BANK_SEL, 1); //����bank 1����Ĵ���
//        icm42688_write_reg(ICM42688_INTF_CONFIG4, 0x02); //����Ϊ4��SPIͨ��

//        icm42688_write_reg(ICM42688_REG_BANK_SEL, 0); //����bank 0����Ĵ���
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
//        reg_val |= (AFS_8G << 5);   //���� ��8g
//        reg_val |= (AODR_50Hz);     //������� 50HZ
//        icm42688_write_reg(ICM42688_ACCEL_CONFIG0, reg_val);

//        bsp_Icm42688GetGres(GFS_1000DPS);
//        icm42688_write_reg(ICM42688_REG_BANK_SEL, 0x00);
//        reg_val = icm42688_read_reg(ICM42688_GYRO_CONFIG0);//page73
//        reg_val |= (GFS_1000DPS << 5);   //���� ��1000dps
//        reg_val |= (GODR_50Hz);     //������� 50HZ
//        icm42688_write_reg(ICM42688_GYRO_CONFIG0, reg_val);

//        icm42688_write_reg(ICM42688_REG_BANK_SEL, 0x00);
//        reg_val = icm42688_read_reg(ICM42688_PWR_MGMT0); //��ȡPWR��MGMT0��ǰ�Ĵ�����ֵ(page72)
//        reg_val &= ~(1 << 5);//ʹ���¶Ȳ���
//        reg_val |= ((3) << 2);//����GYRO_MODE  0:�ر� 1:���� 2:Ԥ�� 3:������
//        reg_val |= (3);//����ACCEL_MODE 0:�ر� 1:�ر� 2:�͹��� 3:������
//        icm42688_write_reg(ICM42688_PWR_MGMT0, reg_val);
//        ICM42688DelayMs(1); //������PWR��MGMT0�Ĵ����� 200us�ڲ������κζ�д�Ĵ����Ĳ���

//        return 0;
//    }
//    return -1;
//}
///*******************************************************************************
//* ��    �ƣ� bsp_Icm42688Init
//* ��    �ܣ� Icm42688 ��������ʼ��
//* ��ڲ����� ��
//* ���ڲ����� 0: ��ʼ���ɹ�  ����ֵ: ��ʼ��ʧ��
//* �������ߣ� Baxiange
//* �������ڣ� 2022-07-25
//* ��    �ģ�
//* �޸����ڣ�
//* ��    ע��
//*******************************************************************************/
//int8_t bsp_Icm42688Init(void)
//{
////    bsp_IcmSpixCsInit();
//		 SPI_Init();

//    return(bsp_Icm42688RegCfg());

//}

///*******************************************************************************
//* ��    �ƣ� bsp_IcmGetTemperature
//* ��    �ܣ� ��ȡIcm42688 �ڲ��������¶�
//* ��ڲ����� ��
//* ���ڲ����� ��
//* �������ߣ� Baxiange
//* �������ڣ� 2022-07-25
//* ��    �ģ�
//* �޸����ڣ�
//* ��    ע�� datasheet page62
//*******************************************************************************/
//int8_t bsp_IcmGetTemperature(int16_t* pTemp)
//{
//    uint8_t buffer[2] = {0};

//    icm42688_read_regs(ICM42688_TEMP_DATA1, buffer, 2);

//    *pTemp = (int16_t)(((int16_t)((buffer[0] << 8) | buffer[1])) / 132.48 + 25);
//    return 0;
//}

///*******************************************************************************
//* ��    �ƣ� bsp_IcmGetAccelerometer
//* ��    �ܣ� ��ȡIcm42688 ���ٶȵ�ֵ
//* ��ڲ����� ������ٶȵ�ֵ
//* ���ڲ����� ��
//* �������ߣ� Baxiange
//* �������ڣ� 2022-07-25
//* ��    �ģ�
//* �޸����ڣ�
//* ��    ע�� datasheet page62
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
//* ��    �ƣ� bsp_IcmGetGyroscope
//* ��    �ܣ� ��ȡIcm42688 �����ǵ�ֵ
//* ��ڲ����� ���������ǵ�ֵ
//* ���ڲ����� ��
//* �������ߣ� Baxiange
//* �������ڣ� 2022-07-25
//* ��    �ģ�
//* �޸����ڣ�
//* ��    ע�� datasheet page63
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
//* ��    �ƣ� bsp_IcmGetRawData
//* ��    �ܣ� ��ȡIcm42688���ٶ�����������
//* ��ڲ����� ����
//* ���ڲ����� ��
//* �������ߣ� Baxiange
//* �������ڣ� 2022-07-25
//* ��    �ģ�
//* �޸����ڣ�
//* ��    ע�� datasheet page62,63
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
  * @brief  ICM426XX�豸��ʼ��
  *         ��λ�������ⲿʱ�Ӻ�RTC�������ź���������
  *         ����Ԥ׼���ݡ������жϡ�����FIFO
  * @note   ��ҪԤ�����úô�������SPI�ӿڡ��ж����š�RTCʱ�����
  * @param  None
  * @retval None
  */
//void icm426xx_deviceInit(void)
//{
//	#ifndef _RAWSIGNAL_ANALYSIS_ICM426XX_
//	// �����ǿ�����˲�����AAF������������������ֲ��еĲ�����
//	uint16_t gyro_aaf_delta = 2;		/* 84 Hz��3dB�� */
//	uint16_t gyro_aaf_deltasqr = gyro_aaf_delta*gyro_aaf_delta;
//	uint8_t gyro_aaf_bitshift = 13;
//	// ���ٶȼƿ�����˲�����AAF������
//	uint16_t accel_aaf_delta = 2;		/* 84 Hz��3dB�� */
//	uint16_t accel_aaf_deltasqr = gyro_aaf_delta*gyro_aaf_delta;
//	uint8_t accel_aaf_bitshift = 13;
//	#endif
//	
//	imuStatus = ICM426XX_STATUS_NOREADY;	// ��־�豸��û׼����
//	
//	imu.quaternion_body2Earth[0] = 1;	// ��ʼ����Ԫ��
//	imu.quaternion_body2Earth[1] = 0;
//	imu.quaternion_body2Earth[2] = 0;
//	imu.quaternion_body2Earth[3] = 0;
//	
//	// - �ȴ��豸�ϵ簲ȫ���
//	HAL_Delay(3);	// 3ms
//	
//	// - ��λ�豸--------------------------------------------------------------------------
//	icm426xx_spi_master_write_register(MPUREG_REG_BANK_SEL, 0);	// �л����Ĵ���BANK 0
//	icm426xx_spi_master_write_register(MPUREG_DEVICE_CONFIG, ICM426XX_DEVICE_CONFIG_RESET_EN);	// �����λ
//	HAL_Delay(2);	// �ȴ��ڲ���λ������ɣ�����1ms
//	
//	// - �����ⲿʱ��Դ��32.768kHz����RTCʱ���---------------------------------------------
//	
//	icm426xx_spi_master_write_register(MPUREG_REG_BANK_SEL,1);	// �л����Ĵ���BANK 1
//	icm426xx_spi_master_write_register(MPUREG_INTF_CONFIG5_B1,
//	                                   ICM426XX_INTF_CONFIG5_PIN9_asCLKIN
//	                                   ); // ����9��Ϊ �ⲿʱ������ģʽ
//	
//	icm426xx_spi_master_write_register(MPUREG_REG_BANK_SEL,0);	// �л����Ĵ���BANK 0
//	// �͹���ģʽ�¼��ٶȼ�ʹ���ڲ�RC������ʹ��PLL ��  RTC
//	icm426xx_spi_master_write_register(MPUREG_INTF_CONFIG1,
//	                                   (ICM426XX_INTF_CONFIG1_ACCEL_LP_CLK_RCOSC | \
//																		 ICM426XX_INTF_CONFIG1_RTC_MODE_EN | \
//																		 0x01)
//	                                   ); 
//	// ʱ������ã�ʱ�������ɶ��Ĵ���������1����׼RTC���ڡ�֡ͬ��ʹ�ܡ�ʹ��ʱ���
//	icm426xx_spi_master_write_register(MPUREG_TMST_CONFIG,
//	                                   (ICM426XX_TMST_CONFIG_TMST_TO_REGS_EN | \
//																		  ICM426XX_TMST_CONFIG_RESOL_16us       | \
//																			ICM426XX_TMST_CONFIG_TMST_FSYNC_EN   | \
//																			ICM426XX_TMST_CONFIG_TMST_EN
//																			)
//																		);
//	
//	// - ���������ǵ��ź�������------------------------------------------------------------
//	
//	// -- �����ǵ��ݲ�����NotchFilter���������ã�����ʵ�����ѡ��
//	//icm426xx_Gyro_ConfigNotchFilter(2,ICM426XX_GYRO_NF_BW_680Hz);
//	
//	// -- �����ǵĿ�����˲�����Anti-Alias Filter������
//	icm426xx_spi_master_write_register(MPUREG_REG_BANK_SEL,1);	// �л����Ĵ���BANK 1
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
//			// ---- ʹ��AAF����ʹ��NF
//			icm426xx_spi_master_write_register(MPUREG_GYRO_CONFIG_STATIC2_B1,
//																				 (ICM426XX_GYRO_AAF_EN | ICM426XX_GYRO_NF_DIS)
//																				);
//		#else
//			// ---- ʹ��AAF��NF
//			icm426xx_spi_master_write_register(MPUREG_GYRO_CONFIG_STATIC2_B1,
//																				 (ICM426XX_GYRO_AAF_EN | ICM426XX_GYRO_NF_EN)
//																				);
//		#endif
//	#else
//		// ---- �ر�AAF��NF���Ӷ��ռ�ԭʼ����
//		icm426xx_spi_master_write_register(MPUREG_GYRO_CONFIG_STATIC2_B1,
//																			 (ICM426XX_GYRO_AAF_DIS | ICM426XX_GYRO_NF_DIS)
//																			);
//	#endif
//																		
//	// -- �����û��ӿ��˲�����UI Filter�������̣�FSR��
//	icm426xx_spi_master_write_register(MPUREG_REG_BANK_SEL,  0);	// �л����Ĵ���BANK 0
//	// 3�ף�200Hz �����ʣ�ODR��������104.8Hz��Ⱥ��ʱ3.8ms��������500dps
//	icm426xx_spi_master_write_register(MPUREG_GYRO_CONFIG0,
//	                                   (_USER_GYRO_FULLSCALE_ | ICM426XX_GYRO_CONFIG0_ODR_200_HZ)
//																		 ); // FSR �� ODR
//	icm426xx_spi_master_write_register(MPUREG_GYRO_CONFIG1,
//	                                   (ICM426XX_GYRO_CONFIG_GYRO_UI_FILT_ORD_3RD_ORDER | (2<<1))
//																		 );	// ����
//	icm426xx_spi_master_write_register(MPUREG_ACCEL_GYRO_CONFIG0,
//	                                   (ICM426XX_GYRO_ACCEL_CONFIG0_ACCEL_FILT_BW_14 | ICM426XX_GYRO_ACCEL_CONFIG0_GYRO_FILT_BW_14)
//																		 ); // �������ٶȼƺ������ǣ�
//				 
//	
//	// - ���ü��ٶȼƵ��ź���--------------------------------------------------------------
//	
//	icm426xx_spi_master_write_register(MPUREG_REG_BANK_SEL,2);	// �л����Ĵ���BANK2
//	// -- ���ٶȼƵĿ�����˲�����Anti-Alias Filter������
//	#ifndef _RAWSIGNAL_ANALYSIS_ICM426XX_
//		icm426xx_spi_master_write_register(MPUREG_ACCEL_CONFIG_STATIC2_B2,
//																			 ((accel_aaf_delta << BIT_ACCEL_AAF_DELT_POS) | \
//																			  ICM426XX_ACCEL_AAF_EN)
//																			);	// ����˳��ʹ����AAF
//		icm426xx_spi_master_write_register(MPUREG_ACCEL_CONFIG_STATIC3_B2,
//																			 (accel_aaf_deltasqr & 0xff)
//																			);
//		icm426xx_spi_master_write_register(MPUREG_ACCEL_CONFIG_STATIC4_B2,
//																			 ( ((accel_aaf_deltasqr>>8) & 0x0f) | (accel_aaf_bitshift<<4) )
//																			);
//	#else
//		// �ر�AAF���Ӷ��ռ�ԭʼ����
//		icm426xx_spi_master_write_register(MPUREG_ACCEL_CONFIG_STATIC2_B2,
//																			 ICM426XX_ACCEL_AAF_DIS);
//	#endif
//																		
//	// -- �����û��ӿ��˲�����UI Filter�������̣�FSR��
//	icm426xx_spi_master_write_register(MPUREG_REG_BANK_SEL,  0);	// �л����Ĵ���BANK 0

//	// 3�ף�200Hz �����ʣ�ODR��������104.8Hz��Ⱥ��ʱ3.8ms��������500dps	
//	icm426xx_spi_master_write_register(MPUREG_ACCEL_CONFIG0,
//	                                   (_USER_ACCL_FULLSCALE_ | ICM426XX_ACCEL_CONFIG0_ODR_200_HZ)
//																		 ); // FSR �� ODR
//	icm426xx_spi_master_write_register(MPUREG_ACCEL_CONFIG1,
//	                                   (ICM426XX_ACCEL_CONFIG_ACCEL_UI_FILT_ORD_3RD_ORDER | (2<<1))
//																		 );	// ����
//																		 // NOTE: ���ﲻ����������Ϊ������������ʱһ������
//	
//	// - ����Ԥ��У׼�õ�6��ƫ������-------------------------------------------------------
//		// �ⲿ���أ�
//	
//	// - �����ж� �������ж����-----------------------------------------------------------
//	
//	icm426xx_spi_master_write_register(MPUREG_REG_BANK_SEL,  0);	// �л����Ĵ���BANK 0
//	// ����INT1�����ģʽ�����죨Push Pull�����ߵ�ƽ��Ч���������ж����
//	icm426xx_spi_master_write_register(MPUREG_INT_CONFIG,
//	                                    (ICM426XX_INT_CONFIG_INT1_DRIVE_CIRCUIT_PP | \
//																			 ICM426XX_INT_CONFIG_INT1_POLARITY_HIGH)
//																		);
//	// �����ж�Դ
//	#ifdef _USER_USE_ICM426XX_FIFO_
//	// �ɴ������ж�Դ��FIFO��ֵ��FIFO��
//	icm426xx_spi_master_write_register(MPUREG_INT_SOURCE0,
//	                                   (BIT_INT_SOURCE0_FIFO_THS_INT1_EN  | \
//																		BIT_INT_SOURCE0_FIFO_FULL_INT1_EN
//																			 )
//																		);
//	#else
//	// �ɴ������ж�Դ��UI�˲��� ���ݿɶ�
//	icm426xx_spi_master_write_register(MPUREG_INT_SOURCE0,
//	                                   BIT_INT_SOURCE0_UI_DRDY_INT1_EN
//																		);
//	#endif
//	// ��������жϷ�ʽ
//	icm426xx_spi_master_write_register(MPUREG_INT_CONFIG0,
//	                                    (ICM426XX_UI_DRDY_INT_CLEAR_ON_STATUS_READ | \
//																			 ICM426XX_FIFO_THS_INT_CLEAR_ON_STATUS_READ | \
//																			 ICM426XX_FIFO_FULL_INT_CLEAR_ON_STATUS_READ
//																			)
//																		);
//	// �����ж��źŵ������ȣ�100us������壬ʹ���ж���Ч���ƣ�����첽��λ�жϽ�
//	icm426xx_spi_master_write_register(MPUREG_INT_CONFIG1,
//	                                    (ICM426XX_INT_TPULSE_DURATION_100_US | \
//																			 ICM426XX_INT_TDEASSERT_ENABLED | \
//																			 ICM426XX_INT_CONFIG1_ASY_RST_ENABLED)
//																		);
//																			 
//	// - ����FIFO�����ݸ�ʽ ������FIFO-----------------------------------------------------
//	
//	icm426xx_spi_master_write_register(MPUREG_REG_BANK_SEL,  0);	// �л����Ĵ���BANK 0
//	
//	//��λFIFO
//	icm426xx_spi_master_write_register(MPUREG_FIFO_CONFIG,
//	                                   ICM426XX_FIFO_CONFIG_MODE_BYPASS
//																		);
//	
//	#ifdef _USER_USE_ICM426XX_FIFO_
//	//����FIFO����Ч���ݱ��淽ʽ��������ʽ�������ֶ�
//	//// -32768��-32767��ʾ��Ч���ݣ� ���������� FIFO�������ʹ��������ݰ�����С�ֶˡ���֯
//	icm426xx_spi_master_write_register(MPUREG_INTF_CONFIG0,
//	                                    (ICM426XX_INTF_CONFIG0_FIFO_SREG_INVALID_IND_EN | \
//																			 ICM426XX_INTF_CONFIG0_FIFO_COUNT_REC_RECORD | \
//																			 ICM426XX_INTF_CONFIG0_FIFO_COUNT_LITTLE_ENDIAN | \
//																			 ICM426XX_INTF_CONFIG0_DATA_LITTLE_ENDIAN
//																			 )
//																		);
//	
//	//����FIFO����ʽΪ ��Packet 3����Header��1Byte��+Accel��6Byte��+Gyro��6Byte��+Temp��1Byte��+Time��2Byte��
//	/* ����ϰ���ȡ��ʹ�ܰ����������жϡ����ø߾�����չ��ʹ��ʱ�����ʹ���¶����ݡ�ʹ�������ǡ�ʹ�ܼ��ٶȼ�*/
//	icm426xx_spi_master_write_register(MPUREG_FIFO_CONFIG1,
//	                                    (ICM426XX_FIFO_CONFIG1_RESUME_PARTIAL_RD_EN | \
//																			 ICM426XX_FIFO_CONFIG1_WM_GT_TH_EN           | \
//																			 ICM426XX_FIFO_CONFIG1_HIRES_DIS             | \
//																			 ICM426XX_FIFO_CONFIG1_TMST_FSYNC_EN         | \
//																			 ICM426XX_FIFO_CONFIG1_TEMP_EN               | \
//																			 ICM426XX_FIFO_CONFIG1_GYRO_EN               | \
//																			 ICM426XX_FIFO_CONFIG1_ACCEL_EN) 
//																		 );
//	// FIFO��������ֵ��WaterMark��Ϊ4�� ����
//	icm426xx_spi_master_write_register(MPUREG_FIFO_CONFIG2,   4);
//	icm426xx_spi_master_write_register(MPUREG_FIFO_CONFIG2+1, 0);
//	
//	//����FIFOΪ����������ģʽ������ʱ�����ݻḲ�Ǿ����ݣ�
//	icm426xx_spi_master_write_register(MPUREG_FIFO_CONFIG,
//	                                   ICM426XX_FIFO_CONFIG_MODE_STOP_ON_FULL
//																		);
//	#endif
//	
//	// - ��Դ�������ã��������Ǻͼ��ٶȼ��ϵ磩--------------------------------------------
//	
//	icm426xx_spi_master_write_register(MPUREG_REG_BANK_SEL,  0);	// �л����Ĵ���BANK 0
//	icm426xx_spi_master_write_register(MPUREG_PWR_MGMT_0,
//	                                    (ICM426XX_PWR_MGMT_0_TEMP_EN | \
//																			 ICM426XX_PWR_MGMT_0_IDLE_EN | \
//																			 ICM426XX_PWR_MGMT_0_GYRO_MODE_LN | \
//																			 ICM426XX_PWR_MGMT_0_ACCEL_MODE_LN
//																			 )
//																		);
//	HAL_Delay(50);	// �ٷ�Ҫ���������л�ģʽ�󣬱��������45ms�����ﱣ�����50ms
//	
//	
//	// - �������------------------------------------------------------------------------

//	icm426xx_spi_master_write_register(MPUREG_REG_BANK_SEL,  0);	// �л����Ĵ���BANK 0
//	imuStatus = ICM426XX_STATUS_NORMAL;
//	return;
//	 
//}