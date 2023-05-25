/*
 * mpu9250.c
 *
 *  Created on: 4 апр. 2023 г.
 *      Author: tochk
 */

#include "mpu9250.h"



#define ENABLE_DEBUG_LOG
#define ENABLE_INTERNAL_MAG_AK8963

#define AUX_READ_TIMEOUT      (200)


//#define MPU9250_DeviceID            ((uint8_t)0x71)
//#define MPU9250_DeviceID            ((uint8_t)0x73)
#define MPU9250_DeviceID            ((uint8_t)0x70)

/*----------- MPU-9250 mode register map for Gyroscope and Accelerometer ------------*/
#define MPU9250_SELF_TEST_XG        ((uint8_t)0x00)
#define MPU9250_SELF_TEST_YG        ((uint8_t)0x01)
#define MPU9250_SELF_TEST_ZG        ((uint8_t)0x02)
#define MPU9250_SELF_TEST_XA        ((uint8_t)0x0D)
#define MPU9250_SELF_TEST_YA        ((uint8_t)0x0E)
#define MPU9250_SELF_TEST_ZA        ((uint8_t)0x0F)
#define MPU9250_XG_OFFSET_H         ((uint8_t)0x13)
#define MPU9250_XG_OFFSET_L         ((uint8_t)0x14)
#define MPU9250_YG_OFFSET_H         ((uint8_t)0x15)
#define MPU9250_YG_OFFSET_L         ((uint8_t)0x16)
#define MPU9250_ZG_OFFSET_H         ((uint8_t)0x17)
#define MPU9250_ZG_OFFSET_L         ((uint8_t)0x18)
#define MPU9250_SMPLRT_DIV          ((uint8_t)0x19)
#define MPU9250_CONFIG              ((uint8_t)0x1A)
#define MPU9250_GYRO_CONFIG         ((uint8_t)0x1B)
#define MPU9250_ACCEL_CONFIG        ((uint8_t)0x1C)
#define MPU9250_ACCEL_CONFIG_2      ((uint8_t)0x1D)
#define MPU9250_LP_ACCEL_ODR        ((uint8_t)0x1E)
#define MPU9250_MOT_THR             ((uint8_t)0x1F)
#define MPU9250_FIFO_EN             ((uint8_t)0x23)
#define MPU9250_I2C_MST_CTRL        ((uint8_t)0x24)
#define MPU9250_I2C_SLV0_ADDR       ((uint8_t)0x25)
#define MPU9250_I2C_SLV0_REG        ((uint8_t)0x26)
#define MPU9250_I2C_SLV0_CTRL       ((uint8_t)0x27)
#define MPU9250_I2C_SLV1_ADDR       ((uint8_t)0x28)
#define MPU9250_I2C_SLV1_REG        ((uint8_t)0x29)
#define MPU9250_I2C_SLV1_CTRL       ((uint8_t)0x2A)
#define MPU9250_I2C_SLV2_ADDR       ((uint8_t)0x2B)
#define MPU9250_I2C_SLV2_REG        ((uint8_t)0x2C)
#define MPU9250_I2C_SLV2_CTRL       ((uint8_t)0x2D)
#define MPU9250_I2C_SLV3_ADDR       ((uint8_t)0x2E)
#define MPU9250_I2C_SLV3_REG        ((uint8_t)0x2F)
#define MPU9250_I2C_SLV3_CTRL       ((uint8_t)0x30)
#define MPU9250_I2C_SLV4_ADDR       ((uint8_t)0x31)
#define MPU9250_I2C_SLV4_REG        ((uint8_t)0x32)
#define MPU9250_I2C_SLV4_DO         ((uint8_t)0x33)
#define MPU9250_I2C_SLV4_CTRL       ((uint8_t)0x34)
#define MPU9250_I2C_SLV4_DI         ((uint8_t)0x35)
#define MPU9250_I2C_MST_STATUS      ((uint8_t)0x36)
#define MPU9250_INT_PIN_CFG         ((uint8_t)0x37)
#define MPU9250_INT_ENABLE          ((uint8_t)0x38)
#define MPU9250_INT_STATUS          ((uint8_t)0x3A)
#define MPU9250_ACCEL_XOUT_H        ((uint8_t)0x3B)
#define MPU9250_ACCEL_XOUT_L        ((uint8_t)0x3C)
#define MPU9250_ACCEL_YOUT_H        ((uint8_t)0x3D)
#define MPU9250_ACCEL_YOUT_L        ((uint8_t)0x3E)
#define MPU9250_ACCEL_ZOUT_H        ((uint8_t)0x3F)
#define MPU9250_ACCEL_ZOUT_L        ((uint8_t)0x40)
#define MPU9250_TEMP_OUT_H          ((uint8_t)0x41)
#define MPU9250_TEMP_OUT_L          ((uint8_t)0x42)
#define MPU9250_GYRO_XOUT_H         ((uint8_t)0x43)
#define MPU9250_GYRO_XOUT_L         ((uint8_t)0x44)
#define MPU9250_GYRO_YOUT_H         ((uint8_t)0x45)
#define MPU9250_GYRO_YOUT_L         ((uint8_t)0x46)
#define MPU9250_GYRO_ZOUT_H         ((uint8_t)0x47)
#define MPU9250_GYRO_ZOUT_L         ((uint8_t)0x48)
#define MPU9250_EXT_SENS_DATA_00    ((uint8_t)0x49)
#define MPU9250_EXT_SENS_DATA_01    ((uint8_t)0x4A)
#define MPU9250_EXT_SENS_DATA_02    ((uint8_t)0x4B)
#define MPU9250_EXT_SENS_DATA_03    ((uint8_t)0x4C)
#define MPU9250_EXT_SENS_DATA_04    ((uint8_t)0x4D)
#define MPU9250_EXT_SENS_DATA_05    ((uint8_t)0x4E)
#define MPU9250_EXT_SENS_DATA_06    ((uint8_t)0x4F)
#define MPU9250_EXT_SENS_DATA_07    ((uint8_t)0x50)
#define MPU9250_EXT_SENS_DATA_08    ((uint8_t)0x51)
#define MPU9250_EXT_SENS_DATA_09    ((uint8_t)0x52)
#define MPU9250_EXT_SENS_DATA_10    ((uint8_t)0x53)
#define MPU9250_EXT_SENS_DATA_11    ((uint8_t)0x54)
#define MPU9250_EXT_SENS_DATA_12    ((uint8_t)0x55)
#define MPU9250_EXT_SENS_DATA_13    ((uint8_t)0x56)
#define MPU9250_EXT_SENS_DATA_14    ((uint8_t)0x57)
#define MPU9250_EXT_SENS_DATA_15    ((uint8_t)0x58)
#define MPU9250_EXT_SENS_DATA_16    ((uint8_t)0x59)
#define MPU9250_EXT_SENS_DATA_17    ((uint8_t)0x5A)
#define MPU9250_EXT_SENS_DATA_18    ((uint8_t)0x5B)
#define MPU9250_EXT_SENS_DATA_19    ((uint8_t)0x5C)
#define MPU9250_EXT_SENS_DATA_20    ((uint8_t)0x5D)
#define MPU9250_EXT_SENS_DATA_21    ((uint8_t)0x5E)
#define MPU9250_EXT_SENS_DATA_22    ((uint8_t)0x5F)
#define MPU9250_EXT_SENS_DATA_23    ((uint8_t)0x60)
#define MPU9250_I2C_SLV0_DO         ((uint8_t)0x63)
#define MPU9250_I2C_SLV1_DO         ((uint8_t)0x64)
#define MPU9250_I2C_SLV2_DO         ((uint8_t)0x65)
#define MPU9250_I2C_SLV3_DO         ((uint8_t)0x66)
#define MPU9250_I2C_MST_DELAY_CTRL  ((uint8_t)0x67)
#define MPU9250_SIGNAL_PATH_RESET   ((uint8_t)0x68)
#define MPU9250_MOT_DETECT_CTRL     ((uint8_t)0x69)
#define MPU9250_USER_CTRL           ((uint8_t)0x6A)
#define MPU9250_PWR_MGMT_1          ((uint8_t)0x6B)
#define MPU9250_PWR_MGMT_2          ((uint8_t)0x6C)
#define MPU9250_FIFO_COUNTH         ((uint8_t)0x72)
#define MPU9250_FIFO_COUNTL         ((uint8_t)0x73)
#define MPU9250_FIFO_R_W            ((uint8_t)0x74)
#define MPU9250_WHO_AM_I            ((uint8_t)0x75)
#define MPU9250_XA_OFFSET_H         ((uint8_t)0x77)
#define MPU9250_XA_OFFSET_L         ((uint8_t)0x78)
#define MPU9250_YA_OFFSET_H         ((uint8_t)0x7A)
#define MPU9250_YA_OFFSET_L         ((uint8_t)0x7B)
#define MPU9250_ZA_OFFSET_H         ((uint8_t)0x7D)
#define MPU9250_ZA_OFFSET_L         ((uint8_t)0x7E)

#define MPU9250_I2C_SLVx_EN         ((uint8_t)0x80)
#define MPU9250_I2C_SLV4_DONE       ((uint8_t)0x40)
#define MPU9250_I2C_SLV4_NACK       ((uint8_t)0x10)

#define AK8963_I2C_ADDR             ((uint8_t)0x0C)   /* 7b'0001100 */
#define AK8963_DeviceID             ((uint8_t)0x48)
/* Read-only Reg */
#define AK8963_WIA                  ((uint8_t)0x00)
#define AK8963_INFO                 ((uint8_t)0x01)
#define AK8963_ST1                  ((uint8_t)0x02)
#define AK8963_HXL                  ((uint8_t)0x03)
#define AK8963_HXH                  ((uint8_t)0x04)
#define AK8963_HYL                  ((uint8_t)0x05)
#define AK8963_HYH                  ((uint8_t)0x06)
#define AK8963_HZL                  ((uint8_t)0x07)
#define AK8963_HZH                  ((uint8_t)0x08)
#define AK8963_ST2                  ((uint8_t)0x09)
/* Write/Read Reg */
#define AK8963_CNTL1                ((uint8_t)0x0A)
#define AK8963_CNTL2                ((uint8_t)0x0B)
#define AK8963_ASTC                 ((uint8_t)0x0C)
#define AK8963_TS1                  ((uint8_t)0x0D)
#define AK8963_TS2                  ((uint8_t)0x0E)
#define AK8963_I2CDIS               ((uint8_t)0x0F)
/* Read-only Reg ( ROM ) */
#define AK8963_ASAX                 ((uint8_t)0x10)
#define AK8963_ASAY                 ((uint8_t)0x11)
#define AK8963_ASAZ                 ((uint8_t)0x12)
/* Status */
#define AK8963_STATUS_DRDY          ((uint8_t)0x01)
#define AK8963_STATUS_DOR           ((uint8_t)0x02)
#define AK8963_STATUS_HOFL          ((uint8_t)0x08)


#define CS_HIGH()		LL_GPIO_SetOutputPin(MPU9250_CS_GPIO, MPU9250_CS_PIN)
#define CS_LOW()		LL_GPIO_ResetOutputPin(MPU9250_CS_GPIO, MPU9250_CS_PIN)

__IO void delay_ms(__IO uint32_t msCnt);

__STATIC_INLINE uint8_t spi_SendData(uint8_t data);
__STATIC_INLINE void spi_WriteData(uint8_t data);
void spi_Init(SPI_TypeDef* SPIx, GPIO_TypeDef *CS_GPIO, uint32_t CS_PIN);

static void WriteReg(uint8_t addr, uint8_t data);
static void WriteRegs(uint8_t addr, uint8_t *data, uint8_t len);
static uint8_t ReadReg(uint8_t addr);
static void ReadRegs(uint8_t addr, uint8_t *data, uint8_t len);
static void AUX_WriteReg(uint8_t slaveAddr, uint8_t addr, uint8_t data);
static uint8_t AUX_ReadReg(uint8_t slaveAddr, uint8_t addr);

static ErrorStatus DeviceCheck(void);
static ErrorStatus AUX_AK8963_DeviceCheck(void);
static void AUX_SlaveConfig(uint8_t slaveNum, uint8_t slaveAddr, uint8_t addr, uint8_t len);
static void AUX_AK8963_Init(void);

/* Variables -------------------------------------------------------------------------------*/
static float gyr_sensadj[3] = {1, 1, 1};
static float acc_sensadj[3] = {1, 1, 1};

static float mag_sensadj[3] = {1, 1, 1};
static float AK8963_ASA[3] = {0};



uint32_t MPU9250_Init(void)
{
	uint8_t res;

	spi_Init(MPU9250_SPI, MPU9250_CS_GPIO, MPU9250_CS_PIN);

	if (DeviceCheck() != SUCCESS)
	{
		return ERROR;
	}

	delay_ms(10);
	WriteReg(MPU9250_PWR_MGMT_1,         0x80);   // Reset the internal registers and restores the default settings
	delay_ms(100);
	WriteReg(MPU9250_PWR_MGMT_1,         0x04);   // Auto selects the best available clock source
	delay_ms(1);
	//WriteReg(MPU9250_INT_PIN_CFG,        0x10);   // INT_ANYRD_2CLEAR
	//delay_ms(1);
	//WriteReg(MPU9250_INT_ENABLE,         0x01);   // Set RAW_RDY_EN
	//delay_ms(1);
	WriteReg(MPU9250_PWR_MGMT_2,         0x00);   // Enable Accel (X,Y,Z axes) & Gyro (X,Y,Z axes)
	delay_ms(1);
	WriteReg(MPU9250_SMPLRT_DIV,         0x00);   // Sample Rate Divider, INTERNAL_SAMPLE_RATE = 1KHz
	delay_ms(1);
	WriteReg(MPU9250_GYRO_CONFIG,        0x18);   // 0x00:250dps, 0x08:500dps, 0x10:1000dps, 0x18:2000dps
	delay_ms(1);
	WriteReg(MPU9250_ACCEL_CONFIG,       0x00);   // 0x00:2g, 0x08:4g, 0x10:8g, 0x18:16g
	delay_ms(1);
	WriteReg(MPU9250_CONFIG,             0x00);   // gyro low pass filter
	delay_ms(1);
	WriteReg(MPU9250_ACCEL_CONFIG_2,     0x05);   // accel low pass filter
	delay_ms(1);
	WriteReg(MPU9250_USER_CTRL,          0x30);   // Set I2C_MST_EN, I2C_IF_DIS
	delay_ms(1);
	WriteReg(MPU9250_I2C_MST_CTRL,       0x5D);   // aux i2c 400kHz
	delay_ms(1);
	WriteReg(MPU9250_I2C_MST_DELAY_CTRL, 0x80);   //
	delay_ms(1);

	/*
	res = ReadReg(MPU9250_GYRO_CONFIG) & 0x18;
	switch (res)
	{
	    case 0x00:  gyr_sensadj[0] = gyr_sensadj[1] = gyr_sensadj[2] = 1.0 / 131.0;   break;
	    case 0x08:  gyr_sensadj[0] = gyr_sensadj[1] = gyr_sensadj[2] = 1.0 / 65.5;    break;
	    case 0x10:  gyr_sensadj[0] = gyr_sensadj[1] = gyr_sensadj[2] = 1.0 / 32.8;    break;
	    case 0x18:  gyr_sensadj[0] = gyr_sensadj[1] = gyr_sensadj[2] = 1.0 / 16.4;    break;
	}

	res = ReadReg(MPU9250_ACCEL_CONFIG) & 0x18;
	switch (res)
	{
	    case 0x00:  acc_sensadj[0] = acc_sensadj[1] = acc_sensadj[2] = 1.0 / 16384;   break;
	    case 0x08:  acc_sensadj[0] = acc_sensadj[1] = acc_sensadj[2] = 1.0 / 8192;    break;
	    case 0x10:  acc_sensadj[0] = acc_sensadj[1] = acc_sensadj[2] = 1.0 / 4096;    break;
	    case 0x18:  acc_sensadj[0] = acc_sensadj[1] = acc_sensadj[2] = 1.0 / 2048;    break;
	}
	*/
/*
#ifdef ENABLE_DEBUG_LOG
	printf(" >>> MPU9250 SENS : %8.6f %8.6f %8.6f, %8.6f %8.6f %8.6f\r\n", gyr_sensadj[0], gyr_sensadj[1], gyr_sensadj[2], acc_sensadj[0], acc_sensadj[1], acc_sensadj[2]);
#endif
*/

#ifdef ENABLE_INTERNAL_MAG_AK8963
	if (AUX_AK8963_DeviceCheck() != SUCCESS)
	{
		return ERROR;
	}
	AUX_AK8963_Init();
	AUX_SlaveConfig(1, AK8963_I2C_ADDR, AK8963_ST1, 8);   // add ak8963 to i2c slave 0
#endif

	delay_ms(5);

}

uint32_t MPU9250_GetData(MPU9250_Data_t *data)
{
#ifdef ENABLE_INTERNAL_MAG_AK8963
	uint8_t readBuf[22] = {0};
	ReadRegs(MPU9250_ACCEL_XOUT_H, readBuf, 22);    /* Gyr, Acc, Mag, Temp */
#else
	uint8_t readBuf[14] = {0};
	ReadRegs(MPU9250_ACCEL_XOUT_H, readBuf, 14);    /* Gyr, Acc, Temp */
#endif

	data->Temp 	=	(readBuf[6]  << 8) | readBuf[7];
	data->Gyr_X =	(readBuf[8]  << 8) | readBuf[9];
	data->Gyr_Y =	(readBuf[10] << 8) | readBuf[11];
	data->Gyr_Z =	(readBuf[12] << 8) | readBuf[13];
	data->Acl_X =	(readBuf[0]  << 8) | readBuf[1];
	data->Acl_Y =	(readBuf[2]  << 8) | readBuf[3];
	data->Acl_Z =	(readBuf[4]  << 8) | readBuf[5];

#ifdef ENABLE_INTERNAL_MAG_AK8963
	data->Mag_X = (readBuf[16] << 8) | readBuf[15];
	data->Mag_Y = (readBuf[18] << 8) | readBuf[17];
	data->Mag_Z = (readBuf[20] << 8) | readBuf[19];
#endif

	return SUCCESS;
}

/*
uint32_t MPU9250_GetData(float *data)
{
	int16_t raw[10] = {0};
	uint32_t status;

	status = MPU9250_GetRawData(raw);

	data[0] = raw[1] * gyr_sensadj[0];  // dps
	data[1] = raw[2] * gyr_sensadj[1];
	data[2] = raw[3] * gyr_sensadj[2];
	data[3] = raw[4] * acc_sensadj[0];  // g
	data[4] = raw[5] * acc_sensadj[1];
	data[5] = raw[6] * acc_sensadj[2];
#ifdef ENABLE_INTERNAL_MAG_AK8963
	data[6] = raw[7] * mag_sensadj[0];  // uT
	data[7] = raw[8] * mag_sensadj[1];
	data[8] = raw[9] * mag_sensadj[2];
#else
	data[6] = 0;  // uT
	data[7] = 0;
	data[8] = 0;
#endif

  return status;
}
*/


static void AUX_AK8963_Init(void)
{
	uint8_t res, asa[3] = {0};

	delay_ms(1);
	AUX_WriteReg(AK8963_I2C_ADDR, AK8963_CNTL2, 0x01);    // Reset Device
	delay_ms(50);
	AUX_WriteReg(AK8963_I2C_ADDR, AK8963_CNTL1, 0x00);    // Power-down mode
	delay_ms(1);
	AUX_WriteReg(AK8963_I2C_ADDR, AK8963_CNTL1, 0x1F);    // Fuse ROM access mode, Read sensitivity adjustment
	delay_ms(10);
	asa[0] = AUX_ReadReg(AK8963_I2C_ADDR, AK8963_ASAX);
	delay_ms(1);
	asa[1] = AUX_ReadReg(AK8963_I2C_ADDR, AK8963_ASAY);
	delay_ms(1);
	asa[2] = AUX_ReadReg(AK8963_I2C_ADDR, AK8963_ASAZ);
	delay_ms(1);
	#ifdef ENABLE_DEBUG_LOG
  	  printf(" >>> AK8963 ASA   : %02X %02X %02X\r\n", asa[0], asa[1], asa[2]);
	#endif
	AUX_WriteReg(AK8963_I2C_ADDR, AK8963_CNTL1, 0x00);    // Power-down mode
	delay_ms(10);
	AUX_WriteReg(AK8963_I2C_ADDR, AK8963_CNTL1, 0x16);    // Continuous measurement mode 2 & 16-bit
	delay_ms(10);

	AK8963_ASA[0] = (asa[0] - 128) * 0.5 / 128 + 1;
	AK8963_ASA[1] = (asa[1] - 128) * 0.5 / 128 + 1;
	AK8963_ASA[2] = (asa[2] - 128) * 0.5 / 128 + 1;
	#ifdef ENABLE_DEBUG_LOG
		printf(" >>> AK8963 ASA   : %8.6f %8.6f %8.6f\r\n", AK8963_ASA[0], AK8963_ASA[1], AK8963_ASA[2]);
	#endif

	res = AUX_ReadReg(AK8963_I2C_ADDR, AK8963_CNTL1) & 0x10;
	switch (res)
	{
    	case 0x00:  mag_sensadj[0] = mag_sensadj[1] = mag_sensadj[2] = 0.6;   break;
    	case 0x10:  mag_sensadj[0] = mag_sensadj[1] = mag_sensadj[2] = 0.15;  break;
	}
	#ifdef ENABLE_DEBUG_LOG
		printf(" >>> AK8963 SENS  : %8.6f %8.6f %8.6f\r\n", mag_sensadj[0], mag_sensadj[1], mag_sensadj[2]);
	#endif

	mag_sensadj[0] *= AK8963_ASA[0];
	mag_sensadj[1] *= AK8963_ASA[1];
	mag_sensadj[2] *= AK8963_ASA[2];
	#ifdef ENABLE_DEBUG_LOG
		printf(" >>> AK8963 SENS  : %8.6f %8.6f %8.6f\r\n", mag_sensadj[0], mag_sensadj[1], mag_sensadj[2]);
	#endif
}



static ErrorStatus DeviceCheck(void)
{
	uint8_t id;

	id = ReadReg(MPU9250_WHO_AM_I);

	#ifdef ENABLE_DEBUG_LOG
		printf(" >>> MPU9250      : 0x%02X\r\n", id);
	#endif

	/*
	if (id != MPU9250_DeviceID)
	{
	return ERROR;
	}
	*/

	return SUCCESS;
}


static ErrorStatus AUX_AK8963_DeviceCheck(void)
{
	uint8_t deviceID;

	delay_ms(1);
	deviceID = AUX_ReadReg(AK8963_I2C_ADDR, AK8963_WIA);
	#ifdef ENABLE_DEBUG_LOG
		printf(" >>> AK8963       : 0x%02X\r\n", deviceID);
	#endif
	if (deviceID != AK8963_DeviceID)
	{
		return ERROR;
	}

  return SUCCESS;
}


/*
 *
 */
__IO void delay_ms(__IO uint32_t msCnt)
{
	uint32_t nCount = msCnt * 7000;
	while(nCount--)
	{
		__asm("NOP");
	}
}


/**
 *  Writing data to the register at address addr
 */
static void WriteReg(uint8_t addr, uint8_t data)
{
	CS_LOW();
	spi_SendData(addr);
	spi_SendData(data);
	CS_HIGH();
}

/**
 *
 */
static void WriteRegs(uint8_t addr, uint8_t *data, uint8_t len)
{
	CS_LOW();
	spi_SendData(addr);
	for (uint32_t i = 0; i < len; i++)
	{
		spi_SendData(*data);
		data ++;
	}
	CS_HIGH();
}

static uint8_t ReadReg(uint8_t addr)
{
  uint8_t data;

  CS_LOW();
  spi_SendData(0x80 | addr);
  data = spi_SendData(0x00);
  CS_HIGH();

  return data;
}

static void ReadRegs(uint8_t addr, uint8_t *data, uint8_t len)
{
	CS_LOW();
	spi_SendData(0x80 | addr);
	for (uint32_t i = 0; i < len; i++)
	{
		*data = spi_SendData(0x00);
		data ++;
	}
	CS_HIGH();

}

static void AUX_WriteReg(uint8_t slaveAddr, uint8_t addr, uint8_t data)
{
  uint8_t  status;
  uint32_t timeout = AUX_READ_TIMEOUT;

  delay_ms(1);
  WriteReg(MPU9250_I2C_SLV4_ADDR, slaveAddr);
  delay_ms(1);
  WriteReg(MPU9250_I2C_SLV4_REG, addr);
  delay_ms(1);
  WriteReg(MPU9250_I2C_SLV4_DO, data);
  delay_ms(1);
  WriteReg(MPU9250_I2C_SLV4_CTRL, MPU9250_I2C_SLVx_EN);

  do {
	  delay_ms(1);
	  status = ReadReg(MPU9250_I2C_MST_STATUS);
  } while (((status & MPU9250_I2C_SLV4_DONE) == 0) && (timeout--));
  delay_ms(1);
}

static uint8_t AUX_ReadReg(uint8_t slaveAddr, uint8_t addr)
{
  uint8_t status;
  uint8_t data;
  uint32_t timeout = AUX_READ_TIMEOUT;

  delay_ms(1);
  WriteReg(MPU9250_I2C_SLV4_ADDR, slaveAddr | 0x80);
  delay_ms(1);
  WriteReg(MPU9250_I2C_SLV4_REG, addr);
  delay_ms(1);
  WriteReg(MPU9250_I2C_SLV4_CTRL, MPU9250_I2C_SLVx_EN);

  do {
	  delay_ms(1);
	  status = ReadReg(MPU9250_I2C_MST_STATUS);
  } while (((status & MPU9250_I2C_SLV4_DONE) == 0) && (timeout--));
  delay_ms(1);
  data = ReadReg(MPU9250_I2C_SLV4_DI);

  return data;
}

static void AUX_SlaveConfig(uint8_t slaveNum, uint8_t slaveAddr, uint8_t addr, uint8_t len)
{
  uint8_t reg;
  uint8_t offset = slaveNum * 3;

  delay_ms(1);
  WriteReg(MPU9250_I2C_SLV0_ADDR + offset, slaveAddr | 0x80);
  delay_ms(1);
  WriteReg(MPU9250_I2C_SLV0_REG + offset, addr);
  delay_ms(1);
  WriteReg(MPU9250_I2C_SLV0_CTRL + offset, MPU9250_I2C_SLVx_EN | len);
  delay_ms(1);
  WriteReg(MPU9250_I2C_SLV4_CTRL, 0x09);
  delay_ms(1);
  reg = ReadReg(MPU9250_I2C_MST_DELAY_CTRL);
  delay_ms(1);
  WriteReg(MPU9250_I2C_MST_DELAY_CTRL, reg | (0x01 << slaveNum));
}



/*
 * spi initialize
 */
void spi_Init(SPI_TypeDef* SPIx, GPIO_TypeDef *CS_GPIO, uint32_t CS_PIN)
{
	  LL_SPI_InitTypeDef SPI_InitStruct = {0};
	  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

	  if (SPIx == SPI1)
	  {
		  /* Peripheral clock enable */
		  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SPI1);

		  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
		  /**SPI1 GPIO Configuration
		  PA5   ------> SPI1_SCK
		  PA6   ------> SPI1_MISO
		  PA7   ------> SPI1_MOSI
		  */
		  GPIO_InitStruct.Pin = LL_GPIO_PIN_5|LL_GPIO_PIN_6|LL_GPIO_PIN_7;
		  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
		  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
		  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
		  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
		  GPIO_InitStruct.Alternate = LL_GPIO_AF_5;
		  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	  }

	  else if (SPIx == SPI2)
	  {
		  /* Peripheral clock enable */
		  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_SPI2);

		  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
		  /**SPI2 GPIO Configuration
		  PB13   ------> SPI2_SCK
		  PB14   ------> SPI2_MISO
		  PB15   ------> SPI2_MOSI
		  */
		  GPIO_InitStruct.Pin = LL_GPIO_PIN_13|LL_GPIO_PIN_14|LL_GPIO_PIN_15;
		  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
		  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
		  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
		  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
		  GPIO_InitStruct.Alternate = LL_GPIO_AF_5;
		  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	  }

	  else if (SPIx == SPI3)
	  {
		  /* Peripheral clock enable */
		  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_SPI3);

		  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
		  /**SPI3 GPIO Configuration
		  PB3   ------> SPI3_SCK
		  PB4   ------> SPI3_MISO
		  PB5   ------> SPI3_MOSI
		  */
		  GPIO_InitStruct.Pin = LL_GPIO_PIN_3|LL_GPIO_PIN_4|LL_GPIO_PIN_5;
		  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
		  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
		  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
		  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
		  GPIO_InitStruct.Alternate = LL_GPIO_AF_6;
		  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	  }

	  /* SPI parameter configuration*/
	  SPI_InitStruct.TransferDirection = LL_SPI_FULL_DUPLEX;
	  SPI_InitStruct.Mode = LL_SPI_MODE_MASTER;
	  SPI_InitStruct.DataWidth = LL_SPI_DATAWIDTH_8BIT;
	  SPI_InitStruct.ClockPolarity = LL_SPI_POLARITY_LOW;
	  SPI_InitStruct.ClockPhase = LL_SPI_PHASE_1EDGE;
	  SPI_InitStruct.NSS = LL_SPI_NSS_SOFT;
	  SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV128;
	  SPI_InitStruct.BitOrder = LL_SPI_MSB_FIRST;
	  SPI_InitStruct.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;
	  SPI_InitStruct.CRCPoly = 10;

	  if (LL_SPI_IsEnabled(SPIx) == 0x00000001U)
		  LL_SPI_Disable(SPIx);
	  LL_SPI_Init(SPIx, &SPI_InitStruct);
	  LL_SPI_SetStandard(SPIx, LL_SPI_PROTOCOL_MOTOROLA);
	  LL_SPI_Enable(SPIx);


	  if (CS_GPIO == GPIOA)
		  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
	  if (CS_GPIO == GPIOB)
		  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);

	  GPIO_InitStruct.Pin = CS_PIN;
	  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
	  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
	  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
	  GPIO_InitStruct.Alternate = LL_GPIO_AF_0;
	  LL_GPIO_Init(CS_GPIO, &GPIO_InitStruct);

	  CS_HIGH();
}

/*
 * Отправка байта в очередь на передачу и получение ответа
 */
__STATIC_INLINE uint8_t spi_SendData(uint8_t data)
{
	 // ждём пока регистр DR скинет данные в сдвиговый регистр
	while(!(MPU9250_SPI->SR & SPI_SR_TXE));
	// отправляем данные
	MPU9250_SPI->DR = (uint16_t)data;
	//ждём пока придёт ответ
	while(!(MPU9250_SPI->SR & SPI_SR_RXNE));
	//считываем полученные данные
	return MPU9250_SPI->DR;
}

/*
 *  Отправка байта в очередь на передачу
 */
__STATIC_INLINE void spi_WriteData(uint8_t data)
{
	 // ждём пока регистр DR скинет данные в сдвиговый регистр
	while(!(MPU9250_SPI->SR & SPI_SR_TXE));
	//отправляем данные
	MPU9250_SPI->DR = (uint16_t)data;
}


