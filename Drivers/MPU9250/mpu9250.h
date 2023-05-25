/*
 * mpu9250.h
 *
 *  Created on: 4 апр. 2023 г.
 *      Author: tochk
 */

#ifndef MPU9250_MPU9250_H_
#define MPU9250_MPU9250_H_


#include "stm32f4xx.h"
#include "stm32f4xx_ll_spi.h"
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_rcc.h"
#include "stm32f4xx_ll_gpio.h"
#include <stdbool.h>


#define MPU9250_SPI			SPI1
#define MPU9250_CS_GPIO		GPIOA
#define MPU9250_CS_PIN		LL_GPIO_PIN_4

typedef struct{
	int16_t		Temp;
	int16_t		Acl_X;
	int16_t		Acl_Y;
	int16_t		Acl_Z;
	int16_t		Gyr_X;
	int16_t		Gyr_Y;
	int16_t		Gyr_Z;
	int16_t		Mag_X;
	int16_t		Mag_Y;
	int16_t		Mag_Z;
}	MPU9250_Data_t;



uint32_t MPU9250_Init(void);
uint32_t MPU9250_GetData(MPU9250_Data_t *data);



#endif /* MPU9250_MPU9250_H_ */
