/*
 * max30102.h
 *
 *  Created on: 24 мар. 2019 г.
 *      Author: Лёша
 */

#ifndef MAX30102_H_
#define MAX30102_H_

#include "stm32f1xx_hal.h"

#define MAX30102_I2C_WRITE_ADDR             0xAE
#define MAX30102_I2C_READ_ADDR              0xAF

#define MAX30102_REG_INT_STATUS_1           0x00
#define MAX30102_REG_INT_STATUS_2           0x01
#define MAX30102_REG_INT_EN_1               0x02
#define MAX30102_REG_INT_EN_2               0x03

#define MAX30102_REG_FIFO_WRITE_PTR         0x04
#define MAX30102_REG_FIFO_OVF_CNT           0x05
#define MAX30102_REG_FIFO_READ_PTR          0x06
#define MAX30102_REG_FIFO_DATA              0x07

#define MAX30102_REG_FIFO_CONFIG            0x08
#define MAX30102_REG_MODE_CONFIG            0x09
#define MAX30102_REG_SPO2_CONFIG            0x0A

#define MAX30102_REG_LED1_PA                0x0C
#define MAX30102_REG_LED2_PA                0x0D

#define MAX30102_REG_MLED_MODE_SLOT_12      0x11
#define MAX30102_REG_MLED_MODE_SLOT_34      0x12

#define MAX30102_REG_REV_ID                 0xFE
#define MAX30102_REG_PART_ID                0xFF

#define MAX30102_MODE_HEART_RATE            0x02
#define MAX30102_MODE_SPO2                  0x03
#define MAX30102_MODE_MULTI_LED             0x07

HAL_StatusTypeDef MAX30102_ReadRegister(I2C_HandleTypeDef *_hi2c, uint8_t regAddress, uint8_t* buffer, uint8_t count);
HAL_StatusTypeDef MAX30102_WriteRegister(I2C_HandleTypeDef *_hi2c, uint8_t regAddress, uint8_t value);

// reads the specified number of bytes from the FIFO DATA register
HAL_StatusTypeDef MAX30102_ReadFifo(I2C_HandleTypeDef *_hi2c, uint8_t* buffer, uint8_t num_bytes);

HAL_StatusTypeDef MAX30102_Init(I2C_HandleTypeDef *_hi2c);
uint8_t MAX30102_GetPartId(I2C_HandleTypeDef *_hi2c);

void MAX30102_ResetFIFO(I2C_HandleTypeDef *_hi2c);
int16_t MAX30102_GetNumSamplesInFifo(I2C_HandleTypeDef *_hi2c);

#endif /* MAX30102_H_ */
