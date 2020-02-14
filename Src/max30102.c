/*
 * max30102.c
 *
 *  Created on: 24 ���. 2019 �.
 *      Author: ˸��
 */

#include "max30102.h"
#include "I2C_Software_Master.h"


HAL_StatusTypeDef MAX30102_ReadRegister(I2C_HandleTypeDef *_hi2c, uint8_t regAddress, uint8_t* buffer, uint8_t count)
{
	if (0 != I2C_SoftWare_Master_Read(MAX30102_I2C_WRITE_ADDR, regAddress, buffer, count))
	{
		return HAL_ERROR;
	}
	return HAL_OK;
}

HAL_StatusTypeDef MAX30102_WriteRegister(I2C_HandleTypeDef *_hi2c, uint8_t regAddress, uint8_t value)
{
	if (0 != I2C_SoftWare_Master_Write(MAX30102_I2C_WRITE_ADDR, regAddress, &value, 1))
	{
		return HAL_ERROR;
	}
	return HAL_OK;
}

HAL_StatusTypeDef MAX30102_ResetFIFO(I2C_HandleTypeDef *_hi2c)
{
    HAL_StatusTypeDef status;
    status = MAX30102_WriteRegister(_hi2c, MAX30102_REG_FIFO_WRITE_PTR, 0x00);  //FIFO_WR_PTR[4:0]
    if (HAL_OK != status) return status;
    status = MAX30102_WriteRegister(_hi2c, MAX30102_REG_FIFO_READ_PTR, 0x00);  //FIFO_RD_PTR[4:0]
    if (HAL_OK != status) return status;
    status = MAX30102_WriteRegister(_hi2c, MAX30102_REG_FIFO_OVF_CNT, 0x00);  //OVF_COUNTER[4:0]
    if (HAL_OK != status) return status;

    return status;
}

void MAX30102_PowerDown(I2C_HandleTypeDef *_hi2c)
{
    MAX30102_WriteRegister(_hi2c, MAX30102_REG_MODE_CONFIG, 0x40);
}


HAL_StatusTypeDef MAX30102_Init(I2C_HandleTypeDef *_hi2c)
{
    // the below code is adopted from https://github.com/aromring/MAX30102_by_RF
    // refer to https://datasheets.maximintegrated.com/en/ds/MAX30102.pdf for details
    HAL_StatusTypeDef status;

//    status = MAX30102_WriteRegister(_hi2c, MAX30102_REG_MODE_CONFIG,
//    		0x01 << 6 // RESET the unit
//    		);
//    if (HAL_OK != status) return status;
//    HAL_Delay(10);

    status = MAX30102_WriteRegister(_hi2c, MAX30102_REG_INT_EN_1, 0xc0);
    if (HAL_OK != status) return status;
    status = MAX30102_WriteRegister(_hi2c, MAX30102_REG_INT_EN_2, 0x00);
    if (HAL_OK != status) return status;

    status = MAX30102_ResetFIFO(_hi2c);
    if (HAL_OK != status) return status;

    status = MAX30102_WriteRegister(_hi2c, MAX30102_REG_FIFO_CONFIG,
            + (0x00 << 5)  // sample averaging: 0 => 1 sample ==> no averaging
            + (0x00 << 4) // fifo rollover=false,
            + 0x0f       // 'fifo almost full' interrupt is fired when FIFO has 17 unread samples or 15 free slots
    		);
    if (HAL_OK != status) return status;

    status = MAX30102_WriteRegister(_hi2c, MAX30102_REG_MODE_CONFIG, 0x02);   //0x02 for Red only, 0x03 for SpO2 mode 0x07 multimode LED
    if (HAL_OK != status) return status;

    status = MAX30102_WriteRegister(_hi2c, MAX30102_REG_SPO2_CONFIG,
			+ (0x01 << 5)   // SPO2_ADC range = 4096nA
			+ (0x03 << 2)   // SPO2 sample rate 5 => 1000 Hz // 4 => 800 Hz // 3 => 400 Hz
			+ 0x03        // LED pulseWidth (411uS)
			);

    if (HAL_OK != status) return status;
    status = MAX30102_WriteRegister(_hi2c, MAX30102_REG_LED1_PA, 0x24);   //Choose value for ~ 7mA for LED1
    if (HAL_OK != status) return status;
    status = MAX30102_WriteRegister(_hi2c, MAX30102_REG_LED2_PA, 0x24);   // Choose value for ~ 7mA for LED2
    if (HAL_OK != status) return status;

    return status;
// ????    MAX30102_WriteRegister(_hi2c, MAX30102_REG_PILOT_PA, 0x7f);   // Choose value for ~ 25mA for Pilot LED}
}

uint8_t MAX30102_GetPartId(I2C_HandleTypeDef *_hi2c)
{
    uint8_t partId = 0;
    MAX30102_ReadRegister(_hi2c, MAX30102_REG_PART_ID, &partId, 1);
    return partId;
}

int16_t MAX30102_GetNumSamplesInFifo(I2C_HandleTypeDef *_hi2c)
{
    uint8_t read_ptr = 0;
    uint8_t write_ptr = 0;
    MAX30102_ReadRegister(_hi2c, MAX30102_REG_FIFO_READ_PTR, &read_ptr, 1);
    MAX30102_ReadRegister(_hi2c, MAX30102_REG_FIFO_WRITE_PTR, &write_ptr, 1);

    int16_t num = write_ptr - read_ptr;
    if (num >= 0 )
    {
        return num;
    }
    else
    {
        return 32 + num;
    }
}

HAL_StatusTypeDef MAX30102_ReadFifo(I2C_HandleTypeDef *_hi2c, uint8_t* buffer, uint8_t num_bytes)
{
    if (0 != MAX30102_ReadRegister(MAX30102_I2C_WRITE_ADDR, MAX30102_REG_FIFO_DATA, buffer, num_bytes))
    {
        return HAL_ERROR;
    }

    return HAL_OK;
}
