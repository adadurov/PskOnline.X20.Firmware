/*
 * max30102.c
 *
 *  Created on: 24 мар. 2019 г.
 *      Author: Лёша
 */

#include "max30102.h"
#include "stm32f1xx_hal.h"


void MAX30102_Init_2(I2C_HandleTypeDef *_hi2c)
{
    // the below code is adopted from https://github.com/aromring/MAX30102_by_RF
    // refer to https://datasheets.maximintegrated.com/en/ds/MAX30102.pdf for details
	HAL_StatusTypeDef status;
    status = MAX30102_WriteRegister(_hi2c, MAX30102_REG_INT_EN_1, 0xc0);
    status = MAX30102_WriteRegister(_hi2c, MAX30102_REG_INT_EN_2, 0x00);
    status = MAX30102_WriteRegister(_hi2c, MAX30102_REG_FIFO_WRITE_PTR, 0x00);  //FIFO_WR_PTR[4:0]
    status = MAX30102_WriteRegister(_hi2c, MAX30102_REG_FIFO_OVF_CNT, 0x00);  //OVF_COUNTER[4:0]
    status = MAX30102_WriteRegister(_hi2c, MAX30102_REG_FIFO_READ_PTR, 0x00);  //FIFO_RD_PTR[4:0]
    status = MAX30102_WriteRegister(_hi2c, MAX30102_REG_FIFO_CONFIG, 0x20 + 0x0f);  //sample avg = 2, fifo rollover=false, fifo almost full = 17
    //    SetMode(_hi2c, MAX30102_MODE_HEART_RATE);
    status = MAX30102_WriteRegister(_hi2c, MAX30102_REG_MODE_CONFIG, 0x03);   //0x02 for Red only, 0x03 for SpO2 mode, 0x07 multimode LED

    status = MAX30102_WriteRegister(_hi2c, MAX30102_REG_SPO2_CONFIG, 0x27);  // SPO2_ADC range = 4096nA, SPO2 sample rate (100 Hz), LED pulseWidth (411uS)
//    MAX30102_WriteRegister(_hi2c, MAX30102_REG_SPO2_CONFIG, 0x20 + 0x14 + 0x03);  // SPO2_ADC range = 4096nA, SPO2 sample rate (1000 Hz), LED pulseWidth (411uS)
    status = MAX30102_WriteRegister(_hi2c, MAX30102_REG_LED1_PA, 0x24);   //Choose value for ~ 7mA for LED1
    status = MAX30102_WriteRegister(_hi2c, MAX30102_REG_LED2_PA, 0x24);   // Choose value for ~ 7mA for LED2

// ????    MAX30102_WriteRegister(_hi2c, MAX30102_REG_PILOT_PA, 0x7f);   // Choose value for ~ 25mA for Pilot LED}
}





HAL_StatusTypeDef MAX30102_ReadRegister(I2C_HandleTypeDef *_hi2c, uint8_t regAddress, uint8_t* buffer, uint8_t count)
{
    if (HAL_OK != HAL_I2C_Master_Transmit(_hi2c, MAX30102_I2C_WRITE_ADDR, &regAddress, 1, 1000))
    {
        return HAL_ERROR;
    }
    return HAL_I2C_Master_Receive(_hi2c, MAX30102_I2C_READ_ADDR, buffer, count, 1000);
}

HAL_StatusTypeDef MAX30102_WriteRegister(I2C_HandleTypeDef *_hi2c, uint8_t regAddress, uint8_t value)
{
    uint8_t buffer[] = { regAddress, value };
    return HAL_I2C_Master_Transmit(_hi2c, MAX30102_I2C_WRITE_ADDR, buffer, 2, 1000);
}

void MAX30102_SetMode(I2C_HandleTypeDef *_hi2c, uint8_t mode)
{
    MAX30102_WriteRegister(_hi2c, MAX30102_REG_MODE_CONFIG, mode);
}

void MAX30102_ResetFIFO(I2C_HandleTypeDef *_hi2c)
{
    MAX30102_WriteRegister(_hi2c, MAX30102_REG_FIFO_WRITE_PTR, 0x00);  //FIFO_WR_PTR[4:0]
    MAX30102_WriteRegister(_hi2c, MAX30102_REG_FIFO_OVF_CNT, 0x00);  //OVF_COUNTER[4:0]
    MAX30102_WriteRegister(_hi2c, MAX30102_REG_FIFO_READ_PTR, 0x00);  //FIFO_RD_PTR[4:0]
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
//    status = MAX30102_WriteRegister(_hi2c, MAX30102_REG_INT_EN_1, 0xc0);
//    if (HAL_OK != status) return status;
//    status = MAX30102_WriteRegister(_hi2c, MAX30102_REG_INT_EN_2, 0x00);
//    if (HAL_OK != status) return status;
//    status = MAX30102_WriteRegister(_hi2c, MAX30102_REG_FIFO_WRITE_PTR, 0x00);  //FIFO_WR_PTR[4:0]
//    if (HAL_OK != status) return status;
//    status = MAX30102_WriteRegister(_hi2c, MAX30102_REG_FIFO_OVF_CNT, 0x00);  //OVF_COUNTER[4:0]
//    if (HAL_OK != status) return status;
//    status = MAX30102_WriteRegister(_hi2c, MAX30102_REG_FIFO_READ_PTR, 0x00);  //FIFO_RD_PTR[4:0]
//    if (HAL_OK != status) return status;
//    status = MAX30102_WriteRegister(_hi2c, MAX30102_REG_FIFO_CONFIG, 0x20 + 0x0f);  //sample avg = 2, fifo rollover=false, fifo almost full = 17
//    if (HAL_OK != status) return status;
//    //    SetMode(_hi2c, MAX30102_MODE_HEART_RATE);
//    status = MAX30102_WriteRegister(_hi2c, MAX30102_REG_MODE_CONFIG, 0x03);   //0x02 for Red only, 0x03 for SpO2 mode, 0x07 multimode LED
//    if (HAL_OK != status) return status;
//
//    status = MAX30102_WriteRegister(_hi2c, MAX30102_REG_SPO2_CONFIG, 0x27);  // SPO2_ADC range = 4096nA, SPO2 sample rate (100 Hz), LED pulseWidth (411uS)
//    if (HAL_OK != status) return status;
////    MAX30102_WriteRegister(_hi2c, MAX30102_REG_SPO2_CONFIG, 0x20 + 0x14 + 0x03);  // SPO2_ADC range = 4096nA, SPO2 sample rate (1000 Hz), LED pulseWidth (411uS)
//    status = MAX30102_WriteRegister(_hi2c, MAX30102_REG_LED1_PA, 0x24);   //Choose value for ~ 7mA for LED1
//    if (HAL_OK != status) return status;
//    status = MAX30102_WriteRegister(_hi2c, MAX30102_REG_LED2_PA, 0x24);   // Choose value for ~ 7mA for LED2
//    if (HAL_OK != status) return status;

    status = MAX30102_WriteRegister(_hi2c, MAX30102_REG_INT_EN_1, 0xc0);
    if (HAL_OK != status) return status;
    status = MAX30102_WriteRegister(_hi2c, MAX30102_REG_INT_EN_2, 0x00);
    if (HAL_OK != status) return status;
    status = MAX30102_WriteRegister(_hi2c, MAX30102_REG_FIFO_WRITE_PTR, 0x00);  //FIFO_WR_PTR[4:0]
    if (HAL_OK != status) return status;
    status = MAX30102_WriteRegister(_hi2c, MAX30102_REG_FIFO_OVF_CNT, 0x00);  //OVF_COUNTER[4:0]
    if (HAL_OK != status) return status;
    status = MAX30102_WriteRegister(_hi2c, MAX30102_REG_FIFO_READ_PTR, 0x00);  //FIFO_RD_PTR[4:0]
    if (HAL_OK != status) return status;
    status = MAX30102_WriteRegister(_hi2c, MAX30102_REG_FIFO_CONFIG, 0x2f);  //sample avg = 2, fifo rollover=false, fifo almost full = 17
    if (HAL_OK != status) return status;
    status = MAX30102_WriteRegister(_hi2c, MAX30102_REG_MODE_CONFIG, 0x02);   //0x02 for Red only, 0x03 for SpO2 mode 0x07 multimode LED
    if (HAL_OK != status) return status;
//    SetMode(_hi2c, MAX30102_MODE_HEART_RATE);

//    MAX30102_WriteRegister(_hi2c, MAX30102_REG_SPO2_CONFIG, 0x27);  // SPO2_ADC range = 4096nA, SPO2 sample rate (100 Hz), LED pulseWidth (411uS)
    status = MAX30102_WriteRegister(_hi2c, MAX30102_REG_SPO2_CONFIG, 0x20 + 0x14 + 0x03);  // SPO2_ADC range = 4096nA, SPO2 sample rate (800 Hz), LED pulseWidth (411uS)
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
    uint8_t reg_fifo_data = MAX30102_REG_FIFO_DATA;
    if (HAL_OK != HAL_I2C_Master_Transmit(_hi2c, MAX30102_I2C_WRITE_ADDR, &reg_fifo_data, 1, 1000))
    {
        return HAL_ERROR;
    }
    return HAL_I2C_Master_Receive(_hi2c, MAX30102_I2C_READ_ADDR, buffer, num_bytes, 1000);
}
