/*
 * i2c_erratum.c
 *
 *  Created on: 24 ט‏ם. 2019 ד.
 *      Author: ֻ¸רא
 */

#include "i2c_erratum.h"

// I2C channel 2
/**
 * \addtogroup i2cConfig I2C_CH1
 */
#define I2C2_DEV			I2C2										/**< The I2C Device.*/
#define I2C2_CLK			RCC_APB1Periph_I2C2							/**< The certain clock the I2C Device uses.*/
#define I2C2_EVT_IRQ		I2C2_EV_IRQn								/**< I2C2 Event Interrupt.*/
#define I2C2_ERR_IRQ		I2C2_ER_IRQn								/**< I2C2 Error Interrupt.*/
#define I2C2_AF				GPIO_AF_I2C2								/**< I2C2 Alternate Function mapping.*/

#define I2C2_SDA_PIN		GPIO_PIN_11									/**< I2C2 Data PIN.*/
#define I2C2_SDA_PORT		GPIOB										/**< The used GPIO Port of the I2C2 Data PIN.*/
#define I2C2_SDA_CLK		RCC_AHB1Periph_GPIOB						/**< The port requires the AHB1 bus clock.*/
#define I2C2_SDA_AF_PIN		GPIO_PinSource11								/**< The used Pin source.*/

#define I2C2_SCL_PIN		GPIO_PIN_10									/**< I2C2 Clock PIN.*/
#define I2C2_SCL_PORT		GPIOB										/**< The used GPIO Port of the I2C2 Clock PIN.*/
#define I2C2_SCL_CLK		RCC_AHB1Periph_GPIOB						/**< The port requires the AHB1 bus clock.*/
#define I2C2_SCL_AF_PIN		GPIO_PinSource10								/**< The used Pin source.*/
/*@}*/

// I2C channel 2
/**
 * \addtogroup i2cConfig I2C_CH1
 */
#define I2C2_DEV			I2C2
#define I2C2_CLK			RCC_APB1Periph_I2C2
#define I2C2_EVT_IRQ		I2C2_EV_IRQn
#define I2C2_ERR_IRQ		I2C2_ER_IRQn
#define I2C2_AF				GPIO_AF_I2C2
//#define I2C2_SDA_PIN		GPIO_PIN_11
#define I2C2_SDA_PORT		GPIOB
#define I2C2_SDA_CLK		RCC_AHB1Periph_GPIOB
#define I2C2_SDA_AF_PIN		GPIO_PinSource11
//#define I2C2_SCL_PIN		GPIO_PIN_10
#define I2C2_SCL_PORT		GPIOB
#define I2C2_SCL_CLK		RCC_AHB1Periph_GPIOB
#define I2C2_SCL_AF_PIN		GPIO_PinSource10
/*@}*/



HAL_StatusTypeDef I2C2_ClearBusyFlagErratum(I2C_HandleTypeDef *instance)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    int timeout =100;
    int timeout_cnt=0;

    // 1. Clear PE bit.
    instance->Instance->CR1 &= ~(0x0001);

    //  2. Configure the SCL and SDA I/Os as General Purpose Output Open-Drain, High level (Write 1 to GPIOx_ODR).
    GPIO_InitStruct.Mode         = GPIO_MODE_OUTPUT_OD;
//    GPIO_InitStruct.Alternate    = GPIO_AF4_I2C2;
    GPIO_InitStruct.Pull         = GPIO_PULLUP;
    GPIO_InitStruct.Speed        = GPIO_SPEED_FREQ_HIGH;

    GPIO_InitStruct.Pin          = I2C2_SCL_PIN;
    HAL_GPIO_Init(I2C2_SCL_PORT, &GPIO_InitStruct);
    HAL_GPIO_WritePin(I2C2_SCL_PORT, I2C2_SCL_PIN, GPIO_PIN_SET);

    GPIO_InitStruct.Pin          = I2C2_SDA_PIN;
    HAL_GPIO_Init(I2C2_SDA_PORT, &GPIO_InitStruct);
    HAL_GPIO_WritePin(I2C2_SDA_PORT, I2C2_SDA_PIN, GPIO_PIN_SET);


    // 3. Check SCL and SDA High level in GPIOx_IDR.
    while (GPIO_PIN_SET != HAL_GPIO_ReadPin(I2C2_SCL_PORT, I2C2_SCL_PIN))
    {
        timeout_cnt++;
        if(timeout_cnt>timeout)
            return HAL_ERROR;
    }

    while (GPIO_PIN_SET != HAL_GPIO_ReadPin(I2C2_SDA_PORT, I2C2_SDA_PIN))
    {
        timeout_cnt++;
        if(timeout_cnt>timeout)
            return HAL_ERROR;
    }

    // 4. Configure the SDA I/O as General Purpose Output Open-Drain, Low level (Write 0 to GPIOx_ODR).
    HAL_GPIO_WritePin(I2C2_SDA_PORT, I2C2_SDA_PIN, GPIO_PIN_RESET);

    //  5. Check SDA Low level in GPIOx_IDR.
    while (GPIO_PIN_RESET != HAL_GPIO_ReadPin(I2C2_SDA_PORT, I2C2_SDA_PIN))
    {
        timeout_cnt++;
        if(timeout_cnt>timeout)
            return HAL_ERROR;
    }

    // 6. Configure the SCL I/O as General Purpose Output Open-Drain, Low level (Write 0 to GPIOx_ODR).
    HAL_GPIO_WritePin(I2C2_SCL_PORT, I2C2_SCL_PIN, GPIO_PIN_RESET);

    //  7. Check SCL Low level in GPIOx_IDR.
    while (GPIO_PIN_RESET != HAL_GPIO_ReadPin(I2C2_SCL_PORT, I2C2_SCL_PIN))
    {
        timeout_cnt++;
        if(timeout_cnt>timeout)
            return HAL_ERROR;
    }

    // 8. Configure the SCL I/O as General Purpose Output Open-Drain, High level (Write 1 to GPIOx_ODR).
    HAL_GPIO_WritePin(I2C2_SCL_PORT, I2C2_SCL_PIN, GPIO_PIN_SET);

    // 9. Check SCL High level in GPIOx_IDR.
    while (GPIO_PIN_SET != HAL_GPIO_ReadPin(I2C2_SCL_PORT, I2C2_SCL_PIN))
    {
        timeout_cnt++;
        if(timeout_cnt>timeout)
            return HAL_ERROR;
    }

    // 10. Configure the SDA I/O as General Purpose Output Open-Drain , High level (Write 1 to GPIOx_ODR).
    HAL_GPIO_WritePin(I2C2_SDA_PORT, I2C2_SDA_PIN, GPIO_PIN_SET);

    // 11. Check SDA High level in GPIOx_IDR.
    while (GPIO_PIN_SET != HAL_GPIO_ReadPin(I2C2_SDA_PORT, I2C2_SDA_PIN))
    {
        timeout_cnt++;
        if(timeout_cnt>timeout)
            return HAL_ERROR;
    }

    // 12. Configure the SCL and SDA I/Os as Alternate function Open-Drain.
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//    GPIO_InitStruct.Alternate = GPIO_AF4_I2C2;

    GPIO_InitStruct.Pin = I2C2_SCL_PIN;
    HAL_GPIO_Init(I2C2_SCL_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = I2C2_SDA_PIN;
    HAL_GPIO_Init(I2C2_SDA_PORT, &GPIO_InitStruct);

    HAL_GPIO_WritePin(I2C2_SCL_PORT, I2C2_SCL_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(I2C2_SDA_PORT, I2C2_SDA_PIN, GPIO_PIN_SET);

    // 13. Set SWRST bit in I2Cx_CR1 register.
    instance->Instance->CR1 |= 0x8000;

    asm("nop");

    // 14. Clear SWRST bit in I2Cx_CR1 register.
    instance->Instance->CR1 &= ~0x8000;

    asm("nop");

    // 15. Enable the I2C peripheral by setting the PE bit in I2Cx_CR1 register
    instance->Instance->CR1 |= 0x0001;

    // Call initialization function.
    return HAL_I2C_Init(instance);
}

