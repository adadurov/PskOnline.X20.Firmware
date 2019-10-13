/*
 * i2c_erratum.h
 *
 *  Created on: 24 ט‏ם. 2019 ד.
 *      Author: ֻ¸רא
 */

#ifndef I2C_ERRATUM_H_
#define I2C_ERRATUM_H_

#include "stm32f1xx_hal.h"

// applies the workaround for I2C2 BUSY issue as per section 2.14.7 of STM32F10xxC/D/E errata sheet
HAL_StatusTypeDef I2C2_ClearBusyFlagErratum(I2C_HandleTypeDef *instance);

#endif /* I2C_ERRATUM_H_ */
