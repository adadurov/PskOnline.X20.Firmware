
#include "stm32f1xx_hal.h"
#include <string.h>
#include <stdint.h>

void I2C_SoftWare_Master_Init(void);
int I2C_SoftWare_Master_Write(uint8_t DeviceAddr, uint8_t RegAddr, uint8_t* pBuffer, uint16_t NumByteToWrite);
int I2C_SoftWare_Master_Read(uint8_t DeviceAddr, uint8_t RegAddr, uint8_t* pBuffer, uint16_t NumByteToRead);
int I2C_SoftWare_Master_ReInit(void);

