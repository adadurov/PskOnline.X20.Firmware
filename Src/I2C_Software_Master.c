
#include "I2C_Software_Master.h"

//#define SCL_PIN 	GPIO_PIN_6
//#define SDA_PIN 	GPIO_PIN_9

#define SCL_PIN 	GPIO_PIN_10
#define SDA_PIN 	GPIO_PIN_11



#define PIN_RESET(port, pin)      port->BSRR = (uint32_t)pin << 16U
#define PIN_SET(port, pin)        port->BSRR = pin


#define SCL_H         PIN_SET(GPIOB, SCL_PIN)
#define SCL_L         PIN_RESET(GPIOB, SCL_PIN)

#define SDA_H         PIN_SET(GPIOB, SDA_PIN)
#define SDA_L         PIN_RESET(GPIOB, SDA_PIN)

#define SCL_read      (GPIOB->IDR & SCL_PIN)
#define SDA_read      (GPIOB->IDR & SDA_PIN)

#define I2C_DIRECTION_TRANSMITTER       ((uint8_t)0x00)
#define I2C_DIRECTION_RECEIVER          ((uint8_t)0x01)


void I2C_delay(void);
void I2C_delay_half(void);

uint8_t I2C_Start(void);
void I2C_Stop(void);
void I2C_Ack(void);
void I2C_NoAck(void);
uint8_t I2C_WaitAck(void);
void I2C_SendByte(uint8_t byte);
uint8_t I2C_ReceiveByte(void);



inline void I2C_delay(void)
{
    volatile int i = 8;
    while (i){
        i--;
        __asm("nop");
    }
}

inline void I2C_delay_half(void)
{
    volatile int i = 4;
    while (i){
        i--;
        __asm("nop");
    }
}


void I2C_SoftWare_Master_Init(void)
{
	  GPIO_InitTypeDef GPIO_InitStruct = {0};

	  /*Configure GPIO pin Output Level */
	  HAL_GPIO_WritePin(GPIOB, SCL_PIN, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOB, SDA_PIN, GPIO_PIN_RESET);

	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

	  GPIO_InitStruct.Pin = SCL_PIN;
	  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	  GPIO_InitStruct.Pin = SDA_PIN;
	  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    
    SCL_H;
    SDA_H;
    
    SCL_L;
    SDA_L;
    
    SCL_H;
    SDA_H;
}



uint8_t I2C_Start(void)
{
    SDA_H;
    SCL_H;
    I2C_delay();
    if (!SDA_read)
        return 0;
    SDA_L;
    I2C_delay();
    if (SDA_read)
        return 0;
    SCL_L;
    I2C_delay();
    return 1;
}

void I2C_Stop(void)
{
    SCL_L;
    I2C_delay_half();
    SDA_L;
    I2C_delay_half();
    SCL_H;
    I2C_delay_half();
    SDA_H;
    I2C_delay_half();
}


void I2C_Ack(void)
{
    SCL_L;
    I2C_delay_half();
    SDA_L;
    I2C_delay_half();
    SCL_H;
    I2C_delay_half();
    SCL_L;
    I2C_delay_half();
}

void I2C_NoAck(void)
{
    SCL_L;
    I2C_delay();
    SDA_H;
    I2C_delay();
    SCL_H;
    I2C_delay();
    SCL_L;
    I2C_delay();
}

uint8_t I2C_WaitAck(void)
{
    SCL_L;
    I2C_delay_half();
    SDA_H;
    I2C_delay_half();
    SCL_H; 
	 
    I2C_delay();
    if (SDA_read)
    {
        SCL_L;
        I2C_delay();
        return 0;
    }

    SCL_L;
    I2C_delay();
    
    return 1;
}

void I2C_SendByte(uint8_t byte)
{
    uint8_t i = 8;
    while (i--)
    {
        SCL_L;
        I2C_delay_half();
        if (byte & 0x80)
            SDA_H;
        else
            SDA_L;
        byte <<= 1;
        I2C_delay_half();
        SCL_H;
        I2C_delay();
    }
    SCL_L;
}

uint8_t I2C_ReceiveByte(void)
{
    uint8_t i = 8;
    uint8_t byte = 0;

    SDA_H;
    while (i--) 
    {
        byte <<= 1;
        SCL_L;
        I2C_delay();
        SCL_H;
        I2C_delay_half();
        if (SDA_read) 
        {
            byte |= 0x01;
        }
    }
    SCL_L;
    return byte;
}

int I2C_SoftWare_Master_Write(uint8_t DeviceAddr, uint8_t RegAddr, uint8_t* pBuffer, uint16_t NumByteToWrite)
{
    int i;
    if (!I2C_Start())
        return I2C_SoftWare_Master_ReInit();
    I2C_SendByte(DeviceAddr | I2C_DIRECTION_TRANSMITTER);
    if (!I2C_WaitAck())
    {
        I2C_Stop();
        return -1;
    }
    
    I2C_SendByte(RegAddr);
    I2C_WaitAck();
    
    for (i = 0; i < NumByteToWrite; i++) 
    {
        I2C_SendByte(pBuffer[i]);
        if (!I2C_WaitAck()) 
	{
            I2C_Stop();
            return I2C_SoftWare_Master_ReInit();
        }
    }
    
    I2C_Stop();
    
    return 0; 
}

int I2C_SoftWare_Master_Read(uint8_t DeviceAddr, uint8_t RegAddr, uint8_t* pBuffer, uint16_t NumByteToRead)
{
    if (!I2C_Start())
        return I2C_SoftWare_Master_ReInit();
    I2C_SendByte(DeviceAddr | I2C_DIRECTION_TRANSMITTER);
    if (!I2C_WaitAck())
    {
        I2C_Stop();
        return I2C_SoftWare_Master_ReInit();
    }
    I2C_SendByte(RegAddr);
    I2C_WaitAck();
    I2C_Start();
    I2C_SendByte(DeviceAddr | I2C_DIRECTION_RECEIVER);
    I2C_WaitAck();
    while (NumByteToRead) 
    {
        *pBuffer = I2C_ReceiveByte();
        if (NumByteToRead == 1)
            I2C_NoAck();
        else
            I2C_Ack();
        pBuffer++;
        NumByteToRead--;
    }
    I2C_Stop();
    
    return 0;
}

int I2C_SoftWare_Master_ReInit(void)
{
    I2C_SoftWare_Master_Init();
  
    return -1;
}
  
