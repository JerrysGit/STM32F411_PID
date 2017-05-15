
#include "stm32f4xx_hal.h"
#include <string.h>
#include <stdint.h>

void I2C_delay(void);
uint8_t I2C_Start(void);
void I2C_Stop(void);
void I2C_Ack(void);
void I2C_NoAck(void);
uint8_t I2C_WaitAck(void);
void I2C_SendByte(uint8_t byte);
uint8_t I2C_ReceiveByte(void);
void I2C_SoftWare_Master_Init(void);
int I2C_SoftWare_Master_Write(uint8_t DeviceAddr, uint8_t RegAddr, uint8_t* pBuffer, uint16_t NumByteToWrite);
int I2C_SoftWare_Master_Read(uint8_t DeviceAddr, uint8_t RegAddr, uint8_t* pBuffer, uint16_t NumByteToRead);
int I2C_SoftWare_Master_ReInit(void);
