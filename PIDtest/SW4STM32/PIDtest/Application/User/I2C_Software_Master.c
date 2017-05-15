
#include "I2C_Software_Master.h"

#define PIN_POS GPIOB
#define SCL_PIN GPIO_PIN_6
#define SDA_PIN GPIO_PIN_9

#define SCL_H         HAL_GPIO_WritePin(PIN_POS, SCL_PIN, GPIO_PIN_SET)//GPIOB->BSRRL = GPIO_Pin_6
#define SCL_L         HAL_GPIO_WritePin(PIN_POS, SCL_PIN, GPIO_PIN_RESET)//GPIOB->BSRRH = GPIO_Pin_6

#define SDA_H         HAL_GPIO_WritePin(PIN_POS, SDA_PIN, GPIO_PIN_SET)//GPIOB->BSRRL = GPIO_Pin_9
#define SDA_L         HAL_GPIO_WritePin(PIN_POS, SDA_PIN, GPIO_PIN_RESET)//GPIOB->BSRRH = GPIO_Pin_9

#define SCL_read      PIN_POS->IDR & SCL_PIN
#define SDA_read      PIN_POS->IDR & SDA_PIN

#define I2C_DIRECTION_TRANSMITTER       ((uint8_t)0x00)
#define I2C_DIRECTION_RECEIVER          ((uint8_t)0x01)

void I2C_SoftWare_Master_Init(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    
    /*SDA GPIO clock enable */ //PB9
    //RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    RCC->AHB1ENR |= 1<<1;
  
    /*SCL GPIO clock enable */ //PB6
    //RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
  
    /* GPIO Configuration */
    /*Configure I2C SCL pin */
    GPIO_InitStructure.Pin = SCL_PIN;
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.Speed = GPIO_SPEED_FAST;
    //GPIO_InitStructure.OType = GPIO_OTYPE_OD;
    GPIO_InitStructure.Pull  = GPIO_PULLUP;
    HAL_GPIO_Init(PIN_POS, &GPIO_InitStructure);
    
    /*Configure I2C SDA pin */
    GPIO_InitStructure.Pin = SDA_PIN;
    HAL_GPIO_Init(PIN_POS, &GPIO_InitStructure);
    
    SCL_H;
    SDA_H;
    
    SCL_L;
    SDA_L;
    
    SCL_H;
    SDA_H;
}

	
void I2C_delay(void)
{
    volatile int i = 10;		  
    while (i){
        i--;
        __asm("nop");
    }
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
    I2C_delay();
    SDA_L;
    I2C_delay();
    SCL_H;
    I2C_delay();
    SDA_H;
    I2C_delay();
}


void I2C_Ack(void)
{
    SCL_L;
    I2C_delay();
    SDA_L;
    I2C_delay();
    SCL_H;
    I2C_delay();
    SCL_L;
    I2C_delay();
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
    I2C_delay();
    SDA_H;
    I2C_delay();
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
        I2C_delay();
        if (byte & 0x80)
            SDA_H;
        else
            SDA_L;
        byte <<= 1;
        I2C_delay();
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
        I2C_delay();
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
  
