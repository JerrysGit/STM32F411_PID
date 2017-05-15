/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <math.h>
#include "matrix.h"
#include "I2C_Software_Master.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;

osThreadId ReadSensorHandle;
osThreadId AttitudeCalHandle;
osThreadId PrintDataHandle;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM4_Init(void);
void ReadSensorLoop(void const * argument);
void AttitudeCalLoop(void const * argument);
void PrintDataLoop(void const * argument);
                                    
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
void GyroInit();
void GyroWriteOne(uint8_t address, uint8_t data);
void GyroReadData(uint8_t address, int readAmount, uint8_t *databuf);
void AccInit();
void user_pwm_setvalue(uint16_t value, uint32_t ch);
void servoControl(char axis, float conAngle);
void moveCommand(float AskedRoll, float AskedPitch);
int8_t u2txbuf[100];
#define print2UART(...) sprintf(u2txbuf, __VA_ARGS__);HAL_UART_Transmit(&huart2, u2txbuf, strlen((char *)u2txbuf), 1000);
#define AccAddress 0x32

char DataIsReady = 0;
uint8_t GyroRawData[6], AccRawData[6];
uint8_t RxBuf;
int16_t GyroOffset[3]={0},AccOffset[3]={0};
uint16_t pwmDutyX, pwmDutyY;
uint32_t time_s,time;
float Roll=.0f, Pitch=.0f, Yaw=.0f;
float Roll_e[3] = {0}, Pitch_e[3] = {0};
Matrix(GyroDCM,3,3);
Matrix(AccDCM,3,3);
typedef union _f2b{
	float F;
	uint8_t b[4];
}F2B;

F2B setRoll, setPitch;

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  MX_TIM4_Init();

  /* USER CODE BEGIN 2 */
  setRoll.F = setPitch.F = .0f;
  I2C_SoftWare_Master_Init();
  HAL_GPIO_WritePin(GPIOD, LD4_Pin, GPIO_PIN_RESET);
  pwmDutyX = pwmDutyY = 400;
  user_pwm_setvalue(pwmDutyX, TIM_CHANNEL_2);
  user_pwm_setvalue(pwmDutyY, TIM_CHANNEL_3);
  HAL_Delay(500);
  GyroInit();
  AccInit();
  eye_Matrix(GyroDCM, 3);
  HAL_GPIO_WritePin(GPIOD, LD4_Pin, GPIO_PIN_SET);
  HAL_UART_Receive_IT(&huart2, &RxBuf, 1);
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of ReadSensor */
  osThreadDef(ReadSensor, ReadSensorLoop, osPriorityHigh, 0, 1024);
  ReadSensorHandle = osThreadCreate(osThread(ReadSensor), NULL);

  /* definition and creation of AttitudeCal */
  osThreadDef(AttitudeCal, AttitudeCalLoop, osPriorityAboveNormal, 0, 1024);
  AttitudeCalHandle = osThreadCreate(osThread(AttitudeCal), NULL);

  /* definition and creation of PrintData */
  osThreadDef(PrintData, PrintDataLoop, osPriorityLow, 0, 1024);
  PrintDataHandle = osThreadCreate(osThread(PrintData), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
 

  /* Start scheduler */
  osKernelStart();
  
  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 200;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV8;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

/* SPI1 init function */
static void MX_SPI1_Init(void)
{

  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }

}

/* TIM4 init function */
static void MX_TIM4_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 100;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 5000;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 100;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.Pulse = 1000;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_TIM_MspPostInit(&htim4);

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
     PC3   ------> I2S2_SD
     PA4   ------> I2S3_WS
     PB10   ------> I2S2_CK
     PC7   ------> I2S3_MCK
     PA9   ------> USB_OTG_FS_VBUS
     PA10   ------> USB_OTG_FS_ID
     PA11   ------> USB_OTG_FS_DM
     PA12   ------> USB_OTG_FS_DP
     PC10   ------> I2S3_CK
     PC12   ------> I2S3_SD
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin 
                          |Audio_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : CS_I2C_SPI_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_I2C_SPI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PDM_OUT_Pin */
  GPIO_InitStruct.Pin = PDM_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(PDM_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : I2S3_WS_Pin */
  GPIO_InitStruct.Pin = I2S3_WS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(I2S3_WS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CLK_IN_Pin */
  GPIO_InitStruct.Pin = CLK_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(CLK_IN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin 
                           Audio_RST_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin 
                          |Audio_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : I2S3_MCK_Pin I2S3_SCK_Pin I2S3_SD_Pin */
  GPIO_InitStruct.Pin = I2S3_MCK_Pin|I2S3_SCK_Pin|I2S3_SD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : VBUS_FS_Pin */
  GPIO_InitStruct.Pin = VBUS_FS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(VBUS_FS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : OTG_FS_ID_Pin OTG_FS_DM_Pin OTG_FS_DP_Pin */
  GPIO_InitStruct.Pin = OTG_FS_ID_Pin|OTG_FS_DM_Pin|OTG_FS_DP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MEMS_INT2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void GyroInit(){
	uint8_t RawData[6];
	int16_t temp16[3];
	GyroWriteOne(0x20, 0x0F);
	for(int i=0;i<100;i++){
		GyroReadData(0x28+0xC0, 6, RawData);
		temp16[0] = (((int16_t)RawData[1]) <<8 ) + RawData[0];
		temp16[1] = (((int16_t)RawData[3]) <<8 ) + RawData[2];
		temp16[2] = (((int16_t)RawData[5]) <<8 ) + RawData[4];

		GyroOffset[0] += temp16[0];
		GyroOffset[1] += temp16[1];
		GyroOffset[2] += temp16[2];

		HAL_Delay(5);
	}
	for(int i=0;i<3;i++)
		GyroOffset[i] /=  100;
}

void GyroWriteOne(uint8_t address, uint8_t data){
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3,GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, &address, 1, 50);
	HAL_SPI_Transmit(&hspi1, &data, 1, 50);
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3,GPIO_PIN_SET);
}

void GyroReadData(uint8_t address, int readAmount, uint8_t *databuf){
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3,GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, &address, 1, 50);
	HAL_SPI_Receive(&hspi1, databuf, readAmount, 50);
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3,GPIO_PIN_SET);
}

void AccInit(){
	//Turn on the Acc by send 0x27 to address 0x20
	uint8_t data = 0x27;
	I2C_SoftWare_Master_Write(AccAddress, 0x20, &data, 1);
}

void user_pwm_setvalue(uint16_t value, uint32_t ch){
	TIM_OC_InitTypeDef sConfigOC;

	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = value;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, ch);
	HAL_TIM_PWM_Start(&htim4, ch);
}

void servoControl(char axis, float conAngle)
{
	if(conAngle >= -8.0f && conAngle <= 8.0f)
	{
		int16_t conDuty = (int16_t)(conAngle * 25.0f);
		switch(axis){
		case 'x':
			pwmDutyX += conDuty;
			if(pwmDutyX > 600)
				pwmDutyX = 600;
			else if(pwmDutyX < 200)
				pwmDutyX = 200;
			user_pwm_setvalue(pwmDutyX, TIM_CHANNEL_2);
			break;
		case 'y':
			pwmDutyY += conDuty;
			if(pwmDutyY > 600)
				pwmDutyY = 600;
			else if(pwmDutyY < 200)
				pwmDutyY = 200;
			user_pwm_setvalue(pwmDutyY, TIM_CHANNEL_3);
			break;
		}
	}

}

void moveCommand(float AskedRoll, float AskedPitch)
{
	Roll_e[0] = Roll - AskedRoll;
	Pitch_e[0] = Pitch - AskedPitch;
	float Kp = 0.01f, Ki = 0.4f, Kd = 0.02f;
	float P_Roll, I_Roll, D_Roll, P_Pitch, I_Pitch, D_Pitch;
	P_Roll = Roll_e[0] - Roll_e[1]; I_Roll = Roll_e[0]; D_Roll = Roll_e[0] - 2 * Roll_e[1] + Roll_e[2];
	P_Pitch = Pitch_e[0] - Pitch_e[1]; I_Pitch = Pitch_e[0]; D_Pitch = Pitch_e[0] - 2 * Pitch_e[1] + Pitch_e[2];
	float RollCommand, PitchCommand;
	float AngleError = 0.3f;

	RollCommand = Kp * P_Roll + Ki * I_Roll + Kd * D_Roll;
	PitchCommand = Kp * P_Pitch + Ki * I_Pitch + Kd * D_Pitch;

	Roll_e[2] = Roll_e[1]; Roll_e[1] = Roll_e[0];
	Pitch_e[2] = Pitch_e[1]; Pitch_e[1] = Pitch_e[0];

	if(RollCommand > AngleError || RollCommand < -AngleError)
		servoControl('x', RollCommand);
	if(PitchCommand > AngleError || PitchCommand < -AngleError)
		servoControl('y', PitchCommand);
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef * huart)
{
	static uint8_t RxState = 0;
	static uint8_t dataCount = 0;
	if(huart->Instance == huart2.Instance)
	{
		switch(RxState)
		{
		case 0:
			if(RxBuf == '^')
			{
				HAL_GPIO_WritePin(GPIOD, LD5_Pin, GPIO_PIN_SET);
				RxState++;
			}
			break;
		case 1:
			if(RxBuf == 'A')
			{
				HAL_GPIO_WritePin(GPIOD, LD6_Pin, GPIO_PIN_SET);
				RxState++;
			}
			else
			{
				HAL_GPIO_WritePin(GPIOD, LD5_Pin, GPIO_PIN_RESET);
				RxState = 0;
			}
			break;
		case 2:
			if(RxBuf == 'R')
				RxState++;
			else if(RxBuf == 'P')
				RxState = 4;
			else
			{
				HAL_GPIO_WritePin(GPIOD, LD5_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOD, LD6_Pin, GPIO_PIN_RESET);
				RxState = 0;
			}
			break;
		case 3:  //catch roll
			setRoll.b[dataCount] = RxBuf;
			dataCount++;
			if(dataCount == 4)
			{
				dataCount = 0;
				RxState = 2;
			}
			break;
		case 4:  //catch Pitch
			setPitch.b[dataCount] = RxBuf;
			dataCount++;
			if(dataCount == 4)
			{
				dataCount = 0;
				RxState = 2;
			}
			break;

		}
	}
	HAL_UART_Receive_IT(&huart2, &RxBuf, 1);
}
/* USER CODE END 4 */

/* ReadSensorLoop function */
void ReadSensorLoop(void const * argument)
{

  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
//	time_s = xTaskGetTickCount();
	GyroReadData(0x28+0xC0, 6, GyroRawData);

	uint8_t address = 0x28;
	for(int i=0;i<6;i++){
		I2C_SoftWare_Master_Read(AccAddress, address, &AccRawData[i], 1);
		address++;
	}


	osDelay(1);
//	time_e = xTaskGetTickCount();
//	time = time_e - time_s;
  }
  /* USER CODE END 5 */ 
}

/* AttitudeCalLoop function */
void AttitudeCalLoop(void const * argument)
{
  /* USER CODE BEGIN AttitudeCalLoop */
  /* Infinite loop */
  int16_t GyroData[3], AccData[3];
  float GyroOmega[3],Gravity[3],AccRoll,AccPitch;
  for(;;)
  {
//	time_s = xTaskGetTickCount();
		GyroData[0] = (((int16_t)GyroRawData[1]) <<8 ) + GyroRawData[0] - GyroOffset[0];
		GyroData[1] = (((int16_t)GyroRawData[3]) <<8 ) + GyroRawData[2] - GyroOffset[1];
		GyroData[2] = (((int16_t)GyroRawData[5]) <<8 ) + GyroRawData[4] - GyroOffset[2];

		for(int i=0;i<3;i++)
			GyroOmega[i] = GyroData[i] * 250.0 / 32767.0;

		float dt = 0.014;
		gyroRot(GyroDCM, GyroOmega[0] * dt, GyroOmega[1] * dt, GyroOmega[2] * dt);

		AccData[0] = (((int16_t)AccRawData[1]) << 8) + AccRawData[0];// - AccOffset[0];
		AccData[1] = (((int16_t)AccRawData[3]) << 8) + AccRawData[2];// - AccOffset[1];
		AccData[2] = (((int16_t)AccRawData[5]) << 8) + AccRawData[4];// - AccOffset[2];

		for(int i=0;i<3;i++)
			Gravity[i] = (((float)AccData[i]) / 32767.0) * 2.0;

		AccPitch = atan2(Gravity[0],sqrt(Gravity[1]*Gravity[1] + Gravity[2]*Gravity[2])) * 180.0 / M_PI;
		if(Gravity[2] > 0)
			AccRoll = atan2(-1.0f * Gravity[1], sqrt(Gravity[2]*Gravity[2] + 0.1f * Gravity[0]*Gravity[0])) * 180.0 / M_PI;
		else
			AccRoll = atan2(-1.0f * Gravity[1], -1.0f * sqrt(Gravity[2]*Gravity[2] + 0.01f * Gravity[0]*Gravity[0])) * 180.0 / M_PI;

		eulRot(AccDCM, AccRoll, AccPitch, Yaw);

		float a = 0.98;
		for(int i=0;i<9;i++)
			GyroDCM[i] = GyroDCM[i] * a + AccDCM[i] * (1 - a);

    osDelay(10);
//    time = xTaskGetTickCount() - time_s;
  }
  /* USER CODE END AttitudeCalLoop */
}

/* PrintDataLoop function */
void PrintDataLoop(void const * argument)
{
  /* USER CODE BEGIN PrintDataLoop */
  /* Infinite loop */
  for(;;)
  {
	mat2ang(GyroDCM, &Roll, &Pitch, &Yaw);
//	mat2ang(AccDCM, &Roll, &Pitch, &Yaw);

	F2B Roll2Send, Pitch2Send, Yaw2Send;
	Roll2Send.F = Roll;
	Pitch2Send.F = Pitch;
//	Yaw2Send.F = Yaw;
//	Roll2Send.F = C_Roll;
//	Pitch2Send.F = C_Pitch;

	uint8_t dataLength = 16;
	uint8_t DataToSend[dataLength];
	for(int i=3;i<7;i++)
		DataToSend[i] = Roll2Send.b[i-3];
	for(int i=7;i<11;i++)
		DataToSend[i] = Pitch2Send.b[i-7];
	for(int i=11;i<15;i++)
			DataToSend[i] = Yaw2Send.b[i-11];
	DataToSend[0] = '^';
	DataToSend[1] = dataLength;
	DataToSend[2] = 'A';

	DataToSend[dataLength-1] = 0;
	for(int i=2;i<dataLength-1;i++)
		DataToSend[dataLength-1] += DataToSend[i];

	HAL_UART_Transmit(&huart2, DataToSend, dataLength, 1000);
//	HAL_UART_Transmit(&huart2, RxBuf, 1, 1000);

	moveCommand(setRoll.F, setPitch.F);

    osDelay(100);
  }
  /* USER CODE END PrintDataLoop */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
