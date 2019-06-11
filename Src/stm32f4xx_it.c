/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
 
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern UART_HandleTypeDef huart1;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */ 
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles TIM1 update interrupt and TIM10 global interrupt.
  */
void TIM1_UP_TIM10_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_UP_TIM10_IRQn 0 */
	HAL_TIM_Base_Stop_IT(&htim2);
	HAL_TIM_Base_Stop_IT(&htim3);
  /* USER CODE END TIM1_UP_TIM10_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_UP_TIM10_IRQn 1 */
  for(i=0;i<3;i++)
  {
	  HAL_I2C_Master_Transmit(&hi2c1,AHT10_Address,&AHT10_MeasureCmd[i],sizeof(AHT10_MeasureCmd[i]),10000);		//å‘é?èŽ·å–æ•°å€¼æŒ‡ä»?
  }
  HAL_Delay(1000);
  HAL_I2C_Master_Receive(&hi2c1,AHT10_Address,&AHT10_Data,sizeof(AHT10_Data),10000);
  temperture=((AHT10_Data[1] << 16) | (AHT10_Data[2] << 8) | AHT10_Data[3]) >> 4;
  temperture=temperture * 100 / 1048576;
  RH=((AHT10_Data[3] & 0x0F) << 16) | (AHT10_Data[4] << 8) | AHT10_Data[5];
  RH=((200 * RH) / 1048576) - 50;
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_Base_Start_IT(&htim3);
  /* USER CODE END TIM1_UP_TIM10_IRQn 1 */
}

/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */
	HAL_TIM_Base_Stop_IT(&htim1);
	HAL_TIM_Base_Stop_IT(&htim3);
  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */
  for(i=0;i<3;i++)
  {
	  HAL_I2C_Master_Receive(&hi2c2,GY30_Address,&GY30_Data[i],sizeof(GY30_Data[i]),10000);
  }
  light=GY30_Data[0];
  light=(light<<8)+GY30_Data[1];
  HAL_TIM_Base_Start_IT(&htim1);
  HAL_TIM_Base_Start_IT(&htim3);
  /* USER CODE END TIM2_IRQn 1 */
}

/**
  * @brief This function handles TIM3 global interrupt.
  */
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */
<<<<<<< HEAD
	uint8_t temp=0xFF;
	uint8_t crc=0xFF;
	uint8_t crc_init=0xFF;														//CRC初始值
=======
  uint8_t temp=0xFF;
	uint8_t crc=0xFF;
	uint8_t crc_init=0xFF;														//CRCåˆå§‹å€¼
>>>>>>> branch 'Sensors' of https://github.com/ly0koS/EnvControl-Software.git
	uint8_t crc_bit;
	j=j+1;
	HAL_TIM_Base_Stop_IT(&htim1);
	HAL_TIM_Base_Stop_IT(&htim2);
  /* USER CODE END TIM3_IRQn 0 */
  HAL_TIM_IRQHandler(&htim3);
  /* USER CODE BEGIN TIM3_IRQn 1 */
  if(j>=16)
  {
	  HAL_I2C_Master_Receive(&hi2c3,SGP30_Address,&SGP30_Data,24,10000);
	  SGP30_Data=SGP30_Data>>8;													//å°†ç©ºçš„ä½Ž8ä½?0ç§»é™¤
	  crc&=SGP30_Data;															//å?8ä½æ ¡éªŒä½
	  SGP30_Data=SGP30_Data>>8;													//å°†ä½Ž8ä½çš„CRCç§»é™¤
	  temp&=SGP30_Data;															//å?8ä½åŒ–å­¦æ±¡æŸ“æµ“åº?
	  SGP30_Data=SGP30_Data>>8;													//å°†ä½Ž8ä½çš„åŒ–å­¦æ±¡æŸ“æµ“åº¦ç§»é™¤
	  co2&=SGP30_Data;															//å?8ä½äºŒæ°§åŒ–ç¢³æµ“åº?
	  crc_init ^= co2;
	  for(crc_bit=8;crc_bit>0;--crc_bit)
	  {
		  if(crc_init&0x80)
			  crc_init=(crc_init<<1)^0x31;										//0x31æ ¡éªŒå¤šé¡¹å¼
		  else
			  crc_init=(crc_init<<1);
	  }
	  crc_init ^= temp;
	  for(crc_bit=8;crc_bit>0;--crc_bit)
	  {
		  if(crc_init&0x80)
			  crc_init=(crc_init<<1)^0x31;										//0x31æ ¡éªŒå¤šé¡¹å¼
		  else
			  crc_init=(crc_init<<1);
	  }
	  if(crc_init!=crc)
	  {
		  co2=0x00;
		  temp=0x00;
	  }
	  j=15;
	HAL_TIM_Base_Start_IT(&htim1);
	HAL_TIM_Base_Start_IT(&htim2);
  }
  /* USER CODE END TIM3_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */
	HAL_Delay(500);
	HAL_UART_Transmit(&huart1,(uint8_t *)temperture,sizeof(temperture),10000);
	HAL_Delay(500);
  /* USER CODE END USART1_IRQn 0 */
	HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */
	HAL_UART_Transmit(&huart1,(uint8_t *)RH,sizeof(RH),10000);
	HAL_Delay(500);
	HAL_UART_Transmit(&huart1,(uint8_t *)light,sizeof(light),10000);
  /* USER CODE END USART1_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
