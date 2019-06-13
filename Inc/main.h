/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
uint8_t AHT10_CalibrateCmd[3];
uint8_t AHT10_MeasureCmd[3];
uint8_t AHT10_Data[6];
uint8_t GY30_Data[3];
uint16_t SGP30_init;
uint16_t SGP30_measure;
uint32_t SGP30_Data;
float temperture;
float RH;
uint8_t co2;
uint8_t light;
uint8_t i,j;
uint8_t GY30_POWERON;
uint8_t GY30_CHRM;
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define I2C2_SDA_Pin GPIO_PIN_0
#define I2C2_SDA_GPIO_Port GPIOF
#define I2C2_SCL_Pin GPIO_PIN_1
#define I2C2_SCL_GPIO_Port GPIOF
#define FSMC_A6_Pin GPIO_PIN_12
#define FSMC_A6_GPIO_Port GPIOF
#define FSMC_D4_Pin GPIO_PIN_7
#define FSMC_D4_GPIO_Port GPIOE
#define FSMC_D5_Pin GPIO_PIN_8
#define FSMC_D5_GPIO_Port GPIOE
#define FSMC_D6_Pin GPIO_PIN_9
#define FSMC_D6_GPIO_Port GPIOE
#define FSMC_D7_Pin GPIO_PIN_10
#define FSMC_D7_GPIO_Port GPIOE
#define FSMC_D8_Pin GPIO_PIN_11
#define FSMC_D8_GPIO_Port GPIOE
#define FSMC_D9_Pin GPIO_PIN_12
#define FSMC_D9_GPIO_Port GPIOE
#define FSMC_D10_Pin GPIO_PIN_13
#define FSMC_D10_GPIO_Port GPIOE
#define FSMC_D11_Pin GPIO_PIN_14
#define FSMC_D11_GPIO_Port GPIOE
#define FSMC_D12_Pin GPIO_PIN_15
#define FSMC_D12_GPIO_Port GPIOE
#define FSMC_D13_Pin GPIO_PIN_8
#define FSMC_D13_GPIO_Port GPIOD
#define FSMC_D14_Pin GPIO_PIN_9
#define FSMC_D14_GPIO_Port GPIOD
#define FSMC_D15_Pin GPIO_PIN_10
#define FSMC_D15_GPIO_Port GPIOD
#define FSMC_D0_Pin GPIO_PIN_14
#define FSMC_D0_GPIO_Port GPIOD
#define FSMC_D1_Pin GPIO_PIN_15
#define FSMC_D1_GPIO_Port GPIOD
#define I2C3_SDA_Pin GPIO_PIN_9
#define I2C3_SDA_GPIO_Port GPIOC
#define I2C3_SCL_Pin GPIO_PIN_8
#define I2C3_SCL_GPIO_Port GPIOA
#define FSMC_D2_Pin GPIO_PIN_0
#define FSMC_D2_GPIO_Port GPIOD
#define FSMC_D3_Pin GPIO_PIN_1
#define FSMC_D3_GPIO_Port GPIOD
#define FSMC_D17_Pin GPIO_PIN_4
#define FSMC_D17_GPIO_Port GPIOD
#define FSMC_D18_Pin GPIO_PIN_5
#define FSMC_D18_GPIO_Port GPIOD
#define FSMC_D16_Pin GPIO_PIN_12
#define FSMC_D16_GPIO_Port GPIOG
#define I2C1_SCL_Pin GPIO_PIN_6
#define I2C1_SCL_GPIO_Port GPIOB
#define I2C1_SDA_Pin GPIO_PIN_7
#define I2C1_SDA_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
#define AHT10_Address (0x38<<1)
#define GY30_Address (0x46<<1)
#define SGP30_Address (0x58<<1)
#define SGP30_measure 0x2008
#define I2C2_SDA_Pin GPIO_PIN_0
#define I2C2_SDA_GPIO_Port GPIOF
#define I2C2_SCL_Pin GPIO_PIN_1
#define I2C2_SCL_GPIO_Port GPIOF
#define FSMC_A6_Pin GPIO_PIN_12
#define FSMC_A6_GPIO_Port GPIOF
#define FSMC_D4_Pin GPIO_PIN_7
#define FSMC_D4_GPIO_Port GPIOE
#define FSMC_D5_Pin GPIO_PIN_8
#define FSMC_D5_GPIO_Port GPIOE
#define FSMC_D6_Pin GPIO_PIN_9
#define FSMC_D6_GPIO_Port GPIOE
#define FSMC_D7_Pin GPIO_PIN_10
#define FSMC_D7_GPIO_Port GPIOE
#define FSMC_D8_Pin GPIO_PIN_11
#define FSMC_D8_GPIO_Port GPIOE
#define FSMC_D9_Pin GPIO_PIN_12
#define FSMC_D9_GPIO_Port GPIOE
#define FSMC_D10_Pin GPIO_PIN_13
#define FSMC_D10_GPIO_Port GPIOE
#define FSMC_D11_Pin GPIO_PIN_14
#define FSMC_D11_GPIO_Port GPIOE
#define FSMC_D12_Pin GPIO_PIN_15
#define FSMC_D12_GPIO_Port GPIOE
#define FSMC_D13_Pin GPIO_PIN_8
#define FSMC_D13_GPIO_Port GPIOD
#define FSMC_D14_Pin GPIO_PIN_9
#define FSMC_D14_GPIO_Port GPIOD
#define FSMC_D15_Pin GPIO_PIN_10
#define FSMC_D15_GPIO_Port GPIOD
#define FSMC_D0_Pin GPIO_PIN_14
#define FSMC_D0_GPIO_Port GPIOD
#define FSMC_D1_Pin GPIO_PIN_15
#define FSMC_D1_GPIO_Port GPIOD
#define I2C3_SDA_Pin GPIO_PIN_9
#define I2C3_SDA_GPIO_Port GPIOC
#define I2C3_SCL_Pin GPIO_PIN_8
#define I2C3_SCL_GPIO_Port GPIOA
#define FSMC_D2_Pin GPIO_PIN_0
#define FSMC_D2_GPIO_Port GPIOD
#define FSMC_D3_Pin GPIO_PIN_1
#define FSMC_D3_GPIO_Port GPIOD
#define FSMC_D17_Pin GPIO_PIN_4
#define FSMC_D17_GPIO_Port GPIOD
#define FSMC_D18_Pin GPIO_PIN_5
#define FSMC_D18_GPIO_Port GPIOD
#define FSMC_D16_Pin GPIO_PIN_12
#define FSMC_D16_GPIO_Port GPIOG
#define I2C1_SCL_Pin GPIO_PIN_6
#define I2C1_SCL_GPIO_Port GPIOB
#define I2C1_SDA_Pin GPIO_PIN_7
#define I2C1_SDA_GPIO_Port GPIOB
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
