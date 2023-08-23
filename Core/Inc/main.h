/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
extern const char application_info[80];
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
void saveAppByte(void);

#define SETTING_PAGE_ADDRESS 0x08007680
//#define APP_ADDRESS 0x08002000

typedef struct settings {
	uint32_t appByte;
	uint32_t validApp;
	uint32_t size;
	uint32_t crc;
	uint8_t bootloader_info[80];
	uint8_t application_info[80];
	uint32_t locks;
	uint32_t leds;
} settings_t;
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SR_LATCH_Pin GPIO_PIN_15
#define SR_LATCH_GPIO_Port GPIOC
#define SR_DATA_Pin GPIO_PIN_2
#define SR_DATA_GPIO_Port GPIOA
#define SR_CLK_Pin GPIO_PIN_3
#define SR_CLK_GPIO_Port GPIOA
#define SR_OE_Pin GPIO_PIN_13
#define SR_OE_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
