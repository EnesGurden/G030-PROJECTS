/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32g0xx_it.c
  * @brief   Interrupt Service Routines.
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
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32g0xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */
uint8_t data[5], dataCounter = 0;




settings_t *settingPage = (settings_t*) SETTING_PAGE_ADDRESS;
settings_t tempSettings;

typedef union {
	uint64_t full;
	struct {
		uint32_t u32_1;
		uint32_t u32_2;
	};
	uint8_t arr[8];
} douWord;

douWord d2;

extern uint16_t wait;


void Flash_write()
{
	FLASH_EraseInitTypeDef eraseConfig;
	uint32_t sectorError;
	uint8_t pageNumber = (SETTING_PAGE_ADDRESS - FLASH_BASE)/FLASH_PAGE_SIZE;
	eraseConfig.TypeErase = FLASH_TYPEERASE_PAGES;
	eraseConfig.Page = pageNumber;
	eraseConfig.NbPages = 1;
	eraseConfig.Banks =1;

//	FLASH->SR &= ~((1<<5)|(1<<7)|(1<<18));
	HAL_FLASH_Unlock();
	HAL_FLASHEx_Erase(&eraseConfig, &sectorError);

	d2.u32_1 = tempSettings.appByte;
	d2.u32_2 = tempSettings.validApp;
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, SETTING_PAGE_ADDRESS, d2.full);

	d2.u32_1 = tempSettings.size;
	d2.u32_2 = tempSettings.crc;
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, SETTING_PAGE_ADDRESS + 8, d2.full);

	for(int i=0;i<80;i++) // SETTING_PAGE_ADDRESS + 16 ile SETTING_PAGE_ADDRESS + 96 adresleri bootlader info icin ayrildi.
	{
		d2.arr[i%8]=tempSettings.bootloader_info[i];
		if(((i+1)%8)==0)
		{
			HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, SETTING_PAGE_ADDRESS + 16 + i - 7, d2.full);
		}
	}

	for(int i=0;i<80;i++)
	{
		d2.arr[i%8]=application_info[i];
		if(((i+1)%8)==0)
		{
			HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, SETTING_PAGE_ADDRESS + 96 + i - 7, d2.full);
		}
	}

	d2.u32_1 = tempSettings.locks;
	d2.u32_2 = tempSettings.leds;
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, SETTING_PAGE_ADDRESS + 176, d2.full);


	HAL_FLASH_Lock();
}
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

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M0+ Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
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
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVC_IRQn 0 */

  /* USER CODE END SVC_IRQn 0 */
  /* USER CODE BEGIN SVC_IRQn 1 */

  /* USER CODE END SVC_IRQn 1 */
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
/* STM32G0xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32g0xx.s).                    */
/******************************************************************************/

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
