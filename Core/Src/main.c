/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
const char application_info[80] = "G030 NCP0202 UART and SR_DATA&SR_CLK Revision 1.0 Date: 28/08/2023  ";
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
RTC_HandleTypeDef hrtc;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_RTC_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void Flash_write();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
extern settings_t *settingPage;
extern settings_t tempSettings;

typedef union
{
	uint16_t Data;
	struct
	{
		uint8_t LED_ON : 1;
		uint8_t LED_BR_LOCK : 1;
		uint8_t LED_LR_LOCK : 1;
		uint8_t LED_HL_LOCK : 1;
		uint8_t LED_ALL_LOCK : 1;
		uint8_t RESERVED : 3;
		uint8_t RESERVED1 : 1;
		uint8_t LED_HL_LOWEST : 1;
		uint8_t RESERVED2 : 1;
		uint8_t LED_PLUG  :1;
		uint8_t LED_BATT : 4;
	};
} payload_SR_Led_struct;
payload_SR_Led_struct payload_SR_Led = {.Data=0};

uint8_t once = 1;
#define ENABLE_UART 0
#define ENABLE_SHIFT 1
void setUartPins(uint8_t enable)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	switch(enable)
	{
	case ENABLE_UART:
		if( once == 1)
		{
			once=0;
			HAL_GPIO_DeInit(GPIOA, GPIO_PIN_3|GPIO_PIN_2);
			MX_USART2_UART_Init();
		}
		break;
	case ENABLE_SHIFT:
		if( once == 0)
		{
			once=1;
			HAL_UART_DeInit(&huart2);
      /*Configure GPIO pin Output Level */
  	  HAL_GPIO_WritePin(GPIOA, SR_DATA_Pin, GPIO_PIN_SET);
  	  /*Configure GPIO pin Output Level */
  	  HAL_GPIO_WritePin(GPIOA, SR_CLK_Pin, GPIO_PIN_RESET);
  	  /*Configure GPIO pins : SR_DATA_Pin SR_CLK_Pin */
  	  GPIO_InitStruct.Pin = SR_DATA_Pin|SR_CLK_Pin;
  	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  	  GPIO_InitStruct.Pull = GPIO_NOPULL;
  	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  	  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
		}
		break;
	default:
		break;
	}
}

void writeShift()
{
  setUartPins(ENABLE_SHIFT);
  HAL_GPIO_WritePin(SR_OE_GPIO_Port, SR_OE_Pin, GPIO_PIN_SET);
  for (int i = 15; i >= 0; i--)
  {

    uint8_t bit = (payload_SR_Led.Data & (1 << i)) >> i == 1 ? 1 : 0;
	HAL_GPIO_WritePin(SR_DATA_GPIO_Port, SR_DATA_Pin, bit);
	HAL_GPIO_WritePin(SR_CLK_GPIO_Port, SR_CLK_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(SR_CLK_GPIO_Port, SR_CLK_Pin, GPIO_PIN_SET);
  }

	HAL_GPIO_WritePin(SR_LATCH_GPIO_Port, SR_LATCH_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(SR_LATCH_GPIO_Port, SR_LATCH_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(SR_LATCH_GPIO_Port, SR_LATCH_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(SR_OE_GPIO_Port, SR_OE_Pin, GPIO_PIN_RESET);
  setUartPins(ENABLE_UART);
}



__RAM_FUNC void Activate_RDP_1_Go_to_Standby(void)
{
      //HAL_FLASH_OB_Launch();
	  SET_BIT(FLASH->CR, FLASH_CR_OPTSTRT);

      //HAL_PWR_EnterSTANDBYMode;
	  SET_BIT(PWR->CR1, PWR_CR1_FPD_LPSLP);

	  /* Set SLEEPDEEP bit of Cortex System Control Register */
	  SET_BIT(SCB->SCR, ((uint32_t)SCB_SCR_SLEEPDEEP_Msk));

	  /* This option is used to ensure that store operations are completed */

	  /* Request Wait For Interrupt */
	  __WFI();

}

void setOptionBytes(void)
{
	if(HAL_FLASH_Unlock() == HAL_OK)
	  {
		  if (HAL_FLASH_OB_Unlock() == HAL_OK)
		  {
			  FLASH_OBProgramInitTypeDef pOBInit;
			  HAL_FLASHEx_OBGetConfig(&pOBInit);
			  if (pOBInit.RDPLevel != OB_RDP_LEVEL_1)
			  {
				  pOBInit.OptionType = OPTIONBYTE_RDP | OPTIONBYTE_USER ;
				  pOBInit.RDPLevel = OB_RDP_LEVEL_1;
//				  pOBInit.USERConfig ^= (1<<26); // |(1<<14) //standby:14, nboot0:26
//				  pOBInit.USERConfig &= ~(1<<14); /* unchecked nRST_STDBY, programlandiktan sonra okumana izin veriyor, seri programlamada silinebilir */
				  HAL_FLASHEx_OBProgram(&pOBInit);

				  HAL_SuspendTick();
			 	  HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);

			 	  /*## Clear all related wakeup flags ########################################*/
			 	  /* Clear PWR wake up Flag */
			 	  __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WUF);

			 	  /* Clear RTC Wake Up timer Flag */
			 	  __HAL_RTC_WAKEUPTIMER_CLEAR_FLAG(&hrtc, RTC_FLAG_WUTF);

			 	  /*## Setting the Wake up time ##############################################*/
			 	  HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, 0x1017, RTC_WAKEUPCLOCK_RTCCLK_DIV16);

				  Activate_RDP_1_Go_to_Standby();
			  }
			  else
			  {

			  }
			  HAL_FLASH_OB_Lock();
		  }
	    HAL_FLASH_Lock();
	  }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
//  NVIC_SystemReset();
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_RTC_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
//  HAL_DBGMCU_DisableDBGStandbyMode();

  tempSettings = *settingPage;
  tempSettings.validApp = 0x02;
  Flash_write();

//  setOptionBytes();
//  HAL_UART_Transmit(&huart2, (uint8_t*)settingPage, 1000, 100);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  payload_SR_Led.Data=0;
	  writeShift();
	  HAL_Delay(500);
	  payload_SR_Led.Data=0xfff;
	  writeShift();
	  HAL_Delay(500);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the peripherals clocks
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  hrtc.Init.OutPutPullUp = RTC_OUTPUT_PULLUP_NONE;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */
  __HAL_UART_ENABLE_IT(&huart2,UART_IT_RXFNE);
  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SR_LATCH_GPIO_Port, SR_LATCH_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SR_OE_GPIO_Port, SR_OE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : SR_LATCH_Pin */
  GPIO_InitStruct.Pin = SR_LATCH_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SR_LATCH_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SR_OE_Pin */
  GPIO_InitStruct.Pin = SR_OE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SR_OE_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
