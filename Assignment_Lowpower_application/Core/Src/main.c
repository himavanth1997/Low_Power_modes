/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define TABLE_OUTLOOK 0
#define TABLE_TEMP 1

#define SET1_NO_OF_CATAGORYS 3
#define SET1_NO_OF_CLASSIFICATION 2

#define SET2_NO_OF_CATAGORYS 3
#define SET2_NO_OF_CLASSIFICATION 2

#define TRAINING_DATA_SET_SIZE 0

#define SET1_SUNNY 0
#define SET1_OVERCAST 1
#define SET1_RAINY 2

#define SET2_HOT 0
#define SET2_MILD 1
#define SET2_COOL 2

#define YES 0
#define NO 1

/*  outlook and temp assumption prediction tables*/
int OUTLOOK_TABLE[SET1_NO_OF_CATAGORYS][SET1_NO_OF_CLASSIFICATION];
int TEMP_TABLE[SET2_NO_OF_CATAGORYS][SET2_NO_OF_CLASSIFICATION];

int outlook_yes_records = 0;
int outlook_no_records = 0;
int temp_yes_records = 0;
int temp_no_records = 0;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
RTC_HandleTypeDef hrtc;

/* USER CODE BEGIN PV */
static __IO uint32_t TimingDelay;
GPIO_InitTypeDef GPIO_InitStructure;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_RTC_Init(void);
/* USER CODE BEGIN PFP */
static void SYSCLKConfig_STOP(void);
static void RTC_Config(void);
/* functions to fit the data */
void fit_data(int table_type, int feature, int prediction);
float calc_probabulity(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	float prob = 0.0;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */
  /* Configure RTC */
  RTC_Config();

  /* Enable Power Clock */
  __HAL_RCC_PWR_CLK_ENABLE();


  /* training sample for table 1*/
  fit_data(TABLE_OUTLOOK, SET1_SUNNY, YES);
  fit_data(TABLE_OUTLOOK, SET1_SUNNY, YES);
  fit_data(TABLE_OUTLOOK, SET1_SUNNY, NO);
  fit_data(TABLE_OUTLOOK, SET1_SUNNY, NO);
  fit_data(TABLE_OUTLOOK, SET1_SUNNY, NO);

  fit_data(TABLE_OUTLOOK, SET1_OVERCAST, YES);
  fit_data(TABLE_OUTLOOK, SET1_OVERCAST, YES);
  fit_data(TABLE_OUTLOOK, SET1_OVERCAST, YES);
  fit_data(TABLE_OUTLOOK, SET1_OVERCAST, YES);

  fit_data(TABLE_OUTLOOK, SET1_RAINY, YES);
  fit_data(TABLE_OUTLOOK, SET1_RAINY, YES);
  fit_data(TABLE_OUTLOOK, SET1_RAINY, YES);
  fit_data(TABLE_OUTLOOK, SET1_RAINY, NO);
  fit_data(TABLE_OUTLOOK, SET1_RAINY, NO);

  /* training sample for table 2 */
  fit_data(TABLE_TEMP, SET2_HOT, YES);
  fit_data(TABLE_TEMP, SET2_HOT, YES);
  fit_data(TABLE_TEMP, SET2_HOT, NO);
  fit_data(TABLE_TEMP, SET2_HOT, NO);

  fit_data(TABLE_TEMP, SET2_MILD, YES);
  fit_data(TABLE_TEMP, SET2_MILD, YES);
  fit_data(TABLE_TEMP, SET2_MILD, YES);
  fit_data(TABLE_TEMP, SET2_MILD, YES);
  fit_data(TABLE_TEMP, SET2_MILD, NO);
  fit_data(TABLE_TEMP, SET2_MILD, NO);

  fit_data(TABLE_TEMP, SET2_COOL, YES);
  fit_data(TABLE_TEMP, SET2_COOL, YES);
  fit_data(TABLE_TEMP, SET2_COOL, YES);
  fit_data(TABLE_TEMP, SET2_COOL, NO);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	    /* Enable GPIOs clock */
	    __HAL_RCC_GPIOA_CLK_ENABLE();
	    __HAL_RCC_GPIOB_CLK_ENABLE();
	    __HAL_RCC_GPIOC_CLK_ENABLE();
	    __HAL_RCC_GPIOH_CLK_ENABLE();

	    /* Set all GPIO in analog state to reduce power consumption,                */
	    /*   except GPIOC to keep user button interrupt enabled                     */
	    /* Note: Debug using ST-Link is not possible during the execution of this   */
	    /*       example because communication between ST-link and the device       */
	    /*       under test is done through UART. All GPIO pins are disabled (set   */
	    /*       to analog input mode) including  UART I/O pins.                    */
	    GPIO_InitStructure.Pin = GPIO_PIN_All;
	    GPIO_InitStructure.Mode = GPIO_MODE_ANALOG;
	    GPIO_InitStructure.Pull = GPIO_NOPULL;

	    HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);
	    HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);
	    HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);
	    HAL_GPIO_Init(GPIOH, &GPIO_InitStructure);

	    /* Disable GPIOs clock */
	    __HAL_RCC_GPIOA_CLK_DISABLE();
	    __HAL_RCC_GPIOB_CLK_DISABLE();
	    __HAL_RCC_GPIOC_CLK_DISABLE();
	    __HAL_RCC_GPIOH_CLK_DISABLE();

	    /* Disable all used wake up source */
	    HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);

	    /* Byee's Theorem   calculating probability for fixed case*/
	    prob = calc_probabulity();

	    if(prob > 0.5)
	    {
	    	/* playing yes */
	    }else
	    {
	    	/* not allowed to play */
	    }

	    __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
	      /* RTC Wake up Interrupt Generation:
	        the wake-up counter is set to its maximum value to yield the longuest
	        stop time to let the current reach its lowest operating point.
	        The maximum value is 0xFFFF, corresponding to about 33 sec. when
	        RTC_WAKEUPCLOCK_RTCCLK_DIV = RTCCLK_Div16 = 16

	        Wake up Time Base = (RTC_WAKEUPCLOCK_RTCCLK_DIV /(LSI))
	        Wake up Time = Wake up Time Base * WakeUpCounter
	          = (RTC_WAKEUPCLOCK_RTCCLK_DIV /(LSI)) * WakeUpCounter
	          ==> WakeUpCounter = Wake up Time / Wake up Time Base

	        To configure the wake up timer to 60s the WakeUpCounter is set to 0xFFFF:
	        Wake up Time Base = 16 /(~32.000KHz) = ~0.5 ms
	        Wake up Time = 0.5 ms  * WakeUpCounter
	        Therefore, with wake-up counter =  0xFFFF  = 65,535
	           Wake up Time =  0,5 ms *  65,535 = 32,7675 s ~ 33 sec. */
	   HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, 0x0FFFF, RTC_WAKEUPCLOCK_RTCCLK_DIV16);


	   /* Enter stop 0 mode */
	   HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);

	   /* ... Stop 0 mode ... */

	   /* Configure system clock after wake-up from stop: enable HSE, PLL and select
	   PLL as system clock source (HSE and PLL are disabled in stop mode) */
	   SYSCLKConfig_STOP();

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_3;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
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
  hrtc.Init.AsynchPrediv = 127; // should  investigate these
  hrtc.Init.SynchPrediv = 255;  // should investigate these
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pins : PC13 PC14 PC15 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PH0 PH1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA1 PA2 PA3
                           PA4 PA5 PA6 PA7
                           PA8 PA9 PA10 PA11
                           PA12 PA13 PA14 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7
                          |GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 PB10
                           PB11 PB12 PB13 PB14
                           PB15 PB3 PB4 PB5
                           PB6 PB7 PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10
                          |GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14
                          |GPIO_PIN_15|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5
                          |GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
/**
  * @brief  RTC Configuration
  *         RTC Clocked by LSI (see HAL_RTC_MspInit)
  * @param  None
  * @retval None
  */
static void RTC_Config(void)
{
  /* Configure RTC */
  /* after MX_RTC_Init : this not done in the MX_RTC_Init*/

  /*##-2- Enable the RTC peripheral Clock ####################################*/
  /* Enable RTC Clock */
  __HAL_RCC_RTC_ENABLE();

  /*##-3- Configure the NVIC for RTC Alarm ###################################*/
  HAL_NVIC_SetPriority(RTC_IRQn, 0x0, 0);
  HAL_NVIC_EnableIRQ(RTC_IRQn);
}

/**
  * @brief  Configures system clock after wake-up from stop: enable HSE, PLL
  *         and select PLL as system clock source.
  * @param  None
  * @retval None
  */
static void SYSCLKConfig_STOP(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  uint32_t pFLatency = 0;

  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();

  /* Get the Oscillators configuration according to the internal RCC registers */
  HAL_RCC_GetOscConfig(&RCC_OscInitStruct);

  /* After wake-up from stop reconfigure the system clock: Enable HSE and PLL */
  RCC_OscInitStruct.OscillatorType  = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState        = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState    = RCC_PLL_ON;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /* Get the Clocks configuration according to the internal RCC registers */
  HAL_RCC_GetClockConfig(&RCC_ClkInitStruct, &pFLatency);

  /* Select PLL as system clock source and keep HCLK, PCLK1 and PCLK2 clocks dividers as before */
  RCC_ClkInitStruct.ClockType     = RCC_CLOCKTYPE_SYSCLK;
  RCC_ClkInitStruct.SYSCLKSource  = RCC_SYSCLKSOURCE_PLLCLK;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, pFLatency) != HAL_OK)
  {
    Error_Handler();
  }
}

/*
 * training data set function
 *
 */
void fit_data(int table_type, int feature, int prediction)
{
	if(table_type == TABLE_OUTLOOK)
	{

		OUTLOOK_TABLE[feature][prediction] = OUTLOOK_TABLE[feature][prediction] + 1;
		/* yes no records for outlook table */
		if(prediction == YES)
		{
			outlook_yes_records++;
		}else if(prediction == NO)
		{
			outlook_no_records++;
		}else
		{

		}

	}else if(table_type == TABLE_TEMP)
	{

		TEMP_TABLE[feature][prediction] = TEMP_TABLE[feature][prediction] + 1;
		/* yes no records for temperature table*/
		if(prediction == YES)
		{
			temp_yes_records++;
		}else if(prediction == NO)
		{
			temp_no_records++;
		}else
		{

		}

	}else
	{
		/* selection table type is wrong*/
	}
}

/*
 * Probability calculation for the data
 */
float calc_probabulity(void)
{
	float prob_resultant = 0.0;
	float prob_yes_today = 0.0;
	float prob_no_today = 0.0;
	float table1_prob = 0.0;
	float table2_prob = 0.0;
	float yes_prob = 0.0;
	float no_prob = 0.0;

	/* assuming the data is sunny and HOT */
	table1_prob = OUTLOOK_TABLE[SET1_SUNNY][YES]/(outlook_yes_records);
	table2_prob = TEMP_TABLE[SET2_HOT][YES]/(temp_yes_records);
	/* calculating the probability of playing yes */
	yes_prob = outlook_yes_records/(outlook_yes_records+outlook_no_records);
	no_prob = outlook_no_records/(outlook_yes_records+outlook_no_records);

	prob_yes_today = table1_prob*table2_prob*yes_prob;
	prob_no_today  = table1_prob*table2_prob*no_prob;
	/* should normalize the value to get the resultant probability */
	prob_resultant = prob_yes_today/(prob_yes_today + prob_no_today);

	return prob_resultant;
}
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
