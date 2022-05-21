/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
const unsigned char billtomb[]={69,14,13,11,22,21,19,38,37,35,70,67};
//								 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, #, *

uint8_t sor=0b1000, olvas=0, pin1=0, kapott=20;
int index=0;
//int pwm_prescale=0, pwm_period=0, pwm_pulse=0;
//int tim_prescale=0, tim_period=0;

GPIO_TypeDef * matrix_GPIO_addr[4]={KB_PC3_OUT_row1_GPIO_Port, KB_PC4_OUT_row2_GPIO_Port, KB_PC5_OUT_row3_GPIO_Port, KB_PC6_OUT_row4_GPIO_Port};

uint16_t matrix_PIN_addr[4]={KB_PC3_OUT_row1_Pin, KB_PC4_OUT_row2_Pin, KB_PC5_OUT_row3_Pin, KB_PC6_OUT_row4_Pin};

GPIO_TypeDef * matrix_GPIO_read[7]={KB_PC6_OUT_row4_GPIO_Port, KB_PC5_OUT_row3_GPIO_Port, KB_PC4_OUT_row2_GPIO_Port, KB_PC3_OUT_row1_GPIO_Port,
									KB_PC2_IN_RIGHT_GPIO_Port, KB_PC1_IN_CENTER_GPIO_Port, KB_PC0_IN_LEFT_GPIO_Port};

uint16_t matrix_PIN_read[7]={KB_PC6_OUT_row4_Pin, KB_PC5_OUT_row3_Pin, KB_PC4_OUT_row2_Pin, KB_PC3_OUT_row1_Pin,
							 KB_PC2_IN_RIGHT_Pin, KB_PC1_IN_CENTER_Pin, KB_PC0_IN_LEFT_Pin};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
void matrix_sorcimzes(uint8_t sor);
void matrix_olvas(void);
void check(int hozott);
void pulse(void);
void blink(void);
void blink_500(void);

/*void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)	//ezt valahonnan kukáztuk a Driver-bõl a TIM2-höz
{
	HAL_GPIO_TogglePin(LED_B_GPIO_Port, LED_B_Pin);
}*/
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
	//uint16_t counter=0;	//timerhez
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
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim2);				//villogtatás

  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);		//PWM
  /* USER CODE END 2 */
 
 

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  matrix_sorcimzes(sor);	  	//PORTC=sor;
	  HAL_Delay(25);
	  matrix_olvas();	  			//olvas= PINC & 0x7F;

	  for(index=0;index<12;index++)
	  {
	  	if(pin1==billtomb[index])
	  	{
	  		kapott=index;
	  		check(kapott); break;
	  	}
	  }

	  if(sor<3) sor++;
	  else {sor=0;}
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

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 15999;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 16;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 100;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 10;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, KB_PC3_OUT_row1_Pin|KB_PC5_OUT_row3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(KB_PC4_OUT_row2_GPIO_Port, KB_PC4_OUT_row2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(KB_PC6_OUT_row4_GPIO_Port, KB_PC6_OUT_row4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : KB_PC0_IN_LEFT_Pin */
  GPIO_InitStruct.Pin = KB_PC0_IN_LEFT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(KB_PC0_IN_LEFT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : KB_PC3_OUT_row1_Pin KB_PC5_OUT_row3_Pin */
  GPIO_InitStruct.Pin = KB_PC3_OUT_row1_Pin|KB_PC5_OUT_row3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : KB_PC1_IN_CENTER_Pin KB_PC2_IN_RIGHT_Pin */
  GPIO_InitStruct.Pin = KB_PC1_IN_CENTER_Pin|KB_PC2_IN_RIGHT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : KB_PC4_OUT_row2_Pin */
  GPIO_InitStruct.Pin = KB_PC4_OUT_row2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(KB_PC4_OUT_row2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : KB_PC6_OUT_row4_Pin */
  GPIO_InitStruct.Pin = KB_PC6_OUT_row4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(KB_PC6_OUT_row4_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_B_Pin */
  GPIO_InitStruct.Pin = LED_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_B_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void matrix_sorcimzes(uint8_t sor)
{
	int i=0;

	while(i<=3)
	{
		if(i==sor)
		{
			HAL_GPIO_WritePin(matrix_GPIO_addr[i], matrix_PIN_addr[i], SET);
		}
		else
		{
			HAL_GPIO_WritePin(matrix_GPIO_addr[i], matrix_PIN_addr[i], RESET);
		}

		i++;
	}
}

void matrix_olvas(void)
{
	int i=0;
	pin1=0;

	while(i<7)
	{
		pin1<<=1;
		pin1|=HAL_GPIO_ReadPin(matrix_GPIO_read[i],matrix_PIN_read[i]);
		i++;
	}

}

void check(int hozott)
{
	switch(hozott)
	{
		case 1: __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 50); HAL_Delay(1000); __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0); break;
		case 2: blink(); break;
		case 3: blink_500(); break;
		case 4: __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 5); break;
		case 5: __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 50); break;
		case 6: __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 98); break;
		case 7: pulse(); break;
		case 8: __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 50); HAL_Delay(1000); __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 100); break;
		default: hozott=0;
	}
}

void blink(void)
{
	int valtozo=20;

	while(valtozo)
	{
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 100);
		HAL_Delay(100);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
		HAL_Delay(100);
		valtozo--;
	}
}

void blink_500(void)
{
	int valtozo=10;

		while(valtozo)
		{
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 100);
			HAL_Delay(500);
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
			HAL_Delay(500);
			valtozo--;
		}
}

void pulse(void)
{
	uint8_t rake=0;
	int fut=50000, pwm1=0;

	while(fut)
	{
		if(pwm1<100000 && rake==0)
		{
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, pwm1);
			pwm1++;
			if(pwm1>=10000) { rake=1; pwm1++; }
		}
		while(rake==1)
		{
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, pwm1);
			pwm1--;
			if(pwm1<=0) rake=0;
		}
		fut--;
	}
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
