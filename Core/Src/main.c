/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
uint8_t Buffer_tx[10] = {1,2,3,4,5,6,7,8,9,10};
char Buffer_rx;

int Flag_Connected = 0;
int  wait_joint = 0;
int join_state = 0;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void Uart_Transmit_AT_Init();
void Send_First_Data();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
//	printf("TX Com");
//}
//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
//	if(Buffer_rx[0] == 'J'){
//		__NOP();
//		if(Buffer_rx[1] == 'O'){
//			while(1);
//		}
//	}
//	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
//}
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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();


  /* USER CODE BEGIN 2 */
  for(int Count;Count <= 150 && Flag_Connected==0; Count++){
	  int Tick_Elapsed1=0;
	  int Tick_Elapsed2=0;
	  Uart_Transmit_AT_Init();
	  HAL_UART_Receive_IT(&huart1, (char *)&Buffer_rx, 1);
	  Tick_Elapsed1 = HAL_GetTick();
	  while(Flag_Connected==0){
		  Tick_Elapsed2 = HAL_GetTick();
		  HAL_UART_Receive_IT(&huart1, (char *)&Buffer_rx, 1);
		  if(Tick_Elapsed2 - Tick_Elapsed1 >= 15000000) break;
	  }
  }

  Send_First_Data();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	if(HAL_GPIO_ReadPin(Buttom_GPIO_Port, Buttom_Pin) == GPIO_PIN_SET){
		HAL_GPIO_WritePin(GPIOA, Relay_1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, Relay_2_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, Relay_3_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, Relay_4_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, Relay_5_Pin, GPIO_PIN_SET);
	}
	else{
		HAL_GPIO_WritePin(GPIOA, Relay_1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, Relay_2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, Relay_3_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, Relay_4_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, Relay_5_Pin, GPIO_PIN_RESET);
	}
	  HAL_UART_Receive_IT(&huart1, (char *)&Buffer_rx, 1);
  }

  /* USER CODE END 3 */
}
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */

  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */

  /*****************************************************/
	if(Buffer_rx == 'J' && join_state == 0 && wait_joint == 1){
		join_state = 1;
	}
	else if(Buffer_rx == 'O' && join_state == 1 && wait_joint == 1){
		join_state = 2;
	}
	else if(Buffer_rx == 'I' && join_state == 2 && wait_joint == 1){
		join_state = 3;
	}
	else if(Buffer_rx == 'N' && join_state == 3 && wait_joint == 1){
		join_state = 4;
	}
	else if(Buffer_rx == 'E' && join_state == 4 && wait_joint == 1){
		join_state = 5;
    }
	else if(Buffer_rx == 'D' && join_state == 5 && wait_joint == 1){
		join_state = 0;
		Flag_Connected = 1;
    }
	else if(wait_joint == 1){
		join_state = 0;
	}
  /*****************************************************/

	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
  /* USER CODE END USART1_IRQn 1 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

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
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, Relay_1_Pin|Relay_2_Pin|Relay_3_Pin|Relay_4_Pin 
                          |Relay_5_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : Buttom_Pin */
  GPIO_InitStruct.Pin = Buttom_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Buttom_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Relay_1_Pin Relay_2_Pin Relay_3_Pin Relay_4_Pin 
                           Relay_5_Pin */
  GPIO_InitStruct.Pin = Relay_1_Pin|Relay_2_Pin|Relay_3_Pin|Relay_4_Pin 
                          |Relay_5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void Uart_Transmit_AT_Init(){
	HAL_UART_Transmit(&huart1, "ATZ\r\n", sizeof("ATZ\r\n"), 1000);
	HAL_Delay(2000);
	HAL_UART_Transmit(&huart1, "AT+VER\r\n", sizeof("AT+VER\r\n"), 1000);
	HAL_Delay(2000);
	HAL_UART_Transmit(&huart1, "AT\r\n", sizeof("AT\r\n"), 1000);
	HAL_Delay(2000);
	HAL_UART_Transmit(&huart1, "AT+DR=0\r\n", sizeof("AT+DR=0\r\n"), 1000);
	HAL_Delay(2000);
	HAL_UART_Transmit(&huart1, "AT\r\n", sizeof("AT\r\n"), 1000);
	HAL_Delay(2000);
	HAL_UART_Transmit(&huart1, "AT+CLASS=CLASSC\r\n", sizeof("AT+CLASS=CLASSC\r\n"), 1000);
	HAL_Delay(2000);
	HAL_UART_Transmit(&huart1, "AT\r\n", sizeof("AT\r\n"), 1000);
	HAL_Delay(2000);
	HAL_UART_Transmit(&huart1, "AT+JOIN\r\n", sizeof("AT+JOIN\r\n"), 1000);
	HAL_Delay(2000);
	wait_joint = 1;
}

void Check_Connection(){
	HAL_UART_Transmit(&huart1, "AT+NJS=?\r\n", sizeof("AT+NJS=?\r\n"), 1000);
	HAL_Delay(2000);

}

void Send_First_Data(){
	HAL_UART_Transmit(&huart1, "AT+SEND=55:SALAM\r\n", sizeof("AT+SEND=55:SALAM\r\n"), 1000);
	HAL_Delay(5000);
	HAL_UART_Transmit(&huart1, "AT+SEND=55:SALAM\r\n", sizeof("AT+SEND=55:SALAM\r\n"), 1000);
	HAL_Delay(5000);
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
