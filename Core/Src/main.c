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
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
__IO ITStatus UartReady = RESET;
uint8_t aTxBuffer[10] = {1,2,3,4,5,6,7,8,9,10};
uint8_t aRxBuffer[RXBUFFERSIZE];

int Join_Flag = 0;
int wait_joint = 0;
int join_state_JOINED = 0;
int join_state_EVT = 0;
int Still_Connect = 0;

char* Message;
char* Lora_Port_Number = "55";
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
void Check_Connection();
void Recive_Data();
void Wait_For_ConnectionStatus();
void Wait_For_Join();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
//	printf("TX Com");
//}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  Join_Flag = 0;
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

  //## Init Lora Module And send join command
  Uart_Transmit_AT_Init();

  //## We should wait for response so we khow it has been joined to Lora network
  Wait_For_Join();

  //Send 2 data so we make sure we get Lora status on first send and the second one to make sure it has been connected
  Send_First_Data();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	//Check IF Lora is still connected or not
	Check_Connection();

	//Wait for Response of Lora if not connected it should Connect again
	Wait_For_ConnectionStatus();
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
	  //Recieve What Lora Gets from Server
	  Recive_Data();
	  //Wait for Response and Check the message

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
  huart1.Init.BaudRate = 9600;
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
  __HAL_RCC_GPIOB_CLK_ENABLE();

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

  /*Configure GPIO pin : PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */
void Uart_Transmit_AT_Init(){
	HAL_UART_Transmit(&huart1,(uint8_t *)("ATZ\r\n"), strlen("ATZ\r\n"), 1000);
	HAL_Delay(2000);
	HAL_UART_Transmit(&huart1,(uint8_t *)("AT+VER=?\r\n"), strlen("AT+VER=?\r"), 1000);
	HAL_Delay(2000);
	HAL_UART_Transmit(&huart1,(uint8_t *)("AT\r\n"), strlen("AT\r\n"), 1000);
	HAL_Delay(2000);
	HAL_UART_Transmit(&huart1,(uint8_t *)("AT+DR=0\r\n"), strlen("AT+DR=0\r\n"), 1000);
	HAL_Delay(2000);
	HAL_UART_Transmit(&huart1,(uint8_t *)("AT\r\n"), strlen("AT\r\n"), 1000);
	HAL_Delay(2000);
	HAL_UART_Transmit(&huart1,(uint8_t *)("AT+CLASS=CLASSC\r\n"), strlen("AT+CLASS=CLASSC\r\n"), 1000);
	HAL_Delay(2000);
	HAL_UART_Transmit(&huart1,(uint8_t *)("AT\r\n"), strlen("AT\r\n"), 1000);
	HAL_Delay(2000);
	HAL_UART_Transmit(&huart1,(uint8_t *)("AT+JOIN\r\n"), strlen("AT+JOIN\r\n"), 1000);
	HAL_Delay(2000);
	wait_joint = 1;
}

void Wait_For_Join(){

	  char* Join_Message = "JOINED";
	  /*##-1- Put UART peripheral in reception process ###########################*/

	  //Wait for JOINED from Lora
	  if(HAL_UART_Receive_IT(&huart1, (uint8_t *)aRxBuffer, RXBUFFERSIZE) != HAL_OK)
	  {
	    Error_Handler();
	  }

	  /*##-2- Wait for the end of the Receive ###################################*/
	  /* While waiting for message to come from the other board, LED1 is
	     blinking according to the following pattern: a double flash every half-second */
	  while (UartReady != SET)
	  {
		  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
	      HAL_Delay(100);
	  	  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
	      HAL_Delay(100);
	  	  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
	      HAL_Delay(100);
	  	  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
	      HAL_Delay(500);
	  }

	  /* Reset reception flag */
	  UartReady = RESET;

	  if((char *)aRxBuffer == Join_Message){
		  Join_Flag = 1;
	  }
}

void Wait_For_ConnectionStatus(){
	  char* Join_Message = "JOINED";
	  char* Status_Message = "1";
	  Still_Connect = 0;
	  /*##-1- Put UART peripheral in reception process ###########################*/

	  //Wait for JOINED from Lora
	  if(HAL_UART_Receive_IT(&huart1, (uint8_t *)aRxBuffer, RXBUFFERSIZE) != HAL_OK)
	  {
	    Error_Handler();
	  }

	  /*##-2- Wait for the end of the Receive ###################################*/
	  /* While waiting for message to come from the other board, LED1 is
	     blinking according to the following pattern: a double flash every half-second */
	  while (UartReady != SET)
	  {
		  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
	      HAL_Delay(100);
	  	  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
	      HAL_Delay(100);
	  	  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
	      HAL_Delay(100);
	  	  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
	      HAL_Delay(500);
	  }

	  /* Reset reception flag */
	  UartReady = RESET;

	  if((char *)aRxBuffer == Status_Message){
		  Still_Connect = 1;
	  }
	  else {
		  if(HAL_UART_Receive_IT(&huart1, (uint8_t *)aRxBuffer, RXBUFFERSIZE) != HAL_OK)
		  	  {
		  	    Error_Handler();
		  	  }

		  	  /*##-2- Wait for the end of the Receive ###################################*/
		  	  /* While waiting for message to come from the other board, LED1 is
		  	     blinking according to the following pattern: a double flash every half-second */
		  	  while (UartReady != SET)
		  	  {
		  		  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
		  	      HAL_Delay(100);
		  	  	  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
		  	      HAL_Delay(100);
		  	  	  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
		  	      HAL_Delay(100);
		  	  	  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
		  	      HAL_Delay(500);
		  	  }

		  	  /* Reset reception flag */
		  	  UartReady = RESET;

		  	  if((char *)aRxBuffer == Join_Message){
		  		  Join_Flag = 1;
		  		  Still_Connect = 1;
		  	  }
	  }
}

void Check_Connection(){ //Check Connection Status
	HAL_UART_Transmit(&huart1,(uint8_t *)("AT\r\n"), strlen("AT\r\n"), 1000);
	HAL_Delay(200);
	HAL_UART_Transmit(&huart1,(uint8_t *)("AT+NJS=?\r\n"), strlen("AT+NJS=?\r\n"), 1000);
	HAL_Delay(2000);

}

void Send_First_Data(){ //First data for getting status and Connection
	HAL_UART_Transmit(&huart1,(uint8_t *)("AT\r\n"), strlen("AT\r\n"), 1000);
	HAL_Delay(200);
	HAL_UART_Transmit(&huart1,(uint8_t *)("AT+SEND=55:Status\r\n"), strlen("AT+SEND=55:SALAM\r\n"), 1000);
	HAL_Delay(5000);
	HAL_UART_Transmit(&huart1,(uint8_t *)("AT\r\n"), strlen("AT\r\n"), 1000);
	HAL_Delay(200);
	HAL_UART_Transmit(&huart1,(uint8_t *)("AT+SEND=55:AmConnected\r\n"), strlen("AT+SEND=55:SALAM\r\n"), 1000);
	HAL_Delay(5000);
}

void Recive_Data(){ //For reciveint data from lora
	HAL_UART_Transmit(&huart1,(uint8_t *)("AT+RECV=?\r\n"), strlen("AT+RECV=?\r\n"), 1000);
	HAL_Delay(2000);

	  /*##-1- Put UART peripheral in reception process ###########################*/

	  //Wait for JOINED from Lora
	  if(HAL_UART_Receive_IT(&huart1, (uint8_t *)aRxBuffer, RXBUFFERSIZE) != HAL_OK)
	  {
	    Error_Handler();
	  }

	  /*##-2- Wait for the end of the Receive ###################################*/
	  /* While waiting for message to come from the other board, LED1 is
	     blinking according to the following pattern: a double flash every half-second */
	  while (UartReady != SET)
	  {
		  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
	      HAL_Delay(100);
	  	  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
	      HAL_Delay(100);
	  	  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
	      HAL_Delay(100);
	  	  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
	      HAL_Delay(500);
	  }

	  /* Reset reception flag */
	  UartReady = RESET;

	  if((char *)aRxBuffer == Lora_Port_Number){
		  __NOP();
	  }
	  else {
		  Message = (char *)aRxBuffer;
	  }

}


void USART1_IRQHandler(void)
{
//  /* USER CODE BEGIN USART1_IRQn 0 */
//
//  /* USER CODE END USART1_IRQn 0 */
//  HAL_UART_IRQHandler(&huart1);
//  /* USER CODE BEGIN USART1_IRQn 1 */
//
//  /*****************************************************/
//	if(aRxBuffer == 'J' && join_state_JOINED == 0 && wait_joint == 1){
//		join_state_JOINED = 1;
//	}
//	else if(aRxBuffer == 'O' && join_state_JOINED == 1 && wait_joint == 1){
//		join_state_JOINED = 2;
//	}
//	else if(aRxBuffer == 'I' && join_state_JOINED == 2 && wait_joint == 1){
//		join_state_JOINED = 3;
//	}
//	else if(aRxBuffer == 'N' && join_state_JOINED == 3 && wait_joint == 1){
//		join_state_JOINED = 4;
//	}
//	else if(aRxBuffer == 'E' && join_state_JOINED == 4 && wait_joint == 1){
//		join_state_JOINED = 5;
//    }
//	else if(aRxBuffer == 'D' && join_state_JOINED == 5 && wait_joint == 1){
//		join_state_JOINED = 0;
//		Join_Flag = 1;
//    }
//	else if(wait_joint == 1){
//		join_state_JOINED = 0;
//	}
//  /*****************************************************/
//	if(aRxBuffer == '1\r' && Still_Connect == 0){
//		Still_Connect = 1;
//	}
//  /*****************************************************/
//	if(aRxBuffer == '+' && join_state_EVT == 0){
//		join_state_EVT = 1;
//	}
//	else if(aRxBuffer == 'E' && join_state_EVT == 1){
//		join_state_EVT = 2;
//	}
//	else if(aRxBuffer == 'V' && join_state_EVT == 2){
//		join_state_EVT = 3;
//	}
//	else if(aRxBuffer == 'T' && join_state_EVT == 3){
//		join_state_EVT = 4;
//	}
//	else if(aRxBuffer == ':' && join_state_EVT == 4){
//		join_state_EVT = 5;
//    }
//	else if(aRxBuffer == 'D' && join_state_EVT == 5){
//		join_state_EVT = 6;
//    }
//	else if(wait_joint == 1){
//		join_state_JOINED = 0;
//	}
//
////	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
//  /* USER CODE END USART1_IRQn 1 */
}

/********************************************************************/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	//Set UartReady
	UartReady = SET;
	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
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
	while(1){
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
		HAL_Delay(2000);
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
