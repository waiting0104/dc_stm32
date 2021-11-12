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
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <string.h>
#include "robot_param.h"
#include "robot_control.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RXBUFSIZE 32
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
Robot robot;

// for system id
int duty = 400;
float t = 0.0f;
int count = 0;
float vel = 0.0f;

float cmd_velocity = 0.0f;
float cmd_angular_velocity = 0.0f;

char pTxData[6];
char pRxData[6];

const int rx_buffer_size = RXBUFSIZE;

uint8_t uart2_rx_buffer[RXBUFSIZE] = {0};
int uart2_buf_index = 0;
uint8_t uart2_rx_byte = 0;

uint8_t uart3_rx_buffer[RXBUFSIZE] = {0};
int uart3_buf_index = 0;
uint8_t uart3_rx_byte = 0;

char pos_info[32];

const int motor_check = 0b10000000;
const int dir_check   = 0b01000000;
const int value_check = 0b00111111;

volatile int lspeed = 0;
volatile int rspeed = 0;
double dlspeed = 0;
double drspeed = 0;
const double spd_ratio = 0.1;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
uint8_t readSerialByte( UART_HandleTypeDef *huart, uint8_t* buf, int* buf_index, const int buf_size);
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
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM5_Init();
  MX_USART2_UART_Init();
  MX_TIM6_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_DMA(&huart2,uart2_rx_buffer,rx_buffer_size);
  HAL_UART_Receive_DMA(&huart3,uart3_rx_buffer,rx_buffer_size);
  Robot_init(&robot);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	memset(pos_info, 0, 32);
	snprintf(pos_info, 32, "%4.4lf %4.4lf\r\n", robot.motor_L.velocity, robot.motor_R.velocity );
	HAL_UART_Transmit(&huart3, (uint8_t *)&pos_info, 32, 10);
//	uart2_rx_byte = readSerialByte(&huart2, uart2_rx_buffer, &uart2_buf_index, rx_buffer_size);
	uart3_rx_byte = readSerialByte(&huart3, uart3_rx_buffer, &uart3_buf_index, rx_buffer_size);
//	if ( uart2_rx_byte != 0){
////		HAL_UART_Transmit(&huart2, &uart2_rx_byte, 1, 10);
//		HAL_UART_Transmit(&huart3, &uart2_rx_byte, 1, 10);
//	}
	if ( uart3_rx_byte != 0){
//		HAL_UART_Transmit(&huart2, &uart3_rx_byte, 1, 10);
//		HAL_UART_Transmit(&huart3, &uart3_rx_byte, 1, 10);
		int motor = uart3_rx_byte & motor_check ? 1:0;
		int dir   = uart3_rx_byte & dir_check ? 1:0; // 0 for negative, 1 for positive
		int value = uart3_rx_byte & value_check;
 		if (motor) {
			// true for left
			lspeed = dir ? value : -value;
			dlspeed = lspeed * spd_ratio;
		} else {
			// false for right
			rspeed = dir ? value : -value;
			drspeed = rspeed * spd_ratio;
		}
	}
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
uint8_t readSerialByte( UART_HandleTypeDef *huart, uint8_t* buf, int* buf_index, const int buf_size){
	uint8_t data = 0;

	int index = huart->hdmarx->Instance->NDTR;
	index = buf_size - index;

	int remainData = index - *buf_index;
	if (remainData < 0) remainData+= buf_size;

	if (remainData > 0) {
		data = buf[*buf_index];
		*buf_index += 1;
		if (*buf_index == buf_size) *buf_index = 0;
	}

	return data;

}
//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
//	if(GPIO_Pin == GPIO_PIN_12){
//		snprintf(pTxData, sizeof(pTxData), "01234");
//		HAL_SPI_TransmitReceive(&hspi1, (uint8_t*)pTxData, (uint8_t*)pRxData, sizeof(pTxData), 1);
//	}
//}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim == robot.INT_TIM){
		Drv_Motor_ReadVelocity(&robot.motor_L);
		Drv_Motor_ReadVelocity(&robot.motor_R);
		Drv_Motor_ControlwithWheelVelocity(&robot.motor_L, dlspeed);
		Drv_Motor_ControlwithWheelVelocity(&robot.motor_R, drspeed);
//		memset(pos_info, 0, 32);
//		snprintf(pos_info, 32, "%4.4lf %4.4lf\r\n", robot.motor_L.velocity, robot.motor_R.velocity );
//		HAL_UART_Transmit(&huart2, (uint8_t *)&pos_info, 32, 10);
//		Robot_ControlwithRobotVelocity(&robot, 0.0, -0.5);
//		snprintf(pTxData, sizeof(pTxData), "Hello");
//		HAL_SPI_TransmitReceive(&hspi1, (uint8_t*)pTxData, (uint8_t*)pRxData, sizeof(pTxData), 1);
		// chirp input for system identify
//		duty = 1400 + 600*sin(M_TWOPI*(0.001+t/40)*t);
//		duty = (int)duty;
		//random input for system id validation
//		count += 1;
//		if(count == 1000){
//			duty = (rand()%1501)+ 400;
//			count = 0;
//		}
		// left motor
//		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
//		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
//		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, duty);
//		HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
		// right motor
		//	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);
		//	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
		//	  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, duty);
		//	  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
//		t += TIM6_DT;

		// sent msg to serial
//		char msg[11];
//		snprintf(msg, sizeof(msg), "%.4f\r\n", velocityL);
//		snprintf(msg, sizeof(msg), "v: %.4f, ref: %.4f, duty: %d\r\n", velocityR, ref_velocityR, dutyR);
//		snprintf(msg, sizeof(msg), "v: %.4f, duty: %d\r\n", velocityR, dutyR);
//		snprintf(msg, sizeof(msg), "%.4f, %.4f\r\n", velocityR, ref_velocityR);
//		HAL_UART_Transmit(&huart2, (uint8_t*)msg, sizeof(msg), 1);
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
