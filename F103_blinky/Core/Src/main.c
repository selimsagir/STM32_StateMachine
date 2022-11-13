/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usart.h"
#include "gpio.h"
#include <stdio.h>
#include <string.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
typedef enum  {
	STATE_A = 0,
	STATE_B,
	STATE_C
}states;

states state_type;

void state_a_function(void);
void state_b_function(void);
void state_c_function(void);
void state_machine_init(void);
void wait(void);

static void (*state_table[])(void)={ state_a_function,
									 state_b_function,
									 state_c_function };
static states current_state;
static int clock;

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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

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
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  state_machine_init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */
	  state_table[current_state]();
	  HAL_Delay(1000);
	  clock++;
	  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void state_machine_init(void)
{
	current_state = STATE_A;
	clock = 0;
}

uint32_t sa_prev_time = 0;
uint32_t sa_now;
float sa_tdelta;
void state_a_function(void)
{
	char buffer[100] = {0};
	if( clock == 2) {
		current_state = STATE_B;
		sa_now = HAL_GetTick();
		sa_tdelta = sa_now - sa_prev_time;
		// Convert to seconds
		sa_tdelta = sa_tdelta/1000;
		sa_prev_time = sa_now;
		sprintf(buffer, "this is output of STATE A : %f seconds ago \n\r", sa_tdelta);
		HAL_UART_Transmit(&huart1, (uint8_t*)buffer, sizeof(buffer), 100);
//		printf("this is output of STATE A : %f seconds ago \n\r", sa_tdelta);
	}
}

uint32_t sb_prev_time = 0;
uint32_t sb_now;
float sb_tdelta;
void state_b_function(void)
{
	char buffer[100] = {0};
	if(clock == 5){
		current_state = STATE_C;
		sb_now = HAL_GetTick();
		sb_tdelta = sb_now - sb_prev_time;
		// Conversion of seconds
		sb_tdelta /= 1000;
		sb_prev_time = sb_now;
		sprintf(buffer, "this is output of STATE B : %f seconds ago \n\r", sb_tdelta);
		HAL_UART_Transmit(&huart1, (uint8_t*)buffer, sizeof(buffer), 100);
//		printf("this is output of STATE B : %f seconds ago \n\r", sb_tdelta);
//		printf("\n\r");

	}
}

uint32_t sc_prev_time = 0;
uint32_t sc_now;
float sc_tdelta;
void state_c_function(void){
	char buffer[100] = {0};
	if(clock == 9){
		clock = 0;
		current_state = STATE_A;
		sc_now = HAL_GetTick();
		sc_tdelta = sc_now - sc_prev_time;
		// Conversion of seconds
		sc_tdelta /= 1000;
		sc_prev_time = sc_now;
		sprintf(buffer, "this is output of STATE C : %f seconds ago \n\r", sc_tdelta);
		HAL_UART_Transmit(&huart1, (uint8_t*)buffer, sizeof(buffer), 100);
//		printf("this is output of STATE B : %f seconds ago \n\r", sc_tdelta);
//		printf("\n\r");

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
