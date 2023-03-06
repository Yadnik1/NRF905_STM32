/* USER CODE BEGIN Header */
/*
 * Author: Wojciech Domski
 * WWW: domski.pl
 *
 */
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
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "NRF905.h"

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
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

int _write(int file, char *ptr, int len)
{
  /* Send the data over USART2 */
  HAL_UART_Transmit(&huart2, (uint8_t*) ptr, len, HAL_MAX_DELAY);
  return len;
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

NRF905_hw_t NRF905_hw;
NRF905_t NRF905;

int master;
int ret;

char buffer[64];
char nrf905_payload_buffer[NRF905_MAX_PAYLOAD + 1];

int message;
int message_length;

uint32_t my_address;
uint32_t receiver_address;

#define ADDRESS_MASTER 0x25D34478
#define ADDRESS_SLAVE  0x6DA0C59B

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
  MX_USART2_UART_Init();
  MX_SPI2_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

	uint32_t uid = 0x00;
	for (uint8_t i = 0; i < 3; ++i) {
		uid += *(uint32_t*) (UID_BASE + i * 4);
	}
	srand(uid);

	NRF905_hw.gpio[NRF905_HW_GPIO_TXEN].pin = TXEN_Pin;
	NRF905_hw.gpio[NRF905_HW_GPIO_TXEN].port = TXEN_GPIO_Port;
	NRF905_hw.gpio[NRF905_HW_GPIO_TRX_EN].pin = TRX_CE_Pin;
	NRF905_hw.gpio[NRF905_HW_GPIO_TRX_EN].port = TRX_CE_GPIO_Port;
	NRF905_hw.gpio[NRF905_HW_GPIO_PWR].pin = PWR_Pin;
	NRF905_hw.gpio[NRF905_HW_GPIO_PWR].port = PWR_GPIO_Port;

	NRF905_hw.gpio[NRF905_HW_GPIO_CD].pin = CD_Pin;
	NRF905_hw.gpio[NRF905_HW_GPIO_CD].port = CD_GPIO_Port;
	NRF905_hw.gpio[NRF905_HW_GPIO_AM].pin = 0;
	NRF905_hw.gpio[NRF905_HW_GPIO_AM].port = NULL;
	NRF905_hw.gpio[NRF905_HW_GPIO_DR].pin = 0;
	NRF905_hw.gpio[NRF905_HW_GPIO_DR].port = NULL;

	NRF905_hw.gpio[NRF905_HW_GPIO_CS].pin = SPI_CS_Pin;
	NRF905_hw.gpio[NRF905_HW_GPIO_CS].port = SPI_CS_GPIO_Port;

	NRF905_hw.tim = &htim1;
	NRF905_hw.spi = &hspi2;

	master = 1;
	if (master == 1) {
		my_address = ADDRESS_MASTER;
		receiver_address = ADDRESS_SLAVE;
	} else {
		my_address = ADDRESS_SLAVE;
		receiver_address = ADDRESS_MASTER;
	}

	if (master == 1) {
		printf("Mode: Master, TX, %08lX\r\n", my_address);
		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
	} else {
		printf("Mode: Slave, RX, %08lX\r\n", my_address);
		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
	}

	NRF905_init(&NRF905, &NRF905_hw);
	NRF905_set_listen_address(&NRF905, receiver_address);

	printf("Reg conf: ");
	for (int i = 0; i < 10; ++i) {
		uint8_t d = NRF905_read_config_register(&NRF905, i);
		printf("%02X, ", d);
	}
	printf("\r\n");

  /* USER CODE END 2 */

	/* Infinite loop /
	/ USER CODE BEGIN WHILE */
	while (1) {
	// Generate a random number.
	uint32_t number = rand();
	// Wait for the BUTTON pin to be pressed.
	while(HAL_GPIO_ReadPin(MODE_GPIO_Port, MODE_Pin) == GPIO_PIN_SET);

	// Initialize the buffer with the number to be sent.
	memcpy(nrf905_payload_buffer, &number, sizeof(number));
	uint8_t len = sizeof(number);

	// Print the number to be sent.
	printf("Sending (%d): %lu\r\n", len, (unsigned long)number);

	// Send the number using NRF905 module in TX mode.
	int ret = NRF905_tx(&NRF905, my_address, nrf905_payload_buffer, len, NRF905_NEXTMODE_RX);

	// Print the return value of NRF905_tx function.
	printf("ret = %d\r\n", ret);

	// Toggle LED Pin.
	HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);

	}
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
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
	while (1) {
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
