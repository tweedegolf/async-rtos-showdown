/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2021 STMicroelectronics.
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <stdatomic.h>
#include <stdbool.h>
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
UART_HandleTypeDef huart3;

/* Definitions for blinkLed */
osThreadId_t blinkLedHandle;
const osThreadAttr_t blinkLed_attributes = { .name = "blinkLed", .stack_size =
		128 * 4, .priority = (osPriority_t) osPriorityAboveNormal, };
/* Definitions for uartWriter */
osThreadId_t uartWriterHandle;
const osThreadAttr_t uartWriter_attributes = { .name = "uartWriter",
		.stack_size = 128 * 4, .priority = (osPriority_t) osPriorityNormal, };
/* Definitions for buttonWaiter */
osThreadId_t buttonWaiterHandle;
const osThreadAttr_t buttonWaiter_attributes = { .name = "buttonWaiter",
		.stack_size = 128 * 4, .priority = (osPriority_t) osPriorityRealtime, };
/* Definitions for uartQueue */
osMessageQueueId_t uartQueueHandle;
const osMessageQueueAttr_t uartQueue_attributes = { .name = "uartQueue" };
/* Definitions for buttonInterruptEvent */
osEventFlagsId_t buttonInterruptEventHandle;
const osEventFlagsAttr_t buttonInterruptEvent_attributes = { .name =
		"buttonInterruptEvent" };
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
void StartBlinkLedTask(void *argument);
void StartUartWriter(void *argument);
void StartButtonWaiterTask(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
atomic_bool buttonPressed;

typedef struct {
	char data[32];
} UartMessage;
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
	/* USER CODE BEGIN 1 */
	atomic_init(&buttonPressed, false);
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
	MX_USART3_UART_Init();
	/* USER CODE BEGIN 2 */

	/* USER CODE END 2 */

	/* Init scheduler */
	osKernelInitialize();

	/* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
	/* USER CODE END RTOS_MUTEX */

	/* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
	/* USER CODE END RTOS_SEMAPHORES */

	/* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
	/* USER CODE END RTOS_TIMERS */

	/* Create the queue(s) */
	/* creation of uartQueue */
	uartQueueHandle = osMessageQueueNew(8, sizeof(UartMessage),
			&uartQueue_attributes);

	/* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
	/* USER CODE END RTOS_QUEUES */

	/* Create the thread(s) */
	/* creation of blinkLed */
	blinkLedHandle = osThreadNew(StartBlinkLedTask, NULL, &blinkLed_attributes);

	/* creation of uartWriter */
	uartWriterHandle = osThreadNew(StartUartWriter, NULL,
			&uartWriter_attributes);

	/* creation of buttonWaiter */
	buttonWaiterHandle = osThreadNew(StartButtonWaiterTask, NULL,
			&buttonWaiter_attributes);

	/* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
	/* USER CODE END RTOS_THREADS */

	/* Create the event(s) */
	/* creation of buttonInterruptEvent */
	buttonInterruptEventHandle = osEventFlagsNew(
			&buttonInterruptEvent_attributes);

	/* USER CODE BEGIN RTOS_EVENTS */
	/* add events, ... */
	/* USER CODE END RTOS_EVENTS */

	/* Start scheduler */
	osKernelStart();

	/* We should never get here as control is now taken by the scheduler */
	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 4;
	RCC_OscInitStruct.PLL.PLLN = 180;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 7;
	RCC_OscInitStruct.PLL.PLLR = 2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Activate the Over-Drive mode
	 */
	if (HAL_PWREx_EnableOverDrive() != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief USART3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART3_UART_Init(void) {

	/* USER CODE BEGIN USART3_Init 0 */

	/* USER CODE END USART3_Init 0 */

	/* USER CODE BEGIN USART3_Init 1 */

	/* USER CODE END USART3_Init 1 */
	huart3.Instance = USART3;
	huart3.Init.BaudRate = 115200;
	huart3.Init.WordLength = UART_WORDLENGTH_8B;
	huart3.Init.StopBits = UART_STOPBITS_1;
	huart3.Init.Parity = UART_PARITY_NONE;
	huart3.Init.Mode = UART_MODE_TX_RX;
	huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart3.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart3) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART3_Init 2 */

	/* USER CODE END USART3_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOG_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, LD1_Pin | LD3_Pin | LD2_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOG, ButtonInterrupt_Pin | ButtonProcessed_Pin,
			GPIO_PIN_RESET);

	/*Configure GPIO pin : USER_Btn_Pin */
	GPIO_InitStruct.Pin = USER_Btn_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : LD1_Pin LD3_Pin LD2_Pin */
	GPIO_InitStruct.Pin = LD1_Pin | LD3_Pin | LD2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pins : ButtonInterrupt_Pin ButtonProcessed_Pin */
	GPIO_InitStruct.Pin = ButtonInterrupt_Pin | ButtonProcessed_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == USER_Btn_Pin) {
		osThreadFlagsSet(buttonWaiterHandle, 1);
	}
}

void CheckStatus(int status) {
	if (status) {
		Error_Handler();
	}
}

UartMessage FormatMessage(int triggerCount, bool buttonPressed) {
	UartMessage message;
	snprintf(message.data, sizeof(message.data), "Button is %i (%i)\n",
			buttonPressed, triggerCount);
	return message;
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartBlinkLedTask */
/**
 * @brief Function implementing the blinkLed thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartBlinkLedTask */
void StartBlinkLedTask(void *argument) {
	/* USER CODE BEGIN 5 */
	/* Infinite loop */
	for (;;) {
		osDelay(100);

		if (atomic_load(&buttonPressed) == GPIO_PIN_RESET) {
			HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_SET);
		}

		osDelay(100);
		HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_RESET);
	}
	/* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartUartWriter */
/**
 * @brief Function implementing the uartWriter thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartUartWriter */
void StartUartWriter(void *argument) {
	/* USER CODE BEGIN StartUartWriter */
	/* Infinite loop */
	for (;;) {
		UartMessage message;

		CheckStatus(osMessageQueueGet(uartQueueHandle, &message, NULL,
		osWaitForever));

		size_t messageLength = strnlen(message.data, sizeof(message.data));
		CheckStatus(
				HAL_UART_Transmit(&huart3, (uint8_t*) &message.data,
						(uint16_t) messageLength, 1000));
	}
	/* USER CODE END StartUartWriter */
}

/* USER CODE BEGIN Header_StartButtonWaiterTask */
/**
 * @brief Function implementing the buttonWaiter thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartButtonWaiterTask */
void StartButtonWaiterTask(void *argument) {
	/* USER CODE BEGIN StartButtonWaiterTask */
	int triggerCount = 0;
	UartMessage message;

	/* Infinite loop */
	for (;;) {
		// Only react to rising edges
		EXTI->RTSR |= USER_Btn_Pin;
		EXTI->FTSR &= ~USER_Btn_Pin;

		HAL_GPIO_WritePin(ButtonProcessed_GPIO_Port, ButtonProcessed_Pin,
				GPIO_PIN_RESET);
		osThreadFlagsWait(1, osFlagsWaitAny, osWaitForever);
		HAL_GPIO_WritePin(ButtonProcessed_GPIO_Port, ButtonProcessed_Pin,
				GPIO_PIN_SET);
		triggerCount++;

		// Set the button pressed variable
		atomic_store(&buttonPressed, true);
		message = FormatMessage(triggerCount, true);
		CheckStatus(
				osMessageQueuePut(uartQueueHandle, &message, 0, osWaitForever));

		// Only react to falling edges
		EXTI->RTSR |= USER_Btn_Pin;
		EXTI->FTSR &= ~USER_Btn_Pin;

		HAL_GPIO_WritePin(ButtonProcessed_GPIO_Port, ButtonProcessed_Pin,
				GPIO_PIN_RESET);
		osThreadFlagsWait(1, osFlagsWaitAny, osWaitForever);
		HAL_GPIO_WritePin(ButtonProcessed_GPIO_Port, ButtonProcessed_Pin,
				GPIO_PIN_SET);
		triggerCount++;

		// Set the button pressed variable
		atomic_store(&buttonPressed, false);
		message = FormatMessage(triggerCount, false);
		CheckStatus(
				osMessageQueuePut(uartQueueHandle, &message, 0, osWaitForever));
	}
	/* USER CODE END StartButtonWaiterTask */
}

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM1 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	/* USER CODE BEGIN Callback 0 */

	/* USER CODE END Callback 0 */
	if (htim->Instance == TIM1) {
		HAL_IncTick();
	}
	/* USER CODE BEGIN Callback 1 */

	/* USER CODE END Callback 1 */
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
		__BKPT(0);
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

