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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "defines.h"
#include "tm_stm32_disco.h"
#include "tm_stm32_delay.h"
#include "tm_stm32_lis302dl_lis3dsh.h"

#include "printf.h"
#include "debug.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef struct lis_angles {
	/* Public */
	int16_t x; /*!< angle value X axis */
	int16_t y; /*!< angle value Y axis */
	int16_t z; /*!< angle value Z axis */
} lis_angles;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_tx;
DMA_HandleTypeDef hdma_usart2_rx;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for LIS302_DSH */
osThreadId_t LIS302_DSHHandle;
const osThreadAttr_t LIS302_DSH_attributes = {
  .name = "LIS302_DSH",
  .priority = (osPriority_t) osPriorityHigh,
  .stack_size = 128 * 4
};
/* Definitions for kalman_filter_t */
osThreadId_t kalman_filter_tHandle;
const osThreadAttr_t kalman_filter_t_attributes = {
  .name = "kalman_filter_t",
  .priority = (osPriority_t) osPriorityNormal1,
  .stack_size = 128 * 4
};
/* Definitions for sender_task */
osThreadId_t sender_taskHandle;
const osThreadAttr_t sender_task_attributes = {
  .name = "sender_task",
  .priority = (osPriority_t) osPriorityNormal1,
  .stack_size = 128 * 4
};
/* Definitions for LIS_queue */
osMessageQueueId_t LIS_queueHandle;
const osMessageQueueAttr_t LIS_queue_attributes = {
  .name = "LIS_queue"
};
/* Definitions for angles_queue */
osMessageQueueId_t angles_queueHandle;
const osMessageQueueAttr_t angles_queue_attributes = {
  .name = "angles_queue"
};
/* USER CODE BEGIN PV */

int16_t x,y,z;
char tx_buffer[20];


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
void StartDefaultTask(void *argument);
void start_task_LIS302(void *argument);
void kalman_filter(void *argument);
void send(void *argument);

/* USER CODE BEGIN PFP */

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{															// in next step dont should read old value so this start stop
	HAL_UART_DMAStop(&huart2);								// proccess was done
	HAL_UART_Transmit_DMA(&huart2, (uint8_t*)tx_buffer, sprintf(tx_buffer, "%d %d %d\n", x,y,z));
}



/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

double expect_estimated = 0;
double _old_estimate = 0;

double kalman_update(double filter_value, double _err_measure)
{
	double _err_estimate = expect_estimated;
	double _last_estimate = _old_estimate;
	double _kalman_gain = _err_estimate / (_err_estimate + _err_measure);
	double _current_estimate = _last_estimate + _kalman_gain * (filter_value - _last_estimate);

	_err_estimate = (1.0 - _kalman_gain) * expect_estimated;
	_old_estimate = _current_estimate;

	return _current_estimate;
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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  __HAL_UART_ENABLE_IT(&huart2,UART_IT_RXNE);	// Receive Data register not empty interrupt
  __HAL_UART_ENABLE_IT(&huart2,UART_IT_TC);	// active IT tranmission complete


     /* Init delay */
     TM_DELAY_Init();

     /* Initialize LEDs */
     TM_DISCO_LedInit();

     /* Detect proper device */
     if (TM_LIS302DL_LIS3DSH_Detect() == TM_LIS302DL_LIS3DSH_Device_LIS302DL) {
         /* Turn on GREEN and RED */
         //TM_DISCO_LedOn(LED_GREEN | LED_RED);
         /* Initialize LIS302DL */
         TM_LIS302DL_LIS3DSH_Init(TM_LIS302DL_Sensitivity_2_3G, TM_LIS302DL_Filter_2Hz);
     } else if (TM_LIS302DL_LIS3DSH_Detect() == TM_LIS302DL_LIS3DSH_Device_LIS3DSH) {
         /* Turn on BLUE and ORANGE */
         //TM_DISCO_LedOn(LED_BLUE | LED_ORANGE);
         /* Initialize LIS3DSH */
         TM_LIS302DL_LIS3DSH_Init(TM_LIS3DSH_Sensitivity_2G, TM_LIS3DSH_Filter_800Hz);
     } else {
         /* Device is not recognized */

         /* Turn on ALL leds */
         //TM_DISCO_LedOn(LED_GREEN | LED_RED | LED_BLUE | LED_ORANGE);
         Error_Handler();
         /* Infinite loop */
     }

     /* Delay for 2 seconds */
//     Delayms(2000);

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
  /* creation of LIS_queue */
  LIS_queueHandle = osMessageQueueNew (16, sizeof(lis_angles), &LIS_queue_attributes);

  /* creation of angles_queue */
  angles_queueHandle = osMessageQueueNew (16, sizeof(lis_angles), &angles_queue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of LIS302_DSH */
  LIS302_DSHHandle = osThreadNew(start_task_LIS302, NULL, &LIS302_DSH_attributes);

  /* creation of kalman_filter_t */
  kalman_filter_tHandle = osThreadNew(kalman_filter, NULL, &kalman_filter_t_attributes);

  /* creation of sender_task */
  sender_taskHandle = osThreadNew(send, NULL, &sender_task_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();
 
  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
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

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, green_Pin|orange_Pin|red_Pin|blue_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : green_Pin orange_Pin red_Pin blue_Pin */
  GPIO_InitStruct.Pin = green_Pin|orange_Pin|red_Pin|blue_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */ 
}

/* USER CODE BEGIN Header_start_task_LIS302 */
/**
* @brief Function implementing the LIS302_DSH thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_start_task_LIS302 */
void start_task_LIS302(void *argument)
{
  /* USER CODE BEGIN start_task_LIS302 */
	  TM_LIS302DL_LIS3DSH_t Axes_Data;
      TM_LIS302DL_LIS3DSH_Device_t statusReadLIS302 = 1;
      osStatus_t status_LIS_queue;
  /* Infinite loop */
  for(;;)
  {
		/* Read axes data from initialized accelerometer */
		TM_LIS302DL_LIS3DSH_ReadAxes(&Axes_Data);
		if (statusReadLIS302 != TM_LIS302DL_LIS3DSH_Device_Error)
		{
			/* Turn LEDS on or off */
					/* Check X axes */
					if (Axes_Data.X > 200) {
						TM_DISCO_LedOn(LED_RED);
					} else {
						TM_DISCO_LedOff(LED_RED);
					}
					if (Axes_Data.X < -200) {
						TM_DISCO_LedOn(LED_GREEN);
					} else {
						TM_DISCO_LedOff(LED_GREEN);
					}
					/* Check Y axes */
					if (Axes_Data.Y > 200) {
						TM_DISCO_LedOn(LED_ORANGE);
					} else {
						TM_DISCO_LedOff(LED_ORANGE);
					}
					if (Axes_Data.Y < -200) {
						TM_DISCO_LedOn(LED_BLUE);
					} else {
						TM_DISCO_LedOff(LED_BLUE);
					}

					status_LIS_queue = osMessageQueuePut(LIS_queueHandle, &Axes_Data, 0U, 0U);
					if(status_LIS_queue != osOK)
						Error_Handler();
		}
		osDelay(10);
  }
  /* USER CODE END start_task_LIS302 */
}

/* USER CODE BEGIN Header_kalman_filter */
/**
* @brief Function implementing the kalman_filter_t thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_kalman_filter */
void kalman_filter(void *argument)
{
  /* USER CODE BEGIN kalman_filter */
	TM_LIS302DL_LIS3DSH_t Axes_Data;
	osStatus_t statusQueue;
	lis_angles lis_ang_put;
  /* Infinite loop */
  for(;;)
  {
	  statusQueue = osMessageQueueGet(LIS_queueHandle,&Axes_Data,0U, osWaitForever);
	  if(statusQueue != osOK)
		  Error_Handler();
	  else
	  {
			lis_ang_put.x = kalman_update(Axes_Data.X, 0.001);
			lis_ang_put.y = kalman_update(Axes_Data.Y, 0.001);
			lis_ang_put.z = kalman_update(Axes_Data.Z, 0.001);

			statusQueue = osMessageQueuePut(angles_queueHandle, &lis_ang_put, 0U, osWaitForever);
			if(statusQueue != osOK)
				Error_Handler();
	  }
    osDelay(10);
  }
  /* USER CODE END kalman_filter */
}

/* USER CODE BEGIN Header_send */
/**
* @brief Function implementing the sender_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_send */
void send(void *argument)
{
  /* USER CODE BEGIN send */
	osStatus_t statusQueue;
	lis_angles lis_ang_get;
	char data[50];
	int i = 0;
  /* Infinite loop */
  for(;;)
  {
	  statusQueue = osMessageQueueGet(angles_queueHandle, &lis_ang_get, 0, osWaitForever);
	  if(statusQueue != osOK)
		  Error_Handler();
	  else
	  {
		  sprintf(data, "%d#%d#%d", lis_ang_get.x,lis_ang_get.y,lis_ang_get.z);

		  if(i % 10 == 0)
			 HAL_UART_Transmit_DMA(&huart2, (uint8_t *)data, sizeof(data));
		  ++i;
	  }
	  osDelay(20);
  }
  /* USER CODE END send */
}

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
