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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32f4xx_hal.h"
#include "tm_stm32_mpu6050.h"
#include "tm_stm32_disco.h"
#include "tm_stm32_delay.h"
#include "tm_stm32_lis302dl_lis3dsh.h"
#include "defines.h"
#include "printf.h"
#include "debug.h"
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct angles {
	/* Public */
	int16_t x; /*!< angle value X axis */
	int16_t y; /*!< angle value Y axis */
	int16_t z; /*!< angle value Z axis */
} angles;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* MPU6050 SETUP BEGIN */
TM_MPU6050_t MPU6050_Sensor;

float last_x_angle;
float last_y_angle;
float last_z_angle;

float gyro_angle_x;
float gyro_angle_y;
float gyro_angle_z;

float gyro_x;
float gyro_y;
float gyro_z;

float accel_x;
float accel_y;
float accel_z;

float accel_angle_x;
float accel_angle_y;
float accel_angle_z;

const float alpha = 0.96;

float angle_x;
float angle_y;
float angle_z;

int gz_threshold = 2;
float GYRO_FACTOR = 131.0;
unsigned long t_now = 0, t_last = 0;
float dt = 0;

float get_last_x_angle() {
	return last_x_angle;
}
float get_last_y_angle() {
	return last_y_angle;
}

float get_last_z_angle() {
	return last_z_angle;
}
const float RADIANS_TO_DEGREES = 57.2958; //180/3.14159

void set_last_read_angle_data(float x, float y, float z) {
	last_x_angle = x;
	last_y_angle = y;
	last_z_angle = z;
}
/* MPU6050 SETUP END */
/* LIS302DL SETUP BEGIN */
#define LENGHT		20
osStatus_t status;
char tx_buffer[50];
int8_t x = 1, y = 1, z = 1;
int i = 0;

int t = 0;


int index_x = 0;
int value_x = 0;
double sum_x = 0;
int readings_x[LENGHT];
int average_x = 0;

int index_y = 0;
int value_y = 0;
double sum_y = 0;
int readings_y [LENGHT];
int average_y = 0;

int index_z = 0;
int value_z = 0;
double sum_z = 0;
int readings_z [LENGHT];
int average_z = 0;

double xk_kalman_prediction_old = 0; 	//Prior Estimate
double pk_error_coverience_new = 1; 	//Error

double expect_estimated;
double _old_estimate;

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

double calculate_kalman_filter(double zk_calculate_value, double r_error_value)
{
	  	double xk_kalman_prediction_new = xk_kalman_prediction_old;
	  	double pk_error_covairence = pk_error_coverience_new;		// use old values for new values

	  	// arrange new calculation
	  	double kk_kalman_gain = pk_error_covairence / (pk_error_covairence + r_error_value);
	  	double xk_kalman_calculated = xk_kalman_prediction_new + kk_kalman_gain * (zk_calculate_value - xk_kalman_prediction_new);
	  	pk_error_covairence = (1 - kk_kalman_gain) * pk_error_coverience_new;

	  	// return new calculation after the kalman filter, pk_error_coverience_new = pk_error_covairence;
	  	xk_kalman_prediction_old = xk_kalman_calculated; 	// this new value will be the next step old value
	  	return xk_kalman_calculated;
}

/* LIS302DL SETUP END */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_tx;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .priority = (osPriority_t) osPriorityNormal1,
  .stack_size = 128 * 4
};
/* Definitions for readMPU6050Task */
osThreadId_t readMPU6050TaskHandle;
const osThreadAttr_t readMPU6050Task_attributes = {
  .name = "readMPU6050Task",
  .priority = (osPriority_t) osPriorityNormal4,
  .stack_size = 128 * 4
};

/* Definitions for computationTask */
osThreadId_t computationTaskHandle;
const osThreadAttr_t computationTask_attributes = {
  .name = "computationTask",
  .priority = (osPriority_t) osPriorityNormal2,
  .stack_size = 128 * 4
};
/* Definitions for senderTask */
osThreadId_t senderTaskHandle;
const osThreadAttr_t senderTask_attributes = {
  .name = "senderTask",
  .priority = (osPriority_t) osPriorityNormal1,
  .stack_size = 128 * 4
};
/* Definitions for readLIS302Task */
osThreadId_t readLIS302TaskHandle;
const osThreadAttr_t readLIS302Task_attributes = {
  .name = "readLIS302Task",
  .priority = (osPriority_t) osPriorityNormal3,
  .stack_size = 128 * 4
};
/* Definitions for kalmanFilter */
osThreadId_t kalmanFilterHandle;
const osThreadAttr_t kalmanFilter_attributes = {
  .name = "kalmanFilter",
  .priority = (osPriority_t) osPriorityNormal2,
  .stack_size = 128 * 4
};
/* Definitions for rowDataQueue */
osMessageQueueId_t rowDataQueueHandle;
const osMessageQueueAttr_t rowDataQueue_attributes = {
  .name = "rowDataQueue"
};
/* Definitions for anglesQueue */
osMessageQueueId_t anglesQueueHandle;
const osMessageQueueAttr_t anglesQueue_attributes = {
  .name = "anglesQueue"
};
/* Definitions for rowLIS302DataQueue */
osMessageQueueId_t rowLIS302DataQueueHandle;
const osMessageQueueAttr_t rowLIS302DataQueue_attributes = {
  .name = "rowLIS302DataQueue"
};
/* Definitions for timer1 */
osTimerId_t timer1Handle;
const osTimerAttr_t timer1_attributes = {
  .name = "timer1"
};
/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);
void DefaultTask(void *argument);
void ReadMPU6050Task(void *argument);
void ComputationTask(void *argument);
void SenderTask(void *argument);
void ReadLIS302Task(void *argument);
void KalmanFilter(void *argument);
void Callback01(void *argument);

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
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  if ((TM_MPU6050_Init(&MPU6050_Sensor, TM_MPU6050_Device_0, TM_MPU6050_Accelerometer_8G, TM_MPU6050_Gyroscope_500s)) != TM_MPU6050_Result_Ok)
	  Error_Handler();

  set_last_read_angle_data(0, 0, 0);

  /* Init delay */
  TM_DELAY_Init();
  /* Initialize LEDs */
  TM_DISCO_LedInit();

  /* Detect proper device */
  if (TM_LIS302DL_LIS3DSH_Detect() == TM_LIS302DL_LIS3DSH_Device_LIS302DL)
  {
      /* Turn on GREEN and RED */
      TM_DISCO_LedOn(LED_GREEN | LED_RED);
      /* Initialize LIS302DL */
      TM_LIS302DL_LIS3DSH_Init(TM_LIS302DL_Sensitivity_2_3G, TM_LIS302DL_Filter_2Hz);
  }
  else if (TM_LIS302DL_LIS3DSH_Detect() == TM_LIS302DL_LIS3DSH_Device_LIS3DSH)
  {
      /* Turn on BLUE and ORANGE */
      TM_DISCO_LedOn(LED_BLUE | LED_ORANGE);
      /* Initialize LIS3DSH */
      TM_LIS302DL_LIS3DSH_Init(TM_LIS3DSH_Sensitivity_2G, TM_LIS3DSH_Filter_800Hz);
  }
  else
  {
      /* Device is not recognized */
      /* Turn on ALL leds */
      TM_DISCO_LedOn(LED_GREEN | LED_RED | LED_BLUE | LED_ORANGE);
      Error_Handler();
  }

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* creation of timer1 */
  timer1Handle = osTimerNew(Callback01, osTimerPeriodic, NULL, &timer1_attributes);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of rowDataQueue */
  rowDataQueueHandle = osMessageQueueNew (5, sizeof(TM_MPU6050_t), &rowDataQueue_attributes);

  /* creation of anglesQueue */
  anglesQueueHandle = osMessageQueueNew (5, sizeof(angles), &anglesQueue_attributes);

  /* creation of rowLIS302DataQueue */
  rowLIS302DataQueueHandle = osMessageQueueNew (5, sizeof(angles), &rowLIS302DataQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(DefaultTask, NULL, &defaultTask_attributes);

  /* creation of readMPU6050Task */
  readMPU6050TaskHandle = osThreadNew(ReadMPU6050Task, NULL, &readMPU6050Task_attributes);

  /* creation of computationTask */
  computationTaskHandle = osThreadNew(ComputationTask, NULL, &computationTask_attributes);

  /* creation of senderTask */
  senderTaskHandle = osThreadNew(SenderTask, NULL, &senderTask_attributes);

  /* creation of readLIS302Task */
  readLIS302TaskHandle = osThreadNew(ReadLIS302Task, NULL, &readLIS302Task_attributes);

  /* creation of kalmanFilter */
  kalmanFilterHandle = osThreadNew(KalmanFilter, NULL, &kalmanFilter_attributes);


  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

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
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_16_9;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  huart2.Init.Mode = UART_MODE_TX;
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

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
void DefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN 5 */

/* USER CODE END Header_ReadMPU6050Task */
void ReadMPU6050Task(void *argument)
{
  /* USER CODE BEGIN ReadMPU6050Task */
	  TM_MPU6050_Result_t statusRead;
	  osStatus_t statusQueue;

  /* Infinite loop */
	for (;;)
	{
		statusRead = TM_MPU6050_ReadAll(&MPU6050_Sensor);
		if (statusRead == TM_MPU6050_Result_Ok)
		{
			statusQueue = osMessageQueuePut(rowDataQueueHandle, &MPU6050_Sensor, 0U, 0U);
			if (statusQueue != osOK)
			{
				osThreadSuspend(computationTaskHandle);
				osThreadSuspend(senderTaskHandle);
			}
			else
			{
				osThreadSuspend(readLIS302TaskHandle);
				osThreadSuspend(kalmanFilterHandle);
				osThreadResume(computationTaskHandle);
			}
		}
		else
		{
			osThreadSuspend(computationTaskHandle);
			osThreadSuspend(senderTaskHandle);
			osThreadResume(readLIS302TaskHandle);
		}
		osDelay(20);
	}
  /* USER CODE END startReadMPU6050Task */
}

/* USER CODE BEGIN Header_startComputationTask */
/**
* @brief Function implementing the computationTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ComputationTask */
void ComputationTask(void *argument)
{
  /* USER CODE BEGIN startComputationTask */
	TM_MPU6050_t MPU6050_Sensor2;
	osStatus_t status;
	angles a1;
  /* Infinite loop */
	for (;;)
	{
	  status = osMessageQueueGet(rowDataQueueHandle, &MPU6050_Sensor2, 0U, osWaitForever);
	  if (status != osOK)
		  osThreadYield();  	// unless be get, it will be on ComputationTask

	  else
	  {

		gyro_x = (MPU6050_Sensor2.Gyroscope_X) / GYRO_FACTOR;
		gyro_y = (MPU6050_Sensor2.Gyroscope_Y) / GYRO_FACTOR;
		gyro_z = (MPU6050_Sensor2.Gyroscope_Z) / GYRO_FACTOR;

		accel_x = MPU6050_Sensor2.Accelerometer_X; // - base_x_accel;
		accel_y = MPU6050_Sensor2.Accelerometer_Y; // - base_y_accel;
		accel_z = MPU6050_Sensor2.Accelerometer_Z; // - base_z_accel;

		accel_angle_y = atan( -1 * accel_x / sqrt(pow(accel_y, 2) + pow(accel_z, 2))) * RADIANS_TO_DEGREES;
		accel_angle_x = atan( accel_y / sqrt(pow(accel_x, 2) + pow(accel_z, 2))) * RADIANS_TO_DEGREES;
		accel_angle_z = 0;  				//Accelerometer doesn't give z-angle

		// Compute the (filtered) gyro angles
		 //dt =(t_now - get_last_time()) / 1000;
		dt = 0.02;

		gyro_angle_x = gyro_x * dt + get_last_x_angle();
		gyro_angle_y = gyro_y * dt + get_last_y_angle();

		// gyro z raw data fluctuation threshold value when gyro doesn't move. It is up to your mpu6050. It is just a personal approach.
		if (gyro_z < gz_threshold && gyro_z > -gz_threshold) // When gyro stands ignore the gyro z small fluctuations to prevent z angle irregular increments
			gyro_z = 0;

		gyro_angle_z = gyro_z * dt + get_last_z_angle();

		// Apply the complementary filter to figure out the change in angle - choice of alpha is
		/* estimated now.  Alpha depends on the sampling rate... */

		angle_x = alpha * gyro_angle_x + (1.0 - alpha) * accel_angle_x;
		angle_y = alpha * gyro_angle_y + (1.0 - alpha) * accel_angle_y;
		angle_z = gyro_angle_z;  //Accelerometer doesn't give z-angle

		/* Update the saved data with the latest values */
		set_last_read_angle_data(angle_x, angle_y, angle_z);

		a1.x = (int) angle_x;
		a1.y = (int) angle_y;
		a1.z = (int) angle_z;
		//		print("2");
		status = osMessageQueuePut(anglesQueueHandle, &a1, 0U, osWaitForever);
		if (status != osOK)
			osThreadYield();

		else
			osThreadResume(senderTaskHandle);
	  }
		osDelay(20);
	}
  /* USER CODE END ComputationTask */
}

/* USER CODE BEGIN Header_startSenderTask */
/**
* @brief Function implementing the senderTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_startSenderTask */
void SenderTask(void *argument)
{
  /* USER CODE BEGIN SenderTask */
	char data[50] = { 0 };
	angles a2;
	osStatus_t status;
	int i = 0;
  /* Infinite loop */
  for(;;)
  {
	status = osMessageQueueGet(anglesQueueHandle, &a2, 0U, osWaitForever);
	if( status != osOK )
	  osThreadYield(); 			/* unless be get, it will be on ComputationTask */

	else
	{
	  sprintf(data, "%d#%d#%d", a2.x, a2.y, a2.z);
	  if(i % 10 == 0)
		  HAL_UART_Transmit_IT(&huart2, (uint8_t *)data, sizeof(data));  //0.55m.sn

	  i++;
	  osDelay(20);
	}
  }
  /* USER CODE END SenderTask */
}

/* USER CODE BEGIN Header_ReadLIS302Task */
/**
* @brief Function implementing the readLIS302Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ReadLIS302Task */
void ReadLIS302Task(void *argument)
{
  /* USER CODE BEGIN startReadLIS302Task */
	TM_LIS302DL_LIS3DSH_Device_t statusReadLIS302 = 1;
	TM_MPU6050_Result_t statusReadMPU6050;
	TM_LIS302DL_LIS3DSH_t Axes_Data;
	osStatus_t statusQueue;
  /* Infinite loop */
  for(;;)
  {
	  statusReadMPU6050 = TM_MPU6050_ReadAll(&MPU6050_Sensor);
	  		if (statusReadMPU6050 == TM_MPU6050_Result_Ok)
	  			osThreadResume(readMPU6050TaskHandle);
		TM_LIS302DL_LIS3DSH_ReadAxes(&Axes_Data);
		if (statusReadLIS302 != TM_LIS302DL_LIS3DSH_Device_Error)
		{
			statusQueue = osMessageQueuePut(rowLIS302DataQueueHandle, &Axes_Data, 0U, 0U);
			if (statusQueue != osOK)
				osThreadYield();
			 else
			 {
				osThreadSuspend(readMPU6050TaskHandle);
				osThreadResume(kalmanFilterHandle);
			 }
		}
		else
			osThreadResume(readMPU6050TaskHandle);

		osDelay(20);
	}
  /* USER CODE END startReadLIS302Task */
}

/* USER CODE BEGIN Header_startKalmanFilter */
/**
* @brief Function implementing the kalmanFilter thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_startKalmanFilter */
void KalmanFilter(void *argument)
{
  /* USER CODE BEGIN startKalmanFilter */
	TM_LIS302DL_LIS3DSH_t Axes_Data;
	osStatus_t statusQueue;
	angles b1;
	/* Infinite loop */
	for (;;) {
		statusQueue = osMessageQueueGet(rowLIS302DataQueueHandle, &Axes_Data,
				0U, osWaitForever);
		if (statusQueue != osOK)
			osThreadResume(readMPU6050TaskHandle); /* unless be get, it will be on startComputationTask */
		else
		{
			b1.x = kalman_update(Axes_Data.X, 0.001);
			b1.y = kalman_update(Axes_Data.Y, 0.001);
			b1.z = kalman_update(Axes_Data.Z, 0.001);
			statusQueue = osMessageQueuePut(anglesQueueHandle, &b1, 0U, osWaitForever);
			if (statusQueue != osOK)
				osThreadResume(readMPU6050TaskHandle);
			 else
				osThreadResume(senderTaskHandle);
		}
		osDelay(20);
	}
  /* USER CODE END startKalmanFilter */
}

/* Callback01 function */
void Callback01(void *argument)
{
  /* USER CODE BEGIN Callback01 */

  /* USER CODE END Callback01 */
}

 /**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM10 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
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
