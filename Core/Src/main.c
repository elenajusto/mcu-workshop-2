/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "infrared_pd.h"
#include <string.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define IPD_STR_LENG 35
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
char msg[23];				// UART Message Buffer
char readI2C[9];			// Read buffer for I2C
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CRC_HandleTypeDef hcrc;

I2C_HandleTypeDef hi2c3;

RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim11;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

// IR Sensor Data Variables
int16_t ObjectTempComp;
int16_t ObjectTempCompChange;
uint8_t MotionDetected;
uint8_t PresenceDetected;

// IR Sensor Configuration Variables
IPD_Instance_t IPD_Instance;
IPD_mcu_type_t mcu = IPD_MCU_STM32;
IPD_algo_conf_t algo_conf;
IPD_device_conf_t device_conf;
IPD_init_err_t status;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CRC_Init(void);
static void MX_I2C3_Init(void);
static void MX_RTC_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM11_Init(void);
/* USER CODE BEGIN PFP */
void startTimer();
int32_t ReadAmbientSensor();
void powerDown();
void cotMode();
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
  MX_CRC_Init();
  MX_I2C3_Init();
  MX_RTC_Init();
  MX_USART2_UART_Init();
  MX_TIM11_Init();
  /* USER CODE BEGIN 2 */

  // Start Timer
  startTimer();

  // Initialise infraredPD instance
  InfraredPD_Initialize(mcu);
  IPD_Instance = InfraredPD_CreateInstance(&algo_conf);

  device_conf.odr = 30;
  device_conf.avg_tmos = 32;
  device_conf.avg_t = 8;

  status = InfraredPD_Start(IPD_Instance, &device_conf, &algo_conf);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	sprintf(msg, "ObjectTemp: %u\n\r", &ObjectTempComp);
	HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 100000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

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
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
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
  * @brief TIM11 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM11_Init(void)
{

  /* USER CODE BEGIN TIM11_Init 0 */

  /* USER CODE END TIM11_Init 0 */

  /* USER CODE BEGIN TIM11_Init 1 */

  /* USER CODE END TIM11_Init 1 */
  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 319;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 999;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM11_Init 2 */

  /* USER CODE END TIM11_Init 2 */

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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim->Instance == TIM11){
		// InfraredPD functions
		IPD_input_t data_in;
		IPD_output_t data_out;

		data_in.t_amb = ReadAmbientSensor();
		//data_in.t_obj = ReadSensor();

		InfraredPD_Update(&IPD_Instance, &data_in, &data_out);

		ObjectTempComp = data_out.t_obj_comp;
		ObjectTempCompChange = data_out.t_obj_change;
		MotionDetected = data_out.mot_flag;
		PresenceDetected = data_out.pres_flag;
	}
}

int32_t ReadAmbientSensor(){
	int32_t ambient;

	// Enter power down mode
	powerDown();

	// Enable access to the embedded functions registers


	// Select read operation mode


	// Set address XXh of the embedded functions register to be read


	// Get register value


	// Disable read operation mode


	// Disable access to the embedded functions registers


	// Enter continuous mode
	cotMode();

	return ambient;
}

void powerDown(){

	char rmsg[8];				 // Read buffer for i2c
    uint8_t dataToWrite = 0x00;  // Data to set ODR[3:0] bits to 0000

	// Read the FUNC_STATUS (25h) register
	HAL_I2C_Mem_Read(&hi2c3, 0xF, 0x25, I2C_MEMADD_SIZE_8BIT, (uint8_t*)rmsg, strlen(rmsg)+1, HAL_MAX_DELAY);

	// Wait that the DRDY bit of the STATUS (23h) register is set to 1
	HAL_Delay(5);

	// Set the ODR[3:0] bits of the CTRL1 (20h) register to 0000
	HAL_I2C_Mem_Write(&hi2c3, 0xF, 0x20, I2C_MEMADD_SIZE_8BIT, &dataToWrite, 1, HAL_MAX_DELAY);

	// Read the FUNC_STATUS (25h) register
	HAL_I2C_Mem_Read(&hi2c3, 0xF, 0x25, I2C_MEMADD_SIZE_8BIT, (uint8_t*)rmsg, strlen(rmsg)+1, HAL_MAX_DELAY);
}

void cotMode(){
	uint8_t dataToCTRL2 = 0x10;
	uint8_t dataToPAGE = 0x64;
	uint8_t dataToFUNC_CFG_ADDR = 0x2A;
	uint8_t dataToFUNC_CFG_DATA = 0x1;
	uint8_t dataToFUNC_CFG_WRITE = 0x0;
	uint8_t dataToFUNC_CFG_ACCESS = 0x0;
	uint8_t dataToCTRL1 = 0x7;

	// Write bit FUNC_CFG_ACCESS = 1 in CTRL2 (21h)
	HAL_I2C_Mem_Write(&hi2c3, 0xF, 0x21, I2C_MEMADD_SIZE_8BIT, &dataToCTRL2, 1, HAL_MAX_DELAY);

	// Write bit FUNC_CFG_WRITE = 1 in PAGE_RW (11h)
	HAL_I2C_Mem_Write(&hi2c3, 0xF, 0x11, I2C_MEMADD_SIZE_8BIT, &dataToPAGE, 1, HAL_MAX_DELAY);

	// Write 2Ah in FUNC_CFG_ADDR (08h)
	HAL_I2C_Mem_Write(&hi2c3, 0xF, 0x08, I2C_MEMADD_SIZE_8BIT, &dataToFUNC_CFG_ADDR, 1, HAL_MAX_DELAY);

	// Write 01h in FUNC_CFG_DATA (09h)
	HAL_I2C_Mem_Write(&hi2c3, 0xF, 0x09, I2C_MEMADD_SIZE_8BIT, &dataToFUNC_CFG_DATA, 1, HAL_MAX_DELAY);

	// Write bit FUNC_CFG_WRITE = 0 in PAGE_RW (11h)
	HAL_I2C_Mem_Write(&hi2c3, 0xF, 0x11, I2C_MEMADD_SIZE_8BIT, &dataToFUNC_CFG_WRITE, 1, HAL_MAX_DELAY);

	// Write bit FUNC_CFG_ACCESS = 0 in CTRL2 (21h)
	HAL_I2C_Mem_Write(&hi2c3, 0xF, 0x21, I2C_MEMADD_SIZE_8BIT, &dataToFUNC_CFG_ACCESS, 1, HAL_MAX_DELAY);

	// Write bits ODR[3:0] in CTRL1 (20h) with the desired value
	HAL_I2C_Mem_Write(&hi2c3, 0xF, 0x20, I2C_MEMADD_SIZE_8BIT, &dataToCTRL1, 1, HAL_MAX_DELAY);
}

void startTimer(){
	 // Enable the TIM11 peripheral
	  __HAL_RCC_TIM11_CLK_ENABLE();

	  // Start the timers
	  HAL_TIM_Base_Start_IT(&htim11);
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
