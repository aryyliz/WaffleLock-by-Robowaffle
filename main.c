/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */


#define BME280_I2C_ADDRESS (0x76<<1) // por default segun la PCB que tenemos
#define BME280_REG_CHIP_ID 0xD0
#define BME280_REG_RESET 0xE0
#define BME280_REG_CTRL_HUM 0xF2
#define BME280_REG_STATUS 0xF3
#define BME280_REG_CTRL_MEAS 0xF4
#define BME280_REG_CONFIG 0xF5
#define BME280_REG_T1 0x88
#define BME280_REG_T2 0x8A
#define BME280_REG_T3 0x8C
#define BME280_REG_TEMP_MSB 0xFA
#define BME280_CHIP_ID_VALUE 0x60

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim6;

/* USER CODE BEGIN PV */

typedef struct{
	uint16_t dig_T1;
	int16_t dig_T2;
	int16_t dig_T3;
}BME_Calibration_Data;

typedef struct{
	uint8_t ID;
	uint8_t deviceReady;
	BME_Calibration_Data coeficientes;
	float temperature;
}BME_Config;

static BME_Config devBME280;
static uint8_t bmeTempRawBuf[3];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM6_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
uint8_t readDigitalValueAir(void);
uint8_t readTempSensor();
void TempSensGoToNormal(void);
void checkTempSensorID(void);
static int BME280_ReadCalibration(void);
static void BME280_ProcessTemperature(int32_t rawTemp);
static void BME280_Init(void);
static float BME280_ReadTemperature(void);

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
  MX_TIM6_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  BME280_Init();
  HAL_TIM_Base_Start_IT(&htim6);
  /* USER CODE END 2 */

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_CSI;
  RCC_OscInitStruct.CSIState = RCC_CSI_ON;
  RCC_OscInitStruct.CSICalibrationValue = RCC_CSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLL1_SOURCE_CSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 50;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1_VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1_VCORANGE_WIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_PCLK3;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the programming delay
  */
  __HAL_FLASH_SET_PROGRAM_DELAY(FLASH_PROGRAMMING_DELAY_2);
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
  hi2c1.Init.Timing = 0x009034B6;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 9999;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 9999;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USER_LED_GPIO_Port, USER_LED_Pin, GPIO_PIN_RESET);
// cuidado de que no este en set
  /*Configure GPIO pin : USER_LED_Pin */
  GPIO_InitStruct.Pin = USER_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(USER_LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SensorDigitalAire_Pin */
  GPIO_InitStruct.Pin = SensorDigitalAire_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SensorDigitalAire_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */



uint8_t readDigitalValueAir(){
	GPIO_PinState state = HAL_GPIO_ReadPin(SensorDigitalAire_GPIO_Port, SensorDigitalAire_Pin);
	if (state == GPIO_PIN_RESET){
		return 1u;
	}
	else{
		return 0u;
	}
}
//Reutilizando código de practicas y proyectos anteriores en C

static int BME280_ReadCalibration(void){
	uint8_t buf[6] = {0};

	if (HAL_I2C_Mem_Read(&hi2c1, BME280_I2C_ADDRESS, BME280_REG_T1, 1, buf, 6, 100) != HAL_OK){
		devBME280.deviceReady = 0u;
		return -1;
	}
	devBME280.coeficientes.dig_T1= (uint16_t)(buf[0] | (buf[1]<< 8));
	devBME280.coeficientes.dig_T2= (int16_t)(buf[2] | (buf[3]<< 8));
	devBME280.coeficientes.dig_T3= (int16_t)(buf[4] | (buf[5]<< 8));
	return 0;
}

static void BME280_ProcessTemperature(int32_t rawTemp){
	int32_t var1 = 0;
	int32_t var2 = 0;
	int32_t resultTemp = 0;
	int32_t t_fine = 0;

	var1 = ((((rawTemp >> 3)- ((int32_t)devBME280.coeficientes.dig_T1 << 1))*(int32_t)devBME280.coeficientes.dig_T2) >> 11);

	var2 = (((((rawTemp >> 4) - (int32_t)devBME280.coeficientes.dig_T1)* ((rawTemp >> 4)- (int32_t)devBME280.coeficientes.dig_T1)) >> 12)* (int32_t)devBME280.coeficientes.dig_T3) >> 14;

	resultTemp= var1 + var2;
	t_fine= resultTemp;
	(void)t_fine;
	resultTemp= (resultTemp*5 + 128)>> 8;

	devBME280.temperature = ((float)resultTemp)/100.0f;
}

static void BME280_Init(void){
	uint8_t chip_id = 0;
	uint8_t reset_cmd = 0xB6;
	uint8_t ctrl_hum = 0x01;
	uint8_t config= 0x00;
	uint8_t ctrl_meas = 0x27;
	devBME280.deviceReady = 0u;
	devBME280.temperature = 0.0f;

	HAL_Delay(10);
	if (HAL_I2C_Mem_Read(&hi2c1, BME280_I2C_ADDRESS, BME280_REG_CHIP_ID, 1, &chip_id, 1, 100) != HAL_OK){
		return;
	}

	devBME280.ID = chip_id;
	if (chip_id != BME280_CHIP_ID_VALUE){
		return;
	}
	if (HAL_I2C_Mem_Write(&hi2c1, BME280_I2C_ADDRESS, BME280_REG_RESET, 1, &reset_cmd, 1, 100) != HAL_OK){
		return;
	}
	HAL_Delay(10);
	if (BME280_ReadCalibration() != 0){
		return;
	}

	HAL_I2C_Mem_Write(&hi2c1, BME280_I2C_ADDRESS, BME280_REG_CTRL_HUM, 1, &ctrl_hum, 1, 50);
	HAL_I2C_Mem_Write(&hi2c1, BME280_I2C_ADDRESS, BME280_REG_CONFIG, 1, &config, 1, 50);
	HAL_I2C_Mem_Write(&hi2c1, BME280_I2C_ADDRESS, BME280_REG_CTRL_MEAS, 1, &ctrl_meas, 1, 50);
	devBME280.deviceReady = 1u;
}

static float BME280_ReadTemperature(void){
	if(devBME280.deviceReady == 0u){
		return -100.0f;
	}
	if(HAL_I2C_Mem_Read(&hi2c1, BME280_I2C_ADDRESS, BME280_REG_TEMP_MSB, 1, bmeTempRawBuf, 3, 100) != HAL_OK){
		return -100.0f;
	}
	// buffer para leer los 3 bytes de info necesarias
	uint8_t tOneData   = bmeTempRawBuf[0];
	uint8_t tTwoData   = bmeTempRawBuf[1];
	uint8_t tThreeData = bmeTempRawBuf[2];
	int32_t rawTemp = ((int32_t)tOneData << 12) | ((int32_t)tTwoData << 4) | (tThreeData >> 4);
	BME280_ProcessTemperature(rawTemp);
	return devBME280.temperature;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim->Instance == TIM6){
		uint8_t mqState = readDigitalValueAir();
		float tempC = BME280_ReadTemperature();

		if ((mqState == 1u) && (tempC >= 45.0f)){//si hay gases detectados en el umbral actual y la temperatura es mayor o igual a 45 C°
			HAL_GPIO_WritePin(USER_LED_GPIO_Port, USER_LED_Pin, 1); //Activa LED (se usará buzzer en futura implementación)
		}
		else{
			HAL_GPIO_WritePin(USER_LED_GPIO_Port, USER_LED_Pin, 0);
		}
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
#ifdef USE_FULL_ASSERT
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
