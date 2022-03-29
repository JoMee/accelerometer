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
#include "stdio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef enum {
  	LSM6DSL_IF_INC_DISABLED =	0x00,
  	LSM6DSL_IF_INC_ENABLED 	=	0x04,
} LSM6DSL_IF_INC_t;

typedef enum {
  	LSM6DSL_BDU_DISABLED 	=	0x00,
  	LSM6DSL_BDU_ENABLED 	=	0x40,
} LSM6DSL_BDU_t;

typedef enum {
  	LSM6DSL_FIFO_MODE_BYPASS 		 	=	0x00,
  	LSM6DSL_FIFO_MODE_FIFO 		 		=	0x01,
  	LSM6DSL_FIFO_MODE_STREAM 		 	=	0x02,
  	LSM6DSL_FIFO_MODE_STF 			 	=	0x03,
  	LSM6DSL_FIFO_MODE_BTS			 	=	0x04,
  	LSM6DSL_FIFO_MODE_DYN_STREAM    	=	0x05,
  	LSM6DSL_FIFO_MODE_DYN_STREAM_2 		=	0x06,
  	LSM6DSL_FIFO_MODE_BTF 		 		=	0x07,
} LSM6DSL_FIFO_MODE_t;

typedef enum {
  	LSM6DSL_ODR_XL_POWER_DOWN 	=0x00,
  	LSM6DSL_ODR_XL_13Hz 		=0x10,
  	LSM6DSL_ODR_XL_26Hz 		=0x20,
  	LSM6DSL_ODR_XL_52Hz 		=0x30,
  	LSM6DSL_ODR_XL_104Hz 		=0x40,
  	LSM6DSL_ODR_XL_208Hz 		=0x50,
  	LSM6DSL_ODR_XL_416Hz 		=0x60,
  	LSM6DSL_ODR_XL_833Hz 		=0x70,
  	LSM6DSL_ODR_XL_1660Hz 		=0x80,
  	LSM6DSL_ODR_XL_3330Hz 		=0x90,
  	LSM6DSL_ODR_XL_6660Hz 		=0xA0,
} LSM6DSL_ODR_XL_t;

typedef enum {
  	LSM6DSL_FS_XL_2g 		 	=0x00,
  	LSM6DSL_FS_XL_16g 		 	=0x04,
  	LSM6DSL_FS_XL_4g 		 	=0x08,
  	LSM6DSL_FS_XL_8g 		 	=0x0C,
} LSM6DSL_FS_XL_t;


typedef enum {
  	LSM6DSL_ODR_G_POWER_DOWN 	=0x00,
  	LSM6DSL_ODR_G_13Hz 		 	=0x10,
  	LSM6DSL_ODR_G_26Hz 		 	=0x20,
  	LSM6DSL_ODR_G_52Hz 		 	=0x30,
  	LSM6DSL_ODR_G_104Hz 		=0x40,
  	LSM6DSL_ODR_G_208Hz 		=0x50,
  	LSM6DSL_ODR_G_416Hz 		=0x60,
  	LSM6DSL_ODR_G_833Hz 		=0x70,
  	LSM6DSL_ODR_G_1660Hz 		=0x80,
  	LSM6DSL_ODR_G_3330Hz 		=0x90,
  	LSM6DSL_ODR_G_6660Hz 		=0xA0,
} LSM6DSL_ODR_G_t;

typedef enum {
  	LSM6DSL_FS_125_DISABLED 	=0x00,
  	LSM6DSL_FS_125_ENABLED 		=0x02,
} LSM6DSL_FS_125_t;

typedef enum {
  	LSM6DSL_FS_G_245dps 		=0x00,
  	LSM6DSL_FS_G_500dps 		=0x04,
  	LSM6DSL_FS_G_1000dps 		=0x08,
  	LSM6DSL_FS_G_2000dps 		=0x0C,
} LSM6DSL_FS_G_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define LSM6DSL_ADDRESS 		0xD4
#define LSM6DSL_ADDRESS_WRITE 	0xD4
#define LSM6DSL_ADDRESS_READ 	(0xD4 | 0x01)

#define LSM6DSL_CTRL3_C 		0x12
#define LSM6DSL_FIFO_CTRL5  	0X0A
#define LSM6DSL_CTRL1_XL		0x10
#define LSM6DSL_CTRL2_G			0x11
#define LSM6DSL_OUTX_L_XL  		0X28


#define LSM6DSL_IF_INC_MASK 	0x04
#define LSM6DSL_BDU_MASK		0x40
#define LSM6DSL_FIFO_MODE_MASK  0x07
#define LSM6DSL_ODR_XL_MASK  	0xF0
#define LSM6DSL_FS_XL_MASK		0x0C
#define LSM6DSL_ODR_G_MASK		0xF0
#define LSM6DSL_FS_125_MASK  	0x02
#define LSM6DSL_FS_G_MASK  		0x0C

#define LSM6DSL_ACC_SENSITIVITY_FOR_FS_2G   0.061

#define X_ODR 104.0f


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


/* CONFIGURE SENSOR */
HAL_StatusTypeDef LSM6DSL_SET_IF_INC(LSM6DSL_IF_INC_t new_value)
{
	uint8_t value;

	if ( HAL_I2C_Mem_Read(&hi2c2, LSM6DSL_ADDRESS_READ, LSM6DSL_CTRL3_C, 1, &value, 1, 20) == HAL_ERROR )
	{
		return HAL_ERROR;
	}

	value &= ~LSM6DSL_IF_INC_MASK;
	value |= new_value;

	if ( HAL_I2C_Mem_Write(&hi2c2, LSM6DSL_ADDRESS_WRITE, LSM6DSL_CTRL3_C, 1, &value, 1, 20) == HAL_ERROR )
	{
		return HAL_ERROR;
	}

	return HAL_OK;
}

HAL_StatusTypeDef LSM6DSL_SET_BDU(LSM6DSL_BDU_t new_value)
{
	uint8_t value;

	if ( HAL_I2C_Mem_Read(&hi2c2, LSM6DSL_ADDRESS_READ, LSM6DSL_CTRL3_C, 1, &value, 1, 20) == HAL_ERROR )
	{
		return HAL_ERROR;
	}

	value &= ~LSM6DSL_BDU_MASK;
	value |= new_value;

	if ( HAL_I2C_Mem_Write(&hi2c2, LSM6DSL_ADDRESS_WRITE, LSM6DSL_CTRL3_C, 1, &value, 1, 20) == HAL_ERROR )
	{
		return HAL_ERROR;
	}

	return HAL_OK;
}

HAL_StatusTypeDef LSM6DSL_SET_FIFO_MODE(LSM6DSL_FIFO_MODE_t new_value)
{
	uint8_t value;

	if ( HAL_I2C_Mem_Read(&hi2c2, LSM6DSL_ADDRESS_READ, LSM6DSL_FIFO_CTRL5, 1, &value, 1, 20) == HAL_ERROR )
	{
		return HAL_ERROR;
	}

	value &= ~LSM6DSL_FIFO_MODE_MASK;
	value |= new_value;

	if ( HAL_I2C_Mem_Write(&hi2c2, LSM6DSL_ADDRESS_WRITE, LSM6DSL_FIFO_CTRL5, 1, &value, 1, 20) == HAL_ERROR )
	{
		return HAL_ERROR;
	}

	return HAL_OK;
}

HAL_StatusTypeDef LSM6DSL_SET_ODR_XL(LSM6DSL_ODR_XL_t new_value)
{
	uint8_t value;

	if ( HAL_I2C_Mem_Read(&hi2c2, LSM6DSL_ADDRESS_READ, LSM6DSL_CTRL1_XL, 1, &value, 1, 20) == HAL_ERROR )
	{
		return HAL_ERROR;
	}

	value &= ~LSM6DSL_ODR_XL_MASK;
	value |= new_value;

	if ( HAL_I2C_Mem_Write(&hi2c2, LSM6DSL_ADDRESS_WRITE, LSM6DSL_CTRL1_XL, 1, &value, 1, 20) == HAL_ERROR )
	{
		return HAL_ERROR;
	}

	return HAL_OK;
}

HAL_StatusTypeDef LSM6DSL_SET_FS_XL(LSM6DSL_FS_XL_t new_value)
{
	uint8_t value;

	if ( HAL_I2C_Mem_Read(&hi2c2, LSM6DSL_ADDRESS_READ, LSM6DSL_CTRL1_XL, 1, &value, 1, 20) == HAL_ERROR )
	{
		return HAL_ERROR;
	}

	value &= ~LSM6DSL_FS_XL_MASK;
	value |= new_value;

	if ( HAL_I2C_Mem_Write(&hi2c2, LSM6DSL_ADDRESS_WRITE, LSM6DSL_CTRL1_XL, 1, &value, 1, 20) == HAL_ERROR )
	{
		return HAL_ERROR;
	}

	return HAL_OK;
}

HAL_StatusTypeDef LSM6DSL_SET_ODR_G(LSM6DSL_ODR_G_t new_value)
{
	uint8_t value;

	if ( HAL_I2C_Mem_Read(&hi2c2, LSM6DSL_ADDRESS_READ, LSM6DSL_CTRL2_G, 1, &value, 1, 20) == HAL_ERROR )
	{
		return HAL_ERROR;
	}

	value &= ~LSM6DSL_ODR_G_MASK;
	value |= new_value;

	if ( HAL_I2C_Mem_Write(&hi2c2, LSM6DSL_ADDRESS_WRITE, LSM6DSL_CTRL2_G, 1, &value, 1, 20) == HAL_ERROR )
	{
		return HAL_ERROR;
	}

	return HAL_OK;
}



HAL_StatusTypeDef LSM6DSL_SET_FS_125(LSM6DSL_FS_125_t new_value)
{
	uint8_t value;

	if ( HAL_I2C_Mem_Read(&hi2c2, LSM6DSL_ADDRESS_READ, LSM6DSL_CTRL2_G, 1, &value, 1, 20) == HAL_ERROR )
	{
		return HAL_ERROR;
	}

	value &= ~LSM6DSL_FS_125_MASK;
	value |= new_value;

	if ( HAL_I2C_Mem_Write(&hi2c2, LSM6DSL_ADDRESS_WRITE, LSM6DSL_CTRL2_G, 1, &value, 1, 20) == HAL_ERROR )
	{
		return HAL_ERROR;
	}

	return HAL_OK;
}

HAL_StatusTypeDef LSM6DSL_SET_FS_G(LSM6DSL_FS_G_t new_value)
{
	uint8_t value;

	if ( HAL_I2C_Mem_Read(&hi2c2, LSM6DSL_ADDRESS_READ, LSM6DSL_CTRL2_G, 1, &value, 1, 20) == HAL_ERROR )
	{
		return HAL_ERROR;
	}

	value &= ~LSM6DSL_FS_G_MASK;
	value |= new_value;

	if ( HAL_I2C_Mem_Write(&hi2c2, LSM6DSL_ADDRESS_WRITE, LSM6DSL_CTRL2_G, 1, &value, 1, 20) == HAL_ERROR )
	{
		return HAL_ERROR;
	}

	return HAL_OK;
}


HAL_StatusTypeDef LSM6DSL_GET_RAW_ACCELEROMETER(uint8_t *buff)
{
	int offset = 0;

	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 2; j++) {
			if ( HAL_I2C_Mem_Read(&hi2c2, LSM6DSL_ADDRESS_READ, LSM6DSL_OUTX_L_XL + offset, 1, &buff[offset], 1, 20) == HAL_ERROR)
			{
				return HAL_ERROR;
			}
			offset++;
		}
	}

	return HAL_OK;
}



HAL_StatusTypeDef CONFIGURE_I2C_SENSOR()
{
	/* Enable register address automatically increasing */
	if ( LSM6DSL_SET_IF_INC(LSM6DSL_IF_INC_ENABLED) == HAL_ERROR)
	{
		return HAL_ERROR;
	}

	/* Enable BDU */
	if ( LSM6DSL_SET_BDU(LSM6DSL_BDU_ENABLED) == HAL_ERROR )
	{
		return HAL_ERROR;
	}

	/* SET FIFO MODE to BYPASS */
	if ( LSM6DSL_SET_FIFO_MODE(LSM6DSL_FIFO_MODE_BYPASS) == HAL_ERROR )
	{
		return HAL_ERROR;
	}

	if ( LSM6DSL_SET_FS_XL(LSM6DSL_FS_XL_2g) == HAL_ERROR )
	{
		return HAL_ERROR;
	}

	if ( LSM6DSL_SET_ODR_G(LSM6DSL_ODR_G_POWER_DOWN) == HAL_ERROR )
	{
		return HAL_ERROR;
	}

	if ( LSM6DSL_SET_FS_125(LSM6DSL_FS_125_DISABLED) == HAL_ERROR )
	{
		return HAL_ERROR;
	}

	if ( LSM6DSL_SET_FS_G(LSM6DSL_FS_G_2000dps) == HAL_ERROR )
	{
		return HAL_ERROR;
	}

	if ( LSM6DSL_SET_ODR_XL(LSM6DSL_ODR_XL_104Hz) == HAL_ERROR )
	{
		return HAL_ERROR;
	}

	return HAL_OK;
}


HAL_StatusTypeDef READ_ACCELEROMETER(int32_t *pData)
{
	 uint8_t dataRaw[6];

	 if ( LSM6DSL_GET_RAW_ACCELEROMETER(dataRaw) == HAL_ERROR )
	 {
		 return HAL_ERROR;
	 }

	 float sensitivity = LSM6DSL_ACC_SENSITIVITY_FOR_FS_2G;

	 pData[0] = (int32_t)( ( (( (int16_t)dataRaw[1] ) << 8) | (int16_t)dataRaw[0] ) * sensitivity);
	 pData[1] = (int32_t)(( ( ((int16_t)dataRaw[3] ) << 8) | (int16_t)dataRaw[2] ) * sensitivity);
	 pData[2] = (int32_t)(( (((int16_t)dataRaw[5] ) << 8) | (int16_t)dataRaw[4] ) * sensitivity);

	 return HAL_OK;
}

int _write(int fd, char* ptr, int len) {
    HAL_UART_Transmit(&huart1, (uint8_t *) ptr, len, HAL_MAX_DELAY);
    return len;
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
	 uint8_t slave_array[256];
	 memset(slave_array, 0, 265);
	 uint8_t regValue[6] = {0, 0, 0, 0, 0, 0};
	 int32_t data[3];

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_I2C2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);

  for (int i = 0; i < 255; i++) {
	  slave_array[i] = HAL_I2C_IsDeviceReady(&hi2c2, i<<1, 1, 10) == HAL_OK;
  }

  if (HAL_I2C_IsDeviceReady(&hi2c2, LSM6DSL_ADDRESS, 1, 10) == HAL_OK) {
	 HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin);
  }
  CONFIGURE_I2C_SENSOR();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
	 READ_ACCELEROMETER(data);
	 printf("%ld\r\n", data[0]);

	  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, data[0] / 10);


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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x00000E14;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 127;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 625;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_GREEN_Pin */
  GPIO_InitStruct.Pin = LED_GREEN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GREEN_GPIO_Port, &GPIO_InitStruct);

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

