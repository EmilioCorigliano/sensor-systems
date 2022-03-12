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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/**
 * @brief enumeration that contains the association between register and their address
 */
typedef enum {
	WHO_AM_I = 0x0F,
	TEMP_CFG_REG = 0x1F,
	CTRL_REG1 = 0x20,
	CTRL_REG4 = 0x23,
	OUT_X = 0x29,
	OUT_Y = 0x2B,
	OUT_Z = 0x2D,
	OUT_TEMP_L = 0x0C,
	OUT_TEMP_H = 0x0D
} Reg;

/**
 * @brief data structure that contains a sample from the accelerometer
 */
typedef struct {
	float acc_x;
	float acc_y;
	float acc_z;
	float acc_t;
} AccelData;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define accel_addr (0b0101000<<1) 	// address of the LIS2DE (shift by one for the R/W bit)
#define SELF_TEST_MIN_CHANGE 50		// minimum change of the acceleration
#define SELF_TEST_MAX_CHANGE 1800	// maximum change of the acceleration
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim11;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */
AccelData accelData; 	// structure that contains the last sample
uint8_t data;			// variable that contains the data to send or to receive
char str[64];			// string that contains the message to send over UART
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM11_Init(void);
/* USER CODE BEGIN PFP */
HAL_StatusTypeDef init();
HAL_StatusTypeDef sample(AccelData *accelData);
HAL_StatusTypeDef writeReg(Reg regAddress, int8_t data);
HAL_StatusTypeDef readReg(Reg regAddress, uint8_t *data, int nData);
float convertToFloat(int8_t msb, uint8_t lsb);
float convertToAcc(int8_t data);
void printSampleDMA(AccelData accelData);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
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
	MX_DMA_Init(); // in order to fix a bug in HAL library
	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_USART2_UART_Init();
	MX_DMA_Init();
	MX_I2C1_Init();
	MX_TIM11_Init();
	/* USER CODE BEGIN 2 */
	// initializing the sensor and, if succeeded, initialize timer
	if (init() == HAL_OK) {
		strcpy(str, "Accelerometer initialized!\r\n");
		HAL_UART_Transmit_DMA(&huart2, str, strlen(str));

		// initializing timer
		HAL_TIM_Base_Start_IT(&htim11);
	} else {
		strcpy(str, "Failed to initialize accelerometer!\r\n");
		HAL_UART_Transmit_DMA(&huart2, str, strlen(str));
	}
	/* USER CODE END 2 */

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
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 16;
	RCC_OscInitStruct.PLL.PLLN = 336;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
	RCC_OscInitStruct.PLL.PLLQ = 7;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void) {

	/* USER CODE BEGIN I2C1_Init 0 */

	/* USER CODE END I2C1_Init 0 */

	/* USER CODE BEGIN I2C1_Init 1 */

	/* USER CODE END I2C1_Init 1 */
	hi2c1.Instance = I2C1;
	hi2c1.Init.ClockSpeed = 100000;
	hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN I2C1_Init 2 */

	/* USER CODE END I2C1_Init 2 */

}

/**
 * @brief TIM11 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM11_Init(void) {

	/* USER CODE BEGIN TIM11_Init 0 */

	/* USER CODE END TIM11_Init 0 */

	/* USER CODE BEGIN TIM11_Init 1 */

	/* USER CODE END TIM11_Init 1 */
	htim11.Instance = TIM11;
	htim11.Init.Prescaler = 2000;
	htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim11.Init.Period = 42000;
	htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim11) != HAL_OK) {
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
static void MX_USART2_UART_Init(void) {

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
	if (HAL_UART_Init(&huart2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART2_Init 2 */

	/* USER CODE END USART2_Init 2 */

}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {

	/* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA1_Stream6_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

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
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : B1_Pin */
	GPIO_InitStruct.Pin = B1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : LD2_Pin */
	GPIO_InitStruct.Pin = LD2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
/**
 * @brief overriding the function in order to sample and send data with DMA
 * every period elapsed
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim == &htim11) {
		sample(&accelData);
		printSampleDMA(accelData);
	}
}

/**
 * @brief initializes the accelerometer
 * @return HAL_OK if the sensor has been configured, HAL_ERROR otherwise
 */
HAL_StatusTypeDef init() {
	// enabling normal mode (1Hz) and enabling X,Y,Z outputs
	if (writeReg(CTRL_REG1, 0b00010111) != HAL_OK) {
		return HAL_ERROR;
	}

	// setting the +/- 2g FSR, Self Test disabled
	if (writeReg(CTRL_REG4, 0b00000000) != HAL_OK) {
		return HAL_ERROR;
	}

	// enabling the temperature sensor
	if (writeReg(TEMP_CFG_REG, 0b11000000) != HAL_OK) {
		return HAL_ERROR;
	}

	// reading the who_am_i register to check if it's the correct sensor
	if (readReg(WHO_AM_I, &data, 1) == HAL_OK && data == 0b00110011) {
		return HAL_OK;
	} else {
		return HAL_ERROR;
	}
}

/**
 * @brief converts a byte into the acceleration in g
 * @param data the byte read from the accelerometer
 * @return the float signed value of the acceleration in g
 */
float convertToAcc(int8_t data) {
	// multiplying the read data by the FSR and dividing by 2^bytes=256
	return (float) data * 4 / 256.0;
}

/**
 * @brief converts two bytes in a float value
 * @param msb the most significant byte of the temperature
 * @param lsb the least significant byte of the temperature
 * @return the float signed value of the acceleration in g
 */
float convertToFloat(int8_t msb, uint8_t lsb) {
	// "concatenates" the two bytes and then divides them by 2^(decimal places), that is 256
	// incremented by 25 (doubt on incrementing it by 22.5) because seems the offset used
	return ((float) (((int16_t) (msb) << 8) | lsb)) / 256.0 + 22.5;
}

/**
 * @brief writes in the register passed the data
 * @param regAddress the address on which we want to write
 * @param data the byte that will be written in the register
 * @return HAL_OK if sensor acknowledged, HAL_ERROR otherwise
 */
HAL_StatusTypeDef writeReg(Reg regAddress, int8_t data) {
	uint8_t datar[2] = { regAddress, data };
	return HAL_I2C_Master_Transmit(&hi2c1, accel_addr, datar, 2, 1000);
}

/**
 * @brief writes in the register passed the data
 * @param regAddress the address on which we want to read
 * @param data the pointer to the array of bytes into we will store the read data
 * @return HAL_OK if sensor acknowledged, HAL_ERROR otherwise
 */
HAL_StatusTypeDef readReg(Reg regAddress, uint8_t *data, int nData) {
	HAL_StatusTypeDef ret = HAL_I2C_Master_Transmit(&hi2c1, accel_addr,
			&regAddress, 1, 1000);
	if (ret == HAL_OK) {
		return HAL_I2C_Master_Receive(&hi2c1, accel_addr | 0b1, data, nData,
				1000);
	} else {
		return ret;
	}
}

/**
 * @brief samples the accelerometer
 * @param accelData the structure to update
 * @return HAL_OK if new data has been sampled, HAL_ERROR otherwise
 */
HAL_StatusTypeDef sample(AccelData *accelData) {
	AccelData tempData;
	uint8_t data, data2;

	// sampling X value
	if (readReg(OUT_X, &data, 1) == HAL_OK) {
		tempData.acc_x = convertToAcc(data);
	} else {
		return HAL_ERROR;
	}

	// sampling Y value
	if (readReg(OUT_Y, &data, 1) == HAL_OK) {
		tempData.acc_y = convertToAcc(data);
	} else {
		return HAL_ERROR;
	}

	// sampling Z value
	if (readReg(OUT_Z, &data, 1) == HAL_OK) {
		tempData.acc_z = convertToAcc(data);
	} else {
		return HAL_ERROR;
	}

	// sampling TEMP value
	writeReg(CTRL_REG4, 0b10000000); // enabling bit to retrieve temperature
	if (readReg(OUT_TEMP_H, &data, 1) == HAL_OK) {
		if (readReg(OUT_TEMP_L, &data2, 1) == HAL_OK) {
			tempData.acc_t = convertToFloat(data, data2);
		} else {
			return HAL_ERROR;
		}
	} else {
		return HAL_ERROR;
	}
	writeReg(CTRL_REG4, 0b00000000); // disabling bit to retrieve acceleration

	memcpy(accelData, &tempData, sizeof(AccelData));
	return HAL_OK;
}

/**
 * @brief prints over the serial communication the sample passed
 * @param accelData the structure to print
 */
void printSampleDMA(AccelData accelData) {
	char str[64];
	int len = sprintf(str,
			"Non Blocking\r\nX: %+.3f\r\nY: %+.3f\r\nZ: %+.3f\r\n\n",
			accelData.acc_x, accelData.acc_y, accelData.acc_z);
	HAL_UART_Transmit_DMA(&huart2, str, len);
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
