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
#include "stdio.h"
#include "string.h"
#include "stdint.h"
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
} AccelData;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define accel_addr (0b0101000<<1) 	// address of the LIS2DE (shift by one for the R/W bit)
#define AUTOINCREMENT (0b1<<7)		// bit to set high if we want the auto increment (to set with the register address)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
DMA_HandleTypeDef hdma_i2c1_rx;

TIM_HandleTypeDef htim11;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */
/* constant messages to transmit */
const char *starting_str = "STARTING DMA PROJECT\r\n";
const char *tim_str = "tim callback\r\n";
const char *i2c_str = "i2c callback\r\n";
const char *error_str = "HAL_ERROR\r\n";

AccelData accelData; // structure that contains the last sample
uint8_t data[8]; // variable that contains the data sent in DMA mode by the sensor
char sample_str[64]; // string that contains the message to send over UART
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
HAL_StatusTypeDef sampleDMA(uint8_t *buf);
HAL_StatusTypeDef writeReg(Reg regAddress, int8_t value);
HAL_StatusTypeDef readRegDMA(Reg regAddress, uint8_t *data, int nData);
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
	MX_DMA_Init();
	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_USART2_UART_Init();
	MX_DMA_Init();
	MX_I2C1_Init();
	MX_TIM11_Init();
	/* USER CODE BEGIN 2 */

	if (init() == HAL_OK) {
		HAL_UART_Transmit(&huart2, starting_str, strlen(starting_str), 100); // DEBUG

		HAL_TIM_Base_Start_IT(&htim11);
	} else {
		HAL_UART_Transmit(&huart2, error_str, strlen(error_str), 100); // DEBUG
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
	/* DMA1_Stream5_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 1, 0);
	HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
	/* DMA1_Stream6_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 1, 0);
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
 * @brief callback of the timer, samples the data in DMA non-blocking mode
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim == &htim11) {
		HAL_UART_Transmit(&huart2, tim_str, strlen(tim_str), 100); // DEBUG
		sampleDMA(data);
	}
}

/**
 * @brief callback of the I2C receiver DMA, called when all the sample is transferred. Converts and prints in DMA mode the sample received
 */
void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c) {
	if (hi2c == &hi2c1) {
		HAL_UART_Transmit(&huart2, i2c_str, strlen(i2c_str), 100);  // DEBUG
		accelData.acc_x = convertToAcc(data[0]);
		accelData.acc_y = convertToAcc(data[2]);
		accelData.acc_z = convertToAcc(data[4]);
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

	return HAL_OK;
}

/**
 * @brief converts a byte into the acceleration in g
 * @param data the byte read from the accelerometer
 * @return the float signed value of the acceleration in g
 */
float convertToAcc(int8_t value) {
	// multiplying the read data by the FSR and dividing by 2^bytes=256
	return (float) value * 4 / 256.0;
}

/**
 * @brief writes in the register passed the data
 * @param regAddress the address on which we want to read
 * @param data the pointer to the array of bytes into we will store the read data
 * @param nData the number of bytes to read
 * @return HAL_OK if sensor acknowledged, HAL_ERROR otherwise
 */
HAL_StatusTypeDef readRegDMA(Reg regAddress, uint8_t *buf, int nData) {
	// setting the autoincrement bit in order to read nData DIFFERENT registers
	// (and not the same register nData times).
	if (nData > 1) {
		regAddress |= AUTOINCREMENT;
	}

	HAL_StatusTypeDef ret = HAL_I2C_Master_Transmit(&hi2c1, accel_addr,
			&regAddress, 1, 1000);
	if (ret == HAL_OK) {
		return HAL_I2C_Master_Receive_DMA(&hi2c1, accel_addr + 1, buf, 5);
	} else {
		HAL_UART_Transmit(&huart2, error_str, strlen(error_str), 100);  // DEBUG
		return ret;
	}
}

/**
 * @brief writes into the register the value passed
 * @param regAddress register to be written
 * @param value the value to write into the register
 * @return the same return of the HAL_I2C_Transmit
 */
HAL_StatusTypeDef writeReg(Reg regAddress, int8_t value) {
	uint8_t datar[2] = { regAddress, value };
	return HAL_I2C_Master_Transmit(&hi2c1, accel_addr, datar, 2, 1000);
}

/**
 * @brief samples the accelerometer in DMA mode.
 * In this case it's only a wrapper to the readRegDMA but it's kept anyway in order to keep the same structure and have a easily understandable code
 * @param accelData the structure to update
 * @return HAL_OK if new data has been sampled, HAL_ERROR otherwise
 */
HAL_StatusTypeDef sampleDMA(uint8_t *buf) {
	// sampling values from X to Z register included
	return readRegDMA(OUT_X, buf, 5);
}

/**
 * @brief prints over the serial communication the sample passed in DMA mode
 * @param accelData the structure to print
 */
void printSampleDMA(AccelData accelData) {
	sprintf(sample_str,
			"Non Blocking I2C DMA\r\nX: %+.3f\r\nY: %+.3f\r\nZ: %+.3f\r\n\n",
			accelData.acc_x, accelData.acc_y, accelData.acc_z);
	HAL_UART_Transmit_DMA(&huart2, sample_str, strlen(sample_str));
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
