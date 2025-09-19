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
#include <stdio.h>
#include <string.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
	uint8_t temperature;
	uint8_t humidity;
	uint8_t temp_decimal;
	uint8_t hum_decimal;
	uint8_t checksum;
} DHT11_Data;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define delay_ms HAL_Delay

//#define ADDRESS   0x3F << 1
#define ADDRESS   0x27 << 1

#define RS1_EN1   0x05
#define RS1_EN0   0x01
#define RS0_EN1   0x04
#define RS0_EN0   0x00
#define BackLight 0x08

#define DHT11_PORT GPIOA
#define DHT11_PIN GPIO_PIN_0

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
int delay = 0;
int value = 0;

DHT11_Data dht11_data;
char uart_buffer[100];  // uart_buffer 변수 선언 추가
char uart_buffer2[100];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
void I2C_ScanAddresses(void);

void delay_us(int us);
void LCD_DATA(uint8_t data);
void LCD_CMD(uint8_t cmd);
void LCD_CMD_4bit(uint8_t cmd);
void LCD_INIT(void);
void LCD_XY(char x, char y);
void LCD_CLEAR(void);
void LCD_PUTS(char *str);

void DHT11_SetPinOutput(void);
void DHT11_SetPinInput(void);
void DHT11_SetPin(GPIO_PinState state);
GPIO_PinState DHT11_ReadPin(void);
void DHT11_DelayUs(uint32_t us);
uint8_t DHT11_Start(void);
uint8_t DHT11_ReadBit(void);
uint8_t DHT11_ReadByte(void);
uint8_t DHT11_ReadData(DHT11_Data *data);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#ifdef __GNUC__
/* With GCC, small printf (option LD Linker->Libraries->Small printf
 set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

/**
 * @brief  Retargets the C library printf function to the USART.
 * @param  None
 * @retval None
 */
PUTCHAR_PROTOTYPE {
	/* Place your implementation of fputc here */
	/* e.g. write a character to the USART1 and Loop until the end of transmission */
	if (ch == '\n')
		HAL_UART_Transmit(&huart2, (uint8_t*) "\r", 1, 0xFFFF);
	HAL_UART_Transmit(&huart2, (uint8_t*) &ch, 1, 0xFFFF);

	return ch;
}

void I2C_ScanAddresses(void) {
	HAL_StatusTypeDef result;
	uint8_t i;

	printf("Scanning I2C addresses...\r\n");

	for (i = 1; i < 128; i++) {
		/*
		 * HAL_I2C_IsDeviceReady: If a device at the specified address exists return HAL_OK.
		 * Since I2C devices must have an 8-bit address, the 7-bit address is shifted left by 1 bit.
		 */
		result = HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t) (i << 1), 1, 10);
		if (result == HAL_OK) {
			printf("I2C device found at address 0x%02X\r\n", i);
		}
	}

	printf("Scan complete.\r\n");
}

void delay_us(int us) {
	value = 3;
	delay = us * value;
	for (int i = 0; i < delay; i++)
		;
}

void LCD_DATA(uint8_t data) {
	uint8_t temp = (data & 0xF0) | RS1_EN1 | BackLight;

	while (HAL_I2C_Master_Transmit(&hi2c1, ADDRESS, &temp, 1, 1000) != HAL_OK)
		;
	temp = (data & 0xF0) | RS1_EN0 | BackLight;
	while (HAL_I2C_Master_Transmit(&hi2c1, ADDRESS, &temp, 1, 1000) != HAL_OK)
		;
	delay_us(4);

	temp = ((data << 4) & 0xF0) | RS1_EN1 | BackLight;
	while (HAL_I2C_Master_Transmit(&hi2c1, ADDRESS, &temp, 1, 1000) != HAL_OK)
		;
	temp = ((data << 4) & 0xF0) | RS1_EN0 | BackLight;
	while (HAL_I2C_Master_Transmit(&hi2c1, ADDRESS, &temp, 1, 1000) != HAL_OK)
		;
	delay_us(50);
}

void LCD_CMD(uint8_t cmd) {
	uint8_t temp = (cmd & 0xF0) | RS0_EN1 | BackLight;
	while (HAL_I2C_Master_Transmit(&hi2c1, ADDRESS, &temp, 1, 1000) != HAL_OK)
		;
	temp = (cmd & 0xF0) | RS0_EN0 | BackLight;
	while (HAL_I2C_Master_Transmit(&hi2c1, ADDRESS, &temp, 1, 1000) != HAL_OK)
		;
	delay_us(4);

	temp = ((cmd << 4) & 0xF0) | RS0_EN1 | BackLight;
	while (HAL_I2C_Master_Transmit(&hi2c1, ADDRESS, &temp, 1, 1000) != HAL_OK)
		;
	temp = ((cmd << 4) & 0xF0) | RS0_EN0 | BackLight;
	while (HAL_I2C_Master_Transmit(&hi2c1, ADDRESS, &temp, 1, 1000) != HAL_OK)
		;
	delay_us(50);
}

void LCD_CMD_4bit(uint8_t cmd) {
	uint8_t temp = ((cmd << 4) & 0xF0) | RS0_EN1 | BackLight;
	while (HAL_I2C_Master_Transmit(&hi2c1, ADDRESS, &temp, 1, 1000) != HAL_OK)
		;
	temp = ((cmd << 4) & 0xF0) | RS0_EN0 | BackLight;
	while (HAL_I2C_Master_Transmit(&hi2c1, ADDRESS, &temp, 1, 1000) != HAL_OK)
		;
	delay_us(50);
}

void LCD_INIT(void) {

	delay_ms(100);

	LCD_CMD_4bit(0x03);
	delay_ms(5);
	LCD_CMD_4bit(0x03);
	delay_us(100);
	LCD_CMD_4bit(0x03);
	delay_us(100);
	LCD_CMD_4bit(0x02);
	delay_us(100);
	LCD_CMD(0x28);  // 4 bits, 2 line, 5x8 font
	LCD_CMD(0x08);  // display off, cursor off, blink off
	LCD_CMD(0x01);  // clear display
	delay_ms(3);
	LCD_CMD(0x06);  // cursor movint direction
	LCD_CMD(0x0C);  // display on, cursor off, blink off
}

void LCD_XY(char x, char y) {
	if (y == 0)
		LCD_CMD(0x80 + x);
	else if (y == 1)
		LCD_CMD(0xC0 + x);
	else if (y == 2)
		LCD_CMD(0x94 + x);
	else if (y == 3)
		LCD_CMD(0xD4 + x);
}

void LCD_CLEAR(void) {
	LCD_CMD(0x01);
	delay_ms(2);
}

void LCD_PUTS(char *str) {
	while (*str)
		LCD_DATA(*str++);

}

// DHT11 함수 구현
void DHT11_SetPinOutput(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	GPIO_InitStruct.Pin = DHT11_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(DHT11_PORT, &GPIO_InitStruct);
}

void DHT11_SetPinInput(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	GPIO_InitStruct.Pin = DHT11_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(DHT11_PORT, &GPIO_InitStruct);
}

void DHT11_SetPin(GPIO_PinState state) {
	HAL_GPIO_WritePin(DHT11_PORT, DHT11_PIN, state);
}

GPIO_PinState DHT11_ReadPin(void) {
	return HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN);
}

void DHT11_DelayUs(uint32_t us) {
	__HAL_TIM_SET_COUNTER(&htim2, 0);
	while (__HAL_TIM_GET_COUNTER(&htim2) < us)
		;
}

uint8_t DHT11_Start(void) {
	uint8_t response = 0;

	// 출력 모드로 설정
	DHT11_SetPinOutput();

	// 시작 신호 전송 (18ms LOW)
	DHT11_SetPin(GPIO_PIN_RESET);
	HAL_Delay(20);  // 18ms -> 20ms로 변경 (더 안정적)

	// HIGH로 변경 후 20-40us 대기
	DHT11_SetPin(GPIO_PIN_SET);
	DHT11_DelayUs(30);

	// 입력 모드로 변경
	DHT11_SetPinInput();

	// DHT11 응답 확인 (80us LOW + 80us HIGH)
	DHT11_DelayUs(40);

	if (!(DHT11_ReadPin())) {
		DHT11_DelayUs(80);
		if (DHT11_ReadPin()) {
			response = 1;
		} else {
			response = 0;
		}
	}

	// HIGH가 끝날 때까지 대기
	while (DHT11_ReadPin())
		;

	return response;
}

uint8_t DHT11_ReadBit(void) {
	// LOW 신호가 끝날 때까지 대기 (50us)
	while (!(DHT11_ReadPin()))
		;

	// HIGH 신호 시작 후 30us 대기
	DHT11_DelayUs(30);

	// 여전히 HIGH면 1, LOW면 0
	if (DHT11_ReadPin()) {
		// HIGH가 끝날 때까지 대기
		while (DHT11_ReadPin())
			;
		return 1;
	} else {
		return 0;
	}
}

uint8_t DHT11_ReadByte(void) {
	uint8_t byte = 0;
	for (int i = 0; i < 8; i++) {
		byte = (byte << 1) | DHT11_ReadBit();
	}
	return byte;
}

uint8_t DHT11_ReadData(DHT11_Data *data) {
	if (!DHT11_Start()) {
		return 0; // 시작 신호 실패
	}

	// 5바이트 데이터 읽기
	data->humidity = DHT11_ReadByte();
	data->hum_decimal = DHT11_ReadByte();
	data->temperature = DHT11_ReadByte();
	data->temp_decimal = DHT11_ReadByte();
	data->checksum = DHT11_ReadByte();

	// 체크섬 확인
	uint8_t calculated_checksum = data->humidity + data->hum_decimal
			+ data->temperature + data->temp_decimal;

	if (calculated_checksum == data->checksum) {
		return 1; // 성공
	} else {
		return 0; // 체크섬 오류
	}
}
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

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_USART2_UART_Init();
	MX_I2C1_Init();
	MX_TIM2_Init();
	/* USER CODE BEGIN 2 */
	// I2C_ScanAddresses();
	HAL_TIM_Base_Start(&htim2);

	// UART 초기화 메시지
	//sprintf(uart_buffer, "DHT11 Temperature & Humidity Sensor Test\r\n");
	HAL_UART_Transmit(&huart2, (uint8_t*) uart_buffer, strlen(uart_buffer),
	HAL_MAX_DELAY);
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {


		if (DHT11_ReadData(&dht11_data)) {
			// 데이터 읽기 성공
//	      sprintf(uart_buffer, "Temperature: %d °C, Humidity: %d%%\r\n",
//	              dht11_data.temperature, dht11_data.humidity); // 터미널에서 섭씨부분 글씨 꺠짐
//	      sprintf(uart_buffer, "Temperature: %d\xC2\xB0C, Humidity: %d%%\r\n",
//	              dht11_data.temperature, dht11_data.humidity);
//	      sprintf(uart_buffer, "Temperature: %d ℃, Humidity: %d%%\r\n",
//	              dht11_data.temperature, dht11_data.humidity);

			sprintf(uart_buffer, "Temperature: %d ℃", dht11_data.temperature);
			sprintf(uart_buffer2, "Humidity: %d%%", dht11_data.humidity);
			printf("\n");

			LCD_INIT();
			LCD_XY(0, 0);
			LCD_PUTS((char*) uart_buffer);
			LCD_XY(0, 1);
			LCD_PUTS((char*) uart_buffer2);

			HAL_UART_Transmit(&huart2, (uint8_t*) uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY);
			HAL_UART_Transmit(&huart2, (uint8_t*) uart_buffer2, strlen(uart_buffer), HAL_MAX_DELAY);
		} else {
			// 데이터 읽기 실패
			sprintf(uart_buffer, "DHT11 Read Error!\r\n");
			HAL_UART_Transmit(&huart2, (uint8_t*) uart_buffer,
					strlen(uart_buffer), HAL_MAX_DELAY);
		}

		// 2초 대기 (DHT11은 최소 2초 간격으로 읽어야 함)
		HAL_Delay(2000);
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

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void) {

	/* USER CODE BEGIN TIM2_Init 0 */

	/* USER CODE END TIM2_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 64 - 1;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 65535;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */

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
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */

	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : B1_Pin */
	GPIO_InitStruct.Pin = B1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : LD2_Pin */
	GPIO_InitStruct.Pin = LD2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

	/* USER CODE BEGIN MX_GPIO_Init_2 */

	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
